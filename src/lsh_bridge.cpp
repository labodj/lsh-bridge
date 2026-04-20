/**
 * @file    lsh_bridge.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the public LSH bridge facade and its internal runtime.
 *
 * Copyright 2026 Jacopo Labardi
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lsh_bridge.hpp"

#include <cstdint>
#include <cstring>
#include <utility>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <Homie.h>
#include <etl/bitset.h>
#include <etl/vector.h>

#include "communication/controller_serial_link.hpp"
#include "communication/mqtt_topics_builder.hpp"
#include "communication/mqtt_publisher.hpp"
#include "constants/communication_protocol.hpp"
#include "constants/configs/homie_identity.hpp"
#include "constants/configs/mqtt.hpp"
#include "constants/configs/runtime.hpp"
#include "constants/configs/virtual_device.hpp"
#include "constants/deserialize_exit_codes.hpp"
#include "constants/payloads.hpp"
#include "debug/debug.hpp"
#include "details_cache_store.hpp"
#include "lsh_node.hpp"
#include "utils/json_scalars.hpp"
#include "utils/time_keeper.hpp"
#include "virtual_device.hpp"

namespace
{

using constants::DeserializeExitCode;

/**
 * @brief Lookup table used to unpack packed actuator-state bytes.
 * @details Bit packing order is shared with the controller protocol:
 *          actuator 0 is bit 0, actuator 7 is bit 7, then packing continues
 *          into the next byte.
 */
constexpr std::uint8_t kPackedBitMask[8] = {0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x20U, 0x40U, 0x80U};

static constexpr const std::uint16_t kBridgeDiagnosticDocSize =
    192U;  //!< Document size used for bridge-local runtime payloads published on `bridge`.
static constexpr const std::uint16_t kBridgeDiagnosticCounterMax =
    0xFFFFU;                                                     //!< Saturation cap for diagnostic counters accumulated by the runtime.
constexpr char kBridgeEventKey[] = "event";                      //!< Field name that identifies the bridge-local runtime event type.
constexpr char kDiagnosticEvent[] = "diagnostic";                //!< Event name emitted for bridge-local diagnostics.
constexpr char kServicePingReplyEvent[] = "service_ping_reply";  //!< Event name emitted when the bridge answers a service-level ping probe.
constexpr char kBridgeDiagnosticKindKey[] = "kind";              //!< Field name that identifies the specific diagnostic kind.
constexpr char kPendingDurationMsKey[] = "pending_ms";  //!< Field name that reports how long an unstable actuator batch stayed open.
constexpr char kMutationCountKey[] =
    "mutation_count";  //!< Field name that reports how many command changes were merged into one dropped batch.
constexpr char kDroppedDeviceCommandsKey[] =
    "dropped_device_commands";  //!< Field name that reports how many device-topic MQTT commands were dropped because the queue was full.
constexpr char kDroppedServiceCommandsKey[] =
    "dropped_service_commands";  //!< Field name that reports how many service-topic MQTT commands were dropped because the queue was full.
constexpr char kControllerConnectedKey[] =
    "controller_connected";  //!< Field name that reports whether the bridge currently sees the controller link as alive.
constexpr char kRuntimeSynchronizedKey[] =
    "runtime_synchronized";  //!< Field name that reports whether the bridge runtime cache is synchronized with the controller.
constexpr char kBootstrapPhaseKey[] = "bootstrap_phase";  //!< Field name that reports the current bridge bootstrap phase.
constexpr char kActuatorStormDroppedDiagnostic[] =
    "actuator_command_storm_dropped";  //!< Diagnostic value emitted when the bridge discards an actuator batch that never became stable.
constexpr char kMqttQueueOverflowDiagnostic[] =
    "mqtt_queue_overflow";  //!< Diagnostic value emitted when MQTT callbacks outpace the bridge queue.
constexpr char kTopologyChangedDiagnostic[] =
    "topology_changed";  //!< Diagnostic value emitted when the controller reports a topology different from the cached one.

/**
 * @brief Resolve the UART used by the bridge runtime.
 *
 * @param serial user-supplied UART pointer, or `nullptr` to select the default UART.
 * @return HardwareSerial* selected UART instance for the bridge/controller link.
 */
[[nodiscard]] auto resolveSerial(HardwareSerial *serial) -> HardwareSerial *
{
    return serial != nullptr ? serial : &Serial2;
}

/**
 * @brief Decide whether bridge logging should be disabled for this runtime.
 *
 * @param mode runtime logging policy selected through `BridgeOptions`.
 * @return true when Homie logging should be disabled.
 * @return false when Homie logging may stay enabled.
 */
[[nodiscard]] auto shouldDisableLogging(lsh::bridge::LoggingMode mode) -> bool
{
    switch (mode)
    {
    case lsh::bridge::LoggingMode::Enabled:
        return false;

    case lsh::bridge::LoggingMode::Disabled:
        return true;

    case lsh::bridge::LoggingMode::AutoFromBuild:
    default:
        // In AutoFromBuild mode the runtime mirrors the compile-time choice:
        // debug builds keep Homie logs enabled, release builds silence them.
#ifdef LSH_DEBUG
        return false;
#else
        return true;
#endif
    }
}

}  // namespace

namespace lsh::bridge
{

class LSHBridge::Impl
{
public:
    /**
     * @brief Outcome of the bridge-side semantic handling for one MQTT command.
     */
    enum class TypedCommandResult : std::uint8_t
    {
        Handled,      //!< The bridge accepted and handled the command.
        Unsupported,  //!< The bridge does not interpret this command type.
        Invalid       //!< The command type is known, but this concrete payload is invalid.
    };

    /**
     * @brief Distinguishes the MQTT topic family that produced an inbound
     * payload.
     */
    enum class MqttCommandSource : std::uint8_t
    {
        Device,  //!< Device-scoped command topic.
        Service  //!< Device-agnostic service topic.
    };

    /**
     * @brief Tracks how far the bridge progressed in the controller sync flow.
     * @details The bridge may already be online while it sits in one of the
     *          waiting states. The controller remains authoritative; the bridge
     *          simply uses the phase to decide which request is safe and useful
     *          to send next.
     */
    enum class BootstrapPhase : std::uint8_t
    {
        WAITING_FOR_DETAILS,               //!< The bridge needs a fresh authoritative `DEVICE_DETAILS` snapshot.
        WAITING_FOR_STATE,                 //!< Topology is known, but the first fresh `STATE` frame has not arrived yet.
        SYNCED,                            //!< Topology and runtime state are both aligned with the controller.
        TOPOLOGY_MIGRATION_PENDING_REBOOT  //!< A different topology was received and cached; the bridge waits to reboot cleanly.
    };

    /**
     * @brief Stores one MQTT payload until the loop is ready to process it.
     * @details MQTT callbacks can fire while the serial path is busy. The bridge
     *          therefore copies complete MQTT frames into a small fixed queue and
     *          drains that queue from the main loop with its own fairness policy.
     */
    struct QueuedMqttCommand
    {
        MqttCommandSource source = MqttCommandSource::Device;
        std::uint16_t length = 0U;
        std::uint8_t payload[constants::controllerSerial::MQTT_COMMAND_MESSAGE_MAX_SIZE]{};
    };

    explicit Impl(BridgeOptions bridgeOptions) :
        options(std::move(bridgeOptions)), serial(resolveSerial(options.serial)), controllerSerialLink(serial, virtualDevice)
    {
        options.serial = serial;
    }

    BridgeOptions options{};
    bool isStarted = false;  //!< True after `begin()` has fully wired serial, Homie and callbacks.
    BootstrapPhase bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;  //!< Current high-level controller sync phase.
    bool bootstrapRequestDue = true;                                      //!< True when the next bootstrap request may be sent immediately.
    bool hasPendingTopologySave = false;        //!< True while a changed topology still has to be persisted to NVS.
    bool isAuthoritativeStateDirty = false;     //!< True after fresh controller state arrived but is not yet mirrored to MQTT/Homie.
    bool canServeCachedStateRequests = false;   //!< True only after this MQTT session has already seen one fresh controller-backed state.
    bool isControllerConnected = false;         //!< Last known controller connectivity bit, used to detect link transitions.
    std::uint32_t lastBootstrapRequestMs = 0U;  //!< Real-time timestamp of the last `ASK_DETAILS` or `ASK_STATE`.
    std::uint32_t lastTopologySaveAttemptMs = 0U;       //!< Real-time timestamp of the last deferred NVS save attempt.
    std::uint32_t lastAuthoritativeStateUpdateMs = 0U;  //!< Real-time timestamp of the latest accepted authoritative state frame.
    etl::vector<LSHNode, constants::virtualDevice::MAX_ACTUATORS> homieNodes{};
    DeviceDetailsSnapshot pendingTopologyDetails{};  //!< Validated topology waiting to be persisted before a controlled reboot.
    VirtualDevice virtualDevice{};
    HardwareSerial *const serial;
    ControllerSerialLink controllerSerialLink;
    AsyncMqttClient *mqttClient = nullptr;
    AsyncMqttClient *mqttMessageBoundClient = nullptr;
    QueuedMqttCommand mqttCommandQueue[constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY]{};
    portMUX_TYPE mqttCommandQueueMux = portMUX_INITIALIZER_UNLOCKED;
    std::uint8_t mqttCommandQueueHead = 0U;
    std::uint8_t mqttCommandQueueTail = 0U;
    std::uint8_t mqttCommandQueueCount = 0U;
    std::uint16_t droppedDeviceMqttCommandCount = 0U;
    std::uint16_t droppedServiceMqttCommandCount = 0U;

    void configureHomie() const
    {
        // Homie's public macros require string literals because they prepend and
        // append the legacy flag bytes through preprocessor concatenation.
        // The bridge therefore configures firmware identity at compile time only.
        Homie_setFirmware(CONFIG_HOMIE_FIRMWARE_NAME, CONFIG_HOMIE_FIRMWARE_VERSION);
        Homie_setBrand(CONFIG_HOMIE_BRAND);

        if (options.disableLedFeedback)
        {
            Homie.disableLedFeedback();
        }

        if (shouldDisableLogging(options.loggingMode))
        {
            Homie.disableLogging();
        }
    }

    /**
     * @brief Rebuilds Homie nodes from the cached controller topology.
     */
    void initializeNodesFromModel()
    {
        homieNodes.clear();
        MqttTopicsBuilder::updateMqttTopics(virtualDevice.getName().c_str());

        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < virtualDevice.getTotalActuators(); ++actuatorIndex)
        {
            const auto actuatorId = virtualDevice.getActuatorId(actuatorIndex);
            homieNodes.emplace_back(controllerSerialLink, virtualDevice, actuatorIndex, actuatorId);
        }
    }

    /**
     * @brief Restore cached controller details from NVS into the in-memory model.
     * @details This is the only place that reads bridge-owned persistent state.
     *          A failed load is intentionally non-fatal: the bridge can still
     *          come online and wait for fresh `DEVICE_DETAILS`.
     *
     * @return true if one valid cached topology has been loaded into
     *         `virtualDevice` and MQTT/Homie nodes have been rebuilt from it.
     * @return false if no valid cache was available; the bridge will stay online
     *         and wait for fresh `DEVICE_DETAILS` from the controller.
     */
    [[nodiscard]] auto loadCachedDetailsFromFlash() -> bool
    {
        DeviceDetailsSnapshot cachedDetails{};
        if (!DetailsCacheStore::load(cachedDetails))
        {
            return false;
        }

        virtualDevice.setDetails(cachedDetails);
        initializeNodesFromModel();
        return true;
    }

    /**
     * @brief Return whether the bridge already knows device-scoped MQTT topics.
     * @details Device topics exist only after the bridge has a validated topology
     *          snapshot. Before that point the bridge supports OTA and the
     *          service topic, but it intentionally avoids publishing under a
     *          made-up controller identity.
     *
     * @return true if the bridge has one cached controller topology and the
     *         derived device-scoped MQTT topics have been built from it.
     * @return false if the bridge is cacheless and therefore cannot safely
     *         expose device-scoped topics yet.
     */
    [[nodiscard]] auto hasDeviceScopedTopics() const -> bool
    {
        return virtualDevice.hasCachedDetails() && !MqttTopicsBuilder::mqttInTopic.empty();
    }

    /**
     * @brief Return whether the bridge can currently publish on its `bridge` topic.
     * @details The bridge topic is device-scoped, so it exists only after a
     *          validated controller topology has been loaded from cache or
     *          learned from the controller and persisted for the next reboot.
     *
     * @return true if the bridge-local MQTT topic string is currently valid.
     * @return false if the bridge has no authoritative controller identity
     *         to publish under.
     */
    [[nodiscard]] auto hasBridgeTopic() const -> bool
    {
        return virtualDevice.hasCachedDetails() && !MqttTopicsBuilder::mqttOutBridgeTopic.empty();
    }

    /**
     * @brief Return the human-readable name of the current bootstrap phase.
     * @details Exposed in service-level ping replies so external observers can
     *          tell whether the bridge is online, waiting for topology, waiting
     *          for state, or about to reboot after a topology migration.
     *
     * @return const char * stable string literal describing `bootstrapPhase`.
     */
    [[nodiscard]] auto bootstrapPhaseName() const -> const char *
    {
        switch (bootstrapPhase)
        {
        case BootstrapPhase::WAITING_FOR_DETAILS:
            return "waiting_details";

        case BootstrapPhase::WAITING_FOR_STATE:
            return "waiting_state";

        case BootstrapPhase::SYNCED:
            return "synced";

        case BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT:
            return "topology_migration_pending_reboot";

        default:
            return "waiting_details";
        }
    }

    /**
     * @brief Drops transient runtime state that depends on a fresh authoritative
     * controller sync.
     */
    void clearPendingRuntimeState()
    {
        controllerSerialLink.clearPendingActuatorBatch();
        isAuthoritativeStateDirty = false;
        canServeCachedStateRequests = false;
    }

    /**
     * @brief Ask the loop to send the next bootstrap request as soon as allowed.
     * @details Requests are rate-limited in the main loop so the bridge never
     *          pounds the controller in a tight retry loop.
     */
    void scheduleBootstrapRequestNow()
    {
        bootstrapRequestDue = true;
        lastBootstrapRequestMs = 0U;
    }

    /**
     * @brief Enter the phase that waits for fresh authoritative `DEVICE_DETAILS`.
     * @details This is the safest phase after a controller `BOOT` because the
     *          bridge must re-confirm the topology before trusting actuator IDs,
     *          button IDs, or the controller-backed event channels.
     */
    void enterWaitingForDetails()
    {
        clearPendingRuntimeState();
        virtualDevice.invalidateRuntimeModel();
        bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;
        scheduleBootstrapRequestNow();
    }

    /**
     * @brief Enter the phase that waits for a fresh authoritative `STATE` frame.
     * @details Used after confirmed topology, after controller reconnect, and
     *          after any other runtime desynchronization where the topology is
     *          trusted but the cached state is not. This helper does not
     *          send serial traffic itself; it only resets state and schedules
     *          the next retry for the main loop.
     */
    void enterWaitingForState()
    {
        clearPendingRuntimeState();
        virtualDevice.invalidateRuntimeModel();
        bootstrapPhase = BootstrapPhase::WAITING_FOR_STATE;
        scheduleBootstrapRequestNow();
    }

    /**
     * @brief Stage a topology migration that must be persisted before rebooting.
     * @details The bridge does not hot-rebuild Homie nodes at runtime because
     *          that logic is both fragile and hard to read. Instead it persists
     *          the new topology, then performs one controlled reboot so startup
     *          can recreate MQTT topics and nodes from a coherent baseline.
     *
     * @param details validated controller topology that differs from the
     *        active cached one and therefore requires one persistent update plus
     *        a controlled reboot.
     */
    void stageTopologyMigration(const DeviceDetailsSnapshot &details)
    {
        clearPendingRuntimeState();
        virtualDevice.invalidateRuntimeModel();
        pendingTopologyDetails = details;
        hasPendingTopologySave = true;
        lastTopologySaveAttemptMs = 0U;
        bootstrapPhase = BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT;
    }

    /**
     * @brief Publishes the cached controller details snapshot to MQTT.
     * @details The topic is retained because it represents controller topology,
     *          not transient runtime state. Callers use this only after a
     *          validated topology is already cached in memory.
     *
     * @return true if the cached topology payload has been serialized and
     *         accepted by the MQTT client.
     * @return false if the bridge has no cached topology yet or if publish
     *         fails because MQTT/session state is not ready.
     */
    [[nodiscard]] auto publishCachedDetails() -> bool
    {
        if (!virtualDevice.hasCachedDetails())
        {
            return false;
        }

        StaticJsonDocument<constants::controllerSerial::JSON_RECEIVED_MAX_SIZE> detailsDoc;
        if (!virtualDevice.populateDetailsDocument(detailsDoc))
        {
            return false;
        }

        return MqttPublisher::sendJson(detailsDoc, MqttTopicsBuilder::mqttOutConfTopic.c_str(), true, 1);
    }

    /**
     * @brief Requests a fresh authoritative state frame from the controller.
     * @details The actual `ASK_STATE` send is deferred to the bootstrap state
     *          machine so the bridge keeps one pacing rule for startup,
     *          reconnect and desynchronization recovery.
     */
    void requestAuthoritativeStateRefresh()
    {
        enterWaitingForState();
    }

    /**
     * @brief Re-synchronizes the MQTT-side peer without tearing down the whole
     * bridge.
     */
    void softResyncMqttPeer()
    {
        if (!virtualDevice.hasCachedDetails())
        {
            canServeCachedStateRequests = false;
            bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;
            scheduleBootstrapRequestNow();
            return;
        }

        if (!publishCachedDetails())
        {
            canServeCachedStateRequests = false;
            return;
        }

        const bool controllerStateUsable = virtualDevice.isRuntimeSynchronized() && controllerSerialLink.isConnected();
        if (controllerStateUsable)
        {
            canServeCachedStateRequests = publishAuthoritativeLshState();
            return;
        }

        canServeCachedStateRequests = false;
        virtualDevice.invalidateRuntimeModel();

        if (bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
        {
            scheduleBootstrapRequestNow();
            return;
        }

        if (bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            enterWaitingForState();
        }
    }

    /**
     * @brief Tracks controller connectivity transitions and reacts to state
     * loss/recovery.
     */
    void refreshControllerConnectivity()
    {
        const bool isConnectedNow = controllerSerialLink.isConnected();
        if (isConnectedNow == isControllerConnected)
        {
            return;
        }

        isControllerConnected = isConnectedNow;
        if (!isConnectedNow)
        {
            // Once serial connectivity is lost, any cached runtime state may be stale.
            // Keep topology, but require a fresh authoritative state before serving
            // cached answers or translating actuator writes again.
            clearPendingRuntimeState();
            virtualDevice.invalidateRuntimeModel();
            if (bootstrapPhase != BootstrapPhase::WAITING_FOR_DETAILS &&
                bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
            {
                bootstrapPhase = BootstrapPhase::WAITING_FOR_STATE;
                scheduleBootstrapRequestNow();
            }
            return;
        }

        if (bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
        {
            scheduleBootstrapRequestNow();
            return;
        }

        if (bootstrapPhase == BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            return;
        }

        if (!virtualDevice.isRuntimeSynchronized())
        {
            // The controller came back while the bridge considers the runtime
            // model stale, so ask immediately for a new authoritative state frame.
            requestAuthoritativeStateRefresh();
        }
    }

    /**
     * @brief Copies one complete MQTT frame into the fixed bridge-side queue.
     * @details The MQTT callback may run while the serial path is busy. The
     *          bridge therefore keeps the callback short: it validates the frame,
     *          copies it into a ring buffer under a critical section, and lets the
     *          main loop parse it later.
     */
    [[nodiscard]] auto enqueueMqttCommand(MqttCommandSource source, const char *payload, std::size_t payloadLength) -> bool
    {
        if (payload == nullptr || payloadLength == 0U || payloadLength > constants::controllerSerial::MQTT_COMMAND_MESSAGE_MAX_SIZE)
        {
            return false;
        }

        bool wasQueued = false;
        portENTER_CRITICAL(&mqttCommandQueueMux);
        if (mqttCommandQueueCount < constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY)
        {
            auto &slot = mqttCommandQueue[mqttCommandQueueTail];
            slot.source = source;
            std::memcpy(slot.payload, payload, payloadLength);
            slot.length = static_cast<std::uint16_t>(payloadLength);
            mqttCommandQueueTail = static_cast<std::uint8_t>((mqttCommandQueueTail + 1U) % constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY);
            ++mqttCommandQueueCount;
            wasQueued = true;
        }
        portEXIT_CRITICAL(&mqttCommandQueueMux);

        return wasQueued;
    }

    /**
     * @brief Pops the oldest queued MQTT frame from the fixed ring buffer.
     * @details Head/tail/count are protected by `mqttCommandQueueMux` because the
     *          callback path and the main loop can touch the queue concurrently.
     */
    [[nodiscard]] auto dequeueMqttCommand(QueuedMqttCommand &outCommand) -> bool
    {
        bool wasDequeued = false;
        portENTER_CRITICAL(&mqttCommandQueueMux);
        if (mqttCommandQueueCount > 0U)
        {
            outCommand = mqttCommandQueue[mqttCommandQueueHead];
            mqttCommandQueueHead = static_cast<std::uint8_t>((mqttCommandQueueHead + 1U) % constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY);
            --mqttCommandQueueCount;
            wasDequeued = true;
        }
        portEXIT_CRITICAL(&mqttCommandQueueMux);

        return wasDequeued;
    }

    /**
     * @brief Drops every queued MQTT frame.
     * @details Used when MQTT disconnects so the bridge does not replay commands
     *          captured before the next MQTT session starts.
     */
    void clearQueuedMqttCommands()
    {
        portENTER_CRITICAL(&mqttCommandQueueMux);
        mqttCommandQueueHead = 0U;
        mqttCommandQueueTail = 0U;
        mqttCommandQueueCount = 0U;
        portEXIT_CRITICAL(&mqttCommandQueueMux);
    }

    /**
     * @brief Forget every bridge-local diagnostic that belongs to the MQTT
     *        session being torn down.
     */
    void clearPendingBridgeDiagnostics()
    {
        controllerSerialLink.clearPendingActuatorStormDiagnostic();

        portENTER_CRITICAL(&mqttCommandQueueMux);
        droppedDeviceMqttCommandCount = 0U;
        droppedServiceMqttCommandCount = 0U;
        portEXIT_CRITICAL(&mqttCommandQueueMux);
    }

    /**
     * @brief Aggregate one MQTT queue overflow occurrence.
     * @details The callback may drop many commands in quick succession. Instead
     *          of publishing one diagnostic per drop, the bridge keeps a small
     *          per-topic-family counter and publishes an aggregated event later
     *          from the main loop.
     */
    void recordDroppedQueuedMqttCommand(MqttCommandSource source)
    {
        portENTER_CRITICAL(&mqttCommandQueueMux);
        if (source == MqttCommandSource::Device)
        {
            if (droppedDeviceMqttCommandCount < kBridgeDiagnosticCounterMax)
            {
                ++droppedDeviceMqttCommandCount;
            }
        }
        else
        {
            if (droppedServiceMqttCommandCount < kBridgeDiagnosticCounterMax)
            {
                ++droppedServiceMqttCommandCount;
            }
        }
        portEXIT_CRITICAL(&mqttCommandQueueMux);
    }

    /**
     * @brief Publish pending bridge-local diagnostics on the `bridge` topic.
     * @details Diagnostics are emitted from the main loop, not from callbacks, so
     *          the bridge never tries to publish MQTT while already executing the
     *          MQTT client's message callback.
     */
    void publishPendingBridgeDiagnostics()
    {
        if (!Homie.isConnected() || !hasBridgeTopic())
        {
            return;
        }

        ControllerSerialLink::ActuatorStormDiagnostic actuatorStormDiagnostic{};
        if (controllerSerialLink.peekPendingActuatorStormDiagnostic(actuatorStormDiagnostic))
        {
            StaticJsonDocument<kBridgeDiagnosticDocSize> diagnosticDoc;
            diagnosticDoc[kBridgeEventKey] = kDiagnosticEvent;
            diagnosticDoc[kBridgeDiagnosticKindKey] = kActuatorStormDroppedDiagnostic;
            diagnosticDoc[kPendingDurationMsKey] = actuatorStormDiagnostic.pendingDurationMs;
            diagnosticDoc[kMutationCountKey] = actuatorStormDiagnostic.mutationCount;

            if (MqttPublisher::sendJson(diagnosticDoc, MqttTopicsBuilder::mqttOutBridgeTopic.c_str(), false, 1))
            {
                controllerSerialLink.clearPendingActuatorStormDiagnostic();
            }
        }

        std::uint16_t droppedDeviceCommands = 0U;
        std::uint16_t droppedServiceCommands = 0U;
        portENTER_CRITICAL(&mqttCommandQueueMux);
        droppedDeviceCommands = droppedDeviceMqttCommandCount;
        droppedServiceCommands = droppedServiceMqttCommandCount;
        portEXIT_CRITICAL(&mqttCommandQueueMux);

        if (droppedDeviceCommands == 0U && droppedServiceCommands == 0U)
        {
            return;
        }

        StaticJsonDocument<kBridgeDiagnosticDocSize> diagnosticDoc;
        diagnosticDoc[kBridgeEventKey] = kDiagnosticEvent;
        diagnosticDoc[kBridgeDiagnosticKindKey] = kMqttQueueOverflowDiagnostic;
        if (droppedDeviceCommands > 0U)
        {
            diagnosticDoc[kDroppedDeviceCommandsKey] = droppedDeviceCommands;
        }
        if (droppedServiceCommands > 0U)
        {
            diagnosticDoc[kDroppedServiceCommandsKey] = droppedServiceCommands;
        }

        if (MqttPublisher::sendJson(diagnosticDoc, MqttTopicsBuilder::mqttOutBridgeTopic.c_str(), false, 1))
        {
            portENTER_CRITICAL(&mqttCommandQueueMux);
            droppedDeviceMqttCommandCount = (droppedDeviceMqttCommandCount > droppedDeviceCommands)
                                                ? static_cast<std::uint16_t>(droppedDeviceMqttCommandCount - droppedDeviceCommands)
                                                : 0U;
            droppedServiceMqttCommandCount = (droppedServiceMqttCommandCount > droppedServiceCommands)
                                                 ? static_cast<std::uint16_t>(droppedServiceMqttCommandCount - droppedServiceCommands)
                                                 : 0U;
            portEXIT_CRITICAL(&mqttCommandQueueMux);
        }
    }

    /**
     * @brief Publish one simple bridge-local diagnostic with no extra payload.
     * @details Used for low-cardinality lifecycle events such as topology
     *          migration. More detailed diagnostics go through the richer code
     *          paths above.
     *
     * @param diagnosticKind short stable identifier of the lifecycle event to
     *        publish, for example `topology_changed`.
     */
    void publishSimpleBridgeDiagnostic(const char *diagnosticKind) const
    {
        if (!Homie.isConnected() || !hasBridgeTopic())
        {
            return;
        }

        StaticJsonDocument<kBridgeDiagnosticDocSize> diagnosticDoc;
        diagnosticDoc[kBridgeEventKey] = kDiagnosticEvent;
        diagnosticDoc[kBridgeDiagnosticKindKey] = diagnosticKind;
        (void)MqttPublisher::sendJson(diagnosticDoc, MqttTopicsBuilder::mqttOutBridgeTopic.c_str(), false, 1);
    }

    /**
     * @brief Publish the service-topic `PING` reply on the bridge-local topic.
     * @details This reply intentionally reports bridge/runtime health only. It
     *          never claims that the controller-backed device topic is usable
     *          unless both controller connectivity and runtime synchronization
     *          are currently true.
     */
    void publishServicePingReply() const
    {
        if (!Homie.isConnected() || !hasBridgeTopic())
        {
            return;
        }

        StaticJsonDocument<kBridgeDiagnosticDocSize> bridgeDoc;
        bridgeDoc[kBridgeEventKey] = kServicePingReplyEvent;
        bridgeDoc[kControllerConnectedKey] = controllerSerialLink.isConnected();
        bridgeDoc[kRuntimeSynchronizedKey] = virtualDevice.isRuntimeSynchronized();
        bridgeDoc[kBootstrapPhaseKey] = bootstrapPhaseName();
        (void)MqttPublisher::sendJson(bridgeDoc, MqttTopicsBuilder::mqttOutBridgeTopic.c_str(), false, 1);
    }

    /**
     * @brief Forget transient MQTT/session state after Wi-Fi or MQTT loss.
     * @details Topology and the in-memory controller model are intentionally
     *          preserved. Only session-scoped items are dropped so a reconnect
     *          can reuse cached topology and re-confirm runtime state.
     */
    void handleDisconnect()
    {
        mqttClient = nullptr;
        clearPendingRuntimeState();
        clearQueuedMqttCommands();
        clearPendingBridgeDiagnostics();
    }

    /**
     * @brief Retry the next bootstrap request at a slow fixed cadence.
     * @details The bridge sends at most one `ASK_DETAILS` or `ASK_STATE` per
     *          retry window. Requests continue even when the controller is
     *          currently silent so the bridge can discover a freshly powered-on
     *          or freshly rebooted core without a misleading "connected" guess.
     *          The fixed cadence keeps the controller firmly in charge and keeps
     *          bootstrap traffic predictable.
     */
    void processBootstrapProgress()
    {
        if (bootstrapPhase == BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            return;
        }

        if (bootstrapPhase != BootstrapPhase::WAITING_FOR_DETAILS && bootstrapPhase != BootstrapPhase::WAITING_FOR_STATE)
        {
            return;
        }

        const auto now = timeKeeper::getRealTime();
        if (!bootstrapRequestDue && (now - lastBootstrapRequestMs) < constants::runtime::BOOTSTRAP_REQUEST_INTERVAL_MS)
        {
            return;
        }

        if (bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
        {
            (void)controllerSerialLink.sendJson(constants::payloads::StaticType::ASK_DETAILS);
        }
        else
        {
            (void)controllerSerialLink.sendJson(constants::payloads::StaticType::ASK_STATE);
        }

        lastBootstrapRequestMs = now;
        bootstrapRequestDue = false;
    }

    /**
     * @brief Persist a changed topology and reboot once persistence succeeds.
     * @details The actual NVS write is deliberately deferred to the main loop so
     *          the serial callback never pays the latency cost of flash I/O.
     *          Until persistence succeeds the bridge stays in
     *          `TOPOLOGY_MIGRATION_PENDING_REBOOT` and intentionally does not
     *          send more bootstrap requests, because the active MQTT/Homie model
     *          is already known to be structurally stale.
     */
    void processPendingTopologyMigration()
    {
        if (!hasPendingTopologySave || bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            return;
        }

        const auto now = timeKeeper::getRealTime();
        if (lastTopologySaveAttemptMs != 0U && (now - lastTopologySaveAttemptMs) < constants::runtime::BOOTSTRAP_REQUEST_INTERVAL_MS)
        {
            return;
        }

        lastTopologySaveAttemptMs = now;
        if (!DetailsCacheStore::save(pendingTopologyDetails))
        {
            return;
        }

        hasPendingTopologySave = false;
        publishSimpleBridgeDiagnostic(kTopologyChangedDiagnostic);
        Homie.reboot();
    }

    void markAuthoritativeStateDirty()
    {
        // A fresh controller-backed state arrived. Delay publication slightly so
        // quick successive state frames collapse into one MQTT/Homie refresh wave.
        isAuthoritativeStateDirty = true;
        canServeCachedStateRequests = false;
        lastAuthoritativeStateUpdateMs = timeKeeper::getRealTime();
    }

    /**
     * @brief Publishes the authoritative compact LSH state topic.
     *
     * @return true if the authoritative packed state has been serialized and
     *         accepted by the MQTT client.
     * @return false if device-scoped topics are unavailable or if MQTT
     *         publish could not be performed.
     */
    [[nodiscard]] auto publishAuthoritativeLshState() -> bool
    {
        using namespace lsh::bridge::protocol;

        if (!hasDeviceScopedTopics())
        {
            return false;
        }

        StaticJsonDocument<constants::controllerSerial::MQTT_SET_STATE_DOC_SIZE> stateDoc;
        stateDoc[KEY_PAYLOAD] = static_cast<std::uint8_t>(Command::ACTUATORS_STATE);
        JsonArray packedStates = stateDoc.createNestedArray(KEY_STATE);

        const auto totalActuators = virtualDevice.getTotalActuators();
        std::uint8_t packedByte = 0U;

        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
        {
            if (virtualDevice.getStateByIndex(actuatorIndex))
            {
                // `actuatorIndex & 0x07U` is equivalent to `actuatorIndex % 8`, but it
                // makes it explicit that only the bit position inside the current byte
                // matters here.
                packedByte |= kPackedBitMask[actuatorIndex & 0x07U];
            }

            const bool isEndOfPackedByte = ((actuatorIndex & 0x07U) == 0x07U);
            const bool isLastActuator = (actuatorIndex == (totalActuators - 1U));
            if (isEndOfPackedByte || isLastActuator)
            {
                packedStates.add(packedByte);
                packedByte = 0U;
            }
        }

        return MqttPublisher::sendJson(stateDoc, MqttTopicsBuilder::mqttOutStateTopic.c_str(), true, 1);
    }

    /**
     * @brief Serves `REQUEST_STATE` directly from cache when the bridge has
     * already published a fresh controller-backed snapshot in the current MQTT
     * session.
     *
     * @return true if the bridge has a fresh controller-backed state snapshot
     *         and successfully republishes it from cache.
     * @return false if the runtime model is stale, the controller link is down,
     *         device topics are unavailable, or the MQTT publish fails.
     */
    [[nodiscard]] auto tryServeCachedStateRequest() -> bool
    {
        if (!canServeCachedStateRequests || !virtualDevice.isRuntimeSynchronized() || !controllerSerialLink.isConnected() ||
            !hasDeviceScopedTopics())
        {
            return false;
        }

        DPL("Serving REQUEST_STATE directly from the synchronized bridge cache.");
        return publishAuthoritativeLshState();
    }

    [[nodiscard]] auto publishAllHomieNodeStates() -> bool
    {
        bool allPublished = true;
        for (const auto &node : homieNodes)
        {
            if (!node.sendState())
            {
                allPublished = false;
            }
        }

        return allPublished;
    }

    [[nodiscard]] auto publishChangedHomieNodeStates() -> bool
    {
        bool allPublished = true;
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < virtualDevice.getTotalActuators(); ++actuatorIndex)
        {
            if (virtualDevice.isActuatorDirty(actuatorIndex))
            {
                if (!homieNodes[actuatorIndex].sendState())
                {
                    allPublished = false;
                }
            }
        }

        return allPublished;
    }

    /**
     * @brief Flushes pending authoritative state changes to MQTT after a short
     * settle window.
     */
    void processPendingStatePublishes()
    {
        if (!isAuthoritativeStateDirty)
        {
            return;
        }

        const auto now = timeKeeper::getRealTime();
        if ((now - lastAuthoritativeStateUpdateMs) < constants::runtime::STATE_PUBLISH_SETTLE_INTERVAL_MS)
        {
            return;
        }

        if (!publishAuthoritativeLshState())
        {
            return;
        }

        bool homiePublished = false;
        if (virtualDevice.consumeFullStatePublishPending())
        {
            // Right after bootstrap or re-sync every node must publish once so Homie
            // definitely matches the fresh authoritative snapshot.
            homiePublished = publishAllHomieNodeStates();
        }
        else
        {
            // Normal steady state: only the actuators touched by the latest
            // authoritative frame need to be mirrored back to Homie.
            homiePublished = publishChangedHomieNodeStates();
        }

        if (!homiePublished)
        {
            DPL("Keeping authoritative state dirty because at least one Homie publish was not accepted.");
            return;
        }

        virtualDevice.clearDirtyActuators();
        isAuthoritativeStateDirty = false;
        canServeCachedStateRequests = true;
    }

    /**
     * @brief Handle a controller `BOOT` frame.
     * @details `BOOT` is treated as a strong hint that topology may have
     *          changed. The bridge therefore stops trusting runtime state and
     *          asks for fresh `DEVICE_DETAILS` before doing anything else.
     */
    void handleControllerBootMessage()
    {
        enterWaitingForDetails();
    }

    /**
     * @brief Handle a freshly received controller `DEVICE_DETAILS` frame.
     * @details If the topology matches the cached one, the bridge can keep
     *          running and only needs a new `STATE`. If the topology changed,
     *          the bridge persists it and then performs one controlled reboot so
     *          startup can rebuild MQTT topics and Homie nodes coherently. On a
     *          first boot without any cache, the bridge also takes this path:
     *          it saves the first validated topology and reboots once so the
     *          steady-state runtime always starts from one coherent cached model.
     */
    void handleControllerDetailsMessage()
    {
        DeviceDetailsSnapshot receivedDetails{};
        if (controllerSerialLink.parseDetailsFromReceived(receivedDetails) != DeserializeExitCode::OK_DETAILS)
        {
            enterWaitingForDetails();
            return;
        }

        if (virtualDevice.matchesDetails(receivedDetails))
        {
            hasPendingTopologySave = false;
            pendingTopologyDetails.clear();
            enterWaitingForState();
            return;
        }

        stageTopologyMigration(receivedDetails);
    }

    /**
     * @brief Handle a freshly received controller `STATE` frame.
     * @details State frames are authoritative only after topology is known. If
     *          the bridge waits for details, it ignores the frame and keeps
     *          asking for topology. A malformed state frame does not reboot the
     *          bridge: topology mismatches fall back to `WAITING_FOR_DETAILS`,
     *          while every other state-shape problem falls back to
     *          `WAITING_FOR_STATE`.
     */
    void handleControllerStateMessage()
    {
        if (bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS || bootstrapPhase == BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            return;
        }

        const auto stateResult = controllerSerialLink.storeStateFromReceived();
        if (stateResult == DeserializeExitCode::OK_STATE)
        {
            controllerSerialLink.reconcileDesiredActuatorStatesFromAuthoritative();
            bootstrapPhase = BootstrapPhase::SYNCED;
            markAuthoritativeStateDirty();
            return;
        }

        if (stateResult == DeserializeExitCode::ERR_ACTUATORS_MISMATCH)
        {
            DPL("Dropping malformed controller STATE frame because its packed size does not match the cached topology. "
                "The bridge will request fresh DEVICE_DETAILS before trusting runtime state again.");
            enterWaitingForDetails();
            return;
        }

        DPL("Dropping malformed controller STATE frame. The bridge will request a fresh authoritative STATE before "
            "serving controller-backed traffic again. Exit code: ",
            static_cast<std::uint8_t>(stateResult));
        enterWaitingForState();
    }

    /**
     * @brief Dispatch one controller-decoded payload into the outer bridge runtime.
     * @details Serial decoding and high-level bridge policy are kept separate on
     *          purpose. `ControllerSerialLink` decides whether a frame is valid;
     *          this switch decides what that validated frame means for bridge
     *          state, MQTT publishing and controller resynchronization.
     *
     * @param code semantic result produced by the serial decoder.
     * @param messageDocument validated payload document that remains owned by
     *        `ControllerSerialLink` for the duration of this callback.
     */
    void handleControllerSerialMessage(constants::DeserializeExitCode code, const JsonDocument &messageDocument)
    {
        switch (code)
        {
        case DeserializeExitCode::OK_BOOT:
            handleControllerBootMessage();
            break;

        case DeserializeExitCode::OK_STATE:
            handleControllerStateMessage();
            break;

        case DeserializeExitCode::OK_DETAILS:
            handleControllerDetailsMessage();
            break;

        case DeserializeExitCode::OK_NETWORK_CLICK:
            if (Homie.isConnected())
            {
                (void)MqttPublisher::sendJson(messageDocument, MqttTopicsBuilder::mqttOutEventsTopic.c_str(), false, 2);
            }
            else
            {
                (void)controllerSerialLink.triggerFailoverFromReceivedClick();
            }
            break;

        case DeserializeExitCode::OK_OTHER_PAYLOAD:
            if (Homie.isConnected())
            {
                (void)MqttPublisher::sendJson(messageDocument, MqttTopicsBuilder::mqttOutEventsTopic.c_str(), false, 2);
            }
            break;

        default:
            break;
        }
    }

    /**
     * @brief React to Homie lifecycle events that matter to bridge runtime.
     *
     * @param event Homie event emitted by the framework.
     */
    void handleHomieEvent(const HomieEvent &event)
    {
        switch (event.type)
        {
        case HomieEventType::WIFI_DISCONNECTED:
        case HomieEventType::MQTT_DISCONNECTED:
            handleDisconnect();
            break;

        case HomieEventType::MQTT_READY:
            if (updateMqttClient() && subscribeMqttTopics())
            {
                softResyncMqttPeer();
            }
            break;

        default:
            break;
        }
    }

    /**
     * @brief Bind the current Homie MQTT client to bridge-side helpers.
     * @details The Homie runtime owns the actual `AsyncMqttClient` instance.
     *          This helper keeps bridge-side callback registration and publisher
     *          binding synchronized with that instance across reconnects.
     *
     * @return true if the current Homie MQTT client is ready for bridge-side use.
     * @return false is currently unreachable in the present implementation, but
     *         the boolean return keeps the call site readable and leaves room
     *         for future initialization checks.
     */
    [[nodiscard]] auto updateMqttClient() -> bool
    {
        // Homie exposes its MQTT client as a reference, so reaching this point
        // always yields one concrete AsyncMqttClient instance.
        auto &nextClient = Homie.getMqttClient();

        if (&nextClient == mqttClient)
        {
            // The same AsyncMqttClient instance survived the reconnect, so only the
            // subscriptions may need refreshing.
            return true;
        }

        mqttClient = &nextClient;
        MqttPublisher::setMqttClient(mqttClient);

        // AsyncMqttClient stores every callback registration. The Homie MQTT
        // client survives reconnects, so the bridge must bind its handler only
        // once.
        if (mqttMessageBoundClient != &nextClient)
        {
            mqttClient->onMessage(LSHBridge::onMqttMessageStatic);
            mqttMessageBoundClient = &nextClient;
        }

        return true;
    }

    /**
     * @brief Subscribe the bridge to device and service MQTT topics.
     * @details The device topic is subscribed only when the bridge already knows
     *          a validated controller identity. The service topic is always
     *          subscribed because it is the recovery/control channel used even
     *          before controller topology is known.
     *
     * @return true if every required subscription for the current runtime phase
     *         has been accepted by the MQTT client.
     * @return false if any mandatory subscription request failed.
     */
    [[nodiscard]] auto subscribeMqttTopics() -> bool
    {
        using constants::mqtt::MQTT_TOPIC_SERVICE;

        // Always unsubscribe first so repeated MQTT_READY events do not leave
        // duplicate subscriptions behind on reconnect.
        if (hasDeviceScopedTopics())
        {
            mqttClient->unsubscribe(MqttTopicsBuilder::mqttInTopic.c_str());
        }
        mqttClient->unsubscribe(MQTT_TOPIC_SERVICE);

        bool isDeviceTopicSubscribed = false;
        if (hasDeviceScopedTopics())
        {
            const std::uint16_t deviceTopicPacketId = mqttClient->subscribe(MqttTopicsBuilder::mqttInTopic.c_str(), 2);
            if (deviceTopicPacketId == 0U)
            {
                return false;
            }

            isDeviceTopicSubscribed = true;
        }

        const std::uint16_t serviceTopicPacketId = mqttClient->subscribe(MQTT_TOPIC_SERVICE, 1);
        if (serviceTopicPacketId == 0U)
        {
            if (isDeviceTopicSubscribed)
            {
                mqttClient->unsubscribe(MqttTopicsBuilder::mqttInTopic.c_str());
            }
            return false;
        }

        return true;
    }

    /**
     * @brief Handles MQTT commands that the bridge understands semantically.
     * @details Unsupported commands fall back to raw forwarding so the
     *          runtime remains compatible with legacy or future controller
     *          commands.
     *
     * @param commandDocument validated MQTT command decoded from the current
     *        inbound frame.
     * @return TypedCommandResult::Handled when the bridge consumed the command.
     * @return TypedCommandResult::Unsupported when the bridge does not assign
     *         any local meaning to that payload type and callers may raw-forward it.
     * @return TypedCommandResult::Invalid when the payload type is known but its
     *         concrete fields do not pass bridge-side validation.
     */
    [[nodiscard]] auto forwardTypedControllerCommand(const JsonDocument &commandDocument) -> TypedCommandResult
    {
        using namespace lsh::bridge::protocol;

        const auto command = static_cast<Command>(commandDocument[KEY_PAYLOAD].as<std::uint8_t>());

        switch (command)
        {
        case Command::REQUEST_DETAILS:
            // DEVICE_DETAILS can be served from the cached validated topology. If
            // that cache is missing, keep the bridge online and ask the
            // controller for a fresh authoritative topology instead.
            if (!virtualDevice.hasCachedDetails())
            {
                enterWaitingForDetails();
            }
            else
            {
                (void)publishCachedDetails();
            }
            return TypedCommandResult::Handled;

        case Command::REQUEST_STATE:
            // Prefer the local cache only when it is known to mirror a controller-
            // backed snapshot from the current session. Otherwise ask the controller.
            if (!tryServeCachedStateRequest() && controllerSerialLink.isConnected())
            {
                if (!virtualDevice.hasCachedDetails() || bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
                {
                    enterWaitingForDetails();
                }
                else
                {
                    requestAuthoritativeStateRefresh();
                }
            }
            return TypedCommandResult::Handled;

        case Command::FAILOVER:
            (void)controllerSerialLink.sendJson(constants::payloads::StaticType::GENERAL_FAILOVER);
            return TypedCommandResult::Handled;

        case Command::SET_SINGLE_ACTUATOR:
        {
            std::uint8_t actuatorId = 0U;
            bool requestedState = false;
            if (!utils::json::tryGetUint8Scalar(commandDocument[KEY_ID], actuatorId) ||
                !utils::json::tryGetBinaryState(commandDocument[KEY_STATE], requestedState))
            {
                return TypedCommandResult::Invalid;
            }

            return controllerSerialLink.stageSingleActuatorCommand(actuatorId, requestedState) ? TypedCommandResult::Handled
                                                                                               : TypedCommandResult::Invalid;
        }

        case Command::SET_STATE:
            return controllerSerialLink.stageDesiredPackedState(commandDocument[KEY_STATE].as<JsonArrayConst>())
                       ? TypedCommandResult::Handled
                       : TypedCommandResult::Invalid;

        case Command::NETWORK_CLICK_REQUEST:
        case Command::NETWORK_CLICK_ACK:
        case Command::FAILOVER_CLICK:
        case Command::NETWORK_CLICK_CONFIRM:
        {
            // Click payloads are tiny and structurally simple, so the bridge rebuilds
            // a compact serial document instead of forwarding unknown fields.
            StaticJsonDocument<constants::controllerSerial::MQTT_RECEIVED_DOC_MAX_SIZE> serialDoc;
            std::uint8_t clickType = 0U;
            std::uint8_t clickableId = 0U;
            std::uint8_t correlationId = 0U;

            if (!utils::json::tryGetUint8Scalar(commandDocument[KEY_TYPE], clickType) ||
                !utils::json::tryGetUint8Scalar(commandDocument[KEY_ID], clickableId) ||
                !utils::json::tryGetUint8Scalar(commandDocument[KEY_CORRELATION_ID], correlationId))
            {
                return TypedCommandResult::Invalid;
            }

            serialDoc[KEY_PAYLOAD] = static_cast<std::uint8_t>(command);
            serialDoc[KEY_TYPE] = clickType;
            serialDoc[KEY_ID] = clickableId;
            serialDoc[KEY_CORRELATION_ID] = correlationId;
            if (!controllerSerialLink.sendJson(serialDoc))
            {
                DPL("Dropping bridge-known MQTT command because the controller TX "
                    "path rejected the serialized payload.");
                return TypedCommandResult::Invalid;
            }

            return TypedCommandResult::Handled;
        }

        default:
            return TypedCommandResult::Unsupported;
        }
    }

    /**
     * @brief Forwards an unsupported command to the controller only when the
     *        two transports already share the same wire codec.
     * @details Raw forwarding is safe only when MQTT and the controller link
     *          already speak the same codec, because the bridge can then copy
     *          the exact payload bytes without interpreting them. When the
     *          codecs differ, the bridge would have to translate a payload
     *          shape it does not understand, which is not robust enough for
     *          mission-critical forwarding.
     *
     * @param payload raw MQTT payload bytes received from the broker.
     * @param payloadLength number of bytes available at `payload`.
     * @return true if the unsupported payload has been forwarded unchanged.
     * @return false if the bridge intentionally dropped the payload because a
     *         safe raw forward was not possible.
     */
    [[nodiscard]] auto forwardUnsupportedCommandToController(const char *payload, std::size_t payloadLength) -> bool
    {
#if (defined(CONFIG_MSG_PACK_MQTT) && defined(CONFIG_MSG_PACK_ARDUINO)) || \
    (!defined(CONFIG_MSG_PACK_MQTT) && !defined(CONFIG_MSG_PACK_ARDUINO))
        return controllerSerialLink.sendJson(payload, payloadLength);
#else
        (void)payload;
        (void)payloadLength;
        return false;
#endif
    }

    /**
     * @brief Process one validated device-topic MQTT command.
     * @details Device-topic commands are device-scoped and therefore may touch
     *          controller-backed state, bridge lifecycle, or both. The bridge
     *          keeps the early returns explicit so it is obvious which commands
     *          are blocked during desynchronization and which ones stay allowed.
     *
     * @param command parsed command identifier.
     * @param payload raw MQTT payload bytes as received from the broker.
     * @param payloadLength number of bytes available at `payload`.
     * @param commandDocument decoded representation of the same MQTT payload.
     */
    void processDeviceTopicCommand(lsh::bridge::protocol::Command command,
                                   const char *payload,
                                   std::size_t payloadLength,
                                   const JsonDocument &commandDocument)
    {
        using namespace lsh::bridge::protocol;

        switch (command)
        {
        case Command::SYSTEM_RESET:
            Homie.reset();
            Homie.setIdle(true);
            return;

        case Command::SYSTEM_REBOOT:
            Homie.reboot();
            return;

        case Command::PING_:
            if (controllerSerialLink.isConnected() && virtualDevice.isRuntimeSynchronized())
            {
                (void)MqttPublisher::sendJson(constants::payloads::StaticType::PING_);
            }
            return;

        case Command::SET_SINGLE_ACTUATOR:
        case Command::SET_STATE:
            // During re-sync or topology migration the bridge must not reinterpret
            // actuator writes against a stale cached model. Dropping them is
            // safer than forwarding ambiguous intent to the controller.
            if (!virtualDevice.isRuntimeSynchronized())
            {
                return;
            }
            break;

        default:
            break;
        }

        const auto typedCommandResult = forwardTypedControllerCommand(commandDocument);
        if (typedCommandResult == TypedCommandResult::Handled)
        {
            return;
        }

        if (typedCommandResult == TypedCommandResult::Invalid)
        {
            DPL("Dropping invalid MQTT command for a bridge-known payload type. "
                "The bridge recognized the command ID but rejected this specific "
                "payload instead of raw-forwarding it to the controller.");
            return;
        }

        if (typedCommandResult == TypedCommandResult::Unsupported)
        {
            if (!forwardUnsupportedCommandToController(payload, payloadLength))
            {
                DPL("Dropping unsupported MQTT command because the bridge cannot "
                    "safely translate an unknown payload across different MQTT "
                    "and serial codecs.");
            }
        }
    }

    /**
     * @brief Process one validated service-topic MQTT command.
     * @details Service-topic commands are bridge-scoped. They must never depend
     *          on controller topology being currently synchronized, because they
     *          are exactly the tools used to diagnose or recover that sync.
     *
     * @param command parsed command identifier received on the service topic.
     */
    void processServiceTopicCommand(lsh::bridge::protocol::Command command)
    {
        using namespace lsh::bridge::protocol;

        switch (command)
        {
        case Command::BOOT:
            softResyncMqttPeer();
            return;

        case Command::SYSTEM_RESET:
            Homie.reset();
            Homie.setIdle(true);
            return;

        case Command::SYSTEM_REBOOT:
            Homie.reboot();
            return;

        case Command::PING_:
            publishServicePingReply();
            return;

        default:
            return;
        }
    }

    /**
     * @brief Decode one queued MQTT command and route it to the proper topic family.
     * @details The queue stores raw bytes because the MQTT callback must stay
     *          short. Parsing happens here, in the main loop, where serial and
     *          MQTT work can be coordinated more safely.
     *
     * @param source topic family from which the payload originally arrived.
     * @param payload raw MQTT payload bytes.
     * @param payloadLength number of bytes available at `payload`.
     */
    void processInboundMqttCommand(MqttCommandSource source, const char *payload, std::size_t payloadLength)
    {
        using namespace lsh::bridge::protocol;

        StaticJsonDocument<constants::controllerSerial::MQTT_RECEIVED_DOC_MAX_SIZE> commandDocument;

#ifdef CONFIG_MSG_PACK_MQTT
        // MQTT control payloads are intentionally shallow: one object plus maybe
        // one state array. Limiting nesting keeps parsing predictable and cheap.
        const DeserializationError deserializationError = deserializeMsgPack(
            commandDocument, reinterpret_cast<const std::uint8_t *>(payload), payloadLength, DeserializationOption::NestingLimit(2));
#else
        // MQTT control payloads are intentionally shallow: one object plus maybe
        // one state array. Limiting nesting keeps parsing predictable and cheap.
        const DeserializationError deserializationError =
            deserializeJson(commandDocument, payload, payloadLength, DeserializationOption::NestingLimit(2));
#endif

        if (deserializationError != DeserializationError::Ok)
        {
            return;
        }

        std::uint8_t rawCommand = 0U;
        if (!utils::json::tryGetUint8Scalar(commandDocument[KEY_PAYLOAD], rawCommand))
        {
            return;
        }

        const auto command = static_cast<Command>(rawCommand);
        if (source == MqttCommandSource::Service)
        {
            processServiceTopicCommand(command);
            return;
        }

        processDeviceTopicCommand(command, payload, payloadLength, commandDocument);
    }

    /**
     * @brief Drains queued MQTT commands with a bounded fairness policy.
     * @details If the UART already has pending RX traffic, the bridge still
     *          processes one queued MQTT command per loop iteration so the queue
     *          cannot starve forever under continuous controller chatter. When
     *          the UART is idle, the bridge drains more aggressively until the
     *          queue empties or fresh serial RX traffic appears.
     */
    void processQueuedMqttCommands()
    {
        const bool serialHadPendingRx = serial->available();
        const std::uint8_t maxCommandsThisLoop = serialHadPendingRx ? 1U : constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY;
        std::uint8_t processedCommands = 0U;

        while (processedCommands < maxCommandsThisLoop)
        {
            QueuedMqttCommand queuedCommand{};
            if (!dequeueMqttCommand(queuedCommand))
            {
                return;
            }

            // The queue stores raw bytes, but once the frame is dequeued it is safe
            // to reinterpret them as `char*` because JSON parsing and MsgPack
            // deserialization both just read the contiguous payload bytes.
            processInboundMqttCommand(queuedCommand.source, reinterpret_cast<const char *>(queuedCommand.payload), queuedCommand.length);
            ++processedCommands;

            if (serial->available())
            {
                return;
            }
        }
    }

    /**
     * @brief Receives complete MQTT frames from the AsyncMqttClient callback.
     *
     * @param topic null-terminated MQTT topic string provided by AsyncMqttClient.
     * @param payload pointer to the current MQTT payload chunk.
     * @param properties MQTT message flags for the current publish.
     * @param len size of the current payload chunk.
     * @param index chunk start offset inside the full publish payload.
     * @param total total payload size announced by AsyncMqttClient.
     */
    void handleMqttMessage(char *topic,
                           char *payload,
                           AsyncMqttClientMessageProperties properties,
                           std::size_t len,
                           std::size_t index,
                           std::size_t total)
    {
        // Topic hashes avoid repeated `strcmp()` calls inside the hot MQTT callback.
        const auto topicHash = djb2_hash(topic);
        MqttCommandSource source = MqttCommandSource::Device;

        if (topicHash == MqttTopicsBuilder::mqttInTopicHash)
        {
            source = MqttCommandSource::Device;
        }
        else if (topicHash == constants::mqtt::MQTT_TOPIC_SERVICE_HASH)
        {
            source = MqttCommandSource::Service;
        }
        else
        {
            return;
        }

        if (total == 0U || len == 0U || properties.retain)
        {
            // Retained commands must not be replayed automatically when the bridge
            // reconnects, otherwise stale writes could be applied long after they
            // were intended.
            return;
        }

        if (total > constants::controllerSerial::MQTT_COMMAND_MESSAGE_MAX_SIZE)
        {
            return;
        }

        // The bridge only accepts complete non-fragmented MQTT frames because it
        // stores them in a fixed queue and parses them later in the main loop.
        if (index != 0U || len != total)
        {
            return;
        }

        if (!enqueueMqttCommand(source, payload, total))
        {
            recordDroppedQueuedMqttCommand(source);
            DPL("Dropping MQTT command because the inbound bridge queue is full. "
                "A producer is sending commands faster than the bridge can safely "
                "coalesce and forward them.");
        }
    }
};

/**
 * @brief Currently active bridge instance used by the static callback trampolines.
 */
LSHBridge *LSHBridge::activeInstance = nullptr;

/**
 * @brief Construct a new bridge facade and its hidden runtime implementation.
 *
 * @param options runtime options.
 */
LSHBridge::LSHBridge(BridgeOptions options) : implementation(std::make_unique<Impl>(std::move(options)))
{}

/**
 * @brief Destroy the bridge facade and detach static callback state if needed.
 */
LSHBridge::~LSHBridge()
{
    if (activeInstance == this)
    {
        MqttPublisher::setMqttClient(nullptr);
        activeInstance = nullptr;
    }
}

/**
 * @brief Initialize serial, restore cached topology if available, and start Homie.
 * @details `HOMIE_RESET` is handled carefully here. New or already-reset
 *          devices enter Homie configuration mode automatically when their
 *          config is invalid, so the bridge only calls `Homie.reset()` when
 *          there is a valid stored Homie configuration that must be erased.
 */
void LSHBridge::begin()
{
    if (this->implementation->isStarted)
    {
        return;
    }

    // Open the USB serial console before any debug log can fire. Without this
    // explicit initialization a debug build compiles the logging calls but the
    // bridge USB port would remain silent.
    DSB();

    activeInstance = this;
    timeKeeper::update();
    this->implementation->controllerSerialLink.begin();
    (void)this->implementation->loadCachedDetailsFromFlash();
    this->implementation->configureHomie();
    Homie.onEvent(onHomieEventStatic);
    this->implementation->isControllerConnected = this->implementation->controllerSerialLink.isConnected();

    auto runtimeDelegate = ControllerSerialLink::MessageCallback::create<Impl, &Impl::handleControllerSerialMessage>(*this->implementation);
    this->implementation->controllerSerialLink.onMessage(runtimeDelegate);

    Homie.setup();
    this->implementation->isStarted = true;
    this->implementation->enterWaitingForDetails();

#ifdef HOMIE_RESET
    if (Homie.isConfigured())
    {
        Homie.reset();
        Homie.setIdle(true);
    }
#endif
}

/**
 * @brief Execute one bridge main-loop iteration.
 */
void LSHBridge::loop()
{
    if (!this->implementation->isStarted)
    {
        return;
    }

    Homie.loop();
    timeKeeper::update();

    if (this->implementation->serial->available())
    {
        this->implementation->controllerSerialLink.processSerialBuffer();
    }

    this->implementation->processQueuedMqttCommands();
    this->implementation->refreshControllerConnectivity();
    this->implementation->processBootstrapProgress();
    this->implementation->processPendingTopologyMigration();
    this->implementation->controllerSerialLink.processPendingActuatorBatch();
    this->implementation->processPendingStatePublishes();
    this->implementation->publishPendingBridgeDiagnostics();
    (void)this->implementation->controllerSerialLink.sendJson(constants::payloads::StaticType::PING_);
}

/**
 * @brief Return whether the cached runtime model is currently synchronized.
 *
 * @return true if the bridge has a fresh controller-backed state snapshot.
 * @return false if the runtime state is stale.
 */
auto LSHBridge::isRuntimeSynchronized() const noexcept -> bool
{
    return this->implementation->virtualDevice.isRuntimeSynchronized();
}

/**
 * @brief Return the cached controller device name.
 *
 * @return const char* device name.
 */
auto LSHBridge::deviceName() const noexcept -> const char *
{
    return this->implementation->virtualDevice.getName().c_str();
}

/**
 * @brief Static trampoline for Homie events.
 *
 * @param event Homie event.
 */
void LSHBridge::onHomieEventStatic(const HomieEvent &event)
{
    if (activeInstance != nullptr && activeInstance->implementation != nullptr)
    {
        activeInstance->implementation->handleHomieEvent(event);
    }
}

/**
 * @brief Static trampoline for AsyncMqttClient messages.
 *
 * @param topic MQTT topic.
 * @param payload MQTT payload.
 * @param properties MQTT message properties.
 * @param len fragment length.
 * @param index fragment index.
 * @param total total payload length.
 */
void LSHBridge::onMqttMessageStatic(char *topic,
                                    char *payload,
                                    AsyncMqttClientMessageProperties properties,
                                    std::size_t len,
                                    std::size_t index,
                                    std::size_t total)
{
    if (activeInstance != nullptr && activeInstance->implementation != nullptr)
    {
        activeInstance->implementation->handleMqttMessage(topic, payload, properties, len, index, total);
    }
}

}  // namespace lsh::bridge
