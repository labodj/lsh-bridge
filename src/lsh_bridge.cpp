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

#include <cstring>
#include <cstdint>
#include <utility>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <Homie.h>
#include <etl/bitset.h>
#include <etl/vector.h>

#include "bridge_runtime_diagnostics.hpp"
#include "bridge_runtime_sync.hpp"
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
#include "mqtt_command_queue.hpp"
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
constexpr char kTopologyChangedDiagnostic[] =
    "topology_changed";  //!< Diagnostic value emitted when the controller reports a topology different from the cached one.
constexpr std::uint16_t kTopologyChangedDiagnosticGraceMs =
    150U;  //!< Short grace window that gives the MQTT client one chance to flush `topology_changed` before rebooting.

/**
 * @brief Validate one MQTT-originated network click command before forwarding it to the controller.
 *
 * @param commandDocument decoded MQTT payload.
 * @param outClickType destination click type byte written only on success.
 * @param outClickableId destination clickable ID written only on success.
 * @param outCorrelationId destination correlation ID written only on success.
 * @return true if every scalar is present and semantically valid for the controller protocol.
 * @return false otherwise.
 */
[[nodiscard]] auto validateInboundNetworkClickCommand(const JsonDocument &commandDocument,
                                                      std::uint8_t &outClickType,
                                                      std::uint8_t &outClickableId,
                                                      std::uint8_t &outCorrelationId) -> bool
{
    using namespace lsh::bridge::protocol;

    if (!utils::json::tryGetUint8Scalar(commandDocument[KEY_TYPE], outClickType) ||
        !utils::json::tryGetUint8Scalar(commandDocument[KEY_ID], outClickableId) ||
        !utils::json::tryGetUint8Scalar(commandDocument[KEY_CORRELATION_ID], outCorrelationId))
    {
        return false;
    }

    switch (static_cast<ProtocolClickType>(outClickType))
    {
    case ProtocolClickType::LONG:
    case ProtocolClickType::SUPER_LONG:
        break;

    default:
        return false;
    }

    return outClickableId != 0U && outCorrelationId != 0U;
}

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
        Handled,        //!< The bridge accepted and handled the command.
        Unsupported,    //!< The bridge does not interpret this command type.
        Invalid,        //!< The command type is known, but this concrete payload is invalid.
        DeliveryFailed  //!< The bridge recognized the command, but could not deliver the resulting side effect.
    };

    using BootstrapPhase = runtime::BootstrapPhase;

    explicit Impl(BridgeOptions bridgeOptions) :
        options(std::move(bridgeOptions)), serial(resolveSerial(options.serial)), controllerSerialLink(serial, virtualDevice)
    {
        options.serial = serial;
    }

    BridgeOptions options{};
    bool isStarted = false;  //!< True after `begin()` has fully wired serial, Homie and callbacks.
    BootstrapPhase bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;  //!< Current high-level controller sync phase.
    bool bootstrapRequestDue = true;                                      //!< True when the next bootstrap request may be sent immediately.
    bool hasPendingTopologySave = false;       //!< True while a changed topology still has to be persisted to NVS.
    bool isAuthoritativeStateDirty = false;    //!< True after fresh controller state arrived but is not yet mirrored to MQTT/Homie.
    bool canServeCachedStateRequests = false;  //!< True only after this MQTT session has already seen one fresh controller-backed state.
    bool isControllerConnected = false;        //!< Last known controller connectivity bit, used to detect link transitions.
    bool mqttResyncPending = false;  //!< True while MQTT-side resync still needs another retry after reconnect or publish refusal.
    bool topologyChangedDiagnosticPending =
        false;                                   //!< True while `topology_changed` still needs one best-effort MQTT publish before reboot.
    std::uint32_t lastBootstrapRequestMs = 0U;   //!< Real-time timestamp of the last `ASK_DETAILS` or `ASK_STATE`.
    std::uint32_t lastMqttResyncAttemptMs = 0U;  //!< Real-time timestamp of the last autonomous MQTT-side resync attempt.
    std::uint32_t lastTopologySaveAttemptMs = 0U;  //!< Real-time timestamp of the last deferred NVS save attempt.
    std::uint32_t topologyRebootPendingSinceMs =
        0U;  //!< Real-time timestamp of the successful topology save that started the reboot window.
    std::uint32_t topologyDiagnosticAcceptedMs = 0U;    //!< Real-time timestamp of the accepted `topology_changed` MQTT publish, if any.
    std::uint32_t lastAuthoritativeStateUpdateMs = 0U;  //!< Real-time timestamp of the latest accepted authoritative state frame.
    etl::vector<LSHNode, constants::virtualDevice::MAX_ACTUATORS> homieNodes{};
    DeviceDetailsSnapshot pendingTopologyDetails{};  //!< Validated topology waiting to be persisted before a controlled reboot.
    VirtualDevice virtualDevice{};
    HardwareSerial *const serial;
    ControllerSerialLink controllerSerialLink;
    AsyncMqttClient *mqttClient = nullptr;
    AsyncMqttClient *mqttMessageBoundClient = nullptr;
    MqttCommandQueue mqttCommandQueue{};

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
        return runtime::bootstrapPhaseName(bootstrapPhase);
    }

    /**
     * @brief Drops transient runtime state that depends on a fresh authoritative
     * controller sync.
     */
    void clearPendingRuntimeState()
    {
        runtime::clearPendingRuntimeState(controllerSerialLink, isAuthoritativeStateDirty, canServeCachedStateRequests);
    }

    /**
     * @brief Ask the loop to send the next bootstrap request as soon as allowed.
     * @details Requests are rate-limited in the main loop so the bridge never
     *          pounds the controller in a tight retry loop.
     */
    void scheduleBootstrapRequestNow()
    {
        runtime::scheduleBootstrapRequestNow(bootstrapRequestDue, lastBootstrapRequestMs);
    }

    /**
     * @brief Enter the phase that waits for fresh authoritative `DEVICE_DETAILS`.
     * @details This is the safest phase after a controller `BOOT` because the
     *          bridge must re-confirm the topology before trusting actuator IDs,
     *          button IDs, or the controller-backed event channels.
     */
    void enterWaitingForDetails()
    {
        runtime::enterWaitingForDetails(controllerSerialLink, virtualDevice, isAuthoritativeStateDirty, canServeCachedStateRequests,
                                        bootstrapPhase, bootstrapRequestDue, lastBootstrapRequestMs);
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
        runtime::enterWaitingForState(controllerSerialLink, virtualDevice, isAuthoritativeStateDirty, canServeCachedStateRequests,
                                      bootstrapPhase, bootstrapRequestDue, lastBootstrapRequestMs);
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
        runtime::stageTopologyMigration(controllerSerialLink, virtualDevice, pendingTopologyDetails, hasPendingTopologySave,
                                        isAuthoritativeStateDirty, canServeCachedStateRequests, details, lastTopologySaveAttemptMs,
                                        bootstrapPhase);
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
        runtime::requestAuthoritativeStateRefresh(controllerSerialLink, virtualDevice, isAuthoritativeStateDirty,
                                                  canServeCachedStateRequests, bootstrapPhase, bootstrapRequestDue, lastBootstrapRequestMs);
    }

    /**
     * @brief Arm the next MQTT-side resync attempt for the earliest safe loop.
     * @details This retry loop is intentionally independent from the controller
     *          bootstrap state machine. It exists to recover retained MQTT
     *          publishes and subscriptions after `MQTT_READY` without assuming
     *          that the controller itself needs another bootstrap request.
     */
    void scheduleMqttResyncNow()
    {
        mqttResyncPending = true;
        lastMqttResyncAttemptMs = 0U;
    }

    /**
     * @brief Re-synchronizes the MQTT-side peer without tearing down the whole
     * bridge.
     */
    [[nodiscard]] auto softResyncMqttPeer() -> bool
    {
        if (!virtualDevice.hasCachedDetails())
        {
            canServeCachedStateRequests = false;
            bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;
            scheduleBootstrapRequestNow();
            mqttResyncPending = false;
            return true;
        }

        if (!publishCachedDetails())
        {
            canServeCachedStateRequests = false;
            mqttResyncPending = true;
            return false;
        }

        const bool controllerStateUsable = virtualDevice.isRuntimeSynchronized() && controllerSerialLink.isConnected();
        if (controllerStateUsable)
        {
            if (!publishAuthoritativeLshState())
            {
                canServeCachedStateRequests = false;
                mqttResyncPending = true;
                return false;
            }

            const bool homiePublished = publishAllHomieNodeStates();
            canServeCachedStateRequests = homiePublished;
            mqttResyncPending = !homiePublished;
            return homiePublished;
        }

        canServeCachedStateRequests = false;
        virtualDevice.invalidateRuntimeModel();
        mqttResyncPending = false;

        if (bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
        {
            scheduleBootstrapRequestNow();
            return true;
        }

        if (bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            enterWaitingForState();
        }
        return true;
    }

    /**
     * @brief Retry MQTT-side resynchronization after reconnect or publish refusal.
     * @details This helper retries subscriptions plus retained `conf` / `state`
     *          republishes at a slow fixed cadence. It deliberately does not
     *          touch controller bootstrap state unless the cache itself has
     *          become unusable.
     */
    void processPendingMqttResync()
    {
        if (!mqttResyncPending || !Homie.isConnected())
        {
            return;
        }

        const auto now = timeKeeper::getRealTime();
        if (lastMqttResyncAttemptMs != 0U && (now - lastMqttResyncAttemptMs) < constants::runtime::BOOTSTRAP_REQUEST_INTERVAL_MS)
        {
            return;
        }

        lastMqttResyncAttemptMs = now;
        if (!updateMqttClient() || !subscribeMqttTopics())
        {
            return;
        }

        if (softResyncMqttPeer())
        {
            mqttResyncPending = false;
        }
    }

    /**
     * @brief Tracks controller connectivity transitions and reacts to state
     * loss/recovery.
     */
    void refreshControllerConnectivity()
    {
        const bool isConnectedNow = controllerSerialLink.isConnected();
        runtime::refreshControllerConnectivity(isConnectedNow, isControllerConnected, controllerSerialLink, virtualDevice,
                                               isAuthoritativeStateDirty, canServeCachedStateRequests, bootstrapPhase, bootstrapRequestDue,
                                               lastBootstrapRequestMs);
    }

    /**
     * @brief Forget every bridge-local diagnostic that belongs to the MQTT
     *        session being torn down.
     */
    void clearPendingBridgeDiagnostics()
    {
        runtime::clearPendingBridgeDiagnostics(controllerSerialLink, mqttCommandQueue);
    }

    /**
     * @brief Publish pending bridge-local diagnostics on the `bridge` topic.
     * @details Diagnostics are emitted from the main loop, not from callbacks, so
     *          the bridge never tries to publish MQTT while already executing the
     *          MQTT client's message callback.
     */
    void publishPendingBridgeDiagnostics()
    {
        runtime::publishPendingBridgeDiagnostics(controllerSerialLink, mqttCommandQueue, Homie.isConnected(), hasBridgeTopic(),
                                                 MqttTopicsBuilder::mqttOutBridgeTopic.c_str());
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
    [[nodiscard]] auto publishSimpleBridgeDiagnostic(const char *diagnosticKind) const -> bool
    {
        return runtime::publishSimpleBridgeDiagnostic(Homie.isConnected(), hasBridgeTopic(), MqttTopicsBuilder::mqttOutBridgeTopic.c_str(),
                                                      diagnosticKind);
    }

    /**
     * @brief Publish the service-topic `PING` reply on the bridge-local topic.
     * @details This reply intentionally reports bridge/runtime health only. It
     *          never claims that the controller-backed device topic is usable
     *          unless both controller connectivity and runtime synchronization
     *          are currently true.
     */
    [[nodiscard]] auto publishServicePingReply() const -> bool
    {
        return runtime::publishServicePingReply(Homie.isConnected(), hasBridgeTopic(), MqttTopicsBuilder::mqttOutBridgeTopic.c_str(),
                                                controllerSerialLink.isConnected(), virtualDevice.isRuntimeSynchronized(),
                                                bootstrapPhaseName());
    }

    /**
     * @brief Forget transient MQTT/session state after Wi-Fi or MQTT loss.
     * @details Topology and the in-memory controller model are intentionally
     *          preserved. Only session-scoped items are dropped so a reconnect
     *          can reuse cached topology and re-confirm runtime state. A
     *          completed topology migration keeps its pending reboot deadline
     *          across disconnects so the bridge cannot stay stuck forever in
     *          `TOPOLOGY_MIGRATION_PENDING_REBOOT`.
     */
    void handleDisconnect()
    {
        mqttClient = nullptr;
        mqttResyncPending = false;
        lastMqttResyncAttemptMs = 0U;
        clearPendingRuntimeState();
        mqttCommandQueue.clear();
        clearPendingBridgeDiagnostics();
    }

    /**
     * @brief Retry the next bootstrap request at a slow fixed cadence.
     * @details The bridge sends at most one `ASK_DETAILS` or `ASK_STATE` per
     *          retry window. Requests continue even when the controller is
     *          currently silent so the bridge can discover a freshly powered-on
     *          or freshly rebooted core without a misleading "connected" guess.
     *          The fixed cadence keeps the controller firmly in charge and keeps
     *          bootstrap traffic predictable. The retry timer advances only
     *          after the UART accepted the whole outbound request.
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
            if (!controllerSerialLink.sendJson(constants::payloads::StaticType::ASK_DETAILS))
            {
                return;
            }
        }
        else
        {
            if (!controllerSerialLink.sendJson(constants::payloads::StaticType::ASK_STATE))
            {
                return;
            }
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
     *          is already known to be structurally stale. After the save
     *          succeeds the bridge grants one short MQTT grace window for the
     *          `topology_changed` diagnostic, but still reboots on a hard
     *          deadline even if MQTT disconnects meanwhile.
     */
    void processPendingTopologyMigration()
    {
        if (bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            return;
        }

        const auto now = timeKeeper::getRealTime();
        if (!hasPendingTopologySave)
        {
            if (topologyChangedDiagnosticPending && publishSimpleBridgeDiagnostic(kTopologyChangedDiagnostic))
            {
                topologyChangedDiagnosticPending = false;
                topologyDiagnosticAcceptedMs = now;
            }

            const bool diagnosticGraceElapsed =
                topologyDiagnosticAcceptedMs != 0U && (now - topologyDiagnosticAcceptedMs) >= kTopologyChangedDiagnosticGraceMs;
            const bool hardDeadlineElapsed =
                topologyRebootPendingSinceMs != 0U && (now - topologyRebootPendingSinceMs) >= constants::runtime::TOPOLOGY_REBOOT_GRACE_MS;
            if (diagnosticGraceElapsed || hardDeadlineElapsed)
            {
                Homie.reboot();
            }
            return;
        }

        if (lastTopologySaveAttemptMs != 0U && (now - lastTopologySaveAttemptMs) < constants::runtime::TOPOLOGY_SAVE_RETRY_INTERVAL_MS)
        {
            return;
        }

        lastTopologySaveAttemptMs = now;
        if (!DetailsCacheStore::save(pendingTopologyDetails))
        {
            return;
        }

        hasPendingTopologySave = false;
        topologyChangedDiagnosticPending = true;
        topologyRebootPendingSinceMs = now;
        topologyDiagnosticAcceptedMs = 0U;
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

        const bool fullStatePublishPending = virtualDevice.isFullStatePublishPending();
        bool homiePublished = false;
        if (fullStatePublishPending)
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

        if (fullStatePublishPending)
        {
            virtualDevice.clearFullStatePublishPending();
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

        case DeserializeExitCode::OK_NETWORK_CLICK_REQUEST:
        {
            if (Homie.isConnected() && MqttPublisher::sendJson(messageDocument, MqttTopicsBuilder::mqttOutEventsTopic.c_str(), false, 2))
            {
                break;
            }

            const auto failoverResult = controllerSerialLink.triggerFailoverFromReceivedClick();
            if (failoverResult != DeserializeExitCode::ERR_NOT_CONNECTED_FAILOVER_SENT &&
                failoverResult != DeserializeExitCode::ERR_NOT_CONNECTED_GENERAL_FAILOVER_SENT)
            {
                DPL("Dropping network click because MQTT publish failed and the "
                    "controller failover command was not accepted by the UART.");
            }
            break;
        }

        case DeserializeExitCode::OK_NETWORK_CLICK_CONFIRM:
            if (Homie.isConnected())
            {
                if (!MqttPublisher::sendJson(messageDocument, MqttTopicsBuilder::mqttOutEventsTopic.c_str(), false, 2))
                {
                    DPL("Dropping controller NETWORK_CLICK_CONFIRM because the MQTT "
                        "client did not accept the publish.");
                }
            }
            break;

        case DeserializeExitCode::OK_OTHER_PAYLOAD:
            if (Homie.isConnected())
            {
                if (!MqttPublisher::sendJson(messageDocument, MqttTopicsBuilder::mqttOutEventsTopic.c_str(), false, 2))
                {
                    DPL("Dropping controller event payload because the MQTT client did not accept the publish.");
                }
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
            scheduleMqttResyncNow();
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
     * @return TypedCommandResult::DeliveryFailed when the command was valid but
     *         the bridge could not deliver the resulting publish or serial write.
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
                return publishCachedDetails() ? TypedCommandResult::Handled : TypedCommandResult::DeliveryFailed;
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
            return controllerSerialLink.sendJson(constants::payloads::StaticType::GENERAL_FAILOVER) ? TypedCommandResult::Handled
                                                                                                    : TypedCommandResult::DeliveryFailed;

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

        case Command::NETWORK_CLICK_ACK:
        case Command::FAILOVER_CLICK:
        {
            // Click payloads are tiny and structurally simple, so the bridge rebuilds
            // a compact serial document instead of forwarding unknown fields.
            StaticJsonDocument<constants::controllerSerial::MQTT_RECEIVED_DOC_MAX_SIZE> serialDoc;
            std::uint8_t clickType = 0U;
            std::uint8_t clickableId = 0U;
            std::uint8_t correlationId = 0U;

            if (!validateInboundNetworkClickCommand(commandDocument, clickType, clickableId, correlationId))
            {
                return TypedCommandResult::Invalid;
            }

            serialDoc[KEY_PAYLOAD] = static_cast<std::uint8_t>(command);
            serialDoc[KEY_TYPE] = clickType;
            serialDoc[KEY_ID] = clickableId;
            serialDoc[KEY_CORRELATION_ID] = correlationId;
            if (!controllerSerialLink.sendJson(serialDoc))
            {
                return TypedCommandResult::DeliveryFailed;
            }

            return TypedCommandResult::Handled;
        }

        case Command::NETWORK_CLICK_REQUEST:
        case Command::NETWORK_CLICK_CONFIRM:
            // These payloads are controller-originated events. Forwarding them
            // from MQTT into the controller would only create a silent no-op on
            // the core side, so the bridge rejects them explicitly.
            return TypedCommandResult::Invalid;

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
                if (!MqttPublisher::sendJson(constants::payloads::StaticType::PING_))
                {
                    DPL("Dropping device-topic PING reply because the MQTT client "
                        "did not accept the publish.");
                }
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

        if (typedCommandResult == TypedCommandResult::DeliveryFailed)
        {
            DPL("Dropping bridge-known MQTT command because the bridge could not "
                "deliver the resulting publish or serial write.");
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
            scheduleMqttResyncNow();
            processPendingMqttResync();
            return;

        case Command::SYSTEM_RESET:
            Homie.reset();
            Homie.setIdle(true);
            return;

        case Command::SYSTEM_REBOOT:
            Homie.reboot();
            return;

        case Command::PING_:
            if (!publishServicePingReply())
            {
                DPL("Dropping service ping reply because the MQTT client did not "
                    "accept the publish.");
            }
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
     *          the UART is idle, the bridge drains more aggressively up to the
     *          dedicated `CONFIG_MQTT_MAX_COMMANDS_PER_LOOP` budget, so queue
     *          capacity and per-loop CPU monopoly stay independently tunable.
     */
    void processQueuedMqttCommands()
    {
        const bool serialHadPendingRx = serial->available();
        const std::uint8_t maxCommandsThisLoop = serialHadPendingRx ? 1U : constants::runtime::MQTT_MAX_COMMANDS_PER_LOOP;
        std::uint8_t processedCommands = 0U;

        while (processedCommands < maxCommandsThisLoop)
        {
            QueuedMqttCommand queuedCommand{};
            if (!mqttCommandQueue.dequeue(queuedCommand))
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
        // Topic hashes keep the hot callback cheap, but a final string compare is
        // still required so a rare hash collision cannot route a foreign topic
        // into the bridge command handler.
        const auto topicHash = djb2_hash(topic);
        MqttCommandSource source = MqttCommandSource::Device;

        if (topicHash == MqttTopicsBuilder::mqttInTopicHash && std::strcmp(topic, MqttTopicsBuilder::mqttInTopic.c_str()) == 0)
        {
            source = MqttCommandSource::Device;
        }
        else if (topicHash == constants::mqtt::MQTT_TOPIC_SERVICE_HASH && std::strcmp(topic, constants::mqtt::MQTT_TOPIC_SERVICE) == 0)
        {
            source = MqttCommandSource::Service;
        }
        else
        {
            return;
        }

        if (properties.retain)
        {
            // Retained commands must not be replayed automatically when the bridge
            // reconnects, otherwise stale writes could be applied long after they
            // were intended.
            mqttCommandQueue.recordRejectedCommand(MqttRejectedCommandReason::Retained);
            DPL("Dropping MQTT command because retained commands are never replayed "
                "into the live bridge runtime.");
            return;
        }

        if (total == 0U || len == 0U)
        {
            mqttCommandQueue.recordRejectedCommand(MqttRejectedCommandReason::Fragmented);
            DPL("Dropping MQTT command because zero-length or incomplete MQTT "
                "payload delivery is not accepted by the bridge.");
            return;
        }

        if (total > constants::controllerSerial::MQTT_COMMAND_MESSAGE_MAX_SIZE)
        {
            mqttCommandQueue.recordRejectedCommand(MqttRejectedCommandReason::Oversize);
            DPL("Dropping MQTT command because the payload is larger than the fixed "
                "bridge-side inbound command buffer.");
            return;
        }

        // The bridge only accepts complete non-fragmented MQTT frames because it
        // stores them in a fixed queue and parses them later in the main loop.
        if (index != 0U || len != total)
        {
            mqttCommandQueue.recordRejectedCommand(MqttRejectedCommandReason::Fragmented);
            DPL("Dropping MQTT command because fragmented MQTT payload delivery is "
                "not accepted by the fixed bridge command queue.");
            return;
        }

        if (!mqttCommandQueue.enqueue(source, payload, total))
        {
            mqttCommandQueue.recordDroppedCommand(source);
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
    this->implementation->processPendingMqttResync();
    this->implementation->processBootstrapProgress();
    this->implementation->processPendingTopologyMigration();
    this->implementation->controllerSerialLink.processPendingActuatorBatch();
    this->implementation->processPendingStatePublishes();
    this->implementation->publishPendingBridgeDiagnostics();
    if (!this->implementation->controllerSerialLink.sendJson(constants::payloads::StaticType::PING_))
    {
        // A rejected heartbeat is harmless here: either the ping interval is
        // still closed or the UART refused the frame. In both cases the next
        // loop iteration will retry under the same normal pacing rules.
    }
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
