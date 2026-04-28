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
#include <cstddef>
#include <cstdint>
#include <new>
#include <utility>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#ifdef ESP32
#include <esp_attr.h>
#include <WiFi.h>
#endif
#include "constants/configs/homie_convention.hpp"
#include <Homie.h>
#include <etl/bitset.h>
#include <etl/vector.h>

#include "bridge_runtime_diagnostics.hpp"
#include "bridge_runtime_sync.hpp"
#include "communication/checked_writer.hpp"
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
#include "mqtt_command_decoder.hpp"
#include "mqtt_command_queue.hpp"
#include "utils/json_scalars.hpp"
#include "utils/time_keeper.hpp"
#include "virtual_device.hpp"

namespace
{

using constants::DeserializeExitCode;

constexpr char kTopologyChangedDiagnostic[] =
    "topology_changed";  //!< Diagnostic value emitted when the controller reports a topology different from the cached one.
constexpr std::uint16_t kTopologyChangedDiagnosticGraceMs =
    150U;  //!< Short grace window that gives the MQTT client one chance to flush `topology_changed` before rebooting.
constexpr std::uint32_t kBridgeBreadcrumbMagic = 0x4C534842UL;  //!< "LSHB" marker for RTC loop-phase breadcrumbs.
constexpr char kBridgeBreadcrumbEvent[] = "diagnostic";
constexpr char kBridgeBreadcrumbKind[] = "last_reset_phase";
constexpr char kBridgeDiagnosticEventKey[] = "event";
constexpr char kBridgeDiagnosticKindKey[] = "kind";
constexpr char kBridgeBreadcrumbPhaseKey[] = "phase";
constexpr char kBridgeBreadcrumbPhaseMsKey[] = "phase_ms";
constexpr char kBridgeBreadcrumbBootCountKey[] = "boot_count";
constexpr std::uint16_t kBridgeBreadcrumbDocSize = 192U;

enum class BridgeLoopPhase : std::uint8_t
{
    Unknown = 0,
    Begin,
    BeginSerial,
    BeginCache,
    BeginConfigureHomie,
    BeginHomieSetup,
    BeginWaitingForDetails,
    LoopStart,
    HomieLoop,
    SerialRx,
    MqttQueue,
    ControllerConnectivity,
    MqttResync,
    Bootstrap,
    TopologyMigration,
    ActuatorBatch,
    StatePublish,
    BridgeDiagnostics,
    SerialPing,
    LoopEnd
};

#ifdef ESP32
RTC_NOINIT_ATTR std::uint32_t bridgeBreadcrumbMagic;
RTC_NOINIT_ATTR std::uint32_t bridgeBreadcrumbBootCount;
RTC_NOINIT_ATTR std::uint32_t bridgeBreadcrumbLastMillis;
RTC_NOINIT_ATTR std::uint8_t bridgeBreadcrumbLastPhase;
#endif

[[nodiscard]] auto bridgeLoopPhaseName(BridgeLoopPhase phase) -> const char *
{
    switch (phase)
    {
    case BridgeLoopPhase::Begin:
        return "begin";
    case BridgeLoopPhase::BeginSerial:
        return "begin_serial";
    case BridgeLoopPhase::BeginCache:
        return "begin_cache";
    case BridgeLoopPhase::BeginConfigureHomie:
        return "begin_configure_homie";
    case BridgeLoopPhase::BeginHomieSetup:
        return "begin_homie_setup";
    case BridgeLoopPhase::BeginWaitingForDetails:
        return "begin_waiting_for_details";
    case BridgeLoopPhase::LoopStart:
        return "loop_start";
    case BridgeLoopPhase::HomieLoop:
        return "homie_loop";
    case BridgeLoopPhase::SerialRx:
        return "serial_rx";
    case BridgeLoopPhase::MqttQueue:
        return "mqtt_queue";
    case BridgeLoopPhase::ControllerConnectivity:
        return "controller_connectivity";
    case BridgeLoopPhase::MqttResync:
        return "mqtt_resync";
    case BridgeLoopPhase::Bootstrap:
        return "bootstrap";
    case BridgeLoopPhase::TopologyMigration:
        return "topology_migration";
    case BridgeLoopPhase::ActuatorBatch:
        return "actuator_batch";
    case BridgeLoopPhase::StatePublish:
        return "state_publish";
    case BridgeLoopPhase::BridgeDiagnostics:
        return "bridge_diagnostics";
    case BridgeLoopPhase::SerialPing:
        return "serial_ping";
    case BridgeLoopPhase::LoopEnd:
        return "loop_end";
    case BridgeLoopPhase::Unknown:
    default:
        return "unknown";
    }
}

[[nodiscard]] auto isBridgeLoopPhaseValueValid(std::uint8_t phase) -> bool
{
    return phase <= static_cast<std::uint8_t>(BridgeLoopPhase::LoopEnd);
}

void recordBridgeLoopPhase(BridgeLoopPhase phase)
{
#ifdef ESP32
    bridgeBreadcrumbMagic = kBridgeBreadcrumbMagic;
    bridgeBreadcrumbLastPhase = static_cast<std::uint8_t>(phase);
    bridgeBreadcrumbLastMillis = millis();
#else
    (void)phase;
#endif
}

[[nodiscard]] auto bridgeBreadcrumbsAreValid() -> bool
{
#ifdef ESP32
    return bridgeBreadcrumbMagic == kBridgeBreadcrumbMagic && isBridgeLoopPhaseValueValid(bridgeBreadcrumbLastPhase);
#else
    return false;
#endif
}

void initializeBridgeBreadcrumbs()
{
#ifdef ESP32
    if (!bridgeBreadcrumbsAreValid())
    {
        bridgeBreadcrumbMagic = kBridgeBreadcrumbMagic;
        bridgeBreadcrumbBootCount = 0U;
        bridgeBreadcrumbLastMillis = 0U;
        bridgeBreadcrumbLastPhase = static_cast<std::uint8_t>(BridgeLoopPhase::Unknown);
    }
    ++bridgeBreadcrumbBootCount;
#endif
}

void writePayloadByte(lsh::bridge::communication::FixedBufferWriter &writer, std::uint8_t byte)
{
    (void)writer.write(byte);
}

template <std::size_t Length> void writePayloadLiteral(lsh::bridge::communication::FixedBufferWriter &writer, const char (&literal)[Length])
{
    (void)writer.write(reinterpret_cast<const std::uint8_t *>(literal), Length - 1U);
}

#ifdef CONFIG_MSG_PACK_MQTT
void writeMsgPackUint8(lsh::bridge::communication::FixedBufferWriter &writer, std::uint8_t value)
{
    if (value <= 0x7FU)
    {
        writePayloadByte(writer, value);
        return;
    }

    writePayloadByte(writer, 0xCCU);
    writePayloadByte(writer, value);
}

void writeMsgPackArrayHeader(lsh::bridge::communication::FixedBufferWriter &writer, std::uint8_t elementCount)
{
    if (elementCount <= 15U)
    {
        writePayloadByte(writer, static_cast<std::uint8_t>(0x90U | elementCount));
        return;
    }

    writePayloadByte(writer, 0xDCU);
    writePayloadByte(writer, 0U);
    writePayloadByte(writer, elementCount);
}
#else
void writeJsonUint8(lsh::bridge::communication::FixedBufferWriter &writer, std::uint8_t value)
{
    if (value >= 100U)
    {
        writePayloadByte(writer, static_cast<std::uint8_t>('0' + (value / 100U)));
        value = static_cast<std::uint8_t>(value % 100U);
        writePayloadByte(writer, static_cast<std::uint8_t>('0' + (value / 10U)));
        writePayloadByte(writer, static_cast<std::uint8_t>('0' + (value % 10U)));
        return;
    }

    if (value >= 10U)
    {
        writePayloadByte(writer, static_cast<std::uint8_t>('0' + (value / 10U)));
        writePayloadByte(writer, static_cast<std::uint8_t>('0' + (value % 10U)));
        return;
    }

    writePayloadByte(writer, static_cast<std::uint8_t>('0' + value));
}
#endif

/**
 * @brief Build the compact authoritative state publish without a JsonDocument.
 * @details `ACTUATORS_STATE` has a fixed shallow shape and the packed bytes are
 *          already validated in `VirtualDevice`. Serializing it directly avoids
 *          ArduinoJson allocation and generic tree walking on the hottest MQTT
 *          publish path while still emitting exactly the active wire codec.
 */
[[nodiscard]] auto buildPackedStatePublishPayload(lsh::bridge::communication::FixedBufferWriter &writer, const VirtualDevice &virtualDevice)
    -> bool
{
    using namespace lsh::bridge::protocol;

    const std::uint8_t packedByteCount = virtualDevice.getPackedStateByteCount();

#ifdef CONFIG_MSG_PACK_MQTT
    writePayloadByte(writer, 0x82U);  // fixmap(2): {"p": ACTUATORS_STATE, "s": packed-bytes}
    writePayloadByte(writer, 0xA1U);
    writePayloadByte(writer, static_cast<std::uint8_t>('p'));
    writeMsgPackUint8(writer, static_cast<std::uint8_t>(Command::ACTUATORS_STATE));
    writePayloadByte(writer, 0xA1U);
    writePayloadByte(writer, static_cast<std::uint8_t>('s'));
    writeMsgPackArrayHeader(writer, packedByteCount);
    for (std::uint8_t byteIndex = 0U; byteIndex < packedByteCount; ++byteIndex)
    {
        writeMsgPackUint8(writer, virtualDevice.getPackedStateByte(byteIndex));
    }
#else
    writePayloadLiteral(writer, "{\"p\":");
    writeJsonUint8(writer, static_cast<std::uint8_t>(Command::ACTUATORS_STATE));
    writePayloadLiteral(writer, ",\"s\":[");
    for (std::uint8_t byteIndex = 0U; byteIndex < packedByteCount; ++byteIndex)
    {
        if (byteIndex != 0U)
        {
            writePayloadByte(writer, static_cast<std::uint8_t>(','));
        }
        writeJsonUint8(writer, virtualDevice.getPackedStateByte(byteIndex));
    }
    writePayloadLiteral(writer, "]}");
#endif

    return !writer.overflowed() && writer.size() != 0U;
}

[[nodiscard]] auto validateInboundNetworkClickFields(std::uint8_t clickType, std::uint8_t clickableId, std::uint8_t correlationId) -> bool;

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

    return validateInboundNetworkClickFields(outClickType, outClickableId, outCorrelationId);
}

[[nodiscard]] auto validateInboundNetworkClickFields(std::uint8_t clickType, std::uint8_t clickableId, std::uint8_t correlationId) -> bool
{
    using namespace lsh::bridge::protocol;

    switch (static_cast<ProtocolClickType>(clickType))
    {
    case ProtocolClickType::LONG:
    case ProtocolClickType::SUPER_LONG:
        break;

    default:
        return false;
    }

    return clickableId != 0U && correlationId != 0U;
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
    if (mode == lsh::bridge::LoggingMode::Enabled)
    {
        return false;
    }
    if (mode == lsh::bridge::LoggingMode::Disabled)
    {
        return true;
    }

    // In AutoFromBuild mode the runtime mirrors the compile-time choice:
    // debug builds keep Homie logs enabled, release builds silence them.
#ifdef LSH_DEBUG
    return false;
#else
    return true;
#endif
}

/**
 * @brief Deserialize only the MQTT command id from one inbound command payload.
 * @details Most bridge commands need only the `p` field. ArduinoJson's filter
 *          keeps the safety net of the normal parser while preventing unrelated
 *          fields from consuming the larger command document on these hot paths.
 */
[[nodiscard]] auto deserializeMqttCommandIdDocument(JsonDocument &commandIdDocument, const char *payload, std::size_t payloadLength)
    -> DeserializationError
{
    using namespace lsh::bridge::protocol;

    StaticJsonDocument<JSON_OBJECT_SIZE(1)> commandIdFilter;
    commandIdFilter[KEY_PAYLOAD] = true;

#ifdef CONFIG_MSG_PACK_MQTT
    return deserializeMsgPack(commandIdDocument, reinterpret_cast<const std::uint8_t *>(payload), payloadLength,
                              DeserializationOption::Filter(commandIdFilter), DeserializationOption::NestingLimit(2));
#else
    return deserializeJson(commandIdDocument, payload, payloadLength, DeserializationOption::Filter(commandIdFilter),
                           DeserializationOption::NestingLimit(2));
#endif
}

/**
 * @brief Deserialize one complete MQTT command payload for commands that need fields beyond `p`.
 */
[[nodiscard]] auto deserializeMqttCommandDocument(JsonDocument &commandDocument, const char *payload, std::size_t payloadLength)
    -> DeserializationError
{
#ifdef CONFIG_MSG_PACK_MQTT
    return deserializeMsgPack(commandDocument, reinterpret_cast<const std::uint8_t *>(payload), payloadLength,
                              DeserializationOption::NestingLimit(2));
#else
    return deserializeJson(commandDocument, payload, payloadLength, DeserializationOption::NestingLimit(2));
#endif
}

/**
 * @brief Parse the command id using the smallest possible ArduinoJson document.
 */
[[nodiscard]] auto parseMqttCommandId(const char *payload, std::size_t payloadLength, lsh::bridge::protocol::Command &outCommand) -> bool
{
    using namespace lsh::bridge::protocol;

    StaticJsonDocument<constants::controllerSerial::MQTT_COMMAND_ID_DOC_SIZE> commandIdDocument;
    if (deserializeMqttCommandIdDocument(commandIdDocument, payload, payloadLength) != DeserializationError::Ok ||
        commandIdDocument.overflowed())
    {
        return false;
    }

    std::uint8_t rawCommand = 0U;
    if (!utils::json::tryGetUint8Scalar(commandIdDocument[KEY_PAYLOAD], rawCommand))
    {
        return false;
    }

    outCommand = static_cast<Command>(rawCommand);
    return true;
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
        serial(resolveSerial(bridgeOptions.serial)), controllerSerialLink(serial, virtualDevice), options(bridgeOptions)
    {
        options.serial = serial;
    }

    HardwareSerial *const serial;
    bool isStarted = false;          //!< True after `begin()` has fully wired serial, Homie and callbacks.
    bool mqttResyncPending = false;  //!< True while MQTT-side resync still needs another retry after reconnect or publish refusal.
    bool topologyChangedDiagnosticPending =
        false;  //!< True while `topology_changed` still needs one best-effort MQTT publish before reboot.
    bool bridgeBreadcrumbDiagnosticPending = false;  //!< True while the previous-boot loop breadcrumb still needs to be published.
    std::uint32_t lastMqttResyncAttemptMs = 0U;      //!< Real-time timestamp of the last autonomous MQTT-side resync attempt.
    std::uint32_t topologyRebootPendingSinceMs =
        0U;  //!< Real-time timestamp of the successful topology save that started the reboot window.
    std::uint32_t topologyDiagnosticAcceptedMs = 0U;  //!< Real-time timestamp of the accepted `topology_changed` MQTT publish, if any.
    std::uint32_t previousBridgeLoopPhaseMs = 0U;     //!< Millis value recorded with the previous-boot loop breadcrumb.
    std::uint32_t previousBridgeBootCount = 0U;       //!< RTC boot counter captured before the current boot increments it.
    BridgeLoopPhase previousBridgeLoopPhase = BridgeLoopPhase::Unknown;  //!< Last recorded phase from the previous boot.
    runtime::RuntimeHotState runtimeState{};  //!< Hot controller-sync state grouped for locality and helper passing.
    VirtualDevice virtualDevice{};
    ControllerSerialLink controllerSerialLink;
    AsyncMqttClient *mqttClient = nullptr;
    AsyncMqttClient *mqttMessageBoundClient = nullptr;
    MqttCommandQueue mqttCommandQueue{};
    BridgeOptions options{};
    etl::vector<LSHNode, constants::virtualDevice::MAX_ACTUATORS> homieNodes{};
    DeviceDetailsSnapshot pendingTopologyDetails{};  //!< Validated topology waiting to be persisted before a controlled reboot.

    void capturePreviousBridgeBreadcrumb()
    {
#ifdef ESP32
        const bool hasPreviousBreadcrumb = bridgeBreadcrumbsAreValid();
        previousBridgeLoopPhase =
            hasPreviousBreadcrumb ? static_cast<BridgeLoopPhase>(bridgeBreadcrumbLastPhase) : BridgeLoopPhase::Unknown;
        previousBridgeLoopPhaseMs = hasPreviousBreadcrumb ? bridgeBreadcrumbLastMillis : 0U;
        previousBridgeBootCount = hasPreviousBreadcrumb ? bridgeBreadcrumbBootCount : 0U;
        bridgeBreadcrumbDiagnosticPending = hasPreviousBreadcrumb && previousBridgeLoopPhase != BridgeLoopPhase::Unknown;
        initializeBridgeBreadcrumbs();
#else
        bridgeBreadcrumbDiagnosticPending = false;
#endif
    }

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

        if (options.disableResetTrigger)
        {
            Homie.disableResetTrigger();
        }

#ifdef ESP32
        if (options.disableWifiSleep)
        {
            WiFi.setSleep(false);
        }
#endif

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

        const std::uint8_t actuatorCount = virtualDevice.getTotalActuators();
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < actuatorCount; ++actuatorIndex)
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
        return runtime::bootstrapPhaseName(runtimeState.bootstrapPhase);
    }

    /**
     * @brief Drops transient runtime state that depends on a fresh authoritative
     * controller sync.
     */
    void clearPendingRuntimeState()
    {
        runtime::clearPendingRuntimeState(controllerSerialLink, runtimeState);
    }

    /**
     * @brief Ask the loop to send the next bootstrap request as soon as allowed.
     * @details Requests are rate-limited in the main loop so the bridge never
     *          pounds the controller in a tight retry loop.
     */
    void scheduleBootstrapRequestNow()
    {
        runtime::scheduleBootstrapRequestNow(runtimeState);
    }

    /**
     * @brief Enter the phase that waits for fresh authoritative `DEVICE_DETAILS`.
     * @details This is the safest phase after a controller `BOOT` because the
     *          bridge must re-confirm the topology before trusting actuator IDs,
     *          button IDs, or the controller-backed event channels.
     */
    void enterWaitingForDetails()
    {
        runtime::enterWaitingForDetails(controllerSerialLink, virtualDevice, runtimeState);
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
        runtime::enterWaitingForState(controllerSerialLink, virtualDevice, runtimeState);
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
        runtime::stageTopologyMigration(controllerSerialLink, virtualDevice, runtimeState, pendingTopologyDetails, details);
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
        runtime::requestAuthoritativeStateRefresh(controllerSerialLink, virtualDevice, runtimeState);
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
            runtimeState.canServeCachedStateRequests = false;
            runtimeState.bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;
            scheduleBootstrapRequestNow();
            mqttResyncPending = false;
            return true;
        }

        if (!publishCachedDetails())
        {
            runtimeState.canServeCachedStateRequests = false;
            mqttResyncPending = true;
            return false;
        }

        const bool controllerStateUsable = virtualDevice.isRuntimeSynchronized() && controllerSerialLink.isConnected();
        if (controllerStateUsable)
        {
            if (!publishAuthoritativeLshState())
            {
                runtimeState.canServeCachedStateRequests = false;
                mqttResyncPending = true;
                return false;
            }

            const bool homiePublished = publishAllHomieNodeStates();
            runtimeState.canServeCachedStateRequests = homiePublished;
            mqttResyncPending = !homiePublished;
            if (homiePublished)
            {
                if (virtualDevice.isFullStatePublishPending())
                {
                    virtualDevice.clearFullStatePublishPending();
                }
                virtualDevice.clearDirtyActuators();
                runtimeState.isAuthoritativeStateDirty = false;
            }
            return homiePublished;
        }

        runtimeState.canServeCachedStateRequests = false;
        virtualDevice.invalidateRuntimeModel();
        mqttResyncPending = false;

        if (runtimeState.bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
        {
            scheduleBootstrapRequestNow();
            return true;
        }

        if (runtimeState.bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
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

        const auto now = timeKeeper::getTime();
        if (lastMqttResyncAttemptMs != 0U && (now - lastMqttResyncAttemptMs) < constants::runtime::BOOTSTRAP_REQUEST_INTERVAL_MS)
        {
            return;
        }

        lastMqttResyncAttemptMs = now;
        updateMqttClient();
        if (!subscribeMqttTopics())
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
        runtime::refreshControllerConnectivity(isConnectedNow, controllerSerialLink, virtualDevice, runtimeState);
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

    [[nodiscard]] auto publishBridgeBreadcrumbDiagnostic() -> bool
    {
        if (!bridgeBreadcrumbDiagnosticPending)
        {
            return true;
        }

        if (!Homie.isConnected() || !hasBridgeTopic())
        {
            return false;
        }

        StaticJsonDocument<kBridgeBreadcrumbDocSize> diagnosticDocument;
        diagnosticDocument[kBridgeDiagnosticEventKey] = kBridgeBreadcrumbEvent;
        diagnosticDocument[kBridgeDiagnosticKindKey] = kBridgeBreadcrumbKind;
        diagnosticDocument[kBridgeBreadcrumbPhaseKey] = bridgeLoopPhaseName(previousBridgeLoopPhase);
        diagnosticDocument[kBridgeBreadcrumbPhaseMsKey] = previousBridgeLoopPhaseMs;
        diagnosticDocument[kBridgeBreadcrumbBootCountKey] = previousBridgeBootCount;

        if (!MqttPublisher::sendJson(diagnosticDocument, MqttTopicsBuilder::mqttOutBridgeTopic.c_str(), true, 1))
        {
            return false;
        }

        bridgeBreadcrumbDiagnosticPending = false;
        return true;
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
        if (runtimeState.bootstrapPhase == BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            return;
        }

        if (runtimeState.bootstrapPhase != BootstrapPhase::WAITING_FOR_DETAILS &&
            runtimeState.bootstrapPhase != BootstrapPhase::WAITING_FOR_STATE)
        {
            return;
        }

        const auto now = timeKeeper::getTime();
        if (!runtimeState.bootstrapRequestDue &&
            (now - runtimeState.lastBootstrapRequestMs) < constants::runtime::BOOTSTRAP_REQUEST_INTERVAL_MS)
        {
            return;
        }

        const auto requestType = runtimeState.bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS
                                     ? constants::payloads::StaticType::ASK_DETAILS
                                     : constants::payloads::StaticType::ASK_STATE;
        if (!controllerSerialLink.sendJson(requestType))
        {
            return;
        }

        runtimeState.lastBootstrapRequestMs = now;
        runtimeState.bootstrapRequestDue = false;
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
        if (runtimeState.bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            return;
        }

        const auto now = timeKeeper::getTime();
        if (!runtimeState.hasPendingTopologySave)
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

        if (runtimeState.lastTopologySaveAttemptMs != 0U &&
            (now - runtimeState.lastTopologySaveAttemptMs) < constants::runtime::TOPOLOGY_SAVE_RETRY_INTERVAL_MS)
        {
            return;
        }

        runtimeState.lastTopologySaveAttemptMs = now;
        if (!DetailsCacheStore::save(pendingTopologyDetails))
        {
            return;
        }

        runtimeState.hasPendingTopologySave = false;
        topologyChangedDiagnosticPending = true;
        topologyRebootPendingSinceMs = now;
        topologyDiagnosticAcceptedMs = 0U;
    }

    void markAuthoritativeStateDirty()
    {
        // A fresh controller-backed state arrived. Delay publication slightly so
        // quick successive state frames collapse into one MQTT/Homie refresh wave.
        runtimeState.isAuthoritativeStateDirty = true;
        runtimeState.canServeCachedStateRequests = false;
        runtimeState.lastAuthoritativeStateUpdateMs = timeKeeper::getTime();
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
        if (!hasDeviceScopedTopics())
        {
            return false;
        }

        char payloadBuffer[constants::controllerSerial::MQTT_PUBLISH_MESSAGE_MAX_SIZE];
        lsh::bridge::communication::FixedBufferWriter payloadWriter(payloadBuffer, sizeof(payloadBuffer));
        if (!buildPackedStatePublishPayload(payloadWriter, virtualDevice))
        {
            return false;
        }

        return MqttPublisher::sendRaw(MqttTopicsBuilder::mqttOutStateTopic.c_str(), true, 1, payloadWriter.data(), payloadWriter.size());
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
        if (!runtimeState.canServeCachedStateRequests || !virtualDevice.isRuntimeSynchronized() || !controllerSerialLink.isConnected() ||
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
            yield();
        }

        return allPublished;
    }

    [[nodiscard]] auto publishChangedHomieNodeStates() -> bool
    {
        bool allPublished = true;
        const std::uint8_t actuatorCount = virtualDevice.getTotalActuators();
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < actuatorCount; ++actuatorIndex)
        {
            if (virtualDevice.isActuatorDirtyUnchecked(actuatorIndex))
            {
                if (!homieNodes[actuatorIndex].sendState())
                {
                    allPublished = false;
                }
                yield();
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
        if (!runtimeState.isAuthoritativeStateDirty)
        {
            return;
        }

        if (!Homie.isConnected())
        {
            return;
        }

        const auto now = timeKeeper::getTime();
        if ((now - runtimeState.lastAuthoritativeStateUpdateMs) < constants::runtime::STATE_PUBLISH_SETTLE_INTERVAL_MS)
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
        runtimeState.isAuthoritativeStateDirty = false;
        runtimeState.canServeCachedStateRequests = true;
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
            runtimeState.hasPendingTopologySave = false;
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
        if (runtimeState.bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS ||
            runtimeState.bootstrapPhase == BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            return;
        }

        const auto stateResult = controllerSerialLink.storeStateFromReceived();
        if (stateResult == DeserializeExitCode::OK_STATE)
        {
            controllerSerialLink.reconcileDesiredActuatorStatesFromAuthoritative();
            runtimeState.bootstrapPhase = BootstrapPhase::SYNCED;
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
        case DeserializeExitCode::OK_OTHER_PAYLOAD:
            if (Homie.isConnected())
            {
                if (!MqttPublisher::sendJson(messageDocument, MqttTopicsBuilder::mqttOutEventsTopic.c_str(), false, 2))
                {
                    DPL(code == DeserializeExitCode::OK_NETWORK_CLICK_CONFIRM
                            ? "Dropping controller NETWORK_CLICK_CONFIRM because the MQTT client did not accept the publish."
                            : "Dropping controller event payload because the MQTT client did not accept the publish.");
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
     */
    void updateMqttClient()
    {
        // Homie exposes its MQTT client as a reference, so reaching this point
        // always yields one concrete AsyncMqttClient instance.
        auto &nextClient = Homie.getMqttClient();

        if (&nextClient == mqttClient)
        {
            // The same AsyncMqttClient instance survived the reconnect, so only the
            // subscriptions may need refreshing.
            return;
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
     * @brief Handles MQTT commands that need fields beyond the command id.
     * @details Stateless commands are handled before the caller allocates a full
     *          document. This function intentionally contains only commands whose
     *          payload must be inspected or normalized before reaching the UART.
     *
     * @param commandDocument validated MQTT command decoded from the current
     *        inbound frame.
     * @return TypedCommandResult::Handled when the bridge consumed the command.
     * @return TypedCommandResult::Unsupported when this helper was called with a
     *         command that should have stayed on the stateless/raw-forward path.
     * @return TypedCommandResult::Invalid when the payload type is known but its
     *         concrete fields do not pass bridge-side validation.
     * @return TypedCommandResult::DeliveryFailed when the command was valid but
     *         the bridge could not deliver the resulting publish or serial write.
     */
    [[nodiscard]] auto forwardTypedControllerCommand(lsh::bridge::protocol::Command command, const JsonDocument &commandDocument)
        -> TypedCommandResult
    {
        using namespace lsh::bridge::protocol;

        switch (command)
        {
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
            std::uint8_t clickType = 0U;
            std::uint8_t clickableId = 0U;
            std::uint8_t correlationId = 0U;

            if (!validateInboundNetworkClickCommand(commandDocument, clickType, clickableId, correlationId))
            {
                return TypedCommandResult::Invalid;
            }

            if (!controllerSerialLink.sendClickCommand(command, clickType, clickableId, correlationId))
            {
                return TypedCommandResult::DeliveryFailed;
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
     * @brief Decode and handle one document-backed MQTT command with a command-specific pool size.
     * @details Only a few device-topic commands need fields beyond `p`. Keeping
     *          their JsonDocument capacity explicit avoids paying the worst-case
     *          `SET_STATE` pool for smaller actuator/click commands while still
     *          preserving ArduinoJson's malformed-payload safety net.
     */
    template <std::uint16_t DocumentCapacity>
    void processDocumentBackedDeviceTopicCommand(lsh::bridge::protocol::Command command, const char *payload, std::size_t payloadLength)
    {
        using namespace lsh::bridge::protocol;

        StaticJsonDocument<DocumentCapacity> commandDocument;
        if (deserializeMqttCommandDocument(commandDocument, payload, payloadLength) != DeserializationError::Ok ||
            commandDocument.overflowed())
        {
            mqttCommandQueue.recordRejectedCommand(MqttRejectedCommandReason::Malformed);
            DPL("Dropping MQTT command because the full payload is not valid JSON/MsgPack "
                "for the active MQTT codec.");
            return;
        }

        std::uint8_t fullDocumentCommandId = 0U;
        if (!utils::json::tryGetUint8Scalar(commandDocument[KEY_PAYLOAD], fullDocumentCommandId) ||
            fullDocumentCommandId != static_cast<std::uint8_t>(command))
        {
            mqttCommandQueue.recordRejectedCommand(MqttRejectedCommandReason::Malformed);
            DPL("Dropping MQTT command because the command id changed between lightweight and full decode.");
            return;
        }

        const auto typedCommandResult = forwardTypedControllerCommand(command, commandDocument);
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

        // Reaching Unsupported here would mean the lightweight routing table and
        // the document-backed handler drifted apart. Drop loudly in debug builds
        // rather than raw-forwarding a bridge-known command through the wrong path.
        DPL("Dropping MQTT command because internal command routing is inconsistent.");
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
     */
    void processDeviceTopicCommand(lsh::bridge::protocol::Command command,
                                   const char *payload,
                                   std::size_t payloadLength,
                                   const DecodedMqttCommand *decodedCommand)
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

        case Command::REQUEST_DETAILS:
            // DEVICE_DETAILS can be served from the cached validated topology. If
            // that cache is missing, keep the bridge online and ask the
            // controller for a fresh authoritative topology instead.
            if (!virtualDevice.hasCachedDetails())
            {
                enterWaitingForDetails();
            }
            else if (!publishCachedDetails())
            {
                DPL("Dropping REQUEST_DETAILS reply because the MQTT client did not accept the cached topology publish.");
            }
            return;

        case Command::REQUEST_STATE:
            // Prefer the local cache only when it is known to mirror a controller-
            // backed snapshot from the current session. Otherwise ask the controller.
            if (!tryServeCachedStateRequest() && controllerSerialLink.isConnected())
            {
                if (!virtualDevice.hasCachedDetails() || runtimeState.bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
                {
                    enterWaitingForDetails();
                }
                else
                {
                    requestAuthoritativeStateRefresh();
                }
            }
            return;

        case Command::FAILOVER:
            if (!controllerSerialLink.sendJson(constants::payloads::StaticType::GENERAL_FAILOVER))
            {
                DPL("Dropping FAILOVER command because the UART did not accept the bridge-local failover frame.");
            }
            return;

        case Command::NETWORK_CLICK_REQUEST:
        case Command::NETWORK_CLICK_CONFIRM:
            // These are controller-originated events. Accepting them from MQTT
            // would turn an upstream event channel into a confusing command loop.
            DPL("Dropping MQTT command because the command id belongs to a controller-originated event.");
            return;

        case Command::SET_SINGLE_ACTUATOR:
            // During re-sync or topology migration the bridge must not reinterpret
            // actuator writes against a stale cached model. Dropping them is
            // safer than forwarding ambiguous intent to the controller.
            if (!virtualDevice.isRuntimeSynchronized())
            {
                return;
            }
            if (decodedCommand != nullptr && decodedCommand->shape == DecodedMqttCommandShape::SetSingleActuator)
            {
                if (!controllerSerialLink.stageSingleActuatorCommand(decodedCommand->actuatorId, decodedCommand->state))
                {
                    DPL("Dropping invalid MQTT SET_SINGLE_ACTUATOR command decoded by the shallow parser.");
                }
                return;
            }
            processDocumentBackedDeviceTopicCommand<constants::controllerSerial::MQTT_SET_SINGLE_ACTUATOR_DOC_SIZE>(command, payload,
                                                                                                                    payloadLength);
            return;

        case Command::SET_STATE:
            // During re-sync or topology migration the bridge must not reinterpret
            // actuator writes against a stale cached model. Dropping them is
            // safer than forwarding ambiguous intent to the controller.
            if (!virtualDevice.isRuntimeSynchronized())
            {
                return;
            }
            if (decodedCommand != nullptr && decodedCommand->shape == DecodedMqttCommandShape::SetPackedState)
            {
                if (!controllerSerialLink.stageDesiredPackedStateBytes(decodedCommand->packedStateBytes, decodedCommand->packedStateLength))
                {
                    DPL("Dropping invalid MQTT SET_STATE command decoded by the shallow parser.");
                }
                return;
            }
            processDocumentBackedDeviceTopicCommand<constants::controllerSerial::MQTT_SET_STATE_DOC_SIZE>(command, payload, payloadLength);
            return;

        case Command::NETWORK_CLICK_ACK:
        case Command::FAILOVER_CLICK:
            if (decodedCommand != nullptr && decodedCommand->shape == DecodedMqttCommandShape::Click)
            {
                if (!validateInboundNetworkClickFields(decodedCommand->clickType, decodedCommand->clickableId,
                                                       decodedCommand->correlationId))
                {
                    DPL("Dropping invalid MQTT click command decoded by the shallow parser.");
                    return;
                }

                if (!controllerSerialLink.sendClickCommand(command, decodedCommand->clickType, decodedCommand->clickableId,
                                                           decodedCommand->correlationId))
                {
                    DPL("Dropping bridge-known MQTT click command because the UART did not accept the direct frame.");
                }
                return;
            }
            processDocumentBackedDeviceTopicCommand<constants::controllerSerial::MQTT_CLICK_DOC_SIZE>(command, payload, payloadLength);
            return;

        default:
            break;
        }

        if (!forwardUnsupportedCommandToController(payload, payloadLength))
        {
            DPL("Dropping unsupported MQTT command because the bridge cannot "
                "safely translate an unknown payload across different MQTT "
                "and serial codecs.");
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
            // The service-topic BOOT only asks the bridge to refresh its MQTT-side
            // mirror. The main loop already owns the resync pacing logic, so keep
            // this path side-effect free and let the queued loop phase execute it.
            scheduleMqttResyncNow();
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

        DecodedMqttCommand decodedCommand{};
        const bool shallowDecoded = decodeMqttCommandShallow(payload, payloadLength, decodedCommand);

        Command command = decodedCommand.command;
        if (!shallowDecoded && !parseMqttCommandId(payload, payloadLength, command))
        {
            mqttCommandQueue.recordRejectedCommand(MqttRejectedCommandReason::Malformed);
            DPL("Dropping MQTT command because the payload is missing a valid uint8 "
                "command id in key 'p'.");
            return;
        }

        if (source == MqttCommandSource::Service)
        {
            processServiceTopicCommand(command);
            return;
        }

        processDeviceTopicCommand(command, payload, payloadLength, shallowDecoded ? &decodedCommand : nullptr);
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
            QueuedMqttCommand queuedCommand;
            if (!mqttCommandQueue.dequeue(queuedCommand))
            {
                return;
            }

            // The queue stores raw bytes, but once the frame is dequeued it is safe
            // to reinterpret them as `const char*`: both the lightweight command-id
            // decode and any later full parse only read the contiguous payload bytes.
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
    void handleMqttMessage(const char *topic,
                           const char *payload,
                           AsyncMqttClientMessageProperties properties,
                           std::size_t len,
                           std::size_t index,
                           std::size_t total)
    {
        MqttCommandSource source = MqttCommandSource::Device;

        if (std::strcmp(topic, MqttTopicsBuilder::mqttInTopic.c_str()) == 0)
        {
            source = MqttCommandSource::Device;
        }
        else if (std::strcmp(topic, constants::mqtt::MQTT_TOPIC_SERVICE) == 0)
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
LSHBridge::LSHBridge(BridgeOptions options)
{
    static_assert(sizeof(Impl) <= detail::BRIDGE_IMPL_STORAGE_SIZE,
                  "LSHBridge::Impl does not fit in opaque storage. Increase CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE.");
    static_assert(alignof(Impl) <= alignof(std::max_align_t), "LSHBridge::Impl alignment is larger than std::max_align_t.");

    this->implementation = new (static_cast<void *>(this->implementationStorage)) Impl(options);
}

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

    if (this->implementation != nullptr)
    {
        this->implementation->~Impl();
        this->implementation = nullptr;
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
    auto &impl = *this->implementation;
    if (impl.isStarted)
    {
        return;
    }

    impl.capturePreviousBridgeBreadcrumb();
    recordBridgeLoopPhase(BridgeLoopPhase::Begin);

    // Open the USB serial console before any debug log can fire. Without this
    // explicit initialization a debug build compiles the logging calls but the
    // bridge USB port would remain silent.
    DSB();

    activeInstance = this;
    timeKeeper::update();
    recordBridgeLoopPhase(BridgeLoopPhase::BeginSerial);
    impl.controllerSerialLink.begin();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::BeginCache);
    (void)impl.loadCachedDetailsFromFlash();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::BeginConfigureHomie);
    impl.configureHomie();
    Homie.onEvent(onHomieEventStatic);
    impl.runtimeState.isControllerConnected = impl.controllerSerialLink.isConnected();

    auto runtimeDelegate = ControllerSerialLink::MessageCallback::create<Impl, &Impl::handleControllerSerialMessage>(impl);
    impl.controllerSerialLink.onMessage(runtimeDelegate);

    recordBridgeLoopPhase(BridgeLoopPhase::BeginHomieSetup);
    Homie.setup();
    impl.isStarted = true;
    recordBridgeLoopPhase(BridgeLoopPhase::BeginWaitingForDetails);
    impl.enterWaitingForDetails();
    yield();

#ifdef HOMIE_RESET
    if (Homie.isConfigured())
    {
        Homie.reset();
        Homie.setIdle(true);
    }
#endif

    recordBridgeLoopPhase(BridgeLoopPhase::LoopEnd);
}

/**
 * @brief Execute one bridge main-loop iteration.
 */
void LSHBridge::loop()
{
    auto &impl = *this->implementation;
    if (!impl.isStarted)
    {
        return;
    }

    recordBridgeLoopPhase(BridgeLoopPhase::LoopStart);
    timeKeeper::update();
    recordBridgeLoopPhase(BridgeLoopPhase::HomieLoop);
    Homie.loop();
    yield();

    if (impl.serial->available())
    {
        recordBridgeLoopPhase(BridgeLoopPhase::SerialRx);
        impl.controllerSerialLink.processSerialBuffer();
        yield();
    }

    recordBridgeLoopPhase(BridgeLoopPhase::MqttQueue);
    impl.processQueuedMqttCommands();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::ControllerConnectivity);
    impl.refreshControllerConnectivity();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::MqttResync);
    impl.processPendingMqttResync();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::Bootstrap);
    impl.processBootstrapProgress();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::TopologyMigration);
    impl.processPendingTopologyMigration();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::ActuatorBatch);
    impl.controllerSerialLink.processPendingActuatorBatch();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::StatePublish);
    impl.processPendingStatePublishes();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::BridgeDiagnostics);
    impl.publishPendingBridgeDiagnostics();
    (void)impl.publishBridgeBreadcrumbDiagnostic();
    yield();
    recordBridgeLoopPhase(BridgeLoopPhase::SerialPing);
    if (!impl.controllerSerialLink.sendJson(constants::payloads::StaticType::PING_))
    {
        // A rejected heartbeat is harmless here: either the ping interval is
        // still closed or the UART refused the frame. In both cases the next
        // loop iteration will retry under the same normal pacing rules.
    }
    recordBridgeLoopPhase(BridgeLoopPhase::LoopEnd);
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
// AsyncMqttClient's callback ABI exposes mutable pointers. The bridge treats
// both buffers as read-only and immediately forwards them to a const-correct
// instance method after the ABI boundary.
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
