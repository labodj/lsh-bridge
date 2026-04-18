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
#include "lsh_node.hpp"
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

/// @brief Document size used for bridge-local diagnostics published on `misc`.
static constexpr const std::uint16_t kBridgeDiagnosticDocSize = 192U;
/// @brief Saturation cap for diagnostic counters accumulated by the runtime.
static constexpr const std::uint16_t kBridgeDiagnosticCounterMax = 0xFFFFU;
/// @brief Field name that identifies the type of bridge-local diagnostic.
constexpr char kBridgeDiagnosticKey[] = "bridge_diagnostic";
/// @brief Field name that reports how long an unstable actuator batch stayed open.
constexpr char kPendingDurationMsKey[] = "pending_ms";
/// @brief Field name that reports how many command changes were merged into one dropped batch.
constexpr char kMutationCountKey[] = "mutation_count";
/// @brief Field name that reports how many device-topic MQTT commands were dropped because the queue was full.
constexpr char kDroppedDeviceCommandsKey[] = "dropped_device_commands";
/// @brief Field name that reports how many service-topic MQTT commands were dropped because the queue was full.
constexpr char kDroppedServiceCommandsKey[] = "dropped_service_commands";
/// @brief Diagnostic value emitted when the bridge discards an actuator batch that never became stable.
constexpr char kActuatorStormDroppedDiagnostic[] = "actuator_command_storm_dropped";
/// @brief Diagnostic value emitted when MQTT callbacks outpace the bridge queue.
constexpr char kMqttQueueOverflowDiagnostic[] = "mqtt_queue_overflow";

[[nodiscard]] auto resolveSerial(HardwareSerial *serial) -> HardwareSerial *
{
    return serial != nullptr ? serial : &Serial2;
}

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

[[nodiscard]] auto tryGetUint8Scalar(const JsonVariantConst value, std::uint8_t &outValue) -> bool
{
    if (value.isNull() || value.is<const char *>() || value.is<bool>() || value.is<JsonArrayConst>() || value.is<JsonObjectConst>())
    {
        return false;
    }

    if (value.is<std::uint8_t>())
    {
        outValue = value.as<std::uint8_t>();
        return true;
    }

    const double rawValue = value.as<double>();
    if (rawValue < 0.0 || rawValue > 255.0)
    {
        return false;
    }

    const auto coercedValue = static_cast<std::uint8_t>(rawValue);
    if (static_cast<double>(coercedValue) != rawValue)
    {
        return false;
    }

    outValue = coercedValue;
    return true;
}

[[nodiscard]] auto tryGetBinaryState(const JsonVariantConst value, bool &outState) -> bool
{
    std::uint8_t rawState = 0U;
    if (!tryGetUint8Scalar(value, rawState) || rawState > 1U)
    {
        return false;
    }

    outState = (rawState == 1U);
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
     * @brief Tracks the blocking controller bootstrap handshake.
     */
    enum class HandshakeState
    {
        IDLE,                 //!< No bootstrap handshake is currently running.
        WAITING_FOR_DETAILS,  //!< Waiting for the controller topology snapshot.
        WAITING_FOR_STATE,    //!< Waiting for the first authoritative actuator state.
        COMPLETE,             //!< Details and state are both valid, runtime may start.
        FAIL_RESTART          //!< Bootstrap failed and the outer loop must start over.
    };

    /**
     * @brief Stores one MQTT payload until the loop is ready to process it.
     * @details MQTT callbacks can fire while the serial path is busy. The bridge
     *          therefore copies complete MQTT frames into a small fixed queue and
     *          drains that queue from the main loop when the serial side is idle.
     */
    struct QueuedMqttCommand
    {
        MqttCommandSource source = MqttCommandSource::Device;
        std::uint16_t length = 0U;
        std::uint8_t payload[constants::controllerSerial::RAW_MESSAGE_MAX_SIZE]{};
    };

    explicit Impl(BridgeOptions bridgeOptions) :
        options(std::move(bridgeOptions)), serial(resolveSerial(options.serial)), controllerSerialLink(serial, virtualDevice)
    {
        options.serial = serial;
    }

    BridgeOptions options{};
    bool isStarted = false;
    HandshakeState handshakeState = HandshakeState::IDLE;
    bool isAuthoritativeStateDirty = false;
    bool canServeCachedStateRequests = false;
    bool isControllerConnected = false;
    std::uint32_t lastAuthoritativeStateUpdateMs = 0U;
    etl::vector<LSHNode, constants::virtualDevice::MAX_ACTUATORS> homieNodes{};
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
     * @brief Publishes the cached controller details snapshot to MQTT.
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
     */
    void requestAuthoritativeStateRefresh()
    {
        clearPendingRuntimeState();
        controllerSerialLink.sendJson(constants::payloads::StaticType::ASK_STATE);
    }

    /**
     * @brief Re-synchronizes the MQTT-side peer without tearing down the whole
     * bridge.
     */
    void softResyncMqttPeer()
    {
        if (!virtualDevice.hasCachedDetails())
        {
            Homie.reboot();
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

        if (controllerSerialLink.isConnected())
        {
            requestAuthoritativeStateRefresh();
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
            return;
        }

        if (!virtualDevice.isRuntimeSynchronized())
        {
            // The controller came back while the bridge still considers the runtime
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
        if (payload == nullptr || payloadLength == 0U || payloadLength > constants::controllerSerial::RAW_MESSAGE_MAX_SIZE)
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
     *          captured in a previous session after reconnect.
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
     * @brief Forget every bridge-local diagnostic that still belongs to an old
     * MQTT session.
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
     * @brief Publish pending bridge-local diagnostics on the `misc` topic.
     * @details Diagnostics are emitted from the main loop, not from callbacks, so
     *          the bridge never tries to publish MQTT while already executing the
     *          MQTT client's message callback.
     */
    void publishPendingBridgeDiagnostics()
    {
        if (!Homie.isConnected())
        {
            return;
        }

        ControllerSerialLink::ActuatorStormDiagnostic actuatorStormDiagnostic{};
        if (controllerSerialLink.peekPendingActuatorStormDiagnostic(actuatorStormDiagnostic))
        {
            StaticJsonDocument<kBridgeDiagnosticDocSize> diagnosticDoc;
            diagnosticDoc[kBridgeDiagnosticKey] = kActuatorStormDroppedDiagnostic;
            diagnosticDoc[kPendingDurationMsKey] = actuatorStormDiagnostic.pendingDurationMs;
            diagnosticDoc[kMutationCountKey] = actuatorStormDiagnostic.mutationCount;

            if (MqttPublisher::sendJson(diagnosticDoc, MqttTopicsBuilder::mqttOutMiscTopic.c_str(), false, 1))
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
        diagnosticDoc[kBridgeDiagnosticKey] = kMqttQueueOverflowDiagnostic;
        if (droppedDeviceCommands > 0U)
        {
            diagnosticDoc[kDroppedDeviceCommandsKey] = droppedDeviceCommands;
        }
        if (droppedServiceCommands > 0U)
        {
            diagnosticDoc[kDroppedServiceCommandsKey] = droppedServiceCommands;
        }

        if (MqttPublisher::sendJson(diagnosticDoc, MqttTopicsBuilder::mqttOutMiscTopic.c_str(), false, 1))
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

    void handleDisconnect()
    {
        mqttClient = nullptr;
        clearPendingRuntimeState();
        clearQueuedMqttCommands();
        clearPendingBridgeDiagnostics();
    }

    /**
     * @brief Performs the blocking `DETAILS -> STATE` bootstrap handshake.
     */
    void bootstrapDevice()
    {
        auto handshakeDelegate = ControllerSerialLink::MessageCallback::create<Impl, &Impl::handleHandshakeMessage>(*this);
        controllerSerialLink.onMessage(handshakeDelegate);

        while (handshakeState != HandshakeState::COMPLETE)
        {
            // Each outer-loop pass represents one full bootstrap attempt.
            // Any fatal handshake condition resets the state machine and starts over.
            handshakeState = HandshakeState::WAITING_FOR_DETAILS;
            std::uint32_t lastRequestMs = 0U;
            bool isRequestDue = true;

            while (handshakeState != HandshakeState::COMPLETE && handshakeState != HandshakeState::FAIL_RESTART)
            {
                const auto now = timeKeeper::getRealTime();

                if (isRequestDue || (now - lastRequestMs) > constants::runtime::BOOTSTRAP_REQUEST_INTERVAL_MS)
                {
                    // Ask first for topology, then for state. If a frame gets lost, the
                    // timer retries the current step until the callback advances the
                    // state machine.
                    if (handshakeState == HandshakeState::WAITING_FOR_DETAILS)
                    {
                        controllerSerialLink.sendJson(constants::payloads::StaticType::ASK_DETAILS);
                    }
                    else
                    {
                        controllerSerialLink.sendJson(constants::payloads::StaticType::ASK_STATE);
                    }

                    lastRequestMs = now;
                    isRequestDue = false;
                }

                if (serial->available())
                {
                    controllerSerialLink.processSerialBuffer();
                }

                yield();
            }
        }

        controllerSerialLink.onMessage(ControllerSerialLink::MessageCallback());
    }

    /**
     * @brief Handles the temporary bootstrap callback registered during
     * `bootstrapDevice()`.
     */
    void handleHandshakeMessage(constants::DeserializeExitCode code, const JsonDocument &receivedDocument)
    {
        (void)receivedDocument;

        if (code == DeserializeExitCode::OK_BOOT)
        {
            if (handshakeState == HandshakeState::WAITING_FOR_STATE)
            {
                handshakeState = HandshakeState::FAIL_RESTART;
            }
            return;
        }

        switch (handshakeState)
        {
        case HandshakeState::WAITING_FOR_DETAILS:
            if (code == DeserializeExitCode::OK_DETAILS)
            {
                if (controllerSerialLink.storeDetailsFromReceived() == DeserializeExitCode::OK_DETAILS)
                {
                    handshakeState = HandshakeState::WAITING_FOR_STATE;
                    // Request the authoritative actuator snapshot immediately
                    // once the topology is known. The outer loop still preserves
                    // the retry cadence, so this only removes the first avoidable delay.
                    controllerSerialLink.sendJson(constants::payloads::StaticType::ASK_STATE);
                }
                else
                {
                    handshakeState = HandshakeState::FAIL_RESTART;
                }
            }
            break;

        case HandshakeState::WAITING_FOR_STATE:
            if (code == DeserializeExitCode::OK_STATE)
            {
                if (controllerSerialLink.storeStateFromReceived() == DeserializeExitCode::OK_STATE)
                {
                    handshakeState = HandshakeState::COMPLETE;
                }
                else
                {
                    handshakeState = HandshakeState::FAIL_RESTART;
                }
            }
            break;

        default:
            break;
        }
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
     */
    [[nodiscard]] auto publishAuthoritativeLshState() -> bool
    {
        using namespace lsh::bridge::protocol;

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
     */
    [[nodiscard]] auto tryServeCachedStateRequest() -> bool
    {
        if (!canServeCachedStateRequests || !virtualDevice.isRuntimeSynchronized() || !controllerSerialLink.isConnected())
        {
            return false;
        }

        DPL("Serving REQUEST_STATE directly from the synchronized bridge cache.");
        return publishAuthoritativeLshState();
    }

    void publishAllHomieNodeStates()
    {
        for (const auto &node : homieNodes)
        {
            node.sendState();
        }
    }

    void publishChangedHomieNodeStates()
    {
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < virtualDevice.getTotalActuators(); ++actuatorIndex)
        {
            if (virtualDevice.isActuatorDirty(actuatorIndex))
            {
                homieNodes[actuatorIndex].sendState();
            }
        }
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

        if (virtualDevice.consumeFullStatePublishPending())
        {
            // Right after bootstrap or re-sync every node must publish once so Homie
            // definitely matches the fresh authoritative snapshot.
            publishAllHomieNodeStates();
        }
        else
        {
            // Normal steady state: only the actuators touched by the latest
            // authoritative frame need to be mirrored back to Homie.
            publishChangedHomieNodeStates();
        }

        virtualDevice.clearDirtyActuators();
        isAuthoritativeStateDirty = false;
        canServeCachedStateRequests = true;
    }

    void handleControllerSerialMessage(constants::DeserializeExitCode code, const JsonDocument &messageDocument)
    {
        switch (code)
        {
        case DeserializeExitCode::OK_BOOT:
            Homie.reboot();
            break;

        case DeserializeExitCode::OK_STATE:
            if (controllerSerialLink.storeStateFromReceived() == DeserializeExitCode::OK_STATE)
            {
                controllerSerialLink.reconcileDesiredActuatorStatesFromAuthoritative();
                markAuthoritativeStateDirty();
            }
            else
            {
                Homie.reboot();
            }
            break;

        case DeserializeExitCode::OK_DETAILS:
            // After bootstrap the bridge treats device topology as immutable.
            // Runtime DEVICE_DETAILS frames are ignored: configuration drift is
            // recovered through BOOT-driven bridge restarts, while transient
            // MQTT resyncs reuse the cached details snapshot directly.
            break;

        case DeserializeExitCode::OK_NETWORK_CLICK:
            if (Homie.isConnected())
            {
                (void)MqttPublisher::sendJson(messageDocument, MqttTopicsBuilder::mqttOutMiscTopic.c_str(), false, 2);
            }
            else
            {
                (void)controllerSerialLink.triggerFailoverFromReceivedClick();
            }
            break;

        case DeserializeExitCode::OK_OTHER_PAYLOAD:
            if (Homie.isConnected())
            {
                (void)MqttPublisher::sendJson(messageDocument, MqttTopicsBuilder::mqttOutMiscTopic.c_str(), false, 2);
            }
            break;

        default:
            break;
        }
    }

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

    [[nodiscard]] auto subscribeMqttTopics() -> bool
    {
        using constants::mqtt::MQTT_TOPIC_SERVICE;

        // Always unsubscribe first so repeated MQTT_READY events do not leave
        // duplicate subscriptions behind on reconnect.
        mqttClient->unsubscribe(MqttTopicsBuilder::mqttInTopic.c_str());
        mqttClient->unsubscribe(MQTT_TOPIC_SERVICE);

        const std::uint16_t deviceTopicPacketId = mqttClient->subscribe(MqttTopicsBuilder::mqttInTopic.c_str(), 2);
        if (deviceTopicPacketId == 0U)
        {
            return false;
        }

        const std::uint16_t serviceTopicPacketId = mqttClient->subscribe(MQTT_TOPIC_SERVICE, 1);
        if (serviceTopicPacketId == 0U)
        {
            mqttClient->unsubscribe(MqttTopicsBuilder::mqttInTopic.c_str());
            return false;
        }

        return true;
    }

    /**
     * @brief Handles MQTT commands that the bridge understands semantically.
     * @details Unsupported commands still fall back to raw forwarding so the
     *          runtime remains compatible with legacy or future controller
     * commands.
     */
    [[nodiscard]] auto forwardTypedControllerCommand(const JsonDocument &commandDocument) -> TypedCommandResult
    {
        using namespace lsh::bridge::protocol;

        const auto command = static_cast<Command>(commandDocument[KEY_PAYLOAD].as<std::uint8_t>());

        switch (command)
        {
        case Command::REQUEST_DETAILS:
            // DEVICE_DETAILS can be served from the cached validated topology. If
            // that cache is unexpectedly missing, the safest recovery is a reboot.
            if (!virtualDevice.hasCachedDetails())
            {
                Homie.reboot();
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
                requestAuthoritativeStateRefresh();
            }
            return TypedCommandResult::Handled;

        case Command::FAILOVER:
            controllerSerialLink.sendJson(constants::payloads::StaticType::GENERAL_FAILOVER);
            return TypedCommandResult::Handled;

        case Command::SET_SINGLE_ACTUATOR:
        {
            std::uint8_t actuatorId = 0U;
            bool requestedState = false;
            if (!tryGetUint8Scalar(commandDocument[KEY_ID], actuatorId) || !tryGetBinaryState(commandDocument[KEY_STATE], requestedState))
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
        case Command::NETWORK_CLICK_CONFIRM:
        {
            // Click payloads are tiny and structurally simple, so the bridge rebuilds
            // a compact serial document instead of forwarding unknown fields.
            StaticJsonDocument<constants::controllerSerial::MQTT_RECEIVED_DOC_MAX_SIZE> serialDoc;
            std::uint8_t clickType = 0U;
            std::uint8_t clickableId = 0U;
            std::uint8_t correlationId = 0U;

            if (!tryGetUint8Scalar(commandDocument[KEY_TYPE], clickType) || !tryGetUint8Scalar(commandDocument[KEY_ID], clickableId) ||
                !tryGetUint8Scalar(commandDocument[KEY_CORRELATION_ID], correlationId))
            {
                return TypedCommandResult::Invalid;
            }

            serialDoc[KEY_PAYLOAD] = static_cast<std::uint8_t>(command);
            serialDoc[KEY_TYPE] = clickType;
            serialDoc[KEY_ID] = clickableId;
            serialDoc[KEY_CORRELATION_ID] = correlationId;
            controllerSerialLink.sendJson(serialDoc);
            return TypedCommandResult::Handled;
        }

        default:
            return TypedCommandResult::Unsupported;
        }
    }

    /**
     * @brief Forwards a command to the controller, translating only when MQTT and
     * serial codecs differ.
     */
    void forwardOrTranslateToController(const char *payload, std::size_t payloadLength, const JsonDocument &commandDocument)
    {
#if (defined(CONFIG_MSG_PACK_MQTT) && defined(CONFIG_MSG_PACK_ARDUINO)) || \
    (!defined(CONFIG_MSG_PACK_MQTT) && !defined(CONFIG_MSG_PACK_ARDUINO))
        controllerSerialLink.sendJson(payload, payloadLength);
#else
        controllerSerialLink.sendJson(commandDocument);
#endif
    }

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
            (void)MqttPublisher::sendJson(constants::payloads::StaticType::PING_);
            return;

        case Command::SET_SINGLE_ACTUATOR:
        case Command::SET_STATE:
            // During re-sync the bridge cannot safely reinterpret actuator writes
            // against a stale cached model, so it forwards them verbatim instead.
            if (!virtualDevice.isRuntimeSynchronized())
            {
                forwardOrTranslateToController(payload, payloadLength, commandDocument);
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
            forwardOrTranslateToController(payload, payloadLength, commandDocument);
        }
    }

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
            (void)MqttPublisher::sendJson(constants::payloads::StaticType::PING_);
            return;

        default:
            return;
        }
    }

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
        if (!tryGetUint8Scalar(commandDocument[KEY_PAYLOAD], rawCommand))
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
     * @brief Drains queued MQTT commands only while the serial side is idle.
     */
    void processQueuedMqttCommands()
    {
        while (!serial->available())
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
        }
    }

    /**
     * @brief Receives complete MQTT frames from the AsyncMqttClient callback.
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

        if (total > constants::controllerSerial::RAW_MESSAGE_MAX_SIZE)
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
 * @brief Initialize serial, bootstrap the controller model and start Homie.
 */
void LSHBridge::begin()
{
    if (this->implementation->isStarted)
    {
        return;
    }

    // Open the USB serial console before any debug log can fire. Without this
    // explicit initialization a debug build compiles the logging calls but
    // still stays silent on the bridge USB port.
    DSB();

    activeInstance = this;
    timeKeeper::update();
    this->implementation->controllerSerialLink.begin();
    this->implementation->configureHomie();
    Homie.onEvent(onHomieEventStatic);
    this->implementation->bootstrapDevice();
    this->implementation->isControllerConnected = this->implementation->controllerSerialLink.isConnected();

    auto runtimeDelegate = ControllerSerialLink::MessageCallback::create<Impl, &Impl::handleControllerSerialMessage>(*this->implementation);
    this->implementation->controllerSerialLink.onMessage(runtimeDelegate);

    this->implementation->initializeNodesFromModel();
    Homie.setup();
    this->implementation->isStarted = true;

#ifdef HOMIE_RESET
    Homie.reset();
    Homie.setIdle(true);
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
    this->implementation->controllerSerialLink.processPendingActuatorBatch();
    this->implementation->processPendingStatePublishes();
    this->implementation->publishPendingBridgeDiagnostics();
    this->implementation->controllerSerialLink.sendJson(constants::payloads::StaticType::PING_);
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
