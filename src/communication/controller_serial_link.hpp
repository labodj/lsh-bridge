/**
 * @file    controller_serial_link.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the bridge serial transport adapter used to talk with the controller.
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

#ifndef LSH_BRIDGE_COMMUNICATION_CONTROLLER_SERIAL_LINK_HPP
#define LSH_BRIDGE_COMMUNICATION_CONTROLLER_SERIAL_LINK_HPP

#include <cstddef>
#include <cstdint>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/semphr.h>
#include <etl/delegate.h>

#include "actuator_state_mask.hpp"
#include "constants/communication_protocol.hpp"
#include "constants/controller_serial.hpp"
#include "constants/configs/controller_serial.hpp"
#include "constants/configs/runtime.hpp"
#include "constants/configs/virtual_device.hpp"
#include "constants/deserialize_exit_codes.hpp"
#include "constants/payloads.hpp"
#include "communication/msgpack_serial_framing.hpp"

struct DeviceDetailsSnapshot;
class VirtualDevice;

/**
 * @brief Handles serial communication with the attached controller.
 */
class ControllerSerialLink
{
public:
    /** @brief Callback invoked after one payload has been decoded. */
    using MessageCallback = etl::delegate<void(constants::DeserializeExitCode, const JsonDocument &)>;

    /**
     * @brief Describes one actuator batch dropped because the incoming intent never became stable enough to send safely.
     */
    struct ActuatorStormDiagnostic
    {
        std::uint32_t pendingDurationMs = 0U;  //!< How long the batch stayed open before the bridge aborted it.
        std::uint16_t mutationCount = 0U;      //!< How many accepted command changes were merged into that unstable batch.
    };

    /**
     * @brief Classify Homie `/set` commands consumed but not staged for the controller.
     */
    enum class HomieRejectedCommandReason : std::uint8_t
    {
        RuntimeDesynchronized,  //!< The bridge was waiting for a fresh authoritative controller state.
        InvalidPayload,         //!< Homie delivered a value other than the accepted boolean literals.
        StageFailed             //!< The node command was valid but could not be staged against the cached topology.
    };

    /**
     * @brief Compact desired-state shadow for the next outbound `SET_STATE` batch.
     * @details Bit positions follow the same dense runtime order used by the
     *          controller protocol and by `VirtualDevice::actuatorsState`.
     */
    using DesiredActuatorStateBitset = ActuatorStateMask;

private:
    HardwareSerial *const serial;                   //!< The serial port used to communicate with the controller.
    VirtualDevice &virtualDevice;                   //!< Device-details holder that stores received details and state.
    std::uint32_t lastSentPayloadTime_ms = 0U;      //!< Last time a payload has been sent to the controller.
    std::uint32_t lastReceivedPayloadTime_ms = 0U;  //!< Last time a valid payload has been received from the controller.
    std::uint32_t firstDesiredActuatorUpdate_ms =
        0U;  //!< Time of the first accepted change in the current batch. Used to detect command storms that keep the quiet window open forever.
    std::uint32_t lastDesiredActuatorUpdate_ms = 0U;  //!< Last time the desired-state shadow changed.
    // This block stores "what the bridge would like the controller state to become".
    // It is intentionally separate from VirtualDevice, which stores only the last
    // controller-confirmed authoritative state.
    DesiredActuatorStateBitset desiredActuatorStates{};  //!< Desired-state shadow used only to build the next outbound `SET_STATE` batch.
    ActuatorStateMask desiredDirtyActuators{};  //!< Outbound per-actuator mask for remote intent not yet confirmed by an authoritative controller state frame.
    std::uint16_t pendingActuatorMutationCount = 0U;   //!< Number of accepted non-duplicate changes merged into the current batch.
    std::uint16_t actuatorCommandBatchRevision = 0U;   //!< Monotonic guard used to avoid committing stale snapshots after serial TX.
    SemaphoreHandle_t actuatorCommandMutex = nullptr;  //!< Blocking mutex that protects the desired-state shadow across short serial sends.
    bool hasSeenControllerTraffic = false;     //!< True after the bridge has decoded at least one valid controller frame in this boot.
    bool actuatorCommandBatchPending = false;  //!< True while a coalesced outbound `SET_STATE` is still waiting for its quiet window.
    bool hasPendingActuatorStormDiagnostic =
        false;                        //!< True while a dropped unstable batch is waiting to be reported to the outer bridge runtime.
    MessageCallback messageCallback;  //!< Registered callback invoked for each decoded payload.
    ActuatorStormDiagnostic
        pendingActuatorStormDiagnostic{};                //!< Details about the last unstable batch dropped by the bridge safety valve.
    std::uint16_t rejectedHomieDesyncCommandCount = 0U;  //!< Homie writes dropped while the runtime model was stale.
    std::uint16_t rejectedHomieInvalidPayloadCommandCount = 0U;  //!< Homie writes dropped because the value was not a boolean literal.
    std::uint16_t rejectedHomieStageFailedCommandCount = 0U;     //!< Homie writes dropped because staging rejected the command.

    // Cold transport scratch space below this point.
    StaticJsonDocument<constants::controllerSerial::JSON_RECEIVED_MAX_SIZE> receivedDoc;  //!< Last received JSON document.
    StaticJsonDocument<constants::controllerSerial::MQTT_RECEIVED_DOC_MAX_SIZE>
        serializationDoc;  //!< Reusable doc for bridge-generated commands and coalesced `SET_STATE` batches.
#ifdef CONFIG_MSG_PACK_ARDUINO
    char
        rxBuffer[constants::controllerSerial::SERIAL_RX_BUFFER_SIZE];  //!< Temporary buffer used for one complete deframed MsgPack payload.
    lsh::bridge::transport::MsgPackFrameReceiver msgPackFrameReceiver;  //!< Incremental deframer used by the serial MsgPack transport.
#else
    char rxBuffer
        [constants::controllerSerial::SERIAL_RX_BUFFER_SIZE];  //!< Temporary buffer used to assemble one newline-terminated JSON frame.
    std::uint16_t rxBufferIndex = 0U;                          //!< Current write index inside `rxBuffer`.
    bool discardInputUntilNewline = false;  //!< True while draining an oversized JSON frame until its terminating newline.
#endif
    StaticSemaphore_t actuatorCommandMutexStorage{};  //!< Storage owned by the bridge-side desired-state mutex.

    /** @brief Lock the desired-state shadow while the bridge updates or serializes it. */
    void lockActuatorCommandState()
    {
        // A normal mutex is used here, not a spinlock, because MQTT/Homie producers
        // may run from a different task than the main bridge loop.
        (void)xSemaphoreTake(this->actuatorCommandMutex, portMAX_DELAY);
    }

    /** @brief Unlock the desired-state shadow after a short critical section. */
    void unlockActuatorCommandState()
    {
        (void)xSemaphoreGive(this->actuatorCommandMutex);
    }

    [[nodiscard]] auto deserialize() -> constants::DeserializeExitCode;
    [[nodiscard]] auto canPing() const -> bool;

    void updateLastSentTime();
    void updateLastReceivedTime();
#ifdef CONFIG_MSG_PACK_ARDUINO
    auto sendFramedMsgPackBuffer(const std::uint8_t *buffer, std::size_t length) -> bool;
#endif
    void realignDesiredStateShadowWithAuthoritativeLocked();
    void clearPendingActuatorBatchLocked();
    void refreshPendingActuatorBatchStateLocked(bool restartSettleWindow = false, bool markNewMutation = false);
    void bumpActuatorCommandBatchRevisionLocked() noexcept;
    [[nodiscard]] auto sendSetStateCommand(const DesiredActuatorStateBitset &desiredSnapshot, std::uint8_t totalActuators) -> bool;

public:
    /** @brief Construct the controller serial transport around one UART and one cached device model. */
    ControllerSerialLink(HardwareSerial *const hardwareSerial, VirtualDevice &device) noexcept :
        serial(hardwareSerial), virtualDevice(device)
#ifdef CONFIG_MSG_PACK_ARDUINO
        ,
        msgPackFrameReceiver(rxBuffer, static_cast<std::uint16_t>(sizeof(rxBuffer)))
#endif
    {
        this->actuatorCommandMutex = xSemaphoreCreateMutexStatic(&this->actuatorCommandMutexStorage);
    }

    /** @brief Initialize the UART used to talk with the controller. */
    void begin() noexcept
    {
        this->serial->begin(constants::controllerSerial::ARDCOM_SERIAL_BAUD, SERIAL_8N1, constants::controllerSerial::ARDCOM_SERIAL_RX_PIN,
                            constants::controllerSerial::ARDCOM_SERIAL_TX_PIN, false,
                            constants::controllerSerial::ARDCOM_SERIAL_TIMEOUT_MS);
    }

    void processSerialBuffer();

    void onMessage(MessageCallback callback);

    [[nodiscard]] auto parseDetailsFromReceived(DeviceDetailsSnapshot &outDetails) const -> constants::DeserializeExitCode;

    [[nodiscard]] auto storeStateFromReceived() const -> constants::DeserializeExitCode;

    [[nodiscard]] auto sendJson(constants::payloads::StaticType payloadType) -> bool;

    [[nodiscard]] auto sendJson(const JsonDocument &json) -> bool;

    [[nodiscard]] auto sendJson(const char *buffer, std::size_t length) -> bool;

    [[nodiscard]] auto stageSingleActuatorCommand(std::uint8_t actuatorId, bool state) -> bool;

    [[nodiscard]] auto stageDesiredPackedState(const JsonArrayConst &packedBytes) -> bool;
    [[nodiscard]] auto stageDesiredPackedStateBytes(const std::uint8_t *packedBytes, std::uint8_t packedByteCount) -> bool;

    void processPendingActuatorBatch();

    void clearPendingActuatorBatch();

    void reconcileDesiredActuatorStatesFromAuthoritative();

    [[nodiscard]] auto peekPendingActuatorStormDiagnostic(ActuatorStormDiagnostic &outDiagnostic) -> bool;

    void clearPendingActuatorStormDiagnostic();

    void recordRejectedHomieCommand(HomieRejectedCommandReason reason);

    void clearRejectedHomieCounters();

    void snapshotRejectedHomieCounters(std::uint16_t &outRejectedDesyncCommands,
                                       std::uint16_t &outRejectedInvalidPayloadCommands,
                                       std::uint16_t &outRejectedStageFailedCommands);

    void consumeRejectedHomieCounters(std::uint16_t rejectedDesyncCommands,
                                      std::uint16_t rejectedInvalidPayloadCommands,
                                      std::uint16_t rejectedStageFailedCommands);

    [[nodiscard]] auto triggerFailoverFromReceivedClick() -> constants::DeserializeExitCode;
    [[nodiscard]] auto sendClickCommand(lsh::bridge::protocol::Command command,
                                        std::uint8_t clickType,
                                        std::uint8_t clickableId,
                                        std::uint8_t correlationId) -> bool;

    [[nodiscard]] auto isConnected() const -> bool;
};

#endif  // LSH_BRIDGE_COMMUNICATION_CONTROLLER_SERIAL_LINK_HPP
