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
#include <etl/bitset.h>
#include <etl/delegate.h>

#include "constants/controller_serial.hpp"
#include "constants/button_click_types.hpp"
#include "constants/configs/controller_serial.hpp"
#include "constants/configs/runtime.hpp"
#include "constants/configs/virtual_device.hpp"
#include "constants/deserialize_exit_codes.hpp"
#include "constants/payloads.hpp"

class VirtualDevice;  //!< FORWARD DECLARATION

/**
 * @brief Handles serial communication with the attached controller.
 * @details The active codec is selected at build time: JSON uses newline-delimited
 *          text frames, while MsgPack is exchanged as raw payload bytes.
 */
class ControllerSerialLink
{
public:
    /**
     * @brief Callback invoked after a payload has been decoded.
     *
     */
    using MessageCallback = etl::delegate<void(constants::DeserializeExitCode, const JsonDocument &)>;

    /**
     * @brief Describes one actuator batch dropped because the incoming intent
     *        never became stable enough to send safely.
     */
    struct ActuatorStormDiagnostic
    {
        std::uint32_t pendingDurationMs = 0U;  //!< How long the batch stayed open before the bridge aborted it.
        std::uint16_t mutationCount = 0U;      //!< How many accepted command changes were merged into that unstable batch.
    };

private:
    std::uint32_t lastSentPayloadTime_ms = 0U;      //!< Last time a payload has been sent to Controllino
    std::uint32_t lastReceivedPayloadTime_ms = 0U;  //!< Last time a valid payload has been received from Controllino
    HardwareSerial *const serial;                   //!< The serial port used to communicate with controllino
    VirtualDevice &virtualDevice;                   //!< Device details holder to store received details
    StaticJsonDocument<constants::controllerSerial::JSON_RECEIVED_MAX_SIZE> receivedDoc;  //!< Last received json document
    StaticJsonDocument<constants::controllerSerial::MQTT_RECEIVED_DOC_MAX_SIZE>
        serializationDoc;  //!< Reusable doc for bridge-generated commands and coalesced SET_STATE batches.
#ifndef CONFIG_MSG_PACK_ARDUINO
    char rxBuffer
        [constants::controllerSerial::RAW_MESSAGE_MAX_SIZE];  //!< Temporary buffer used to assemble one newline-terminated JSON frame
    std::uint16_t rxBufferIndex = 0U;                         //!< Current write index inside rxBuffer
    bool discardInputUntilNewline = false;                    //!< True while draining an oversized JSON frame until its terminating newline
#endif
    // This block stores "what the bridge would like the controller state to become".
    // It is intentionally separate from VirtualDevice, which stores only the last
    // controller-confirmed authoritative state.
    bool desiredActuatorStates
        [constants::virtualDevice::MAX_ACTUATORS]{};  //!< Desired-state shadow used only to build the next outbound SET_STATE batch.
    etl::bitset<constants::virtualDevice::MAX_ACTUATORS>
        desiredDirtyActuators{};  //!< Outbound per-actuator mask for remote intent not yet confirmed by an authoritative controller state frame.
    bool actuatorCommandBatchPending = false;  //!< True while a coalesced outbound SET_STATE is still waiting for its quiet window.
    std::uint32_t firstDesiredActuatorUpdate_ms =
        0U;  //!< Time of the first accepted change in the current batch. Used to detect command storms that keep the quiet window open forever.
    std::uint32_t lastDesiredActuatorUpdate_ms = 0U;  //!< Last time the desired-state shadow changed.
    std::uint16_t pendingActuatorMutationCount = 0U;  //!< Number of accepted non-duplicate changes merged into the current batch.
    bool hasPendingActuatorStormDiagnostic =
        false;  //!< True while a dropped unstable batch is waiting to be reported to the outer bridge runtime.
    ActuatorStormDiagnostic
        pendingActuatorStormDiagnostic{};              //!< Details about the last unstable batch dropped by the bridge safety valve.
    StaticSemaphore_t actuatorCommandMutexStorage{};   //!< Storage owned by the bridge-side desired-state mutex.
    SemaphoreHandle_t actuatorCommandMutex = nullptr;  //!< Blocking mutex that protects the desired-state shadow across short serial sends.
    MessageCallback messageCallback;                   //!< Registered callback invoked for each decoded payload

    void lockActuatorCommandState()
    {
        // A normal mutex is used here, not a spinlock, because the coalescing path
        // may keep the lock while it writes one compact SET_STATE frame to serial.
        (void)xSemaphoreTake(this->actuatorCommandMutex, portMAX_DELAY);
    }

    void unlockActuatorCommandState()
    {
        (void)xSemaphoreGive(this->actuatorCommandMutex);
    }

    // Process
    [[nodiscard]] auto deserialize() -> constants::DeserializeExitCode;  // Deserialize a json
    [[nodiscard]] auto canPing() const -> bool;                          // Return if the bridge can send a ping now

    void updateLastSentTime();      // Set last time payload has been sent to now
    void updateLastReceivedTime();  // Set last time payload has been received to now
    void
    realignDesiredStateShadowWithAuthoritativeLocked();  // Copy the last controller-confirmed state into the desired-state shadow. Must be called with actuatorCommandMutex held.
    void
    clearPendingActuatorBatchLocked();  // Forget every pending desired-state change and reset batch bookkeeping. Must be called with actuatorCommandMutex held.
    void refreshPendingActuatorBatchStateLocked(
        bool restartSettleWindow = false,
        bool markNewMutation =
            false);  // Recomputes pending/timestamp/counter invariants from dirty bits. Must be called with actuatorCommandMutex held.

public:
    /**
     * @brief Construct a new ControllerSerialLink object.
     *
     * @param hardwareSerial serial port used to talk with the controller.
     * @param device cached bridge-side device model that receives validated details and state.
     */
    ControllerSerialLink(HardwareSerial *const hardwareSerial, VirtualDevice &device) noexcept :
        serial(hardwareSerial), virtualDevice(device)
    {
        this->actuatorCommandMutex = xSemaphoreCreateMutexStatic(&this->actuatorCommandMutexStorage);
    }

    /**
     * @brief Initialize serial communication with the attached controller.
     *
     */
    void begin() noexcept
    {
        this->serial->begin(constants::controllerSerial::ARDCOM_SERIAL_BAUD, SERIAL_8N1, constants::controllerSerial::ARDCOM_SERIAL_RX_PIN,
                            constants::controllerSerial::ARDCOM_SERIAL_TX_PIN, false, 5U);
        this->serial->setTimeout(constants::controllerSerial::ARDCOM_SERIAL_TIMEOUT_MS);
    }

    // --- Main I/O Methods ---

    void processSerialBuffer();  // Main serial processing function

    void onMessage(MessageCallback callback);  // Register payload callback

    [[nodiscard]] auto storeDetailsFromReceived() const
        -> constants::DeserializeExitCode;  // Validate and store last received details payload

    [[nodiscard]] auto storeStateFromReceived() const
        -> constants::DeserializeExitCode;  // Validate and store last received packed state payload

    // --- Public Send Methods ---

    void sendJson(constants::payloads::StaticType payloadType);  // Send a static JSON payload

    void sendJson(const JsonDocument &json);  // Send a JsonDocument using the current serial codec

    /**
     * @brief Convenience JSON-text overload kept for callers with a
     *        null-terminated literal.
     */
    void sendJson(const char *jsonString);

    void sendJson(const char *buffer, std::size_t length);  // Forward a raw payload buffer using the active serial codec

    [[nodiscard]] auto stageSingleActuatorCommand(std::uint8_t actuatorId, bool state) -> bool;  // Stage a single desired actuator state

    [[nodiscard]] auto stageDesiredPackedState(const JsonArrayConst &packedBytes) -> bool;  // Stage a packed desired-state snapshot

    void processPendingActuatorBatch();  // Emit one coalesced SET_STATE after the quiet window

    void clearPendingActuatorBatch();  // Drop pending desired-state updates and realign the shadow

    void reconcileDesiredActuatorStatesFromAuthoritative();  // Reconcile desired-state shadow with authoritative state

    [[nodiscard]] auto peekPendingActuatorStormDiagnostic(ActuatorStormDiagnostic &outDiagnostic)
        -> bool;  // Copy the last dropped unstable-batch diagnostic without clearing it

    void clearPendingActuatorStormDiagnostic();  // Clear the last dropped unstable-batch diagnostic after the outer bridge has reported it

    [[nodiscard]] auto triggerFailoverFromReceivedClick()
        -> constants::DeserializeExitCode;  // Create and send a failover from the last received click payload

    // --- Getters ---

    [[nodiscard]] auto getReceivedDoc() const -> const JsonDocument &;  // Get the last received JsonDocument

    // --- Utility Methods ---

    [[nodiscard]] auto isConnected() const -> bool;  // Return if attached device is connected
};

#endif  // LSH_BRIDGE_COMMUNICATION_CONTROLLER_SERIAL_LINK_HPP
