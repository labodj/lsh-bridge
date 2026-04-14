#ifndef LSHESP_COMMUNICATION_ARDCOM_HPP
#define LSHESP_COMMUNICATION_ARDCOM_HPP

#include <cstdint>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <etl/bitset.h>
#include <etl/delegate.h>

#include "constants/ardcom.hpp"
#include "constants/buttonclicktypes.hpp"
#include "constants/configs/ardcom.hpp"
#include "constants/configs/vdev.hpp"
#include "constants/deserializeexitcodes.hpp"
#include "constants/payloads.hpp"

class VirtualDevice; //!< FORWARD DECLARATION

/**
 * @brief Handles serial communication with the attached controller.
 * @details The active codec is selected at build time: JSON uses newline-delimited
 *          text frames, while MsgPack is exchanged as raw payload bytes.
 */

class ArdCom
{
public:
    // Callback type definition
    using MessageCallback = etl::delegate<void(constants::DeserializeExitCode, const JsonDocument &)>;

private:
    static constexpr std::uint16_t ACTUATOR_COMMAND_SETTLE_INTERVAL_MS = 50U; //!< Quiet window used to coalesce bursts of actuator commands into one SET_STATE.

    std::uint32_t lastSentPayloadTime_ms = 0U;                                 //!< Last time a payload has been sent to Controllino
    std::uint32_t lastReceivedPayloadTime_ms = 0U;                             //!< Last time a valid payload has been received from Controllino
    HardwareSerial *const serial;                                              //!< The serial port used to communicate with controllino
    VirtualDevice &m_virtualDevice;                                            //!< Device details holder to store received details
    StaticJsonDocument<constants::ardCom::JSON_RECEIVED_MAX_SIZE> receivedDoc; //!< Last received json document
    StaticJsonDocument<constants::ardCom::MQTT_RECEIVED_DOC_MAX_SIZE> serializationDoc; //!< Reusable doc for bridge-generated commands and coalesced SET_STATE batches.
#ifndef CONFIG_MSG_PACK_ARDUINO
    char rxBuffer[constants::ardCom::RAW_MESSAGE_MAX_SIZE];
    std::uint16_t rxBufferIndex = 0;
    bool discardInputUntilNewline = false;
#endif
    bool desiredActuatorStates[constants::vDev::MAX_ACTUATORS]{}; //!< Desired-state shadow used only to build the next outbound SET_STATE batch.
    etl::bitset<constants::vDev::MAX_ACTUATORS> desiredDirtyActuators{}; //!< Outbound per-actuator mask for remote intent not yet confirmed by an authoritative controller state frame.
    bool actuatorCommandBatchPending = false;                     //!< True while a coalesced outbound SET_STATE is still waiting for its quiet window.
    std::uint32_t lastDesiredActuatorUpdate_ms = 0U;             //!< Last time the desired-state shadow changed.
    std::uint32_t desiredActuatorRevision = 0U; //!< Monotonic token to detect commands arriving while a coalesced batch is being built/sent.
    portMUX_TYPE actuatorCommandMux = portMUX_INITIALIZER_UNLOCKED;
    MessageCallback messageCallback; // Member to hold the callback function.

    // Process
    auto deserialize() -> constants::DeserializeExitCode; // Deserialize a json

    void updateLastSentTime();     // Set last time payload has been sent to now
    void updateLastReceivedTime(); // Set last time payload has been received to now
    void refreshPendingActuatorBatchStateLocked(bool restartSettleWindow = false); // Recomputes pending/timestamp invariants from dirty bits. Must be called with actuatorCommandMux held.

public:
    /**
     * @brief Construct a new Ard Com object.
     *
     * @param hardwareSerial the hardware serial to which arduino is connected to.
     * @param device device details holder object, to store received details.
     */
    ArdCom(HardwareSerial *const hardwareSerial,
           VirtualDevice &device) noexcept : serial(hardwareSerial),
                                             m_virtualDevice(device)
    {
    }

    void begin() noexcept
    {
        this->serial->begin(constants::ardCom::ARDCOM_SERIAL_BAUD, SERIAL_8N1,
                            constants::ardCom::ARDCOM_SERIAL_RX_PIN,
                            constants::ardCom::ARDCOM_SERIAL_TX_PIN,
                            false, 5U);
        this->serial->setTimeout(constants::ardCom::ARDCOM_SERIAL_TIMEOUT_MS);
    }

    // --- Main I/O Methods ---
    void processSerialBuffer();               // The main processing function
    void onMessage(MessageCallback callback); // Method to allow external code (like main) to register a handler
    auto storeDetailsFromReceived() const -> constants::DeserializeExitCode;
    auto storeStateFromReceived() const -> constants::DeserializeExitCode;

    // --- Public Send Methods ---
    void sendJson(constants::payloads::StaticType payloadType);                // Send a static JSON
    void sendJson(const JsonDocument &json);                                   // Send a JSON
    void sendJson(const char *jsonString);                                     // Send a raw, null-terminated JSON string (JSON mode only)
    void sendJson(const char *buffer, size_t length);                          // Forwards a raw payload using the active serial codec
    [[nodiscard]] auto stageSingleActuatorCommand(uint8_t actuatorId, bool state) -> bool; // Updates the desired-state shadow for one actuator and arms the batch window.
    [[nodiscard]] auto stageDesiredPackedState(const JsonArrayConst &packedBytes) -> bool;  // Replaces the desired-state shadow from a packed SET_STATE payload and arms the batch window.
    void processPendingActuatorBatch();                                        // Emits one coalesced SET_STATE after a short quiet window.
    void clearPendingActuatorBatch();                                          // Drops pending desired-state updates and re-aligns the shadow with the authoritative state.
    void reconcileDesiredActuatorStatesFromAuthoritative();                    // Merges the authoritative controller state back into the outbound desired shadow.
    auto triggerFailoverFromReceivedClick() -> constants::DeserializeExitCode; // Creates and sends a "failover" command based on the last received click message.

    // --- Getters ---
    auto getReceivedDoc() const -> const JsonDocument &;

    // --- Utility Methods ---
    [[nodiscard]] auto isConnected() const -> bool; // Returns if controllino is connected or not
    [[nodiscard]] auto canPing() const -> bool;     // Returns if the device can send ping or not
};

#endif // LSHESP_COMMUNICATION_ARDCOM_HPP
