#include "communication/ardcom.hpp"

#include <cstddef>
#include <cstring>

#include <ArduinoJson.h>
#include <etl/bitset.h>

#include "constants/ardcom.hpp"
#include "constants/communicationprotocol.hpp"
#include "constants/configs/ping.hpp"
#include "constants/payloads.hpp"
#include "debug/debug.hpp"
#include "utils/payloads.hpp"
#include "utils/timekeeper.hpp"
#include "virtualdevice.hpp"

using namespace constants::payloads;
using constants::DeserializeExitCode;

namespace
{
    constexpr std::uint8_t BIT_MASK_8[8] = {
        0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x20U, 0x40U, 0x80U};

    auto tryGetUint8Scalar(const JsonVariantConst value, std::uint8_t &out) -> bool
    {
        if (value.isNull() || value.is<const char *>() || value.is<bool>() ||
            value.is<JsonArrayConst>() || value.is<JsonObjectConst>())
        {
            return false;
        }

        if (value.is<std::uint8_t>())
        {
            out = value.as<std::uint8_t>();
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

        out = coercedValue;
        return true;
    }

    auto validatePositiveUniqueIds(const JsonArrayConst &ids, std::size_t maxIds) -> bool
    {
        if (ids.size() > maxIds)
        {
            return false;
        }

        etl::bitset<256U> seenIds;
        for (const JsonVariantConst idVariant : ids)
        {
            std::uint8_t id = 0U;
            if (!tryGetUint8Scalar(idVariant, id))
            {
                return false;
            }
            if (id == 0U || seenIds.test(id))
            {
                return false;
            }
            seenIds.set(id);
        }

        return true;
    }

} // namespace

/**
 * @brief Send a static Json payload.
 *
 * @param payloadType type of the payload.
 */
void ArdCom::sendJson(constants::payloads::StaticType payloadType)
{
    using constants::payloads::StaticType;
    if (payloadType == StaticType::PING_ && !this->canPing())
    {
        return;
    }

// Determine which payload to get at compile time
#ifdef CONFIG_MSG_PACK_ARDUINO
    constexpr bool useMsgPack = true;
#else
    constexpr bool useMsgPack = false;
#endif

    const auto payloadToSend = utils::payloads::get<useMsgPack>(payloadType);

    if (!payloadToSend.empty())
    {
        this->serial->write(payloadToSend.data(), payloadToSend.size());
        this->updateLastSentTime();
    }
}

/**
 * @brief Sends a JsonDocument via Serial.
 *
 * @param json JsonDocument to be sent.
 */
void ArdCom::sendJson(const JsonDocument &json)
{
    DP_CONTEXT();
    DP("↑ Json to send: ");
    DPJ(json);
#ifdef CONFIG_MSG_PACK_ARDUINO
    serializeMsgPack(json, (*this->serial));
#else
    serializeJson(json, (*this->serial));
    // Append the newline delimiter ONLY for JSON-based communication.
    // MsgPack doesn't need it.
    this->serial->write("\n", 1);
#endif // CONFIG_MSG_PACK_ARDUINO
    this->updateLastSentTime();
}

/**
 * @brief Forwards a raw, null-terminated JSON string (Don't use for msgPack!).
 * @details This is a convenience overload. It uses `print()` which is suitable for C-style strings.
 *          In MessagePack mode this overload is intentionally rejected because a
 *          UTF-8 JSON string is not a valid raw MessagePack payload.
 * @param jsonString A constant pointer to a null-terminated C-style string.
 */
void ArdCom::sendJson(const char *jsonString)
{
    DP_CONTEXT();
    if (!jsonString)
        return;

#ifdef CONFIG_MSG_PACK_ARDUINO
    DPL("sendJson(const char*) is only valid in JSON serial mode.");
    return;
#else
    this->serial->print(jsonString);
    this->serial->write("\n", 1);
    this->updateLastSentTime();
#endif
}

/**
 * @brief Forwards a raw buffer of a known size using the active serial codec.
 * @details This is the most efficient method for forwarding payloads (like from MQTT)
 *          as it uses a single, non-iterative `write()` operation and doesn't
 *          rely on null-termination.
 * @param buffer A constant pointer to the start of the buffer.
 * @param length The number of bytes to write from the buffer.
 */
void ArdCom::sendJson(const char *buffer, size_t length)
{
    DP_CONTEXT();
    if (!buffer || length == 0)
        return;
#ifdef CONFIG_MSG_PACK_ARDUINO
    this->serial->write(reinterpret_cast<const uint8_t *>(buffer), length);
#else
    this->serial->write(reinterpret_cast<const uint8_t *>(buffer), length);
    // Append the newline delimiter ONLY for JSON-based communication.
    this->serial->write("\n", 1);
#endif
    this->updateLastSentTime();
}

void ArdCom::refreshPendingActuatorBatchStateLocked(bool restartSettleWindow)
{
    this->actuatorCommandBatchPending = (this->desiredDirtyActuators.count() > 0U);
    if (!this->actuatorCommandBatchPending)
    {
        this->lastDesiredActuatorUpdate_ms = 0U;
    }
    else if (restartSettleWindow)
    {
        this->lastDesiredActuatorUpdate_ms = timeKeeper::getRealTime();
    }
}

auto ArdCom::stageSingleActuatorCommand(uint8_t actuatorId, bool state) -> bool
{
    DP_CONTEXT();
    DPL("↑ ID: ", actuatorId, " | state: ", state);

    std::uint8_t actuatorIndex = 0U;
    if (!this->m_virtualDevice.tryGetActuatorIndex(actuatorId, actuatorIndex))
    {
        return false;
    }

    const bool authoritativeState = this->m_virtualDevice.getStateByIndex(actuatorIndex);

    portENTER_CRITICAL(&this->actuatorCommandMux);
    // If this exact actuator already has the same target value staged in the
    // currently pending batch, treat the repeat as a no-op so it doesn't keep
    // stretching the settle window for everyone else.
    if (this->actuatorCommandBatchPending &&
        this->desiredDirtyActuators.test(actuatorIndex) &&
        this->desiredActuatorStates[actuatorIndex] == state)
    {
        DPL("Ignoring duplicate pending actuator command for ID ", actuatorId);
        portEXIT_CRITICAL(&this->actuatorCommandMux);
        return true;
    }

    this->desiredActuatorStates[actuatorIndex] = state;
    if (state == authoritativeState)
    {
        this->desiredDirtyActuators.reset(actuatorIndex);
    }
    else
    {
        this->desiredDirtyActuators.set(actuatorIndex);
    }

    this->refreshPendingActuatorBatchStateLocked(true);
    ++this->desiredActuatorRevision;
    portEXIT_CRITICAL(&this->actuatorCommandMux);
    return true;
}

auto ArdCom::stageDesiredPackedState(const JsonArrayConst &packedBytes) -> bool
{
    const auto totalActuators = this->m_virtualDevice.getTotalActuators();
    const std::size_t expectedBytes = (static_cast<std::size_t>(totalActuators) + 7U) / 8U;
    if (packedBytes.isNull() || packedBytes.size() != expectedBytes)
    {
        return false;
    }

    if (totalActuators == 0U)
    {
        return true;
    }

    bool desiredSnapshot[constants::vDev::MAX_ACTUATORS]{};
    std::uint8_t actuatorIndex = 0U;

    for (JsonVariantConst packedByteVariant : packedBytes)
    {
        std::uint8_t packedByte = 0U;
        if (!tryGetUint8Scalar(packedByteVariant, packedByte))
        {
            return false;
        }

        for (std::uint8_t bitIndex = 0U; bitIndex < 8U && actuatorIndex < totalActuators; ++bitIndex)
        {
            desiredSnapshot[actuatorIndex] = (packedByte & BIT_MASK_8[bitIndex]) != 0U;
            ++actuatorIndex;
        }
    }

    portENTER_CRITICAL(&this->actuatorCommandMux);
    bool snapshotAlreadyPending = this->actuatorCommandBatchPending;
    if (snapshotAlreadyPending)
    {
        for (std::uint8_t index = 0U; index < totalActuators; ++index)
        {
            if (this->desiredActuatorStates[index] != desiredSnapshot[index])
            {
                snapshotAlreadyPending = false;
                break;
            }
        }
    }

    // Repeating the exact same full-state snapshot while it is already pending
    // should not restart the settle window.
    if (snapshotAlreadyPending)
    {
        DPL("Ignoring duplicate pending SET_STATE snapshot.");
        portEXIT_CRITICAL(&this->actuatorCommandMux);
        return true;
    }

    this->desiredDirtyActuators.reset();
    for (std::uint8_t index = 0U; index < totalActuators; ++index)
    {
        this->desiredActuatorStates[index] = desiredSnapshot[index];
        if (desiredSnapshot[index] != this->m_virtualDevice.getStateByIndex(index))
        {
            this->desiredDirtyActuators.set(index);
        }
    }
    this->refreshPendingActuatorBatchStateLocked(true);
    ++this->desiredActuatorRevision;
    portEXIT_CRITICAL(&this->actuatorCommandMux);

    return true;
}

void ArdCom::processPendingActuatorBatch()
{
    const auto now = timeKeeper::getRealTime();
    const auto totalActuators = this->m_virtualDevice.getTotalActuators();
    if (totalActuators == 0U)
    {
        return;
    }

    bool desiredSnapshot[constants::vDev::MAX_ACTUATORS]{};
    bool batchPending = false;
    std::uint32_t lastDesiredUpdate_ms = 0U;
    std::uint32_t desiredRevision = 0U;

    portENTER_CRITICAL(&this->actuatorCommandMux);
    for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
    {
        desiredSnapshot[actuatorIndex] = this->desiredActuatorStates[actuatorIndex];
    }
    batchPending = this->actuatorCommandBatchPending;
    lastDesiredUpdate_ms = this->lastDesiredActuatorUpdate_ms;
    desiredRevision = this->desiredActuatorRevision;
    portEXIT_CRITICAL(&this->actuatorCommandMux);

    if (!batchPending)
    {
        return;
    }

    if ((now - lastDesiredUpdate_ms) < ACTUATOR_COMMAND_SETTLE_INTERVAL_MS)
    {
        return;
    }

    bool differsFromAuthoritative = false;
    for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
    {
        if (desiredSnapshot[actuatorIndex] != this->m_virtualDevice.getStateByIndex(actuatorIndex))
        {
            differsFromAuthoritative = true;
            break;
        }
    }

    if (!differsFromAuthoritative)
    {
        portENTER_CRITICAL(&this->actuatorCommandMux);
        if (this->desiredActuatorRevision == desiredRevision)
        {
            this->desiredDirtyActuators.reset();
            this->refreshPendingActuatorBatchStateLocked();
            DPL("Dropping pending actuator batch: already matches authoritative state.");
        }
        portEXIT_CRITICAL(&this->actuatorCommandMux);
        return;
    }

    using namespace LSH::protocol;
    this->serializationDoc.clear();
    this->serializationDoc[KEY_PAYLOAD] = static_cast<uint8_t>(Command::SET_STATE);
    auto packedStates = this->serializationDoc.createNestedArray(KEY_STATE);

    std::uint8_t packedByte = 0U;
    for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
    {
        if (desiredSnapshot[actuatorIndex])
        {
            packedByte |= BIT_MASK_8[actuatorIndex & 0x07U];
        }

        const bool endOfPackedByte = ((actuatorIndex & 0x07U) == 0x07U);
        const bool lastActuator = actuatorIndex == (totalActuators - 1U);
        if (endOfPackedByte || lastActuator)
        {
            packedStates.add(packedByte);
            packedByte = 0U;
        }
    }

    this->sendJson(this->serializationDoc);

    portENTER_CRITICAL(&this->actuatorCommandMux);
    if (this->desiredActuatorRevision == desiredRevision)
    {
        this->desiredDirtyActuators.reset();
        this->refreshPendingActuatorBatchStateLocked();
    }
    portEXIT_CRITICAL(&this->actuatorCommandMux);
}

void ArdCom::clearPendingActuatorBatch()
{
    portENTER_CRITICAL(&this->actuatorCommandMux);
    for (std::uint8_t actuatorIndex = 0U; actuatorIndex < this->m_virtualDevice.getTotalActuators(); ++actuatorIndex)
    {
        const bool authoritativeState = this->m_virtualDevice.getStateByIndex(actuatorIndex);
        this->desiredActuatorStates[actuatorIndex] = authoritativeState;
    }
    this->desiredDirtyActuators.reset();
    this->refreshPendingActuatorBatchStateLocked();
    ++this->desiredActuatorRevision;
    portEXIT_CRITICAL(&this->actuatorCommandMux);
}

void ArdCom::reconcileDesiredActuatorStatesFromAuthoritative()
{
    const auto totalActuators = this->m_virtualDevice.getTotalActuators();

    portENTER_CRITICAL(&this->actuatorCommandMux);
    if (this->actuatorCommandBatchPending)
    {
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
        {
            const bool authoritativeState = this->m_virtualDevice.getStateByIndex(actuatorIndex);
            if (this->desiredDirtyActuators.test(actuatorIndex))
            {
                if (this->desiredActuatorStates[actuatorIndex] == authoritativeState)
                {
                    this->desiredDirtyActuators.reset(actuatorIndex);
                }
            }
            else
            {
                this->desiredActuatorStates[actuatorIndex] = authoritativeState;
            }
        }

        const bool wasBatchPending = this->actuatorCommandBatchPending;
        this->refreshPendingActuatorBatchStateLocked();
        if (wasBatchPending && !this->actuatorCommandBatchPending)
        {
            DPL("Cleared pending actuator batch during authoritative reconciliation.");
        }
    }
    else
    {
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
        {
            this->desiredActuatorStates[actuatorIndex] = this->m_virtualDevice.getStateByIndex(actuatorIndex);
        }
        this->desiredDirtyActuators.reset();
        this->refreshPendingActuatorBatchStateLocked();
        ++this->desiredActuatorRevision;
    }
    portEXIT_CRITICAL(&this->actuatorCommandMux);
}

/**
 * @brief Clean implementation for registering the callback.
 */
void ArdCom::onMessage(MessageCallback callback)
{
    this->messageCallback = callback;
}

/**
 * @brief Reads the serial buffer, processes all complete messages, and invokes a callback for each.
 * @details JSON transport uses newline-delimited text frames. MessagePack transport
 *          relies on ArduinoJson stream deserialization directly over the serial link.
 *          For each complete message the method deserializes the payload and invokes
 *          the registered callback delegate if it's valid.
 */
void ArdCom::processSerialBuffer()
{
#ifdef CONFIG_MSG_PACK_ARDUINO
    if (!this->serial->available())
    {
        return;
    }

    this->receivedDoc.clear();
    const DeserializationError err = deserializeMsgPack(this->receivedDoc, *this->serial);
    if (err != DeserializationError::Ok)
    {
        DPL("Fatal deserialization error: ", err.c_str());
        return;
    }

    this->updateLastReceivedTime();
    const auto messageType = this->deserialize();
    if (this->messageCallback.is_valid())
    {
        this->messageCallback(messageType, this->receivedDoc);
    }
    return;

#else
    while (this->serial->available())
    {
        char receivedChar = this->serial->read();

        if (this->discardInputUntilNewline)
        {
            if (receivedChar == '\n')
            {
                this->discardInputUntilNewline = false;
            }
            continue;
        }

        if (receivedChar == '\n')
        {
            if (this->rxBufferIndex == 0)
                continue;

            this->rxBuffer[this->rxBufferIndex] = '\0';
            this->receivedDoc.clear();
            DeserializationError err = deserializeJson(this->receivedDoc, this->rxBuffer, this->rxBufferIndex);
            this->rxBufferIndex = 0;

            if (err == DeserializationError::Ok)
            {
                this->updateLastReceivedTime();
                const auto messageType = this->deserialize();
                if (this->messageCallback.is_valid())
                {
                    // The receivedDoc is now populated. It uses zero-copy and points directly
                    // into rxBuffer. The messageCallback is responsible for either copying the data
                    // into a persistent storage (like VirtualDevice) or re-serializing it for forwarding.
                    // The rxBuffer will be reused for the next message, so its content is volatile
                    this->messageCallback(messageType, this->receivedDoc);
                }
            }
            else
            {
                DPL("Deserialization err: ", err.c_str(), " on message: ", this->rxBuffer);
            }
        }
        else if (receivedChar >= 32)
        {
            if (this->rxBufferIndex < sizeof(this->rxBuffer) - 1)
            {
                this->rxBuffer[this->rxBufferIndex++] = receivedChar;
            }
            else
            {
                // Ignore the rest of the oversized frame until its newline arrives,
                // otherwise the payload tail could be mistaken for a new command.
                this->rxBufferIndex = 0;
                this->discardInputUntilNewline = true;
                DPL("Buffer overflow during serial read! Message discarded.");
            }
        }
    }
#endif
}

/**
 * @brief Get the last received json document.
 *
 * @return const StaticJsonDocument<ardComConstants::JSON_RECEIVED_MAX_SIZE>& last received json document.
 */
auto ArdCom::getReceivedDoc() const -> const JsonDocument &
{
    return this->receivedDoc;
}

/**
 * @brief Deserializes the last received JSON document and identifies the payload type.
 * @details This method is state-agnostic. It only determines what kind of message was
 *          received and returns a corresponding exit code. The application loop is
 *          responsible for acting on this information.
 * @return A DeserializeExitCode representing the received message type.
 */
auto ArdCom::deserialize() -> constants::DeserializeExitCode
{
    DP_CONTEXT();
    using namespace LSH::protocol;

    const JsonVariantConst commandVariant = this->receivedDoc[KEY_PAYLOAD];
    if (commandVariant.isNull())
    {
        return DeserializeExitCode::ERR_MISSING_KEY_PAYLOAD;
    }

    std::uint8_t cmd = 0U;
    if (!tryGetUint8Scalar(commandVariant, cmd))
    {
        return DeserializeExitCode::ERR_UNKNOWN_PAYLOAD;
    }

    switch (static_cast<Command>(cmd))
    {
    case Command::DEVICE_DETAILS:
        return DeserializeExitCode::OK_DETAILS;
    case Command::ACTUATORS_STATE:
        return DeserializeExitCode::OK_STATE;
    case Command::NETWORK_CLICK_REQUEST:
    case Command::NETWORK_CLICK_CONFIRM:
        return DeserializeExitCode::OK_NETWORK_CLICK;
    case Command::BOOT:
        return DeserializeExitCode::OK_BOOT;
    case Command::PING_:
        return DeserializeExitCode::OK_PING;
    default:
        return DeserializeExitCode::OK_OTHER_PAYLOAD;
    }
}

/**
 * @brief Validates and stores received bitpacked state.
 * @details The state is sent as an array of bytes where each byte contains 8 actuator states.
 *          Byte 0, bit 0 = actuator 0; Byte 0, bit 7 = actuator 7; Byte 1, bit 0 = actuator 8, etc.
 * @return constants::DeserializeExitCode the result of sanity checks.
 */
auto ArdCom::storeStateFromReceived() const -> constants::DeserializeExitCode
{
    DP_CONTEXT();
    using namespace LSH::protocol;

    const JsonArrayConst packedBytes = this->receivedDoc[KEY_STATE].as<JsonArrayConst>();
    if (packedBytes.isNull())
    {
        return DeserializeExitCode::ERR_MISSING_KEY_STATE;
    }

    const std::size_t expectedBytes = (static_cast<std::size_t>(this->m_virtualDevice.getTotalActuators()) + 7U) / 8U;
    if (packedBytes.size() != expectedBytes)
    {
        return DeserializeExitCode::ERR_ACTUATORS_MISMATCH;
    }

    for (JsonVariantConst packedByte : packedBytes)
    {
        std::uint8_t validatedPackedByte = 0U;
        if (!tryGetUint8Scalar(packedByte, validatedPackedByte))
        {
            return DeserializeExitCode::ERR_STATE_VALUE_IMPLAUSIBLE;
        }
    }

    this->m_virtualDevice.setStateFromPackedBytes(packedBytes);

    return DeserializeExitCode::OK_STATE;
}

/**
 * @brief Validate and store the bridge-relevant subset of received details.
 * @details ArdCom extracts the JSON objects and delegates the model update to
 *          VirtualDevice. The bridge only persists what it needs after the
 *          handshake: name, actuator IDs and actuator state baseline. Button IDs
 *          are validated for shape/sanity but are not retained. The device name
 *          must be non-empty and fit the compiled `CONFIG_MAX_NAME_LENGTH`
 *          limit of the bridge build.
 * @return constants::DeserializeExitCode the result of validation and saving.
 */
auto ArdCom::storeDetailsFromReceived() const -> constants::DeserializeExitCode
{
    DP_CONTEXT();
    using namespace LSH::protocol;

    // Get device name from JSON
    const JsonVariantConst protocolMajor = this->receivedDoc[KEY_PROTOCOL_MAJOR];
    std::uint8_t validatedProtocolMajor = 0U;
    if (!tryGetUint8Scalar(protocolMajor, validatedProtocolMajor))
    {
        DPL("Error: Missing or invalid 'v' key in device details JSON.");
        return DeserializeExitCode::ERR_MISSING_KEY_PROTOCOL_MAJOR;
    }

    if (validatedProtocolMajor != WIRE_PROTOCOL_MAJOR)
    {
        DPL("Error: Protocol major mismatch. Received ", validatedProtocolMajor, ", expected ", WIRE_PROTOCOL_MAJOR, ".");
        return DeserializeExitCode::ERR_PROTOCOL_MAJOR_MISMATCH;
    }

    const char *const jsonDeviceName = this->receivedDoc[KEY_NAME];
    if (!jsonDeviceName)
    {
        return DeserializeExitCode::ERR_NO_NAME;
    }

    const std::size_t deviceNameLength = std::strlen(jsonDeviceName);
    if (deviceNameLength == 0U)
    {
        return DeserializeExitCode::ERR_NO_NAME;
    }
    if (deviceNameLength > constants::vDev::MAX_NAME_LENGTH)
    {
        DPL("Error: Device name is too long for this bridge build.");
        return DeserializeExitCode::ERR_NAME_TOO_LONG;
    }

    // Get actuators array from JSON
    const JsonArrayConst actuatorsIDs = this->receivedDoc[KEY_ACTUATORS_ARRAY];
    if (actuatorsIDs.isNull())
    {
        DPL("Error: Missing 'a' key in device details JSON.");
        return DeserializeExitCode::ERR_MISSING_KEY_ACTUATORS_IDS;
    }

    // Get buttons array from JSON
    const JsonArrayConst buttonsIDs = this->receivedDoc[KEY_BUTTONS_ARRAY];
    if (buttonsIDs.isNull())
    {
        DPL("Error: Missing 'b' key in device details JSON.");
        return DeserializeExitCode::ERR_MISSING_KEY_BUTTONS_IDS;
    }

    if (!validatePositiveUniqueIds(actuatorsIDs, constants::vDev::MAX_ACTUATORS))
    {
        DPL("Error: Invalid actuator IDs in device details JSON.");
        return DeserializeExitCode::ERR_ACTUATOR_ID_IMPLAUSIBLE;
    }

    if (!validatePositiveUniqueIds(buttonsIDs, constants::vDev::MAX_BUTTONS))
    {
        DPL("Error: Invalid button IDs in device details JSON.");
        return DeserializeExitCode::ERR_BUTTON_ID_IMPLAUSIBLE;
    }

    // Delegate the configuration to VirtualDevice.
    // The code is now cleaner, more robust, and respects separation of concerns.
    this->m_virtualDevice.setName(jsonDeviceName);
    this->m_virtualDevice.setActuatorsIds(actuatorsIDs);

    return DeserializeExitCode::OK_DETAILS;
}

/**
 * @brief Return if device can ping (minimum interval check).
 *
 * @return true if device can ping.
 * @return false if device can't ping.
 */
auto ArdCom::canPing() const -> bool
{
    // DP_CONTEXT(); // Bloats the serial
    using constants::ping::PING_INTERVAL_CONTROLLINO_MS;
    return (timeKeeper::getTime() - this->lastSentPayloadTime_ms > PING_INTERVAL_CONTROLLINO_MS);
}

/**
 * @brief Set last time payload has been sent to now.
 *
 */
void ArdCom::updateLastSentTime()
{
    // DP_CONTEXT(); pollutes serial
    this->lastSentPayloadTime_ms = timeKeeper::getTime();
}

/**
 * @brief Set last time payload has been received to now.
 *
 */
void ArdCom::updateLastReceivedTime()
{
    // DP_CONTEXT(); pollutes serial
    this->lastReceivedPayloadTime_ms = timeKeeper::getTime();
}

/**
 * @brief Creates and sends a "failover" command for a specific click.
 * @details This function is called when a network click cannot be forwarded to MQTT.
 *          It reuses the information (click type, button ID) from the last received
 *          click message (`receivedDoc`) to construct and send a specific failover
 *          command back to the Controllino.
 * @return The result code of the operation.
 */
auto ArdCom::triggerFailoverFromReceivedClick() -> constants::DeserializeExitCode
{
    DP_CONTEXT();
    using namespace LSH::protocol;

    if (!this->receivedDoc.containsKey(KEY_TYPE) || !this->receivedDoc.containsKey(KEY_ID) || !this->receivedDoc.containsKey(KEY_CORRELATION_ID))
    {
        DPL("Cannot create specific failover, sending general failover.");
        this->sendJson(StaticType::GENERAL_FAILOVER);
        return DeserializeExitCode::ERR_NOT_CONNECTED_GENERAL_FAILOVER_SENT;
    }

    JsonVariantConst jsonClickType = this->receivedDoc[KEY_TYPE];
    JsonVariantConst jsonButtonID = this->receivedDoc[KEY_ID];

    this->serializationDoc.clear();

    this->serializationDoc[KEY_PAYLOAD] = static_cast<uint8_t>(Command::FAILOVER_CLICK);
    this->serializationDoc[KEY_TYPE] = jsonClickType;
    this->serializationDoc[KEY_ID] = jsonButtonID;
    this->serializationDoc[KEY_CORRELATION_ID] = this->receivedDoc[KEY_CORRELATION_ID];

    this->sendJson(this->serializationDoc);
    return DeserializeExitCode::ERR_NOT_CONNECTED_FAILOVER_SENT;
}

/**
 * @brief Returns if ESP is connected or not to attached device, the assumption is based on last received message time.
 *
 * @return true if attached device is connected.
 * @return false if attached device isn't connected.
 */
auto ArdCom::isConnected() const -> bool
{
    DP_CONTEXT();
    using constants::ping::CONNECTION_TIMEOUT_CONTROLLINO_MS;
    return ((timeKeeper::getTime() - this->lastReceivedPayloadTime_ms) < CONNECTION_TIMEOUT_CONTROLLINO_MS);
}
