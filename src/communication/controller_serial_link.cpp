/**
 * @file    controller_serial_link.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the serial transport layer between the bridge and the
 * controller.
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

#include "communication/controller_serial_link.hpp"

#include <cstddef>
#include <cstring>

#include <ArduinoJson.h>
#include <etl/bitset.h>

#include "constants/controller_serial.hpp"
#include "constants/communication_protocol.hpp"
#include "constants/configs/ping.hpp"
#include "constants/payloads.hpp"
#include "debug/debug.hpp"
#include "utils/json_scalars.hpp"
#include "utils/payloads.hpp"
#include "utils/time_keeper.hpp"
#include "virtual_device.hpp"

using namespace constants::payloads;
using constants::DeserializeExitCode;
#ifdef CONFIG_MSG_PACK_ARDUINO
using lsh::bridge::transport::MsgPackFrameConsumeResult;
using lsh::bridge::transport::MsgPackFrameWriter;
#endif

namespace
{
/**
 * @brief Validated view of the fields that make up one `DEVICE_DETAILS` payload.
 */
struct ValidatedDetailsPayload
{
    const char *deviceName = nullptr;
    JsonArrayConst actuatorIds;
    JsonArrayConst buttonIds;
};

/**
 * @brief Validated scalar view of one controller-side network click payload.
 */
struct ValidatedNetworkClickPayload
{
    std::uint8_t clickType = 0U;
    std::uint8_t clickableId = 0U;
    std::uint8_t correlationId = 0U;
};

/**
 * @brief Lookup table used to unpack one packed state byte into 8 boolean actuator states.
 * @details Keeping the masks in a table makes the unpacking loop easy to read and
 *          avoids repeated shift operations on smaller microcontrollers.
 */
constexpr std::uint8_t kPackedBitMask[8] = {0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x20U, 0x40U, 0x80U};

/**
 * @brief Validate that one JSON ID array contains only unique positive IDs.
 *
 * @param idList JSON array to validate.
 * @param maxAllowedIds maximum number of elements accepted by the current build.
 * @return true if `idList` contains only unique IDs in the range `[1, 255]`.
 * @return false if the array is too large or contains invalid/duplicate IDs.
 */
auto validatePositiveUniqueIds(const JsonArrayConst &idList, std::size_t maxAllowedIds) -> bool
{
    if (idList.size() > maxAllowedIds)
    {
        return false;
    }

    etl::bitset<256U> seenIds;
    for (const JsonVariantConst idVariant : idList)
    {
        std::uint8_t validatedId = 0U;
        if (!utils::json::tryGetUint8Scalar(idVariant, validatedId))
        {
            return false;
        }
        if (validatedId == 0U || seenIds.test(validatedId))
        {
            return false;
        }
        seenIds.set(validatedId);
    }

    return true;
}

/**
 * @brief Validate the last received `DEVICE_DETAILS` payload without mutating the runtime model.
 *
 * @param receivedDoc decoded controller frame to validate.
 * @param outPayload destination view populated only on success.
 * @return DeserializeExitCode::OK_DETAILS if the payload is structurally valid.
 * @return any other `DeserializeExitCode` if a required field is missing or invalid.
 */
auto validateReceivedDetailsPayload(const JsonDocument &receivedDoc, ValidatedDetailsPayload &outPayload) -> DeserializeExitCode
{
    using namespace lsh::bridge::protocol;

    const JsonVariantConst protocolMajor = receivedDoc[KEY_PROTOCOL_MAJOR];
    std::uint8_t validatedProtocolMajor = 0U;
    if (!utils::json::tryGetUint8Scalar(protocolMajor, validatedProtocolMajor))
    {
        DPL("Error: Missing or invalid 'v' key in device details JSON.");
        return DeserializeExitCode::ERR_MISSING_KEY_PROTOCOL_MAJOR;
    }

    if (validatedProtocolMajor != WIRE_PROTOCOL_MAJOR)
    {
        DPL("Error: Protocol major mismatch. Received ", validatedProtocolMajor, ", expected ", WIRE_PROTOCOL_MAJOR, ".");
        return DeserializeExitCode::ERR_PROTOCOL_MAJOR_MISMATCH;
    }

    outPayload.deviceName = receivedDoc[KEY_NAME];
    if (outPayload.deviceName == nullptr)
    {
        return DeserializeExitCode::ERR_NO_NAME;
    }

    const std::size_t deviceNameLength = std::strlen(outPayload.deviceName);
    if (deviceNameLength == 0U)
    {
        return DeserializeExitCode::ERR_NO_NAME;
    }

    if (deviceNameLength > constants::virtualDevice::MAX_NAME_LENGTH)
    {
        DPL("Error: Device name is too long for this bridge build.");
        return DeserializeExitCode::ERR_NAME_TOO_LONG;
    }

    outPayload.actuatorIds = receivedDoc[KEY_ACTUATORS_ARRAY].as<JsonArrayConst>();
    if (outPayload.actuatorIds.isNull())
    {
        DPL("Error: Missing 'a' key in device details JSON.");
        return DeserializeExitCode::ERR_MISSING_KEY_ACTUATORS_IDS;
    }

    outPayload.buttonIds = receivedDoc[KEY_BUTTONS_ARRAY].as<JsonArrayConst>();
    if (outPayload.buttonIds.isNull())
    {
        DPL("Error: Missing 'b' key in device details JSON.");
        return DeserializeExitCode::ERR_MISSING_KEY_BUTTONS_IDS;
    }

    if (!validatePositiveUniqueIds(outPayload.actuatorIds, constants::virtualDevice::MAX_ACTUATORS))
    {
        DPL("Error: Invalid actuator IDs in device details JSON.");
        return DeserializeExitCode::ERR_ACTUATOR_ID_IMPLAUSIBLE;
    }

    if (!validatePositiveUniqueIds(outPayload.buttonIds, constants::virtualDevice::MAX_BUTTONS))
    {
        DPL("Error: Invalid button IDs in device details JSON.");
        return DeserializeExitCode::ERR_BUTTON_ID_IMPLAUSIBLE;
    }

    return DeserializeExitCode::OK_DETAILS;
}

/**
 * @brief Validate one controller-side `NETWORK_CLICK_*` payload.
 *
 * @param receivedDoc decoded controller frame to validate.
 * @param outPayload destination populated only when validation succeeds.
 * @return DeserializeExitCode::OK_NETWORK_CLICK_REQUEST when the payload is a valid request.
 * @return DeserializeExitCode::OK_NETWORK_CLICK_CONFIRM when the payload is a valid confirm.
 * @return another click-specific `DeserializeExitCode` when required fields are missing or malformed.
 */
auto validateReceivedNetworkClickPayload(const JsonDocument &receivedDoc, ValidatedNetworkClickPayload &outPayload) -> DeserializeExitCode
{
    using namespace lsh::bridge::protocol;

    std::uint8_t rawCommandId = 0U;
    if (!utils::json::tryGetUint8Scalar(receivedDoc[KEY_PAYLOAD], rawCommandId))
    {
        return DeserializeExitCode::ERR_UNKNOWN_PAYLOAD;
    }

    switch (static_cast<Command>(rawCommandId))
    {
    case Command::NETWORK_CLICK_REQUEST:
        break;

    case Command::NETWORK_CLICK_CONFIRM:
        break;

    default:
        return DeserializeExitCode::ERR_UNKNOWN_PAYLOAD;
    }

    if (!utils::json::tryGetUint8Scalar(receivedDoc[KEY_TYPE], outPayload.clickType))
    {
        return DeserializeExitCode::ERR_NO_CLICK_TYPE;
    }

    switch (static_cast<ProtocolClickType>(outPayload.clickType))
    {
    case ProtocolClickType::LONG:
    case ProtocolClickType::SUPER_LONG:
        break;

    default:
        return DeserializeExitCode::ERR_UNKNOWN_CLICK_TYPE;
    }

    if (!utils::json::tryGetUint8Scalar(receivedDoc[KEY_ID], outPayload.clickableId) || outPayload.clickableId == 0U)
    {
        return DeserializeExitCode::ERR_LONG_CLICKED_BUTTON_IMPLAUSIBLE;
    }

    if (!utils::json::tryGetUint8Scalar(receivedDoc[KEY_CORRELATION_ID], outPayload.correlationId) || outPayload.correlationId == 0U)
    {
        return DeserializeExitCode::ERR_CLICK_CORRELATION_ID_IMPLAUSIBLE;
    }

    return static_cast<Command>(rawCommandId) == Command::NETWORK_CLICK_REQUEST ? DeserializeExitCode::OK_NETWORK_CLICK_REQUEST
                                                                                : DeserializeExitCode::OK_NETWORK_CLICK_CONFIRM;
}

}  // namespace

#ifdef CONFIG_MSG_PACK_ARDUINO
/**
 * @brief Send one raw MsgPack payload through the framed serial transport.
 * @details The framing layer is transport-only: callers still pass the pure
 *          MsgPack payload bytes, while this helper adds the opening delimiter,
 *          escapes reserved bytes on the fly and closes the frame.
 *
 * @param buffer pointer to the pure MsgPack payload bytes.
 * @param length number of payload bytes available in `buffer`.
 * @return true if the full payload and both frame delimiters were accepted by the UART driver.
 * @return false if the UART driver rejected part of the frame.
 */
auto ControllerSerialLink::sendFramedMsgPackBuffer(const std::uint8_t *buffer, std::size_t length) -> bool
{
    if (buffer == nullptr || length == 0U)
    {
        return false;
    }

    MsgPackFrameWriter framedWriter(*this->serial);
    if (!framedWriter.beginFrame())
    {
        return false;
    }

    const std::size_t writtenPayloadBytes = framedWriter.write(buffer, length);
    const bool frameEnded = framedWriter.endFrame();
    return writtenPayloadBytes == length && frameEnded;
}
#endif

/**
 * @brief Send one compile-time pre-serialized static control payload.
 * @details This path is used for tiny pre-generated bridge/control payloads
 *          such as `PING`, `ASK_DETAILS`, `ASK_STATE` and the generic
 *          failover frame. No temporary JSON document is built at runtime.
 *
 * @param payloadType semantic identifier of the pre-generated payload bytes to
 *        send over the active serial codec.
 * @return true if the UART accepted the complete pre-generated payload.
 * @return false if throttling or a partial UART write prevented a full send.
 */
auto ControllerSerialLink::sendJson(constants::payloads::StaticType payloadType) -> bool
{
    using constants::payloads::StaticType;
    if (payloadType == StaticType::PING_ && !this->canPing())
    {
        return false;
    }

    // Select the prebuilt static payload table that matches the active wire format.
#ifdef CONFIG_MSG_PACK_ARDUINO
    constexpr bool useMsgPack = true;
#else
    constexpr bool useMsgPack = false;
#endif

    const auto payloadToSend = utils::payloads::getSerial<useMsgPack>(payloadType);

    if (!payloadToSend.empty())
    {
        const std::size_t writtenBytes = this->serial->write(payloadToSend.data(), payloadToSend.size());
        if (writtenBytes != payloadToSend.size())
        {
            return false;
        }
        this->updateLastSentTime();
        return true;
    }

    return false;
}

/**
 * @brief Sends a JsonDocument via Serial.
 * @details The active wire codec is selected at build time. JSON builds
 *          serialize newline-delimited text frames, while MsgPack builds emit
 *          the compact binary representation through the framed serial
 *          transport used on the controller link.
 *
 * @param json document to serialize and send over the active serial codec.
 * @return true if the full serialized payload has been accepted by the UART.
 * @return false if serialization produced no bytes or the UART accepted only a
 *         partial frame.
 */
auto ControllerSerialLink::sendJson(const JsonDocument &json) -> bool
{
    DP_CONTEXT();
    DP("↑ Json to send: ");
    DPJ(json);
#ifdef CONFIG_MSG_PACK_ARDUINO
    const std::size_t expectedPayloadBytes = measureMsgPack(json);
    MsgPackFrameWriter framedWriter(*this->serial);
    if (!framedWriter.beginFrame())
    {
        return false;
    }

    const std::size_t writtenPayloadBytes = serializeMsgPack(json, framedWriter);
    const bool frameEnded = framedWriter.endFrame();
    if (writtenPayloadBytes == 0U || writtenPayloadBytes != expectedPayloadBytes || !frameEnded)
    {
        return false;
    }
#else
    const std::size_t expectedPayloadBytes = measureJson(json);
    const std::size_t writtenPayloadBytes = serializeJson(json, (*this->serial));
    // Append the newline delimiter ONLY for JSON-based communication.
    // MsgPack doesn't need it.
    const std::size_t writtenDelimiterBytes = this->serial->write("\n", 1);
    if (writtenPayloadBytes != expectedPayloadBytes || writtenDelimiterBytes != 1U)
    {
        return false;
    }
#endif  // CONFIG_MSG_PACK_ARDUINO
    this->updateLastSentTime();
    return true;
}

/**
 * @brief Forwards a raw buffer of a known size using the active serial codec.
 * @details This is the most efficient method for forwarding payloads (like from
 * MQTT) as it uses a single, non-iterative `write()` operation and doesn't rely
 * on null-termination.
 * @param buffer A constant pointer to the start of the buffer.
 * @param length The number of bytes to write from the buffer.
 * @return true if the full raw payload frame has been accepted by the UART.
 * @return false if the buffer is invalid or the UART accepted only part of the frame.
 */
auto ControllerSerialLink::sendJson(const char *buffer, size_t length) -> bool
{
    DP_CONTEXT();
    if (!buffer || length == 0)
        return false;
#ifdef CONFIG_MSG_PACK_ARDUINO
    if (!this->sendFramedMsgPackBuffer(reinterpret_cast<const std::uint8_t *>(buffer), length))
    {
        return false;
    }
#else
    const std::size_t writtenPayloadBytes = this->serial->write(reinterpret_cast<const uint8_t *>(buffer), length);
    // Append the newline delimiter ONLY for JSON-based communication.
    const std::size_t writtenDelimiterBytes = this->serial->write("\n", 1);
    if (writtenPayloadBytes != length || writtenDelimiterBytes != 1U)
    {
        return false;
    }
#endif
    this->updateLastSentTime();
    return true;
}

/**
 * @brief Copy the last controller-confirmed state into the desired-state shadow.
 * @details The bridge uses this helper when it decides that pending MQTT-side
 *          intent must be discarded, for example after a command storm
 *          or after losing runtime synchronization.
 */
void ControllerSerialLink::realignDesiredStateShadowWithAuthoritativeLocked()
{
    for (std::uint8_t actuatorIndex = 0U; actuatorIndex < this->virtualDevice.getTotalActuators(); ++actuatorIndex)
    {
        this->desiredActuatorStates[actuatorIndex] = this->virtualDevice.getStateByIndex(actuatorIndex);
    }
}

/**
 * @brief Forget every pending desired-state change and reset batch bookkeeping.
 * @details This helper is the "safe abort" path for the coalescing logic. It
 *          drops unconfirmed remote intent, realigns the desired-state shadow
 *          with the last authoritative controller snapshot, and clears all
 *          batch timers/counters.
 *
 */
void ControllerSerialLink::clearPendingActuatorBatchLocked()
{
    this->realignDesiredStateShadowWithAuthoritativeLocked();
    this->desiredDirtyActuators.reset();
    this->refreshPendingActuatorBatchStateLocked();
}

/**
 * @brief Recompute the pending SET_STATE batch invariants from the dirty mask.
 *
 * @param restartSettleWindow if true, restart the quiet window from the
 *        current real time.
 * @param markNewMutation if true, count this refresh as one newly accepted
 *        command change merged into the current batch.
 */
void ControllerSerialLink::refreshPendingActuatorBatchStateLocked(bool restartSettleWindow, bool markNewMutation)
{
    const bool wasBatchPending = this->actuatorCommandBatchPending;
    this->actuatorCommandBatchPending = (this->desiredDirtyActuators.count() > 0U);
    if (!this->actuatorCommandBatchPending)
    {
        this->firstDesiredActuatorUpdate_ms = 0U;
        this->lastDesiredActuatorUpdate_ms = 0U;
        this->pendingActuatorMutationCount = 0U;
    }
    else if (!wasBatchPending)
    {
        const auto now = timeKeeper::getRealTime();
        this->firstDesiredActuatorUpdate_ms = now;
        this->lastDesiredActuatorUpdate_ms = now;
        this->pendingActuatorMutationCount = markNewMutation ? 1U : 0U;
    }
    else if (restartSettleWindow)
    {
        this->lastDesiredActuatorUpdate_ms = timeKeeper::getRealTime();
    }

    if (this->actuatorCommandBatchPending && markNewMutation && wasBatchPending &&
        this->pendingActuatorMutationCount < constants::runtime::ACTUATOR_COMMAND_MAX_MUTATION_COUNT)
    {
        ++this->pendingActuatorMutationCount;
    }
}

/**
 * @brief Stage one actuator command inside the desired-state shadow.
 *
 * @param actuatorId logical actuator ID received from MQTT/Homie.
 * @param state desired actuator state.
 * @return true if the command has been accepted or was a harmless duplicate.
 * @return false if the actuator ID does not exist in the cached topology.
 */
auto ControllerSerialLink::stageSingleActuatorCommand(uint8_t actuatorId, bool state) -> bool
{
    DP_CONTEXT();
    DPL("↑ ID: ", actuatorId, " | state: ", state);

    // MQTT and Homie address actuators by logical wire ID, while internal arrays
    // are indexed by compact position. Resolve the ID once before touching the
    // desired-state shadow.
    std::uint8_t actuatorIndex = 0U;
    if (!this->virtualDevice.tryGetActuatorIndex(actuatorId, actuatorIndex))
    {
        return false;
    }

    const bool authoritativeState = this->virtualDevice.getStateByIndex(actuatorIndex);

    this->lockActuatorCommandState();
    // If this exact actuator already has the same target value staged in the
    // currently pending batch, treat the repeat as a no-op so it doesn't keep
    // stretching the settle window for everyone else.
    if (this->actuatorCommandBatchPending && this->desiredDirtyActuators.test(actuatorIndex) &&
        this->desiredActuatorStates[actuatorIndex] == state)
    {
        DPL("Ignoring duplicate pending actuator command for ID ", actuatorId);
        this->unlockActuatorCommandState();
        return true;
    }

    this->desiredActuatorStates[actuatorIndex] = state;
    if (state == authoritativeState)
    {
        // If the requested state already matches reality, there is nothing left to
        // send for this actuator in the pending batch.
        this->desiredDirtyActuators.reset(actuatorIndex);
    }
    else
    {
        this->desiredDirtyActuators.set(actuatorIndex);
    }

    this->refreshPendingActuatorBatchStateLocked(true, true);
    this->unlockActuatorCommandState();
    return true;
}

/**
 * @brief Stage a full packed SET_STATE request inside the desired-state shadow.
 * @details The bridge unpacks the bytes into a temporary snapshot, compares it
 *          with both the currently pending snapshot and the authoritative
 *          controller state, then keeps only the actuators that need to
 *          be sent in the next coalesced outbound batch.
 *
 * @param packedBytes packed actuator-state bytes.
 * @return true if the payload is valid and has been staged.
 * @return false if the payload shape or contents are invalid.
 */
auto ControllerSerialLink::stageDesiredPackedState(const JsonArrayConst &packedBytes) -> bool
{
    const auto totalActuators = this->virtualDevice.getTotalActuators();
    const std::size_t expectedBytes = (static_cast<std::size_t>(totalActuators) + 7U) / 8U;
    if (packedBytes.isNull() || packedBytes.size() != expectedBytes)
    {
        return false;
    }

    if (totalActuators == 0U)
    {
        return true;
    }

    bool desiredSnapshot[constants::virtualDevice::MAX_ACTUATORS]{};
    std::uint8_t actuatorIndex = 0U;

    // First validate and unpack the payload outside the critical section. The
    // lock is only taken once the whole snapshot is known to be well-formed.
    for (JsonVariantConst packedByteVariant : packedBytes)
    {
        std::uint8_t packedByte = 0U;
        if (!utils::json::tryGetUint8Scalar(packedByteVariant, packedByte))
        {
            return false;
        }

        for (std::uint8_t bitIndex = 0U; bitIndex < 8U && actuatorIndex < totalActuators; ++bitIndex)
        {
            desiredSnapshot[actuatorIndex] = (packedByte & kPackedBitMask[bitIndex]) != 0U;
            ++actuatorIndex;
        }
    }

    this->lockActuatorCommandState();
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
        this->unlockActuatorCommandState();
        return true;
    }

    this->desiredDirtyActuators.reset();
    for (std::uint8_t index = 0U; index < totalActuators; ++index)
    {
        this->desiredActuatorStates[index] = desiredSnapshot[index];
        if (desiredSnapshot[index] != this->virtualDevice.getStateByIndex(index))
        {
            this->desiredDirtyActuators.set(index);
        }
    }
    this->refreshPendingActuatorBatchStateLocked(true, true);
    this->unlockActuatorCommandState();

    return true;
}

/**
 * @brief Send the coalesced SET_STATE batch once the quiet window expires.
 * @details This method snapshots the desired-state shadow, waits for a short
 *          period without newer writes, then emits one packed SET_STATE frame
 *          that represents the full desired batch. The desired-state mutex
 *          stays held from the last "is this snapshot still valid?" decision
 *          through the serial write itself, so a newer MQTT/Homie command
 *          cannot slip in after the bridge already committed to sending the
 *          current packed batch. If a producer keeps changing the desired state
 *          for too long or too many times in a single batch, the bridge treats
 *          that traffic as unstable and drops the batch once the safety limits
 *          are exceeded. If the UART refuses the outbound frame, the batch is
 *          left pending so the next loop can retry the same authoritative
 *          intent instead of silently forgetting it.
 */
void ControllerSerialLink::processPendingActuatorBatch()
{
    const auto now = timeKeeper::getRealTime();
    const auto totalActuators = this->virtualDevice.getTotalActuators();
    if (totalActuators == 0U)
    {
        return;
    }

    bool desiredSnapshot[constants::virtualDevice::MAX_ACTUATORS]{};
    bool batchPending = false;
    std::uint32_t firstDesiredUpdate_ms = 0U;
    std::uint32_t lastDesiredUpdate_ms = 0U;
    std::uint16_t pendingMutationCount = 0U;

    this->lockActuatorCommandState();
    for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
    {
        desiredSnapshot[actuatorIndex] = this->desiredActuatorStates[actuatorIndex];
    }
    batchPending = this->actuatorCommandBatchPending;
    firstDesiredUpdate_ms = this->firstDesiredActuatorUpdate_ms;
    lastDesiredUpdate_ms = this->lastDesiredActuatorUpdate_ms;
    pendingMutationCount = this->pendingActuatorMutationCount;

    if (!batchPending)
    {
        this->unlockActuatorCommandState();
        return;
    }

    const bool settleWindowElapsed = (now - lastDesiredUpdate_ms) >= constants::runtime::ACTUATOR_COMMAND_SETTLE_INTERVAL_MS;
    const bool pendingWindowExceeded =
        firstDesiredUpdate_ms != 0U && (now - firstDesiredUpdate_ms) >= constants::runtime::ACTUATOR_COMMAND_MAX_PENDING_MS;
    const bool mutationBudgetExceeded = pendingMutationCount >= constants::runtime::ACTUATOR_COMMAND_MAX_MUTATION_COUNT;

    if (!settleWindowElapsed && !pendingWindowExceeded && !mutationBudgetExceeded)
    {
        this->unlockActuatorCommandState();
        return;
    }

    if (pendingWindowExceeded || mutationBudgetExceeded)
    {
        // This is the "command storm" safety valve. The bridge prefers to
        // wait for a stable intent, but it refuses to keep one batch open forever
        // under flapping or malicious traffic. In that case it discards the
        // unconfirmed desired state and trusts only the last authoritative
        // controller snapshot again.
        this->pendingActuatorStormDiagnostic.pendingDurationMs = (now - firstDesiredUpdate_ms);
        this->pendingActuatorStormDiagnostic.mutationCount = pendingMutationCount;
        this->hasPendingActuatorStormDiagnostic = true;
        this->clearPendingActuatorBatchLocked();
        this->unlockActuatorCommandState();

        DPL("Dropping unstable actuator batch after ", (now - firstDesiredUpdate_ms), " ms and ", pendingMutationCount,
            " accepted changes. The desired-state shadow has been realigned "
            "with the last authoritative controller state.");
        return;
    }

    bool differsFromAuthoritative = false;
    for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
    {
        if (desiredSnapshot[actuatorIndex] != this->virtualDevice.getStateByIndex(actuatorIndex))
        {
            differsFromAuthoritative = true;
            break;
        }
    }

    if (!differsFromAuthoritative)
    {
        this->desiredDirtyActuators.reset();
        this->refreshPendingActuatorBatchStateLocked();
        this->unlockActuatorCommandState();
        DPL("Dropping pending actuator batch: already matches authoritative "
            "state.");
        return;
    }

    using namespace lsh::bridge::protocol;
    this->serializationDoc.clear();
    this->serializationDoc[KEY_PAYLOAD] = static_cast<uint8_t>(Command::SET_STATE);
    auto packedStates = this->serializationDoc.createNestedArray(KEY_STATE);

    std::uint8_t packedByte = 0U;
    for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
    {
        if (desiredSnapshot[actuatorIndex])
        {
            // `actuatorIndex & 0x07U` means "bit position inside the current output
            // byte". Every 8 actuators one packed byte is emitted.
            packedByte |= kPackedBitMask[actuatorIndex & 0x07U];
        }

        const bool endOfPackedByte = ((actuatorIndex & 0x07U) == 0x07U);
        const bool lastActuator = actuatorIndex == (totalActuators - 1U);
        if (endOfPackedByte || lastActuator)
        {
            packedStates.add(packedByte);
            packedByte = 0U;
        }
    }

    if (!this->sendJson(this->serializationDoc))
    {
        this->unlockActuatorCommandState();
        DPL("Serial TX did not accept the coalesced SET_STATE batch. Keeping the "
            "desired-state batch pending for retry.");
        return;
    }

    this->desiredDirtyActuators.reset();
    this->refreshPendingActuatorBatchStateLocked();
    this->unlockActuatorCommandState();
}

/**
 * @brief Drop every pending desired-state change and realign the shadow.
 * @details This is used when the runtime loses authoritative synchronization
 *          and must forget any MQTT-side desired state until a fresh
 *          controller-backed snapshot is available again.
 */
void ControllerSerialLink::clearPendingActuatorBatch()
{
    this->lockActuatorCommandState();
    this->clearPendingActuatorBatchLocked();
    this->unlockActuatorCommandState();
}

/**
 * @brief Reconcile pending desired states against the latest authoritative state.
 * @details After a controller state frame arrives, commands that are already
 *          satisfied are removed from the dirty mask, while untouched actuators
 *          are updated so the desired-state shadow keeps matching reality.
 */
void ControllerSerialLink::reconcileDesiredActuatorStatesFromAuthoritative()
{
    const auto totalActuators = this->virtualDevice.getTotalActuators();

    this->lockActuatorCommandState();
    if (this->actuatorCommandBatchPending)
    {
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
        {
            const bool authoritativeState = this->virtualDevice.getStateByIndex(actuatorIndex);
            if (this->desiredDirtyActuators.test(actuatorIndex))
            {
                // This actuator has an unconfirmed target. If the controller
                // reports that target as real, the pending intent has been satisfied.
                if (this->desiredActuatorStates[actuatorIndex] == authoritativeState)
                {
                    this->desiredDirtyActuators.reset(actuatorIndex);
                }
            }
            else
            {
                // No pending remote intent for this actuator: keep the desired shadow
                // aligned with the newly received authoritative reality.
                this->desiredActuatorStates[actuatorIndex] = authoritativeState;
            }
        }

        const bool wasBatchPending = this->actuatorCommandBatchPending;
        this->refreshPendingActuatorBatchStateLocked();
        if (wasBatchPending && !this->actuatorCommandBatchPending)
        {
            DPL("Cleared pending actuator batch during authoritative "
                "reconciliation.");
        }
    }
    else
    {
        // When no batch is pending, the whole desired shadow simply tracks the
        // latest authoritative state one-to-one.
        this->realignDesiredStateShadowWithAuthoritativeLocked();
        this->desiredDirtyActuators.reset();
        this->refreshPendingActuatorBatchStateLocked();
    }
    this->unlockActuatorCommandState();
}

/**
 * @brief Copy the last pending unstable-batch diagnostic without clearing it.
 * @details The outer bridge runtime uses this method to build a bridge-local
 *          MQTT diagnostic event on the `bridge` topic only after publishing is
 *          known to be possible.
 *
 * @param outDiagnostic destination structure that receives the diagnostic.
 * @return true if a diagnostic was available.
 * @return false if no unstable batch has been dropped since the last clear.
 */
auto ControllerSerialLink::peekPendingActuatorStormDiagnostic(ActuatorStormDiagnostic &outDiagnostic) -> bool
{
    bool hasDiagnostic = false;
    this->lockActuatorCommandState();
    if (this->hasPendingActuatorStormDiagnostic)
    {
        outDiagnostic = this->pendingActuatorStormDiagnostic;
        hasDiagnostic = true;
    }
    this->unlockActuatorCommandState();
    return hasDiagnostic;
}

/**
 * @brief Clear the last unstable-batch diagnostic after the outer bridge has
 *        reported it.
 */
void ControllerSerialLink::clearPendingActuatorStormDiagnostic()
{
    this->lockActuatorCommandState();
    this->hasPendingActuatorStormDiagnostic = false;
    this->pendingActuatorStormDiagnostic = ActuatorStormDiagnostic{};
    this->unlockActuatorCommandState();
}

/**
 * @brief Register the high-level payload callback used by the outer bridge runtime.
 *
 * @param callback delegate invoked after one complete controller frame has been
 *        decoded and classified.
 */
void ControllerSerialLink::onMessage(MessageCallback callback)
{
    this->messageCallback = callback;
}

/**
 * @brief Reads the serial buffer, processes at most one complete message, and invokes
 *        the registered callback for it.
 * @details JSON transport uses newline-delimited text frames. MsgPack
 *          transport uses a SLIP-like delimiter-and-escape layer on top of the
 *          pure MsgPack payload, so the serial path never relies on stream
 *          timeouts as implicit framing. Each call also obeys a raw-byte
 *          fairness budget so noisy UART traffic cannot monopolize one bridge
 *          loop iteration. When a complete payload is decoded and validated,
 *          the method invokes the registered callback and returns immediately.
 */
void ControllerSerialLink::processSerialBuffer()
{
    std::uint16_t consumedBytes = 0U;

#ifdef CONFIG_MSG_PACK_ARDUINO
    const std::uint32_t nowRealTime_ms = timeKeeper::getRealTime();
    this->msgPackFrameReceiver.resetIfIdle(nowRealTime_ms, constants::controllerSerial::ARDCOM_SERIAL_MSGPACK_FRAME_IDLE_TIMEOUT_MS);

    while (this->serial->available() && consumedBytes < constants::controllerSerial::SERIAL_MAX_RX_BYTES_PER_LOOP)
    {
        const int rawByte = this->serial->read();
        if (rawByte < 0)
        {
            break;
        }
        ++consumedBytes;

        const auto consumeResult = this->msgPackFrameReceiver.consumeByte(static_cast<std::uint8_t>(rawByte), nowRealTime_ms);
        if (consumeResult == MsgPackFrameConsumeResult::FrameDiscarded)
        {
            DPL("Discarded malformed framed MsgPack controller payload.");
            continue;
        }

        if (consumeResult != MsgPackFrameConsumeResult::FrameComplete)
        {
            continue;
        }

        this->receivedDoc.clear();
        const DeserializationError deserializationError =
            deserializeMsgPack(this->receivedDoc, this->msgPackFrameReceiver.frameData(), this->msgPackFrameReceiver.frameLength());
        this->msgPackFrameReceiver.reset();
        if (deserializationError != DeserializationError::Ok)
        {
            DPL("MsgPack deserialization error: ", deserializationError.c_str());
            return;
        }

        this->updateLastReceivedTime();
        const auto messageType = this->deserialize();
        if (this->messageCallback.is_valid())
        {
            this->messageCallback(messageType, this->receivedDoc);
        }
        return;
    }
    return;

#else
    while (this->serial->available() && consumedBytes < constants::controllerSerial::SERIAL_MAX_RX_BYTES_PER_LOOP)
    {
        const int rawByte = this->serial->read();
        if (rawByte < 0)
        {
            break;
        }
        ++consumedBytes;

        char receivedChar = static_cast<char>(rawByte);

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
            {
                continue;
            }

            this->rxBuffer[this->rxBufferIndex] = '\0';
            this->receivedDoc.clear();
            DeserializationError deserializationError = deserializeJson(this->receivedDoc, this->rxBuffer, this->rxBufferIndex);
            this->rxBufferIndex = 0;

            if (deserializationError == DeserializationError::Ok)
            {
                this->updateLastReceivedTime();
                const auto messageType = this->deserialize();
                if (this->messageCallback.is_valid())
                {
                    // The receivedDoc is populated. It uses zero-copy and points
                    // directly into rxBuffer. The messageCallback is responsible for
                    // either copying the data into a persistent storage (like
                    // VirtualDevice) or re-serializing it for forwarding. The rxBuffer
                    // will be reused for the next message, so its content is volatile
                    this->messageCallback(messageType, this->receivedDoc);
                }
                return;
            }
            else
            {
                DPL("Deserialization err: ", deserializationError.c_str(), " on message: ", this->rxBuffer);
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
 * @brief Deserializes the last received JSON document and identifies the
 * payload type.
 * @details This method is state-agnostic. It only determines what kind of
 * message was received and returns a corresponding exit code. The application
 * loop is responsible for acting on this information.
 * @return A DeserializeExitCode representing the received message type.
 */
auto ControllerSerialLink::deserialize() -> constants::DeserializeExitCode
{
    DP_CONTEXT();
    using namespace lsh::bridge::protocol;

    const JsonVariantConst commandVariant = this->receivedDoc[KEY_PAYLOAD];
    if (commandVariant.isNull())
    {
        return DeserializeExitCode::ERR_MISSING_KEY_PAYLOAD;
    }

    std::uint8_t rawCommandId = 0U;
    if (!utils::json::tryGetUint8Scalar(commandVariant, rawCommandId))
    {
        return DeserializeExitCode::ERR_UNKNOWN_PAYLOAD;
    }

    switch (static_cast<Command>(rawCommandId))
    {
    case Command::DEVICE_DETAILS:
        return DeserializeExitCode::OK_DETAILS;
    case Command::ACTUATORS_STATE:
        return DeserializeExitCode::OK_STATE;
    case Command::NETWORK_CLICK_REQUEST:
    case Command::NETWORK_CLICK_CONFIRM:
    {
        ValidatedNetworkClickPayload clickPayload{};
        return validateReceivedNetworkClickPayload(this->receivedDoc, clickPayload);
    }
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
 * @details The state is sent as an array of bytes where each byte contains 8
 * actuator states. Byte 0, bit 0 = actuator 0; Byte 0, bit 7 = actuator 7; Byte
 * 1, bit 0 = actuator 8, etc.
 * @return constants::DeserializeExitCode the result of sanity checks.
 */
auto ControllerSerialLink::storeStateFromReceived() const -> constants::DeserializeExitCode
{
    DP_CONTEXT();
    using namespace lsh::bridge::protocol;

    const JsonArrayConst packedBytes = this->receivedDoc[KEY_STATE].as<JsonArrayConst>();
    if (packedBytes.isNull())
    {
        return DeserializeExitCode::ERR_MISSING_KEY_STATE;
    }

    const std::size_t expectedBytes = (static_cast<std::size_t>(this->virtualDevice.getTotalActuators()) + 7U) / 8U;
    if (packedBytes.size() != expectedBytes)
    {
        return DeserializeExitCode::ERR_ACTUATORS_MISMATCH;
    }

    for (JsonVariantConst packedByte : packedBytes)
    {
        std::uint8_t validatedPackedByte = 0U;
        if (!utils::json::tryGetUint8Scalar(packedByte, validatedPackedByte))
        {
            return DeserializeExitCode::ERR_STATE_VALUE_IMPLAUSIBLE;
        }
    }

    this->virtualDevice.setStateFromPackedBytes(packedBytes);

    return DeserializeExitCode::OK_STATE;
}

/**
 * @brief Validate and materialize received details without mutating the active model.
 * @details The bridge uses this method when it must compare a fresh controller
 *          topology against the currently cached one before deciding whether a
 *          reboot is required.
 *
 * @param outDetails destination snapshot that receives the validated topology.
 * @return constants::DeserializeExitCode::OK_DETAILS if the last received
 *         payload is a valid `DEVICE_DETAILS` frame and has been copied into
 *         `outDetails`.
 * @return any other `DeserializeExitCode` if validation fails; in that case the
 *         caller must treat `outDetails` as cleared/transient data only.
 */
auto ControllerSerialLink::parseDetailsFromReceived(DeviceDetailsSnapshot &outDetails) const -> constants::DeserializeExitCode
{
    DP_CONTEXT();
    ValidatedDetailsPayload detailsPayload{};
    const auto result = validateReceivedDetailsPayload(this->receivedDoc, detailsPayload);
    if (result != DeserializeExitCode::OK_DETAILS)
    {
        return result;
    }

    outDetails.clear();
    outDetails.name.assign(detailsPayload.deviceName);
    for (const JsonVariantConst idVariant : detailsPayload.actuatorIds)
    {
        outDetails.actuatorIds.push_back(idVariant.as<std::uint8_t>());
    }

    for (const JsonVariantConst idVariant : detailsPayload.buttonIds)
    {
        outDetails.buttonIds.push_back(idVariant.as<std::uint8_t>());
    }

    return DeserializeExitCode::OK_DETAILS;
}

/**
 * @brief Return if device can ping (minimum interval check).
 *
 * @return true if device can ping.
 * @return false if device can't ping.
 */
auto ControllerSerialLink::canPing() const -> bool
{
    // DP_CONTEXT(); // Bloats the serial
    using constants::ping::PING_INTERVAL_CONTROLLINO_MS;
    return (timeKeeper::getTime() - this->lastSentPayloadTime_ms > PING_INTERVAL_CONTROLLINO_MS);
}

/**
 * @brief Set last time payload has been sent to now.
 *
 */
void ControllerSerialLink::updateLastSentTime()
{
    // DP_CONTEXT(); pollutes serial
    this->lastSentPayloadTime_ms = timeKeeper::getTime();
}

/**
 * @brief Cache the receive timestamp of the most recent valid controller frame.
 * @details This is also the moment from which the bridge may honestly claim
 *          that the controller link is alive for this boot session.
 */
void ControllerSerialLink::updateLastReceivedTime()
{
    // DP_CONTEXT(); pollutes serial
    this->hasSeenControllerTraffic = true;
    this->lastReceivedPayloadTime_ms = timeKeeper::getTime();
}

/**
 * @brief Creates and sends a "failover" command for a specific click.
 * @details This function is called when a network click cannot be forwarded to
 * MQTT. It reuses the information (click type, button ID) from the last
 * received click message (`receivedDoc`) to construct and send a specific
 * failover command back to the Controllino.
 * @return DeserializeExitCode::ERR_NOT_CONNECTED_FAILOVER_SENT if the specific
 *         click failover command has been accepted by the UART.
 * @return DeserializeExitCode::ERR_NOT_CONNECTED_GENERAL_FAILOVER_SENT if the
 *         bridge could not build the specific click payload but a generic
 *         failover command has been accepted by the UART.
 * @return DeserializeExitCode::ERR_NOT_FORWARDED_OTHER_PROBLEM if the UART did
 *         not accept the fallback command and the click could therefore not be
 *         forwarded nor failed over safely.
 */
auto ControllerSerialLink::triggerFailoverFromReceivedClick() -> constants::DeserializeExitCode
{
    DP_CONTEXT();
    using namespace lsh::bridge::protocol;

    ValidatedNetworkClickPayload clickPayload{};
    const auto validationResult = validateReceivedNetworkClickPayload(this->receivedDoc, clickPayload);
    if (validationResult != DeserializeExitCode::OK_NETWORK_CLICK_REQUEST)
    {
        DPL("Cannot create specific failover, sending general failover.");
        return this->sendJson(StaticType::GENERAL_FAILOVER) ? DeserializeExitCode::ERR_NOT_CONNECTED_GENERAL_FAILOVER_SENT
                                                            : DeserializeExitCode::ERR_NOT_FORWARDED_OTHER_PROBLEM;
    }

    this->serializationDoc.clear();

    this->serializationDoc[KEY_PAYLOAD] = static_cast<uint8_t>(Command::FAILOVER_CLICK);
    this->serializationDoc[KEY_TYPE] = clickPayload.clickType;
    this->serializationDoc[KEY_ID] = clickPayload.clickableId;
    this->serializationDoc[KEY_CORRELATION_ID] = clickPayload.correlationId;

    return this->sendJson(this->serializationDoc) ? DeserializeExitCode::ERR_NOT_CONNECTED_FAILOVER_SENT
                                                  : DeserializeExitCode::ERR_NOT_FORWARDED_OTHER_PROBLEM;
}

/**
 * @brief Return whether the controller is considered connected right now.
 * @details The bridge reports the controller as connected only after it has
 *          decoded at least one valid frame in this boot session and that frame
 *          is inside the configured liveness timeout window. The explicit
 *          `hasSeenControllerTraffic` gate prevents the runtime from treating
 *          a never-seen controller as connected.
 *
 * @return true if attached device is connected.
 * @return false if attached device isn't connected.
 */
auto ControllerSerialLink::isConnected() const -> bool
{
    using constants::ping::CONNECTION_TIMEOUT_CONTROLLINO_MS;
    return this->hasSeenControllerTraffic &&
           ((timeKeeper::getTime() - this->lastReceivedPayloadTime_ms) < CONNECTION_TIMEOUT_CONTROLLINO_MS);
}
