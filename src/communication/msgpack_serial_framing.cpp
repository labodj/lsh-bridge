/**
 * @file    msgpack_serial_framing.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the serial framing helpers used for MsgPack payloads on the controller link.
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

#include "communication/msgpack_serial_framing.hpp"

namespace lsh::bridge::transport
{
/**
 * @brief Construct one incremental MsgPack frame receiver around caller-owned storage.
 *
 * @param buffer destination storage used for the deframed payload bytes.
 * @param capacity maximum number of payload bytes that fit in `buffer`.
 */
MsgPackFrameReceiver::MsgPackFrameReceiver(char *buffer, std::uint16_t capacity) noexcept : frameBuffer(buffer), frameCapacity(capacity)
{}

/**
 * @brief Forget the current receive state and return to the idle state.
 * @details The destination buffer content is left untouched because the caller
 *          only trusts the first `frameLength()` bytes after `FrameComplete`.
 */
void MsgPackFrameReceiver::reset() noexcept
{
    this->frameLengthBytes = 0U;
    this->lastByteTimeMs = 0U;
    this->escapePending = false;
    this->discardUntilFrameEnd = false;
}

/**
 * @brief Drop a partially received frame that has been idle for too long.
 * @details This timeout is only a housekeeping guard for truncated frames. It
 *          is not part of the framing itself and does not define frame
 *          boundaries.
 *
 * @param nowMs current real-time tick in milliseconds.
 * @param idleTimeoutMs maximum allowed silence while one frame is in progress.
 */
void MsgPackFrameReceiver::resetIfIdle(std::uint32_t nowMs, std::uint32_t idleTimeoutMs) noexcept
{
    if (idleTimeoutMs == 0U)
    {
        return;
    }

    if (!this->discardUntilFrameEnd && !this->escapePending && this->frameLengthBytes == 0U)
    {
        return;
    }

    if ((nowMs - this->lastByteTimeMs) >= idleTimeoutMs)
    {
        this->reset();
    }
}

/**
 * @brief Append one deframed byte to the payload buffer.
 *
 * @param byte payload byte to append.
 * @return true if the byte fit in the configured destination buffer.
 * @return false if appending it would overflow the buffer.
 */
auto MsgPackFrameReceiver::appendByte(std::uint8_t byte) -> bool
{
    if (this->frameLengthBytes >= this->frameCapacity)
    {
        return false;
    }

    this->frameBuffer[this->frameLengthBytes++] = static_cast<char>(byte);
    return true;
}

/**
 * @brief Enter discard mode until the next frame delimiter arrives.
 * @details Once a frame is known to be corrupted or too large, the receiver
 *          ignores every following byte until the next `END` delimiter so the
 *          next payload starts from a clean state.
 */
void MsgPackFrameReceiver::startDiscarding() noexcept
{
    this->frameLengthBytes = 0U;
    this->escapePending = false;
    this->discardUntilFrameEnd = true;
}

/**
 * @brief Feed one raw serial byte into the framed MsgPack receiver.
 * @details The transport uses a SLIP-like byte-stuffed frame:
 *          `END + escaped(msgpack_payload) + END`.
 *          The receiver therefore only needs to understand the delimiter and
 *          the two escaped reserved bytes; the payload remains pure MsgPack.
 *
 * @param byte raw byte read from UART.
 * @param nowMs current real-time tick in milliseconds.
 * @return MsgPackFrameConsumeResult describing whether a frame became ready or
 *         a corrupted frame has just been dropped.
 */
auto MsgPackFrameReceiver::consumeByte(std::uint8_t byte, std::uint32_t nowMs) -> MsgPackFrameConsumeResult
{
    this->lastByteTimeMs = nowMs;

    if (this->discardUntilFrameEnd)
    {
        if (byte == MSGPACK_FRAME_END)
        {
            this->reset();
            return MsgPackFrameConsumeResult::FrameDiscarded;
        }
        return MsgPackFrameConsumeResult::Incomplete;
    }

    if (this->escapePending)
    {
        this->escapePending = false;
        switch (byte)
        {
        case MSGPACK_FRAME_ESCAPED_END:
            if (this->appendByte(MSGPACK_FRAME_END))
            {
                return MsgPackFrameConsumeResult::Incomplete;
            }
            this->startDiscarding();
            return MsgPackFrameConsumeResult::Incomplete;

        case MSGPACK_FRAME_ESCAPED_ESCAPE:
            if (this->appendByte(MSGPACK_FRAME_ESCAPE))
            {
                return MsgPackFrameConsumeResult::Incomplete;
            }
            this->startDiscarding();
            return MsgPackFrameConsumeResult::Incomplete;

        default:
            if (byte == MSGPACK_FRAME_END)
            {
                this->reset();
                return MsgPackFrameConsumeResult::FrameDiscarded;
            }
            this->startDiscarding();
            return MsgPackFrameConsumeResult::Incomplete;
        }
    }

    if (byte == MSGPACK_FRAME_END)
    {
        if (this->frameLengthBytes == 0U)
        {
            return MsgPackFrameConsumeResult::Incomplete;
        }
        return MsgPackFrameConsumeResult::FrameComplete;
    }

    if (byte == MSGPACK_FRAME_ESCAPE)
    {
        this->escapePending = true;
        return MsgPackFrameConsumeResult::Incomplete;
    }

    if (this->appendByte(byte))
    {
        return MsgPackFrameConsumeResult::Incomplete;
    }

    this->startDiscarding();
    return MsgPackFrameConsumeResult::Incomplete;
}

/**
 * @brief Return the beginning of the completed payload buffer.
 *
 * @return const std::uint8_t * pointer to the first deframed payload byte.
 */
auto MsgPackFrameReceiver::frameData() const -> const std::uint8_t *
{
    return reinterpret_cast<const std::uint8_t *>(this->frameBuffer);
}

/**
 * @brief Return the current payload length assembled by the receiver.
 *
 * @return std::uint16_t number of valid payload bytes stored in `frameBuffer`.
 */
auto MsgPackFrameReceiver::frameLength() const -> std::uint16_t
{
    return this->frameLengthBytes;
}

/**
 * @brief Construct one streaming MsgPack frame writer around a UART.
 *
 * @param serialPort UART used to send the framed payload.
 */
MsgPackFrameWriter::MsgPackFrameWriter(HardwareSerial &serialPort) noexcept : serial(serialPort)
{}

/**
 * @brief Emit the frame delimiter that starts one new MsgPack transport frame.
 * @details The opening delimiter also flushes any stale partial state that may
 *          still exist on the receiver after line noise or a peer reset.
 *
 * @return true if the delimiter byte has been accepted by the UART driver.
 * @return false otherwise.
 */
auto MsgPackFrameWriter::beginFrame() -> bool
{
    return this->serial.write(MSGPACK_FRAME_END) == 1U;
}

/**
 * @brief Emit the frame delimiter that terminates one MsgPack transport frame.
 *
 * @return true if the delimiter byte has been accepted by the UART driver.
 * @return false otherwise.
 */
auto MsgPackFrameWriter::endFrame() -> bool
{
    return this->serial.write(MSGPACK_FRAME_END) == 1U;
}

/**
 * @brief Write one payload byte, escaping reserved transport markers when needed.
 *
 * @param byte raw MsgPack payload byte to send.
 * @return true if the byte, or its escaped representation, has been accepted.
 * @return false if the UART driver rejected part of the write.
 */
auto MsgPackFrameWriter::writeEscapedByte(std::uint8_t byte) -> bool
{
    if (byte == MSGPACK_FRAME_END)
    {
        return this->serial.write(MSGPACK_FRAME_ESCAPE) == 1U && this->serial.write(MSGPACK_FRAME_ESCAPED_END) == 1U;
    }

    if (byte == MSGPACK_FRAME_ESCAPE)
    {
        return this->serial.write(MSGPACK_FRAME_ESCAPE) == 1U && this->serial.write(MSGPACK_FRAME_ESCAPED_ESCAPE) == 1U;
    }

    return this->serial.write(byte) == 1U;
}

/**
 * @brief Write one raw payload byte through the framed MsgPack transport.
 *
 * @param byte raw MsgPack payload byte.
 * @return std::size_t `1` if the payload byte was accepted, `0` otherwise.
 */
auto MsgPackFrameWriter::write(std::uint8_t byte) -> std::size_t
{
    return this->writeEscapedByte(byte) ? 1U : 0U;
}

/**
 * @brief Write a contiguous raw payload buffer through the framed MsgPack transport.
 *
 * @param buffer pointer to the raw MsgPack payload bytes.
 * @param size number of payload bytes available in `buffer`.
 * @return std::size_t number of payload bytes accepted from `buffer`.
 */
auto MsgPackFrameWriter::write(const std::uint8_t *buffer, std::size_t size) -> std::size_t
{
    if (buffer == nullptr)
    {
        return 0U;
    }

    std::size_t writtenPayloadBytes = 0U;
    while (writtenPayloadBytes < size)
    {
        std::size_t contiguousSafeBytes = 0U;
        while ((writtenPayloadBytes + contiguousSafeBytes) < size)
        {
            const std::uint8_t byte = buffer[writtenPayloadBytes + contiguousSafeBytes];
            if (byte == MSGPACK_FRAME_END || byte == MSGPACK_FRAME_ESCAPE)
            {
                break;
            }
            ++contiguousSafeBytes;
        }

        if (contiguousSafeBytes > 0U)
        {
            const std::size_t writtenChunkBytes = this->serial.write(buffer + writtenPayloadBytes, contiguousSafeBytes);
            writtenPayloadBytes += writtenChunkBytes;
            if (writtenChunkBytes != contiguousSafeBytes)
            {
                break;
            }

            if (writtenPayloadBytes == size)
            {
                break;
            }
        }

        if (!this->writeEscapedByte(buffer[writtenPayloadBytes]))
        {
            break;
        }

        ++writtenPayloadBytes;
    }

    return writtenPayloadBytes;
}

}  // namespace lsh::bridge::transport
