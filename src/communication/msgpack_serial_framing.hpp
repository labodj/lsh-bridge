/**
 * @file    msgpack_serial_framing.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the serial framing helpers used for MsgPack payloads on the controller link.
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

#ifndef LSH_BRIDGE_COMMUNICATION_MSGPACK_SERIAL_FRAMING_HPP
#define LSH_BRIDGE_COMMUNICATION_MSGPACK_SERIAL_FRAMING_HPP

#include <cstddef>
#include <cstdint>

#include <Arduino.h>

namespace lsh::bridge::transport
{
constexpr std::uint8_t MSGPACK_FRAME_END = 0xC0U;             //!< Delimiter that separates adjacent framed MsgPack payloads.
constexpr std::uint8_t MSGPACK_FRAME_ESCAPE = 0xDBU;          //!< Escape marker emitted before reserved bytes inside the frame body.
constexpr std::uint8_t MSGPACK_FRAME_ESCAPED_END = 0xDCU;     //!< Escaped representation of `MSGPACK_FRAME_END`.
constexpr std::uint8_t MSGPACK_FRAME_ESCAPED_ESCAPE = 0xDDU;  //!< Escaped representation of `MSGPACK_FRAME_ESCAPE`.

/**
 * @brief Result of feeding one serial byte into the MsgPack frame receiver.
 */
enum class MsgPackFrameConsumeResult : std::uint8_t
{
    Incomplete,     //!< The receiver accepted the byte, but the frame is not complete yet.
    FrameComplete,  //!< One complete deframed payload is ready in the destination buffer.
    FrameDiscarded  //!< A previously corrupted or overflowing frame has just been dropped at its terminating delimiter.
};

/**
 * @brief Incremental receiver for the SLIP-like MsgPack serial framing used on the controller link.
 */
class MsgPackFrameReceiver
{
private:
    char *const frameBuffer;              //!< External storage that receives the deframed MsgPack payload bytes.
    const std::uint16_t frameCapacity;    //!< Maximum number of payload bytes accepted before the frame is discarded.
    std::uint16_t frameLengthBytes = 0U;  //!< Number of payload bytes currently stored in `frameBuffer`.
    std::uint32_t lastByteTimeMs = 0U;    //!< Real-time timestamp of the most recent byte that touched this receiver.
    bool escapePending = false;           //!< True after an escape marker until the following escaped byte arrives.
    bool discardUntilFrameEnd = false;    //!< True while draining a corrupted frame until the next delimiter.

    [[nodiscard]] auto appendByte(std::uint8_t byte) -> bool;
    void startDiscarding() noexcept;

public:
    MsgPackFrameReceiver(char *buffer, std::uint16_t capacity) noexcept;

    void reset() noexcept;
    void resetIfIdle(std::uint32_t nowMs, std::uint32_t idleTimeoutMs) noexcept;
    [[nodiscard]] auto consumeByte(std::uint8_t byte, std::uint32_t nowMs) -> MsgPackFrameConsumeResult;
    [[nodiscard]] auto frameData() const -> const std::uint8_t *;
    [[nodiscard]] auto frameLength() const -> std::uint16_t;
};

/**
 * @brief Streaming writer that applies the MsgPack serial framing while bytes are sent to UART.
 */
class MsgPackFrameWriter final : public Print
{
private:
    HardwareSerial &serial;  //!< UART used to emit the framed payload.

    [[nodiscard]] auto writeEscapedByte(std::uint8_t byte) -> bool;

public:
    explicit MsgPackFrameWriter(HardwareSerial &serialPort) noexcept;

    auto beginFrame() -> bool;
    auto endFrame() -> bool;
    auto write(std::uint8_t byte) -> std::size_t override;
    auto write(const std::uint8_t *buffer, std::size_t size) -> std::size_t override;
};

}  // namespace lsh::bridge::transport

#endif  // LSH_BRIDGE_COMMUNICATION_MSGPACK_SERIAL_FRAMING_HPP
