#pragma once

#include <cstdint>
#include "constants/communication_protocol.hpp"

namespace lsh::bridge::communication
{
/**
 * @brief Write one uint8 using the shortest canonical MsgPack unsigned form.
 */
template <typename Writer> void writeMsgPackUint8(Writer &writer, std::uint8_t value)
{
    if (value <= 0x7FU)
    {
        (void)writer.write(value);
        return;
    }

    const std::uint8_t bytes[] = {0xCCU, value};
    (void)writer.write(bytes, sizeof(bytes));
}

/**
 * @brief Write a MsgPack array header for the packed state byte array.
 */
template <typename Writer> void writeMsgPackArrayPrefix(Writer &writer, std::uint8_t elementCount)
{
    if (elementCount <= 15U)
    {
        (void)writer.write(static_cast<std::uint8_t>(0x90U | elementCount));
        return;
    }

    const std::uint8_t bytes[] = {0xDCU, 0x00U, elementCount};
    (void)writer.write(bytes, sizeof(bytes));
}

template <typename Writer>
void writeClick(Writer &writer, protocol::Command command, std::uint8_t clickType, std::uint8_t clickableId, std::uint8_t correlationId)
{
    using namespace protocol;
    (void)writer.write(static_cast<std::uint8_t>(0x84U));  // fixmap, 4 key/value pairs
    (void)writer.write(static_cast<std::uint8_t>(0xA1U));
    (void)writer.write(static_cast<std::uint8_t>(KEY_PAYLOAD[0]));
    writeMsgPackUint8(writer, static_cast<std::uint8_t>(command));
    (void)writer.write(static_cast<std::uint8_t>(0xA1U));
    (void)writer.write(static_cast<std::uint8_t>(KEY_TYPE[0]));
    writeMsgPackUint8(writer, clickType);
    (void)writer.write(static_cast<std::uint8_t>(0xA1U));
    (void)writer.write(static_cast<std::uint8_t>(KEY_ID[0]));
    writeMsgPackUint8(writer, clickableId);
    (void)writer.write(static_cast<std::uint8_t>(0xA1U));
    (void)writer.write(static_cast<std::uint8_t>(KEY_CORRELATION_ID[0]));
    writeMsgPackUint8(writer, correlationId);
}

template <typename Writer, typename PackedByte> void writePackedState(Writer &writer, std::uint8_t byteCount, PackedByte packedByte)
{
    using namespace protocol;
    (void)writer.write(static_cast<std::uint8_t>(0x82U));  // fixmap, 2 key/value pairs
    (void)writer.write(static_cast<std::uint8_t>(0xA1U));
    (void)writer.write(static_cast<std::uint8_t>(KEY_PAYLOAD[0]));
    writeMsgPackUint8(writer, static_cast<std::uint8_t>(Command::SET_STATE));
    (void)writer.write(static_cast<std::uint8_t>(0xA1U));
    (void)writer.write(static_cast<std::uint8_t>(KEY_STATE[0]));
    writeMsgPackArrayPrefix(writer, byteCount);
    for (std::uint8_t byteIndex = 0U; byteIndex < byteCount; ++byteIndex)
    {
        writeMsgPackUint8(writer, packedByte(byteIndex));
    }
}
}  // namespace lsh::bridge::communication
