/**
 * @file    mqtt_command_decoder.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements shallow MQTT command decoding for bridge hot paths.
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

#include "mqtt_command_decoder.hpp"

namespace lsh::bridge
{
namespace
{
struct DecodedFields
{
    bool hasCommand = false;
    bool hasId = false;
    bool hasState = false;
    bool hasPackedState = false;
    bool hasType = false;
    bool hasCorrelationId = false;
};

#ifndef CONFIG_MSG_PACK_MQTT
class JsonCursor
{
public:
    JsonCursor(const char *payload, std::size_t payloadLength) noexcept : current(payload), end(payload + payloadLength)
    {}

    void skipWhitespace() noexcept
    {
        while (current < end && (*current == ' ' || *current == '\n' || *current == '\r' || *current == '\t'))
        {
            ++current;
        }
    }

    [[nodiscard]] auto consume(char expected) noexcept -> bool
    {
        skipWhitespace();
        if (current >= end || *current != expected)
        {
            return false;
        }
        ++current;
        return true;
    }

    [[nodiscard]] auto peek(char expected) noexcept -> bool
    {
        skipWhitespace();
        return current < end && *current == expected;
    }

    [[nodiscard]] auto readOneByteKey(char &outKey) noexcept -> bool
    {
        skipWhitespace();
        if (current >= end || *current != '"')
        {
            return false;
        }
        ++current;
        if (current >= end)
        {
            return false;
        }
        outKey = *current++;
        if (current >= end || *current != '"')
        {
            return false;
        }
        ++current;
        return true;
    }

    [[nodiscard]] auto readUint8(std::uint8_t &outValue) noexcept -> bool
    {
        skipWhitespace();
        if (current >= end || *current < '0' || *current > '9')
        {
            return false;
        }

        std::uint16_t value = 0U;
        do
        {
            value = static_cast<std::uint16_t>((value * 10U) + static_cast<std::uint16_t>(*current - '0'));
            if (value > 255U)
            {
                return false;
            }
            ++current;
        } while (current < end && *current >= '0' && *current <= '9');

        outValue = static_cast<std::uint8_t>(value);
        return true;
    }

    [[nodiscard]] auto readBinaryState(bool &outState) noexcept -> bool
    {
        skipWhitespace();
        if (remainingStartsWith("true"))
        {
            current += 4;
            outState = true;
            return true;
        }
        if (remainingStartsWith("false"))
        {
            current += 5;
            outState = false;
            return true;
        }

        std::uint8_t rawState = 0U;
        if (!readUint8(rawState) || rawState > 1U)
        {
            return false;
        }
        outState = rawState != 0U;
        return true;
    }

    [[nodiscard]] auto readPackedState(std::uint8_t *outBytes, std::uint8_t &outLength) noexcept -> bool
    {
        if (!consume('['))
        {
            return false;
        }

        outLength = 0U;
        skipWhitespace();
        if (consume(']'))
        {
            return true;
        }

        while (true)
        {
            if (outLength >= constants::controllerSerial::MQTT_PACKED_STATE_BYTES)
            {
                return false;
            }

            std::uint8_t value = 0U;
            if (!readUint8(value))
            {
                return false;
            }
            outBytes[outLength++] = value;

            skipWhitespace();
            if (consume(']'))
            {
                return true;
            }
            if (!consume(','))
            {
                return false;
            }
        }
    }

    [[nodiscard]] auto atEnd() noexcept -> bool
    {
        skipWhitespace();
        return current == end;
    }

private:
    const char *current = nullptr;
    const char *end = nullptr;

    [[nodiscard]] auto remainingStartsWith(const char *literal) const noexcept -> bool
    {
        const char *cursor = current;
        while (*literal != '\0')
        {
            if (cursor >= end || *cursor != *literal)
            {
                return false;
            }
            ++cursor;
            ++literal;
        }
        return true;
    }
};

[[nodiscard]] auto decodeJsonMqttCommand(const char *payload, std::size_t payloadLength, DecodedMqttCommand &outCommand) -> bool
{
    using namespace protocol;

    JsonCursor cursor(payload, payloadLength);
    DecodedFields fields{};
    DecodedMqttCommand command{};

    if (!cursor.consume('{'))
    {
        return false;
    }

    if (cursor.peek('}'))
    {
        return false;
    }

    while (true)
    {
        char key = '\0';
        if (!cursor.readOneByteKey(key) || !cursor.consume(':'))
        {
            return false;
        }

        switch (key)
        {
        case KEY_PAYLOAD[0]:
        {
            std::uint8_t rawCommand = 0U;
            if (!cursor.readUint8(rawCommand))
            {
                return false;
            }
            command.command = static_cast<Command>(rawCommand);
            fields.hasCommand = true;
            break;
        }

        case KEY_ID[0]:
            if (!cursor.readUint8(command.actuatorId))
            {
                return false;
            }
            command.clickableId = command.actuatorId;
            fields.hasId = true;
            break;

        case KEY_STATE[0]:
            if (cursor.peek('['))
            {
                if (!cursor.readPackedState(command.packedStateBytes, command.packedStateLength))
                {
                    return false;
                }
                fields.hasPackedState = true;
            }
            else if (cursor.readBinaryState(command.state))
            {
                fields.hasState = true;
            }
            else
            {
                return false;
            }
            break;

        case KEY_TYPE[0]:
            if (!cursor.readUint8(command.clickType))
            {
                return false;
            }
            fields.hasType = true;
            break;

        case KEY_CORRELATION_ID[0]:
            if (!cursor.readUint8(command.correlationId))
            {
                return false;
            }
            fields.hasCorrelationId = true;
            break;

        default:
            return false;
        }

        if (cursor.consume('}'))
        {
            break;
        }
        if (!cursor.consume(','))
        {
            return false;
        }
    }

    if (!fields.hasCommand || !cursor.atEnd())
    {
        return false;
    }

    if (command.command == Command::SET_SINGLE_ACTUATOR && fields.hasId && fields.hasState)
    {
        command.shape = DecodedMqttCommandShape::SetSingleActuator;
    }
    else if (command.command == Command::SET_STATE && fields.hasPackedState)
    {
        command.shape = DecodedMqttCommandShape::SetPackedState;
    }
    else if ((command.command == Command::NETWORK_CLICK_ACK || command.command == Command::FAILOVER_CLICK) && fields.hasType &&
             fields.hasId && fields.hasCorrelationId)
    {
        command.shape = DecodedMqttCommandShape::Click;
    }

    outCommand = command;
    return true;
}
#else
class MsgPackCursor
{
public:
    MsgPackCursor(const char *payload, std::size_t payloadLength) noexcept :
        current(reinterpret_cast<const std::uint8_t *>(payload)), end(reinterpret_cast<const std::uint8_t *>(payload) + payloadLength)
    {}

    [[nodiscard]] auto readMapSize(std::uint8_t &outSize) noexcept -> bool
    {
        std::uint8_t marker = 0U;
        if (!readByte(marker))
        {
            return false;
        }
        if ((marker & 0xF0U) == 0x80U)
        {
            outSize = static_cast<std::uint8_t>(marker & 0x0FU);
            return true;
        }
        if (marker == 0xDEU)
        {
            std::uint16_t size = 0U;
            if (!readUint16(size) || size > 255U)
            {
                return false;
            }
            outSize = static_cast<std::uint8_t>(size);
            return true;
        }
        return false;
    }

    [[nodiscard]] auto readOneByteKey(char &outKey) noexcept -> bool
    {
        std::uint8_t marker = 0U;
        if (!readByte(marker))
        {
            return false;
        }

        std::uint8_t length = 0U;
        if ((marker & 0xE0U) == 0xA0U)
        {
            length = static_cast<std::uint8_t>(marker & 0x1FU);
        }
        else if (marker == 0xD9U)
        {
            if (!readByte(length))
            {
                return false;
            }
        }
        else
        {
            return false;
        }

        if (length != 1U || current >= end)
        {
            return false;
        }
        outKey = static_cast<char>(*current++);
        return true;
    }

    [[nodiscard]] auto readUint8(std::uint8_t &outValue) noexcept -> bool
    {
        std::uint8_t marker = 0U;
        if (!readByte(marker))
        {
            return false;
        }

        if (marker <= 0x7FU)
        {
            outValue = marker;
            return true;
        }
        if (marker == 0xCCU)
        {
            return readByte(outValue);
        }
        if (marker == 0xCDU)
        {
            std::uint16_t value = 0U;
            if (!readUint16(value) || value > 255U)
            {
                return false;
            }
            outValue = static_cast<std::uint8_t>(value);
            return true;
        }
        return false;
    }

    [[nodiscard]] auto readBinaryState(bool &outState) noexcept -> bool
    {
        std::uint8_t marker = 0U;
        if (!peekByte(marker))
        {
            return false;
        }
        if (marker == 0xC2U || marker == 0xC3U)
        {
            (void)readByte(marker);
            outState = marker == 0xC3U;
            return true;
        }

        std::uint8_t rawState = 0U;
        if (!readUint8(rawState) || rawState > 1U)
        {
            return false;
        }
        outState = rawState != 0U;
        return true;
    }

    [[nodiscard]] auto readPackedState(std::uint8_t *outBytes, std::uint8_t &outLength) noexcept -> bool
    {
        std::uint8_t marker = 0U;
        if (!readByte(marker))
        {
            return false;
        }

        std::uint8_t length = 0U;
        if ((marker & 0xF0U) == 0x90U)
        {
            length = static_cast<std::uint8_t>(marker & 0x0FU);
        }
        else if (marker == 0xDCU)
        {
            std::uint16_t size = 0U;
            if (!readUint16(size) || size > constants::controllerSerial::MQTT_PACKED_STATE_BYTES)
            {
                return false;
            }
            length = static_cast<std::uint8_t>(size);
        }
        else
        {
            return false;
        }

        if (length > constants::controllerSerial::MQTT_PACKED_STATE_BYTES)
        {
            return false;
        }

        outLength = length;
        for (std::uint8_t index = 0U; index < length; ++index)
        {
            if (!readUint8(outBytes[index]))
            {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] auto nextValueIsArray() const noexcept -> bool
    {
        std::uint8_t marker = 0U;
        return peekByte(marker) && (((marker & 0xF0U) == 0x90U) || marker == 0xDCU);
    }

    [[nodiscard]] auto atEnd() const noexcept -> bool
    {
        return current == end;
    }

private:
    const std::uint8_t *current = nullptr;
    const std::uint8_t *end = nullptr;

    [[nodiscard]] auto readByte(std::uint8_t &outByte) noexcept -> bool
    {
        if (current >= end)
        {
            return false;
        }
        outByte = *current++;
        return true;
    }

    [[nodiscard]] auto peekByte(std::uint8_t &outByte) const noexcept -> bool
    {
        if (current >= end)
        {
            return false;
        }
        outByte = *current;
        return true;
    }

    [[nodiscard]] auto readUint16(std::uint16_t &outValue) noexcept -> bool
    {
        if (static_cast<std::size_t>(end - current) < 2U)
        {
            return false;
        }
        outValue = static_cast<std::uint16_t>((static_cast<std::uint16_t>(current[0]) << 8U) | current[1]);
        current += 2;
        return true;
    }
};

[[nodiscard]] auto decodeMsgPackMqttCommand(const char *payload, std::size_t payloadLength, DecodedMqttCommand &outCommand) -> bool
{
    using namespace protocol;

    MsgPackCursor cursor(payload, payloadLength);
    DecodedFields fields{};
    DecodedMqttCommand command{};
    std::uint8_t mapSize = 0U;
    if (!cursor.readMapSize(mapSize) || mapSize == 0U)
    {
        return false;
    }

    for (std::uint8_t fieldIndex = 0U; fieldIndex < mapSize; ++fieldIndex)
    {
        char key = '\0';
        if (!cursor.readOneByteKey(key))
        {
            return false;
        }

        switch (key)
        {
        case KEY_PAYLOAD[0]:
        {
            std::uint8_t rawCommand = 0U;
            if (!cursor.readUint8(rawCommand))
            {
                return false;
            }
            command.command = static_cast<Command>(rawCommand);
            fields.hasCommand = true;
            break;
        }

        case KEY_ID[0]:
            if (!cursor.readUint8(command.actuatorId))
            {
                return false;
            }
            command.clickableId = command.actuatorId;
            fields.hasId = true;
            break;

        case KEY_STATE[0]:
            if (cursor.nextValueIsArray())
            {
                if (!cursor.readPackedState(command.packedStateBytes, command.packedStateLength))
                {
                    return false;
                }
                fields.hasPackedState = true;
            }
            else
            {
                if (!cursor.readBinaryState(command.state))
                {
                    return false;
                }
                fields.hasState = true;
            }
            break;

        case KEY_TYPE[0]:
            if (!cursor.readUint8(command.clickType))
            {
                return false;
            }
            fields.hasType = true;
            break;

        case KEY_CORRELATION_ID[0]:
            if (!cursor.readUint8(command.correlationId))
            {
                return false;
            }
            fields.hasCorrelationId = true;
            break;

        default:
            return false;
        }
    }

    if (!fields.hasCommand || !cursor.atEnd())
    {
        return false;
    }

    if (command.command == Command::SET_SINGLE_ACTUATOR && fields.hasId && fields.hasState)
    {
        command.shape = DecodedMqttCommandShape::SetSingleActuator;
    }
    else if (command.command == Command::SET_STATE && fields.hasPackedState)
    {
        command.shape = DecodedMqttCommandShape::SetPackedState;
    }
    else if ((command.command == Command::NETWORK_CLICK_ACK || command.command == Command::FAILOVER_CLICK) && fields.hasType &&
             fields.hasId && fields.hasCorrelationId)
    {
        command.shape = DecodedMqttCommandShape::Click;
    }

    outCommand = command;
    return true;
}
#endif
}  // namespace

auto decodeMqttCommandShallow(const char *payload, std::size_t payloadLength, DecodedMqttCommand &outCommand) -> bool
{
    if (payload == nullptr || payloadLength == 0U)
    {
        return false;
    }

#ifdef CONFIG_MSG_PACK_MQTT
    return decodeMsgPackMqttCommand(payload, payloadLength, outCommand);
#else
    return decodeJsonMqttCommand(payload, payloadLength, outCommand);
#endif
}

}  // namespace lsh::bridge
