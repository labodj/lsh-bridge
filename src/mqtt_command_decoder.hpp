/**
 * @file    mqtt_command_decoder.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Shallow MQTT command decoder for bridge hot paths.
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

#ifndef LSH_BRIDGE_MQTT_COMMAND_DECODER_HPP
#define LSH_BRIDGE_MQTT_COMMAND_DECODER_HPP

#include <cstddef>
#include <cstdint>

#include "constants/communication_protocol.hpp"
#include "constants/controller_serial.hpp"

namespace lsh::bridge
{

enum class DecodedMqttCommandShape : std::uint8_t
{
    CommandOnly,
    SetSingleActuator,
    SetPackedState,
    Click
};

struct DecodedMqttCommand
{
    static constexpr std::uint16_t kPackedStateCapacity =
        constants::controllerSerial::MQTT_PACKED_STATE_BYTES == 0U ? 1U : constants::controllerSerial::MQTT_PACKED_STATE_BYTES;

    protocol::Command command = protocol::Command::BOOT;
    DecodedMqttCommandShape shape = DecodedMqttCommandShape::CommandOnly;
    std::uint8_t actuatorId = 0U;
    bool state = false;
    std::uint8_t packedStateLength = 0U;
    std::uint8_t packedStateBytes[kPackedStateCapacity]{};
    std::uint8_t clickType = 0U;
    std::uint8_t clickableId = 0U;
    std::uint8_t correlationId = 0U;
};

[[nodiscard]] auto decodeMqttCommandShallow(const char *payload, std::size_t payloadLength, DecodedMqttCommand &outCommand) -> bool;

}  // namespace lsh::bridge

#endif  // LSH_BRIDGE_MQTT_COMMAND_DECODER_HPP
