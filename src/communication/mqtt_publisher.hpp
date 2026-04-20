/**
 * @file    mqtt_publisher.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the MQTT publishing helpers used by the bridge runtime.
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

#ifndef LSH_BRIDGE_COMMUNICATION_MQTT_PUBLISHER_HPP
#define LSH_BRIDGE_COMMUNICATION_MQTT_PUBLISHER_HPP

#include <cstdint>

#include <ArduinoJson.h>

#include "constants/payloads.hpp"

class AsyncMqttClient;  //!< Forward declaration of the AsyncMqttClient owned by Homie.

namespace MqttPublisher
{
void setMqttClient(AsyncMqttClient *client);

[[nodiscard]] auto sendJson(const JsonDocument &jsonDoc, const char *topic, bool retain, std::uint8_t qos) -> bool;

[[nodiscard]] auto sendJson(constants::payloads::StaticType payloadType) -> bool;
}  // namespace MqttPublisher

#endif  // LSH_BRIDGE_COMMUNICATION_MQTT_PUBLISHER_HPP
