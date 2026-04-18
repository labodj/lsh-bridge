/**
 * @file    mqtt_topics_builder.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the MQTT topic builder used by the bridge runtime.
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

#ifndef LSH_BRIDGE_COMMUNICATION_MQTT_TOPICS_BUILDER_HPP
#define LSH_BRIDGE_COMMUNICATION_MQTT_TOPICS_BUILDER_HPP

#include <cstdint>

#include <etl/string.h>

#include "constants/configs/mqtt.hpp"

/**
 * @brief MQTT topics builder.
 *
 */
namespace MqttTopicsBuilder
{
extern etl::string<constants::mqtt::MQTT_BASE_TOPIC_LENGTH>
    mqttBaseTopic;  //!< Common `base/deviceName/` prefix reused by all bridge topics.
extern etl::string<constants::mqtt::MQTT_MAX_IN_TOPIC_LENGTH> mqttInTopic;  //!< Device-scoped inbound command topic.
extern std::uint32_t mqttInTopicHash;  //!< Precomputed hash of `mqttInTopic`, used inside the hot MQTT callback.
extern etl::string<constants::mqtt::MQTT_MAX_OUT_CONF_TOPIC_LENGTH>
    mqttOutConfTopic;  //!< Outbound topic that publishes validated cached details.
extern etl::string<constants::mqtt::MQTT_MAX_OUT_STATE_TOPIC_LENGTH>
    mqttOutStateTopic;  //!< Outbound topic that publishes authoritative packed actuator state.
extern etl::string<constants::mqtt::MQTT_MAX_OUT_MISC_TOPIC_LENGTH>
    mqttOutMiscTopic;  //!< Outbound topic for click events and miscellaneous controller payloads.

void updateMqttTopics(const char *deviceName);  // Rebuild every topic after the controller name becomes known
}  // namespace MqttTopicsBuilder

#endif  // LSH_BRIDGE_COMMUNICATION_MQTT_TOPICS_BUILDER_HPP
