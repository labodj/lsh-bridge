/**
 * @file    mqtt_topics_builder.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the MQTT topic builder used by the bridge runtime.
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

#include "communication/mqtt_topics_builder.hpp"

#include <cstdint>

#include "constants/configs/mqtt.hpp"
#include "debug/debug.hpp"
namespace MqttTopicsBuilder
{
etl::string<constants::mqtt::MQTT_BASE_TOPIC_LENGTH> mqttBaseTopic{};                 //!< MQTT base topic
etl::string<constants::mqtt::MQTT_MAX_IN_TOPIC_LENGTH> mqttInTopic{};                 //!< MQTT input topic
etl::string<constants::mqtt::MQTT_MAX_OUT_CONF_TOPIC_LENGTH> mqttOutConfTopic{};      //!< MQTT output config topic
etl::string<constants::mqtt::MQTT_MAX_OUT_STATE_TOPIC_LENGTH> mqttOutStateTopic{};    //!< MQTT output state topic
etl::string<constants::mqtt::MQTT_MAX_OUT_EVENTS_TOPIC_LENGTH> mqttOutEventsTopic{};  //!< MQTT output controller-backed events topic
etl::string<constants::mqtt::MQTT_MAX_OUT_BRIDGE_TOPIC_LENGTH> mqttOutBridgeTopic{};  //!< MQTT output bridge-local runtime topic

/**
 * @brief Builds MQTT topics.
 *
 * Eg: LSH/deviceName/InTopic, LSH/deviceName/OutTopic
 *
 * @param deviceName name of the attached device.
 */
void updateMqttTopics(const char *const deviceName)
{
    DP_CONTEXT();
    DPL("↑ Device name: ", deviceName);

    using constants::mqtt::MQTT_TOPIC_BASE;
    using constants::mqtt::MQTT_TOPIC_BRIDGE;
    using constants::mqtt::MQTT_TOPIC_CONF;
    using constants::mqtt::MQTT_TOPIC_EVENTS;
    using constants::mqtt::MQTT_TOPIC_INPUT;
    using constants::mqtt::MQTT_TOPIC_STATE;

    mqttBaseTopic.clear();
    mqttInTopic.clear();
    mqttOutConfTopic.clear();
    mqttOutStateTopic.clear();
    mqttOutEventsTopic.clear();
    mqttOutBridgeTopic.clear();

    mqttBaseTopic.append(MQTT_TOPIC_BASE).append("/").append(deviceName).append("/");
    DPL("MQTT base topic: ", mqttBaseTopic.c_str());

    mqttInTopic.append(mqttBaseTopic).append(MQTT_TOPIC_INPUT);
    DPL("MQTT input topic: ", mqttInTopic.c_str());

    mqttOutConfTopic.append(mqttBaseTopic).append(MQTT_TOPIC_CONF);
    DPL("MQTT output conf topic: ", mqttOutConfTopic.c_str());

    mqttOutStateTopic.append(mqttBaseTopic).append(MQTT_TOPIC_STATE);
    DPL("MQTT output state topic: ", mqttOutStateTopic.c_str());

    mqttOutEventsTopic.append(mqttBaseTopic).append(MQTT_TOPIC_EVENTS);
    DPL("MQTT output events topic: ", mqttOutEventsTopic.c_str());

    mqttOutBridgeTopic.append(mqttBaseTopic).append(MQTT_TOPIC_BRIDGE);
    DPL("MQTT output bridge topic: ", mqttOutBridgeTopic.c_str());
}

}  // namespace MqttTopicsBuilder
