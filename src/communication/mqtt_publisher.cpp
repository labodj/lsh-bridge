/**
 * @file    mqtt_publisher.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the MQTT publishing helpers used by the bridge runtime.
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

#include "communication/mqtt_publisher.hpp"

#include <ArduinoJson.h>
#include <AsyncMqttClient.h>

#include "communication/mqtt_topics_builder.hpp"
#include "constants/controller_serial.hpp"
#include "debug/debug.hpp"
#include "utils/payloads.hpp"

namespace
{
AsyncMqttClient *mqttClient = nullptr;  //!< Active MQTT client bound by the bridge runtime.
}  // namespace

namespace MqttPublisher
{

/**
 * @brief Set MQTT client.
 *
 * @param client client to set.
 */
void setMqttClient(AsyncMqttClient *const client)
{
    mqttClient = client;
}

/**
 * @brief Send a Json message to MQTT.
 *
 * @param jsonDoc the json payload to send.
 * @param topic the topic.
 * @param retain if the message must be retained by MQTT broker.
 * @param qos MQTT QoS used for the publish call.
 * @return true if message is published.
 * @return false if message isn't published.
 */
auto sendJson(const JsonDocument &jsonDoc, const char *const topic, bool retain, std::uint8_t qos) -> bool
{
    DP_CONTEXT();
    DP("↑ Topic: ", topic, " | Retain: ", retain, " | Json to send: ");
    DPJ(jsonDoc);

    using constants::controllerSerial::MQTT_PUBLISH_MESSAGE_MAX_SIZE;

    if (mqttClient == nullptr || topic == nullptr || topic[0] == '\0')
    {
        return false;
    }

    char buffer[MQTT_PUBLISH_MESSAGE_MAX_SIZE];
#ifdef CONFIG_MSG_PACK_MQTT
    const size_t size = serializeMsgPack(jsonDoc, buffer, MQTT_PUBLISH_MESSAGE_MAX_SIZE);
#else
    const size_t expectedSize = measureJson(jsonDoc);
    const size_t size = serializeJson(jsonDoc, buffer, MQTT_PUBLISH_MESSAGE_MAX_SIZE);
    if (size != expectedSize)
    {
        return false;
    }
#endif  // CONFIG_MSG_PACK_MQTT
    if (size == 0U)
    {
        return false;
    }
    DPL("Json size: ", size);
    // `publish()` expects a raw byte pointer plus its exact size. `buffer`
    // already contains a contiguous serialized payload in the active codec.
    return static_cast<bool>(mqttClient->publish(topic, qos, retain, static_cast<const char *>(buffer), size));
}

/**
 * @brief Send one compile-time pre-serialized static payload to MQTT.
 *
 * @param payloadType type of the json to send.
 * @return true if message has been sent.
 * @return false if message hs not been sent.
 */
auto sendJson(constants::payloads::StaticType payloadType) -> bool
{
    if (mqttClient == nullptr || MqttTopicsBuilder::mqttOutEventsTopic.empty())
    {
        return false;
    }

    // Select the prebuilt static payload table that matches the active MQTT codec.
#ifdef CONFIG_MSG_PACK_MQTT
    constexpr bool useMsgPack = true;
#else
    constexpr bool useMsgPack = false;
#endif

    const auto payloadToSend = utils::payloads::getMqtt<useMsgPack>(payloadType);
    if (payloadToSend.empty())
    {
        return false;
    }

    // Static payloads are pre-serialized at compile time, so this path can
    // publish them directly without building a temporary JsonDocument.
    return static_cast<bool>(mqttClient->publish(MqttTopicsBuilder::mqttOutEventsTopic.c_str(), 1, false,
                                                 reinterpret_cast<const char *>(payloadToSend.data()), payloadToSend.size()));
}

}  // namespace MqttPublisher
