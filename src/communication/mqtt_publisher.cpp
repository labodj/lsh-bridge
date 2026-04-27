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

#include "communication/checked_writer.hpp"
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
 * @brief Publish one already-serialized MQTT payload.
 *
 * @param topic destination topic.
 * @param retain MQTT retain flag.
 * @param qos MQTT QoS.
 * @param payload serialized payload bytes.
 * @param payloadSize number of bytes to publish.
 * @return true if AsyncMqttClient accepted the publish and returned a packet id.
 * @return false if the client/topic/payload is not publishable right now.
 */
auto sendRaw(const char *const topic, bool retain, std::uint8_t qos, const char *const payload, std::size_t payloadSize) -> bool
{
    if (mqttClient == nullptr || topic == nullptr || topic[0] == '\0' || payload == nullptr || payloadSize == 0U)
    {
        return false;
    }

    return mqttClient->publish(topic, qos, retain, payload, payloadSize) != 0U;
}

/**
 * @brief Serialize and publish one ArduinoJson document through the active MQTT codec.
 * @details JSON MQTT builds emit JSON text, while MsgPack MQTT builds serialize
 *          the same document model as MessagePack bytes.
 *
 * @param jsonDoc document payload to send.
 * @param topic the topic.
 * @param retain if the message must be retained by MQTT broker.
 * @param qos MQTT QoS used for the publish call.
 * @return true if message is published.
 * @return false if message isn't published.
 */
auto sendJson(const JsonDocument &jsonDoc, const char *const topic, bool retain, std::uint8_t qos) -> bool
{
    DP_CONTEXT();
    DP("↑ Topic: ", topic, " | Retain: ", retain, " | Document to send: ");
    DPJ(jsonDoc);

    using constants::controllerSerial::MQTT_PUBLISH_MESSAGE_MAX_SIZE;

    if (mqttClient == nullptr || topic == nullptr || topic[0] == '\0' || jsonDoc.overflowed())
    {
        return false;
    }

    char buffer[MQTT_PUBLISH_MESSAGE_MAX_SIZE];
    lsh::bridge::communication::FixedBufferWriter bufferWriter(buffer, MQTT_PUBLISH_MESSAGE_MAX_SIZE);
#ifdef CONFIG_MSG_PACK_MQTT
    const size_t size = serializeMsgPack(jsonDoc, bufferWriter);
#else
    const size_t size = serializeJson(jsonDoc, bufferWriter);
#endif  // CONFIG_MSG_PACK_MQTT
    if (size == 0U || bufferWriter.overflowed())
    {
        return false;
    }
    DPL("Serialized MQTT payload size: ", size);

    return sendRaw(topic, retain, qos, bufferWriter.data(), bufferWriter.size());
}

/**
 * @brief Send one compile-time pre-serialized static payload to MQTT.
 *
 * @param payloadType pre-serialized payload type to send.
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
    return sendRaw(MqttTopicsBuilder::mqttOutEventsTopic.c_str(), false, 1, reinterpret_cast<const char *>(payloadToSend.data()),
                   payloadToSend.size());
}

}  // namespace MqttPublisher
