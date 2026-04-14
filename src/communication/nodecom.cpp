#include "communication/nodecom.hpp"

#include <ArduinoJson.h>
#include <AsyncMqttClient.h>

#include "communication/mqtttopicsbuilder.hpp"
#include "constants/configs/ping.hpp"
#include "constants/payloads.hpp"
#include "constants/ardcom.hpp"
#include "debug/debug.hpp"
#include "utils/timekeeper.hpp"
#include "utils/payloads.hpp"

namespace NodeCom
{
    AsyncMqttClient *mqttClient = nullptr; //!< MQTT client

    /**
     * @brief Set MQTT client.
     *
     * @param mqttClient client to set.
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
     * @return true if message is published.
     * @return false if message isn't published.
     */
    auto sendJson(const JsonDocument &jsonDoc, const char *const topic, bool retain, std::uint8_t qos) -> bool
    {
        DP_CONTEXT();
        DP("↑ Topic: ", topic, " | Retain: ", retain, " | Json to send: ");
        DPJ(jsonDoc);

        using constants::ardCom::RAW_MESSAGE_MAX_SIZE;

        if (mqttClient == nullptr)
        {
            return false;
        }

        char buffer[RAW_MESSAGE_MAX_SIZE];
#ifdef CONFIG_MSG_PACK_MQTT
        const size_t size = serializeMsgPack(jsonDoc, buffer, RAW_MESSAGE_MAX_SIZE);
#else
        const size_t size = serializeJson(jsonDoc, buffer, RAW_MESSAGE_MAX_SIZE);
#endif // CONFIG_MSG_PACK_MQTT
        DPL("Json size: ", size);
        return static_cast<bool>(mqttClient->publish(topic, qos, retain, static_cast<const char *>(buffer), size));
    }

    /**
     * @brief Send a static Json to MQTT using default topic.
     *
     * @param payloadType type of the json to send.
     * @return true if message has been sent.
     * @return false if message hs not been sent.
     */
    auto sendJson(constants::payloads::StaticType payloadType) -> bool
    {
        if (mqttClient == nullptr)
        {
            return false;
        }

// Determine which payload to get at compile time
#ifdef CONFIG_MSG_PACK_MQTT
        constexpr bool useMsgPack = true;
#else
        constexpr bool useMsgPack = false;
#endif

        const auto payloadToSend = utils::payloads::get<useMsgPack>(payloadType);

        if (payloadToSend.empty())
        {
            return false;
        }

        return static_cast<bool>(
            mqttClient->publish(
                MqttTopicsBuilder::mqttOutMiscTopic.c_str(),
                1,
                false,
                reinterpret_cast<const char *>(payloadToSend.data()),
                payloadToSend.size()));
    }
} // namespace NodeCom
