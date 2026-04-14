#include "communication/mqtttopicsbuilder.hpp"

#include <cstdint>

#include "constants/configs/mqtt.hpp"
#include "debug/debug.hpp"
#include "utils/hash.hpp"

namespace MqttTopicsBuilder
{
    etl::string<constants::mqtt::MQTT_BASE_TOPIC_LENGTH> mqttBaseTopic{};              //!< MQTT base topic
    etl::string<constants::mqtt::MQTT_MAX_IN_TOPIC_LENGTH> mqttInTopic{};              //!< MQTT input topic
    std::uint32_t mqttInTopicHash = 0;                                                 //!< MQTT input topic DJB2 hash
    etl::string<constants::mqtt::MQTT_MAX_OUT_CONF_TOPIC_LENGTH> mqttOutConfTopic{};   //!< MQTT output config topic
    etl::string<constants::mqtt::MQTT_MAX_OUT_STATE_TOPIC_LENGTH> mqttOutStateTopic{}; //!< MQTT output state topic
    etl::string<constants::mqtt::MQTT_MAX_OUT_MISC_TOPIC_LENGTH> mqttOutMiscTopic{};   //!< MQTT output miscellaneous topic

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
        using constants::mqtt::MQTT_TOPIC_CONF;
        using constants::mqtt::MQTT_TOPIC_INPUT;
        using constants::mqtt::MQTT_TOPIC_MISC;
        using constants::mqtt::MQTT_TOPIC_STATE;

        mqttBaseTopic.clear();
        mqttInTopic.clear();
        mqttOutConfTopic.clear();
        mqttOutStateTopic.clear();
        mqttOutMiscTopic.clear();

        mqttBaseTopic
            .append(MQTT_TOPIC_BASE)
            .append("/")
            .append(deviceName)
            .append("/");
        DPL("MQTT base topic: ", mqttBaseTopic.c_str());

        mqttInTopic
            .append(mqttBaseTopic)
            .append(MQTT_TOPIC_INPUT);
        DPL("MQTT input topic: ", mqttInTopic.c_str());

        mqttInTopicHash=djb2_hash(mqttInTopic.c_str());

        mqttOutConfTopic
            .append(mqttBaseTopic)
            .append(MQTT_TOPIC_CONF);
        DPL("MQTT output conf topic: ", mqttOutConfTopic.c_str());

        mqttOutStateTopic
            .append(mqttBaseTopic)
            .append(MQTT_TOPIC_STATE);
        DPL("MQTT output state topic: ", mqttOutStateTopic.c_str());

        mqttOutMiscTopic
            .append(mqttBaseTopic)
            .append(MQTT_TOPIC_MISC);
        DPL("MQTT output misc topic: ", mqttOutMiscTopic.c_str());
    }

} // namespace MqttTopicsBuilder