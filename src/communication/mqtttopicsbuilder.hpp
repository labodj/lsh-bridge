#ifndef LSHESP_COMMUNICATION_MQTT_TOPICS_BUILDER_HPP
#define LSHESP_COMMUNICATION_MQTT_TOPICS_BUILDER_HPP

#include <cstdint>
#include <etl/string.h>

#include "constants/configs/mqtt.hpp"

/**
 * @brief MQTT topics builder.
 *
 */
namespace MqttTopicsBuilder
{
    extern etl::string<constants::mqtt::MQTT_BASE_TOPIC_LENGTH> mqttBaseTopic;              //!< MQTT base topic
    extern etl::string<constants::mqtt::MQTT_MAX_IN_TOPIC_LENGTH> mqttInTopic;              //!< MQTT input topic
    extern std::uint32_t mqttInTopicHash;                                                   //!< MQTT input topic DJB2 hash
    extern etl::string<constants::mqtt::MQTT_MAX_OUT_CONF_TOPIC_LENGTH> mqttOutConfTopic;   //!< MQTT config output topic
    extern etl::string<constants::mqtt::MQTT_MAX_OUT_STATE_TOPIC_LENGTH> mqttOutStateTopic; //!< MQTT state output topic
    extern etl::string<constants::mqtt::MQTT_MAX_OUT_MISC_TOPIC_LENGTH> mqttOutMiscTopic;   //!< MQTT miscellaneous output topic
    
    void updateMqttTopics(const char *deviceName);                                          //!< Build MQTT topics
} // namespace MqttTopicsBuilder

#endif // LSHESP_COMMUNICATION_MQTT_TOPICS_BUILDER_HPP
