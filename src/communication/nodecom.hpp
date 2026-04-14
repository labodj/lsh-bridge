#ifndef LSHESP_COMMUNICATION_NODECOM_HPP
#define LSHESP_COMMUNICATION_NODECOM_HPP

#include <ArduinoJson.h>

#include "communication/mqtttopicsbuilder.hpp"
#include "constants/payloads.hpp"

class AsyncMqttClient; //!< FORWARD DECLARATION

/**
 * @brief Holds communication based on MQTT with a MQTT broker.
 *
 */
namespace NodeCom
{
    extern AsyncMqttClient *mqttClient; //!< MQTT client

    void setMqttClient(AsyncMqttClient *client); // Set MQTT client

    // Send to MQTT
    auto sendJson(const JsonDocument &jsonDoc, const char *topic, bool retain, std::uint8_t qos) -> bool; // Send a json
    auto sendJson(constants::payloads::StaticType payloadType) -> bool;                                   // Send a static Json, use default topic
} // namespace NodeCom

#endif // LSHESP_COMMUNICATION_NODECOM_HPP
