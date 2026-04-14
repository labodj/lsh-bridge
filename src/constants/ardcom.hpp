#ifndef LSHESP_CONSTANTS_ARDCOM_HPP
#define LSHESP_CONSTANTS_ARDCOM_HPP

#include <bit>
#include <cstdint>

#include <ArduinoJson.h>

#include "constants/configs/vdev.hpp"

namespace constants
{
    namespace ardCom
    {
        // --- FOR THE RAW TEXT BUFFER (stack) ---
        // EG: {"p":1,"v":2,"n":"name","a":[255,255,...],"b":[255,255,...]}
        // Formula: 34 (fixed chars) + NAME_LEN + (MAX_ACT * 4) + (MAX_BTN * 4) + 2 (terminators)
        constexpr std::uint16_t RAW_MESSAGE_MAX_SIZE = std::bit_ceil(34U + vDev::MAX_NAME_LENGTH +
                                                                     (vDev::MAX_ACTUATORS * 4) +
                                                                     (vDev::MAX_BUTTONS * 4) + 2U);

        // --- FOR ARDUINOJSON's MEMORY POOL (persistent) ---
        /*
         Sent details Json Document size, the size is computed here https://arduinojson.org/v6/assistant/
         IMPORTANT: We are assuming that all keys strings and values strings are const char *
         {"p":1,"v":2,"n":"name","a":[255,255,...],"b":[255,255,...]} -> JSON_ARRAY_SIZE(CONFIG_MAX_ACTUATORS) + JSON_ARRAY_SIZE(CONFIG_MAX_CLICKABLES) + JSON_OBJECT_SIZE(5)
                                                                       -> 8*CONFIG_MAX_ACTUATORS + 8*CONFIG_MAX_CLICKABLES + 40
        The bare minimum is 81 with no actuators nor clickables nor name
        */
        constexpr std::uint16_t JSON_RECEIVED_MAX_SIZE = std::bit_ceil(JSON_OBJECT_SIZE(5) +
                                                                       JSON_ARRAY_SIZE(vDev::MAX_ACTUATORS) +
                                                                       JSON_ARRAY_SIZE(vDev::MAX_BUTTONS) +
                                                                       13U + vDev::MAX_NAME_LENGTH);
        /*
        MQTT Received Json Document Size
        MQTT control payloads are deserialized in full. Supported incoming commands are intentionally
        small (`SET_STATE`, `SET_SINGLE_ACTUATOR`, click responses, reboot/reset/ping/service commands),
        so a compact static document is sufficient and avoids ArduinoJson filter corner cases.

        {"p":12,"s":[packedByte0,...]} -> JSON_ARRAY_SIZE(ceil(CONFIG_MAX_ACTUATORS / 8)) + JSON_OBJECT_SIZE(2) + 4
        {"p":17,"t":2,"i":255,"c":255} -> JSON_OBJECT_SIZE(4) + 4
        */
        constexpr std::uint16_t MQTT_SET_STATE_DOC_SIZE = std::bit_ceil(JSON_ARRAY_SIZE((vDev::MAX_ACTUATORS + 7U) / 8U) +
                                                                        JSON_OBJECT_SIZE(2) + 4U);
        constexpr std::uint16_t MQTT_CLICK_DOC_SIZE = std::bit_ceil(JSON_OBJECT_SIZE(4) + 4U);
        constexpr std::uint16_t MQTT_RECEIVED_DOC_MAX_SIZE =
            MQTT_SET_STATE_DOC_SIZE < MQTT_CLICK_DOC_SIZE ? MQTT_CLICK_DOC_SIZE : MQTT_SET_STATE_DOC_SIZE;
        static_assert(MQTT_RECEIVED_DOC_MAX_SIZE >= JSON_OBJECT_SIZE(4), "MQTT_RECEIVED_DOC_MAX_SIZE must fit click payloads.");

    } // namespace ardCom
} // namespace constants

#endif // LSHESP_CONSTANTS_ARDCOM_HPP
