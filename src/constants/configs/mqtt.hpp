#ifndef LSHESP_CONSTANTS_CONFIGS_MQTT_HPP
#define LSHESP_CONSTANTS_CONFIGS_MQTT_HPP

#include <cstdint>

#include "utils/hash.hpp"
#include "vdev.hpp"

/**
 * @brief MQTT topics.
 *
 */
namespace constants
{
        namespace mqtt
        {
#ifndef CONFIG_MQTT_TOPIC_BASE
                static constexpr const char *MQTT_TOPIC_BASE = "LSH"; //!< Default Root topic
#else
                static constexpr const char *MQTT_TOPIC_BASE = CONFIG_MQTT_TOPIC_BASE; //!< Root topic
#endif // CONFIG_MQTT_TOPIC_BASE

#ifndef CONFIG_MQTT_TOPIC_INPUT
                static constexpr const char *MQTT_TOPIC_INPUT = "IN"; //!< Default Input topic
#else
                static constexpr const char *MQTT_TOPIC_INPUT = CONFIG_MQTT_TOPIC_INPUT; //!< Input topic
#endif // CONFIG_MQTT_TOPIC_INPUT

#ifndef CONFIG_MQTT_TOPIC_STATE
                static constexpr const char *MQTT_TOPIC_STATE = "state"; //!< Default state topic
#else
                static constexpr const char *MQTT_TOPIC_STATE = CONFIG_MQTT_TOPIC_STATE; //!< State topic
#endif // CONFIG_MQTT_TOPIC_STATE

#ifndef CONFIG_MQTT_TOPIC_CONF
                static constexpr const char *MQTT_TOPIC_CONF = "conf"; //!< Default Config topic
#else
                static constexpr const char *MQTT_TOPIC_CONF = CONFIG_MQTT_TOPIC_CONF; //!< Config topic
#endif // CONFIG_MQTT_TOPIC_CONF

#ifndef CONFIG_MQTT_TOPIC_CMD
                static constexpr const char *MQTT_TOPIC_MISC = "misc"; //!< Default Miscellaneous topic
#else
                static constexpr const char *MQTT_TOPIC_MISC = CONFIG_MQTT_TOPIC_MISC; //!< Miscellaneous topic
#endif // CONFIG_MQTT_TOPIC_CMD

#ifndef CONFIG_MQTT_TOPIC_SERVICE
                static constexpr const char *MQTT_TOPIC_SERVICE = "LSH/Node-RED/SRV"; //!< Default Service (device agnostic, like broadcast) topic
#else
                static constexpr const char *MQTT_TOPIC_SERVICE = CONFIG_MQTT_TOPIC_SERVICE; //!< Service topic
#endif // CONFIG_MQTT_TOPIC_SERVICE
                static constexpr const auto MQTT_TOPIC_SERVICE_HASH = djb2_hash(MQTT_TOPIC_SERVICE);

                static constexpr const std::uint8_t MQTT_BASE_TOPIC_LENGTH = std::char_traits<char>::length(MQTT_TOPIC_BASE) + 1U + vDev::MAX_NAME_LENGTH + 3U;       // EG: LSH/deviceName/
                static constexpr const std::uint8_t MQTT_MAX_IN_TOPIC_LENGTH = MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_INPUT) + 2U;        // EG: LSH/deviceName/IN
                static constexpr const std::uint8_t MQTT_MAX_OUT_STATE_TOPIC_LENGTH = MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_STATE) + 2U; // EG: LSH/deviceName/state
                static constexpr const std::uint8_t MQTT_MAX_OUT_CONF_TOPIC_LENGTH = MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_CONF) + 2U;   // EG: LSH/deviceName/conf
                static constexpr const std::uint8_t MQTT_MAX_OUT_MISC_TOPIC_LENGTH = MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_MISC) + 2U;   // EG: LSH/deviceName/misc

                // Compile-time checks to ensure buffer sizes are sane.
                // If these assertions fail, the build will stop with a clear error message,
                // preventing runtime buffer overflows due to misconfiguration in platformio.ini.
                static_assert(MQTT_BASE_TOPIC_LENGTH > (std::char_traits<char>::length(MQTT_TOPIC_BASE) + vDev::MAX_NAME_LENGTH),
                              "CONFIG_MQTT_BASE_TOPIC_LENGTH is too small for the base topic and device name.");

                static_assert(MQTT_MAX_IN_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_INPUT)),
                              "CONFIG_MQTT_MAX_IN_TOPIC_LENGTH is too small.");

                static_assert(MQTT_MAX_OUT_STATE_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_STATE)),
                              "CONFIG_MQTT_MAX_OUT_STATE_TOPIC_LENGTH is too small.");

                static_assert(MQTT_MAX_OUT_CONF_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_CONF)),
                              "CONFIG_MQTT_MAX_OUT_CONF_TOPIC_LENGTH is too small.");

                static_assert(MQTT_MAX_OUT_MISC_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_MISC)),
                              "CONFIG_MQTT_MAX_OUT_MISC_TOPIC_LENGTH is too small.");
        } // namespace mqttConfigs

} // namespace constants

#endif // LSHESP_CONSTANTS_CONFIGS_MQTT_HPP
