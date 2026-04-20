/**
 * @file    mqtt.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines compile-time MQTT topic settings and derived buffer sizes.
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

#ifndef LSH_BRIDGE_CONSTANTS_CONFIGS_MQTT_HPP
#define LSH_BRIDGE_CONSTANTS_CONFIGS_MQTT_HPP

#include <cstdint>

#include "utils/hash.hpp"
#include "virtual_device.hpp"

/**
 * @brief MQTT topics.
 *
 */
namespace constants
{
namespace mqtt
{
#ifndef CONFIG_MQTT_TOPIC_BASE
static constexpr const char *MQTT_TOPIC_BASE = "LSH";  //!< Default Root topic
#else
static constexpr const char *MQTT_TOPIC_BASE = CONFIG_MQTT_TOPIC_BASE;  //!< Root topic
#endif  // CONFIG_MQTT_TOPIC_BASE

#ifndef CONFIG_MQTT_TOPIC_INPUT
static constexpr const char *MQTT_TOPIC_INPUT = "IN";  //!< Default Input topic
#else
static constexpr const char *MQTT_TOPIC_INPUT = CONFIG_MQTT_TOPIC_INPUT;  //!< Input topic
#endif  // CONFIG_MQTT_TOPIC_INPUT

#ifndef CONFIG_MQTT_TOPIC_STATE
static constexpr const char *MQTT_TOPIC_STATE = "state";  //!< Default state topic
#else
static constexpr const char *MQTT_TOPIC_STATE = CONFIG_MQTT_TOPIC_STATE;  //!< State topic
#endif  // CONFIG_MQTT_TOPIC_STATE

#ifndef CONFIG_MQTT_TOPIC_CONF
static constexpr const char *MQTT_TOPIC_CONF = "conf";  //!< Default Config topic
#else
static constexpr const char *MQTT_TOPIC_CONF = CONFIG_MQTT_TOPIC_CONF;  //!< Config topic
#endif  // CONFIG_MQTT_TOPIC_CONF

#ifndef CONFIG_MQTT_TOPIC_EVENTS
static constexpr const char *MQTT_TOPIC_EVENTS = "events";  //!< Default controller-backed events topic.
#else
static constexpr const char *MQTT_TOPIC_EVENTS = CONFIG_MQTT_TOPIC_EVENTS;  //!< Controller-backed events topic.
#endif  // CONFIG_MQTT_TOPIC_EVENTS

#ifndef CONFIG_MQTT_TOPIC_BRIDGE
static constexpr const char *MQTT_TOPIC_BRIDGE = "bridge";  //!< Default bridge-local runtime topic.
#else
static constexpr const char *MQTT_TOPIC_BRIDGE = CONFIG_MQTT_TOPIC_BRIDGE;  //!< Bridge-local runtime topic.
#endif  // CONFIG_MQTT_TOPIC_BRIDGE

#ifndef CONFIG_MQTT_TOPIC_SERVICE
static constexpr const char *MQTT_TOPIC_SERVICE = "LSH/Node-RED/SRV";  //!< Default Service (device agnostic, like broadcast) topic
#else
static constexpr const char *MQTT_TOPIC_SERVICE = CONFIG_MQTT_TOPIC_SERVICE;  //!< Service topic
#endif  // CONFIG_MQTT_TOPIC_SERVICE
static constexpr const auto MQTT_TOPIC_SERVICE_HASH =
    djb2_hash(MQTT_TOPIC_SERVICE);  //!< Precomputed hash of the service topic for fast MQTT dispatch.

static constexpr const std::uint8_t MQTT_BASE_TOPIC_LENGTH = std::char_traits<char>::length(MQTT_TOPIC_BASE) + 1U +
                                                             virtualDevice::MAX_NAME_LENGTH +
                                                             3U;  //!< Maximum length of the common `LSH/deviceName/` topic prefix.
static constexpr const std::uint8_t MQTT_MAX_IN_TOPIC_LENGTH =
    MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_INPUT) +
    2U;  //!< Maximum length of the inbound command topic, for example `LSH/deviceName/IN`.
static constexpr const std::uint8_t MQTT_MAX_OUT_STATE_TOPIC_LENGTH =
    MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_STATE) +
    2U;  //!< Maximum length of the outbound state topic, for example `LSH/deviceName/state`.
static constexpr const std::uint8_t MQTT_MAX_OUT_CONF_TOPIC_LENGTH =
    MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_CONF) +
    2U;  //!< Maximum length of the outbound configuration topic, for example `LSH/deviceName/conf`.
static constexpr const std::uint8_t MQTT_MAX_OUT_EVENTS_TOPIC_LENGTH =
    MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_EVENTS) +
    2U;  //!< Maximum length of the outbound controller-backed events topic, for example `LSH/deviceName/events`.
static constexpr const std::uint8_t MQTT_MAX_OUT_BRIDGE_TOPIC_LENGTH =
    MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_BRIDGE) +
    2U;  //!< Maximum length of the outbound bridge-local topic, for example `LSH/deviceName/bridge`.

// Compile-time checks to ensure buffer sizes are sane.
// If these assertions fail, the build will stop with a clear error message,
// preventing runtime buffer overflows due to misconfiguration in platformio.ini.
static_assert(MQTT_BASE_TOPIC_LENGTH > (std::char_traits<char>::length(MQTT_TOPIC_BASE) + virtualDevice::MAX_NAME_LENGTH),
              "Derived MQTT base topic length overflowed uint8_t. Shorten CONFIG_MQTT_TOPIC_BASE or CONFIG_MAX_NAME_LENGTH.");

static_assert(
    MQTT_MAX_IN_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_INPUT)),
    "Derived MQTT input topic length overflowed uint8_t. Shorten CONFIG_MQTT_TOPIC_INPUT, CONFIG_MQTT_TOPIC_BASE or CONFIG_MAX_NAME_LENGTH.");

static_assert(
    MQTT_MAX_OUT_STATE_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_STATE)),
    "Derived MQTT state topic length overflowed uint8_t. Shorten CONFIG_MQTT_TOPIC_STATE, CONFIG_MQTT_TOPIC_BASE or CONFIG_MAX_NAME_LENGTH.");

static_assert(
    MQTT_MAX_OUT_CONF_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_CONF)),
    "Derived MQTT config topic length overflowed uint8_t. Shorten CONFIG_MQTT_TOPIC_CONF, CONFIG_MQTT_TOPIC_BASE or CONFIG_MAX_NAME_LENGTH.");

static_assert(
    MQTT_MAX_OUT_EVENTS_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_EVENTS)),
    "Derived MQTT events topic length overflowed uint8_t. Shorten CONFIG_MQTT_TOPIC_EVENTS, CONFIG_MQTT_TOPIC_BASE or CONFIG_MAX_NAME_LENGTH.");

static_assert(
    MQTT_MAX_OUT_BRIDGE_TOPIC_LENGTH > (MQTT_BASE_TOPIC_LENGTH + std::char_traits<char>::length(MQTT_TOPIC_BRIDGE)),
    "Derived MQTT bridge topic length overflowed uint8_t. Shorten CONFIG_MQTT_TOPIC_BRIDGE, CONFIG_MQTT_TOPIC_BASE or CONFIG_MAX_NAME_LENGTH.");
}  // namespace mqtt

}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONFIGS_MQTT_HPP
