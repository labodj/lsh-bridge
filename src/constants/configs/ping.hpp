/**
 * @file    ping.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines compile-time liveness timers for the controller link.
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

#ifndef LSH_BRIDGE_CONSTANTS_CONFIGS_PING_HPP
#define LSH_BRIDGE_CONSTANTS_CONFIGS_PING_HPP

#include <cstdint>

/**
 * @brief Timers for ping and timeout.
 *
 */
namespace constants
{
namespace ping
{
#ifndef CONFIG_PING_INTERVAL_CONTROLLINO_MS
static constexpr const std::uint16_t PING_INTERVAL_CONTROLLINO_MS = 10000U;  //!< Default ping interval
#else
static constexpr const std::uint16_t PING_INTERVAL_CONTROLLINO_MS = CONFIG_PING_INTERVAL_CONTROLLINO_MS;  //!< Ping interval
#endif  // CONFIG_PING_INTERVAL_CONTROLLINO_MS

#ifndef CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS
static constexpr const std::uint16_t CONNECTION_TIMEOUT_CONTROLLINO_MS =
    PING_INTERVAL_CONTROLLINO_MS + 200U;  //!< Default connection timeout
#else
static constexpr const std::uint16_t CONNECTION_TIMEOUT_CONTROLLINO_MS = CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS;  //!< Connection timeout
#endif  // CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS
}  // namespace ping
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONFIGS_PING_HPP
