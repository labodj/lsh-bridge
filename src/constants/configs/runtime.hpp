/**
 * @file    runtime.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines compile-time runtime-policy knobs for lsh-bridge.
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

#ifndef LSH_BRIDGE_CONSTANTS_CONFIGS_RUNTIME_HPP
#define LSH_BRIDGE_CONSTANTS_CONFIGS_RUNTIME_HPP

#include <cstdint>

namespace constants
{
namespace runtime
{
#ifndef CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS
static constexpr const std::uint16_t BOOTSTRAP_REQUEST_INTERVAL_MS =
    500U;  //!< Delay between controller detail/state requests during bootstrap and resync.
#else
static constexpr const std::uint16_t BOOTSTRAP_REQUEST_INTERVAL_MS =
    CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS;  //!< Delay between controller detail/state requests during bootstrap and resync.
#endif  // CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS

#ifndef CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS
static constexpr const std::uint16_t STATE_PUBLISH_SETTLE_INTERVAL_MS =
    40U;  //!< Quiet window used before publishing cached state changes to MQTT and Homie.
#else
static constexpr const std::uint16_t STATE_PUBLISH_SETTLE_INTERVAL_MS =
    CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS;  //!< Quiet window used before publishing cached state changes to MQTT and Homie.
#endif  // CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS

#ifndef CONFIG_MQTT_COMMAND_QUEUE_CAPACITY
static constexpr const std::uint8_t MQTT_COMMAND_QUEUE_CAPACITY =
    8U;  //!< Maximum number of complete MQTT frames buffered while the serial side is busy.
#else
static constexpr const std::uint8_t MQTT_COMMAND_QUEUE_CAPACITY =
    CONFIG_MQTT_COMMAND_QUEUE_CAPACITY;  //!< Maximum number of complete MQTT frames buffered while the serial side is busy.
#endif  // CONFIG_MQTT_COMMAND_QUEUE_CAPACITY

#ifndef CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS
static constexpr const std::uint16_t ACTUATOR_COMMAND_SETTLE_INTERVAL_MS =
    50U;  //!< Quiet window used to coalesce bursts of actuator commands into one SET_STATE.
#else
static constexpr const std::uint16_t ACTUATOR_COMMAND_SETTLE_INTERVAL_MS =
    CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS;  //!< Quiet window used to coalesce bursts of actuator commands into one SET_STATE.
#endif  // CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS

#ifndef CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS
static constexpr const std::uint16_t ACTUATOR_COMMAND_MAX_PENDING_MS =
    1000U;  //!< Hard safety limit for an unstable pending actuator batch.
#else
static constexpr const std::uint16_t ACTUATOR_COMMAND_MAX_PENDING_MS =
    CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS;  //!< Hard safety limit for an unstable pending actuator batch.
#endif  // CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS

#ifndef CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT
static constexpr const std::uint16_t ACTUATOR_COMMAND_MAX_MUTATION_COUNT =
    32U;  //!< Maximum number of accepted command changes merged into one unstable actuator batch.
#else
static constexpr const std::uint16_t ACTUATOR_COMMAND_MAX_MUTATION_COUNT =
    CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT;  //!< Maximum number of accepted command changes merged into one unstable actuator batch.
#endif  // CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT

static_assert(MQTT_COMMAND_QUEUE_CAPACITY > 0U, "MQTT_COMMAND_QUEUE_CAPACITY must be at least 1.");
static_assert(ACTUATOR_COMMAND_SETTLE_INTERVAL_MS > 0U, "ACTUATOR_COMMAND_SETTLE_INTERVAL_MS must be greater than 0.");
static_assert(ACTUATOR_COMMAND_MAX_PENDING_MS > 0U, "ACTUATOR_COMMAND_MAX_PENDING_MS must be greater than 0.");
static_assert(ACTUATOR_COMMAND_MAX_MUTATION_COUNT > 0U, "ACTUATOR_COMMAND_MAX_MUTATION_COUNT must be greater than 0.");
}  // namespace runtime
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONFIGS_RUNTIME_HPP
