/**
 * @file    virtual_device.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines compile-time capacity limits for the cached virtual device
 * model.
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

#ifndef LSH_BRIDGE_CONSTANTS_CONFIGS_VIRTUAL_DEVICE_HPP
#define LSH_BRIDGE_CONSTANTS_CONFIGS_VIRTUAL_DEVICE_HPP

#include <cstdint>

/**
 * @brief Virtual device limits compiled into the bridge.
 *
 */
namespace constants
{
namespace virtualDevice
{
#ifndef CONFIG_MAX_ACTUATORS
static constexpr const std::uint8_t MAX_ACTUATORS = 12U;  //!< Default max actuators
#else
static constexpr const std::uint8_t MAX_ACTUATORS = CONFIG_MAX_ACTUATORS;  //!< Max actuators
#endif  // CONFIG_MAX_ACTUATORS

#ifndef CONFIG_MAX_BUTTONS
static constexpr const std::uint8_t MAX_BUTTONS = 12U;  //!< Default max buttons
#else
static constexpr const std::uint8_t MAX_BUTTONS = CONFIG_MAX_BUTTONS;  //!< Max buttons
#endif  // CONFIG_MAX_BUTTONS

#ifndef CONFIG_MAX_NAME_LENGTH
static constexpr const std::uint8_t MAX_NAME_LENGTH = 4U;  //!< Default max device name length
#else
static constexpr const std::uint8_t MAX_NAME_LENGTH = CONFIG_MAX_NAME_LENGTH;  //!< Max device name length
#endif  // CONFIG_MAX_NAME_LENGTH
}  // namespace virtualDevice

}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONFIGS_VIRTUAL_DEVICE_HPP
