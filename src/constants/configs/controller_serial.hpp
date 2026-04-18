/**
 * @file    controller_serial.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines compile-time serial configuration values for the controller
 * link.
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

#ifndef LSH_BRIDGE_CONSTANTS_CONFIGS_CONTROLLER_SERIAL_HPP
#define LSH_BRIDGE_CONSTANTS_CONFIGS_CONTROLLER_SERIAL_HPP

#include <cstdint>

/**
 * @brief Serial configuration used to talk with the attached controller.
 * @details The public compile-time macros keep their historical
 *          `CONFIG_ARDCOM_*` names for backward compatibility with existing
 *          PlatformIO environments, even though the internal code now uses the
 *          clearer `controllerSerial` naming.
 *
 */
namespace constants
{
namespace controllerSerial
{
#ifndef CONFIG_ARDCOM_SERIAL_RX_PIN
static constexpr const std::uint8_t ARDCOM_SERIAL_RX_PIN = 16U;  //!< Default Serial RX pin connected with Arduino
#else
static constexpr const std::uint8_t ARDCOM_SERIAL_RX_PIN = CONFIG_ARDCOM_SERIAL_RX_PIN;  //!< Serial RX pin connected with Arduino
#endif  // CONFIG_ARDCOM_SERIAL_RX_PIN

#ifndef CONFIG_ARDCOM_SERIAL_TX_PIN
static constexpr const std::uint8_t ARDCOM_SERIAL_TX_PIN = 17U;  //!< Default Serial TX pin connected with Arduino
#else
static constexpr const std::uint8_t ARDCOM_SERIAL_TX_PIN = CONFIG_ARDCOM_SERIAL_TX_PIN;  //!< Serial TX pin connected with Arduino
#endif  // CONFIG_ARDCOM_SERIAL_TX_PIN

#ifndef CONFIG_ARDCOM_SERIAL_BAUD
static constexpr const std::uint32_t ARDCOM_SERIAL_BAUD = 250000U;  //!< Default Serial baud connected with Arduino
#else
static constexpr const std::uint32_t ARDCOM_SERIAL_BAUD = CONFIG_ARDCOM_SERIAL_BAUD;  //!< Serial baud connected with Arduino
#endif  // CONFIG_ARDCOM_SERIAL_BAUD

#ifndef CONFIG_ARDCOM_SERIAL_TIMEOUT_MS
static constexpr const std::uint8_t ARDCOM_SERIAL_TIMEOUT_MS = 5U;  //!< Default Serial connected with Arduino timeout
#else
static constexpr const std::uint8_t ARDCOM_SERIAL_TIMEOUT_MS = CONFIG_ARDCOM_SERIAL_TIMEOUT_MS;  //!< Serial connected with Arduino timeout
#endif  // CONFIG_ARDCOM_SERIAL_TIMEOUT_MS
}  // namespace controllerSerial
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONFIGS_CONTROLLER_SERIAL_HPP
