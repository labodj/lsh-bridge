/**
 * @file    button_click_types.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines the click types used by bridge-side button-related payloads.
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

#ifndef LSH_BRIDGE_CONSTANTS_BUTTON_CLICK_TYPES_HPP
#define LSH_BRIDGE_CONSTANTS_BUTTON_CLICK_TYPES_HPP

#include <cstdint>

/**
 * @brief Namespace for constants.
 */
namespace constants
{
/**
 * @brief Click types.
 *
 */
enum class ButtonClickType : std::uint8_t
{
    SHORT,      //!< Short click
    LONG,       //!< Long click
    SUPER_LONG  //!< Super long click
};
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_BUTTON_CLICK_TYPES_HPP
