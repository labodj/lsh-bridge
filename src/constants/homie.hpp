/**
 * @file    homie.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines Homie string literals reused across the bridge runtime.
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

#ifndef LSH_BRIDGE_CONSTANTS_HOMIE_HPP
#define LSH_BRIDGE_CONSTANTS_HOMIE_HPP

#include <WString.h>

/**
 * @brief Homie messages.
 *
 */
namespace constants
{
namespace homie
{
static constexpr const char *BOOL_TRUE_LITERAL = "true";                   //!< Literal "true"
static constexpr const char *BOOL_FALSE_LITERAL = "false";                 //!< Literal "false"
static constexpr const char *HOMIE_PROPERTY_DATATYPE_BOOLEAN = "boolean";  //!< Homie boolean data type
static constexpr const char *HOMIE_ACTUATOR_TYPE = "switch";               //!< Homie actuator type
static constexpr const char *HOMIE_PROPERTY_ADVERTISE = "state";           //!< Homie advertise
}  // namespace homie
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_HOMIE_HPP
