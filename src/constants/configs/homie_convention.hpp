/**
 * @file    homie_convention.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Enforces the Homie convention profile supported by lsh-bridge.
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

#ifndef LSH_BRIDGE_CONSTANTS_CONFIGS_HOMIE_CONVENTION_HPP
#define LSH_BRIDGE_CONSTANTS_CONFIGS_HOMIE_CONVENTION_HPP

/**
 * lsh-bridge and the current Node-RED consumer intentionally standardize on
 * Homie v5. The convention selector must be a project-wide PlatformIO build
 * flag, not a local source-level default, because the Homie dependency also
 * compiles its own translation units and they must see the same value.
 */
#ifndef HOMIE_CONVENTION_VERSION
#error "lsh-bridge requires -D HOMIE_CONVENTION_VERSION=5 in the embedding PlatformIO environment."
#endif

#if HOMIE_CONVENTION_VERSION != 5
#error "lsh-bridge supports only HOMIE_CONVENTION_VERSION=5. Update the embedding build flags."
#endif

namespace constants
{
namespace homieConvention
{
inline constexpr int HOMIE_CONVENTION_VERSION_REQUIRED = 5;  //!< Homie convention major version required by the bridge.
}  // namespace homieConvention
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONFIGS_HOMIE_CONVENTION_HPP
