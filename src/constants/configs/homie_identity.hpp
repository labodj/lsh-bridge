/**
 * @file    homie_identity.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines compile-time Homie identity strings used by the bridge.
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

#ifndef LSH_BRIDGE_CONSTANTS_CONFIGS_HOMIE_IDENTITY_HPP
#define LSH_BRIDGE_CONSTANTS_CONFIGS_HOMIE_IDENTITY_HPP

#include <cstddef>

/**
 * @brief Compile-time Homie identity strings.
 * @details These macros must expand to string literals because the Homie
 *          helper macros build the legacy flagged payload format through
 *          preprocessor string-literal concatenation.
 */
namespace constants
{
namespace homieIdentity
{
static constexpr const std::size_t MAX_FIRMWARE_NAME_LENGTH = 32U;     //!< Maximum firmware name length accepted by Homie.
static constexpr const std::size_t MAX_FIRMWARE_VERSION_LENGTH = 16U;  //!< Maximum firmware version length accepted by Homie.
// Homie stores the brand in a 22-byte buffer including the null terminator.
static constexpr const std::size_t MAX_BRAND_LENGTH = 21U;  //!< Maximum brand length accepted by Homie, excluding the null terminator.

#ifndef CONFIG_HOMIE_FIRMWARE_NAME
#define CONFIG_HOMIE_FIRMWARE_NAME "lsh-homie"
#endif  // CONFIG_HOMIE_FIRMWARE_NAME

#ifndef CONFIG_HOMIE_FIRMWARE_VERSION
#define CONFIG_HOMIE_FIRMWARE_VERSION "1.1.0"
#endif  // CONFIG_HOMIE_FIRMWARE_VERSION

#ifndef CONFIG_HOMIE_BRAND
#define CONFIG_HOMIE_BRAND "LaboSmartHome"
#endif  // CONFIG_HOMIE_BRAND

static_assert(sizeof(CONFIG_HOMIE_FIRMWARE_NAME) > 1U, "CONFIG_HOMIE_FIRMWARE_NAME must not be empty.");
static_assert(sizeof(CONFIG_HOMIE_FIRMWARE_NAME) <= (MAX_FIRMWARE_NAME_LENGTH + 1U),
              "CONFIG_HOMIE_FIRMWARE_NAME exceeds the Homie firmware-name limit.");

static_assert(sizeof(CONFIG_HOMIE_FIRMWARE_VERSION) > 1U, "CONFIG_HOMIE_FIRMWARE_VERSION must not be empty.");
static_assert(sizeof(CONFIG_HOMIE_FIRMWARE_VERSION) <= (MAX_FIRMWARE_VERSION_LENGTH + 1U),
              "CONFIG_HOMIE_FIRMWARE_VERSION exceeds the Homie firmware-version limit.");

static_assert(sizeof(CONFIG_HOMIE_BRAND) > 1U, "CONFIG_HOMIE_BRAND must not be empty.");
static_assert(sizeof(CONFIG_HOMIE_BRAND) <= (MAX_BRAND_LENGTH + 1U), "CONFIG_HOMIE_BRAND exceeds the Homie brand limit.");
}  // namespace homieIdentity
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONFIGS_HOMIE_IDENTITY_HPP
