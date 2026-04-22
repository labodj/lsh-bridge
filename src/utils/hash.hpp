/**
 * @file    hash.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Provides the topic-hash helper used by the bridge MQTT dispatch path.
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

#ifndef LSH_BRIDGE_UTILS_HASH_HPP
#define LSH_BRIDGE_UTILS_HASH_HPP

#include <cstdint>

/**
 * @brief Computes the DJB2-style hash used for MQTT topic dispatch.
 * @details The bridge compares MQTT topics inside a callback that should stay
 *          very cheap. Pre-hashing well-known topic strings avoids repeated
 *          `strcmp()` calls. Returning `0` for null or empty strings makes
 *          invalid topic inputs easy to reject.
 *
 * @param str null-terminated string to hash.
 * @return std::uint32_t DJB2 hash, or `0` when `str` is null or empty.
 */
__attribute__((always_inline)) constexpr auto djb2_hash(const char *str) -> std::uint32_t
{
    if (str == nullptr || str[0] == '\0')
    {
        return 0U;
    }

    std::uint32_t hash = 5381U;
    unsigned char c = 0U;

    while ((c = *str++))
    {
        hash = ((hash << 5) + hash) ^ static_cast<std::uint32_t>(c);
    }

    return hash;
}

#endif  // LSH_BRIDGE_UTILS_HASH_HPP
