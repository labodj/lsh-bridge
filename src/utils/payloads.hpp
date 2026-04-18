/**
 * @file    payloads.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Selects the generated static payload bytes for the active bridge codec.
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

#ifndef LSH_BRIDGE_UTILS_PAYLOADS_HPP
#define LSH_BRIDGE_UTILS_PAYLOADS_HPP

#include <span>
#include "constants/payloads.hpp"

namespace utils::payloads
{
/**
 * @brief Gets a span pointing to the correct pre-defined static payload bytes.
 * @details Some bridge commands are always the same (`PING_`, `ASK_STATE`,
 *          `ASK_DETAILS`, ...). Instead of rebuilding those tiny payloads at
 *          runtime, the protocol generator emits them once as compile-time
 *          byte arrays and this helper simply selects the correct table for
 *          the active codec.
 *
 * JSON payloads include the newline delimiter. MsgPack payloads are emitted
 * as raw bytes because the serial link no longer adds extra framing.
 */
template <bool IsMsgPack> [[nodiscard]] constexpr auto get(constants::payloads::StaticType type) -> std::span<const std::uint8_t>
{
    using namespace constants::payloads;

    if constexpr (IsMsgPack)
    {
        switch (type)
        {
        case StaticType::BOOT:
            return MSGPACK_BOOT_BYTES;
        case StaticType::PING_:
            return MSGPACK_PING_BYTES;
        case StaticType::ASK_DETAILS:
            return MSGPACK_ASK_DETAILS_BYTES;
        case StaticType::ASK_STATE:
            return MSGPACK_ASK_STATE_BYTES;
        case StaticType::GENERAL_FAILOVER:
            return MSGPACK_GENERAL_FAILOVER_BYTES;
        default:
            // Returning an empty span makes unsupported payload requests
            // cheap to detect at the call site without allocating anything.
            return {};
        }
    }
    else  // JSON
    {
        switch (type)
        {
        case StaticType::BOOT:
            return JSON_BOOT_BYTES;
        case StaticType::PING_:
            return JSON_PING_BYTES;
        case StaticType::ASK_DETAILS:
            return JSON_ASK_DETAILS_BYTES;
        case StaticType::ASK_STATE:
            return JSON_ASK_STATE_BYTES;
        case StaticType::GENERAL_FAILOVER:
            return JSON_GENERAL_FAILOVER_BYTES;
        default:
            // Returning an empty span makes unsupported payload requests
            // cheap to detect at the call site without allocating anything.
            return {};
        }
    }
}
}  // namespace utils::payloads

#endif  // LSH_BRIDGE_UTILS_PAYLOADS_HPP
