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
 * @brief Gets the final serial transport bytes for one compile-time static payload.
 * @details Some bridge commands are always the same (`PING_`, `ASK_STATE`,
 *          `ASK_DETAILS`, ...). Instead of rebuilding those tiny payloads at
 *          runtime, the protocol generator emits them once as compile-time
 *          byte arrays in both raw and serial-ready forms. This helper selects
 *          the serial-ready bytes for the active controller-link codec.
 */
template <bool IsMsgPack> [[nodiscard]] constexpr auto getSerial(constants::payloads::StaticType type) -> std::span<const std::uint8_t>
{
    using namespace constants::payloads;

    if constexpr (IsMsgPack)
    {
        switch (type)
        {
        case StaticType::BOOT:
            return MSGPACK_SERIAL_BOOT_BYTES;
        case StaticType::PING_:
            return MSGPACK_SERIAL_PING_BYTES;
        case StaticType::ASK_DETAILS:
            return MSGPACK_SERIAL_ASK_DETAILS_BYTES;
        case StaticType::ASK_STATE:
            return MSGPACK_SERIAL_ASK_STATE_BYTES;
        case StaticType::GENERAL_FAILOVER:
            return MSGPACK_SERIAL_GENERAL_FAILOVER_BYTES;
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
            return JSON_SERIAL_BOOT_BYTES;
        case StaticType::PING_:
            return JSON_SERIAL_PING_BYTES;
        case StaticType::ASK_DETAILS:
            return JSON_SERIAL_ASK_DETAILS_BYTES;
        case StaticType::ASK_STATE:
            return JSON_SERIAL_ASK_STATE_BYTES;
        case StaticType::GENERAL_FAILOVER:
            return JSON_SERIAL_GENERAL_FAILOVER_BYTES;
        default:
            // Returning an empty span makes unsupported payload requests
            // cheap to detect at the call site without allocating anything.
            return {};
        }
    }
}

/**
 * @brief Gets the raw MQTT transport bytes for one compile-time static payload.
 * @details MQTT carries the logical payload bytes directly, so this selector
 *          intentionally returns the raw transport form without newline
 *          delimiters or controller-link framing.
 */
template <bool IsMsgPack> [[nodiscard]] constexpr auto getMqtt(constants::payloads::StaticType type) -> std::span<const std::uint8_t>
{
    using namespace constants::payloads;

    if constexpr (IsMsgPack)
    {
        switch (type)
        {
        case StaticType::BOOT:
            return MSGPACK_RAW_BOOT_BYTES;
        case StaticType::PING_:
            return MSGPACK_RAW_PING_BYTES;
        case StaticType::ASK_DETAILS:
            return MSGPACK_RAW_ASK_DETAILS_BYTES;
        case StaticType::ASK_STATE:
            return MSGPACK_RAW_ASK_STATE_BYTES;
        case StaticType::GENERAL_FAILOVER:
            return MSGPACK_RAW_GENERAL_FAILOVER_BYTES;
        default:
            return {};
        }
    }
    else  // JSON
    {
        switch (type)
        {
        case StaticType::BOOT:
            return JSON_RAW_BOOT_BYTES;
        case StaticType::PING_:
            return JSON_RAW_PING_BYTES;
        case StaticType::ASK_DETAILS:
            return JSON_RAW_ASK_DETAILS_BYTES;
        case StaticType::ASK_STATE:
            return JSON_RAW_ASK_STATE_BYTES;
        case StaticType::GENERAL_FAILOVER:
            return JSON_RAW_GENERAL_FAILOVER_BYTES;
        default:
            return {};
        }
    }
}
}  // namespace utils::payloads

#endif  // LSH_BRIDGE_UTILS_PAYLOADS_HPP
