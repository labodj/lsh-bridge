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

#include <array>
#include <span>
#include "constants/payloads.hpp"

namespace utils::payloads
{
namespace
{
using PayloadSpan = std::span<const std::uint8_t>;
using PayloadTable = std::array<PayloadSpan, 5U>;

[[nodiscard]] constexpr auto selectStaticPayloadSpan(constants::payloads::StaticType type, const PayloadTable &payloads)
    -> std::span<const std::uint8_t>
{
    const auto index = static_cast<std::size_t>(type);
    if (index >= payloads.size())
    {
        return {};
    }

    return payloads[index];
}
}  // namespace

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
        constexpr PayloadTable payloads = {
            MSGPACK_SERIAL_BOOT_BYTES,
            MSGPACK_SERIAL_PING_BYTES,
            MSGPACK_SERIAL_ASK_DETAILS_BYTES,
            MSGPACK_SERIAL_ASK_STATE_BYTES,
            MSGPACK_SERIAL_GENERAL_FAILOVER_BYTES,
        };
        return selectStaticPayloadSpan(type, payloads);
    }

    constexpr PayloadTable payloads = {
        JSON_SERIAL_BOOT_BYTES,
        JSON_SERIAL_PING_BYTES,
        JSON_SERIAL_ASK_DETAILS_BYTES,
        JSON_SERIAL_ASK_STATE_BYTES,
        JSON_SERIAL_GENERAL_FAILOVER_BYTES,
    };
    return selectStaticPayloadSpan(type, payloads);
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
        constexpr PayloadTable payloads = {
            MSGPACK_RAW_BOOT_BYTES,
            MSGPACK_RAW_PING_BYTES,
            MSGPACK_RAW_ASK_DETAILS_BYTES,
            MSGPACK_RAW_ASK_STATE_BYTES,
            MSGPACK_RAW_GENERAL_FAILOVER_BYTES,
        };
        return selectStaticPayloadSpan(type, payloads);
    }

    constexpr PayloadTable payloads = {
        JSON_RAW_BOOT_BYTES, JSON_RAW_PING_BYTES, JSON_RAW_ASK_DETAILS_BYTES, JSON_RAW_ASK_STATE_BYTES, JSON_RAW_GENERAL_FAILOVER_BYTES,
    };
    return selectStaticPayloadSpan(type, payloads);
}
}  // namespace utils::payloads

#endif  // LSH_BRIDGE_UTILS_PAYLOADS_HPP
