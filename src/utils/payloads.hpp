#ifndef LSHESP_UTILS_PAYLOADS_HPP
#define LSHESP_UTILS_PAYLOADS_HPP

#include <span>
#include "constants/payloads.hpp"

namespace utils::payloads
{
    /**
     * @brief Gets a span pointing to the correct pre-defined static payload bytes.
     *
     * JSON payloads include the newline delimiter. MsgPack payloads are emitted
     * as raw bytes because the serial link no longer adds extra framing.
     */
    template <bool IsMsgPack>
    [[nodiscard]] constexpr auto get(constants::payloads::StaticType type) -> std::span<const std::uint8_t>
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
                return {};
            }
        }
        else // JSON
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
                return {};
            }
        }
    }
} // namespace utils::payloads

#endif // LSHESP_UTILS_PAYLOADS_HPP
