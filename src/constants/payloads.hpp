/**
 * @file Auto-generated from shared/lsh_protocol.json.
 * @brief Defines target-specific pre-serialized static payload bytes.
 * @note Do not edit manually. Run tools/generate_lsh_protocol.py instead.
 */

#ifndef LSHESP_CONSTANTS_PAYLOADS_HPP
#define LSHESP_CONSTANTS_PAYLOADS_HPP

#include <stdint.h>
#include <array>

namespace constants::payloads
{
    enum class StaticType : uint8_t
    {
        BOOT,
        PING_,
        ASK_DETAILS,
        ASK_STATE,
        GENERAL_FAILOVER,
    };

    // --- BOOT ---
    inline constexpr std::array<uint8_t, 8> JSON_BOOT_BYTES = {'{', '"', 'p', '"', ':', '4', '}', '\n'};
    inline constexpr std::array<uint8_t, 4> MSGPACK_BOOT_BYTES = {0x81, 0xA1, 0x70, 0x04};

    // --- PING ---
    inline constexpr std::array<uint8_t, 8> JSON_PING_BYTES = {'{', '"', 'p', '"', ':', '5', '}', '\n'};
    inline constexpr std::array<uint8_t, 4> MSGPACK_PING_BYTES = {0x81, 0xA1, 0x70, 0x05};

    // --- ASK_DETAILS ---
    inline constexpr std::array<uint8_t, 9> JSON_ASK_DETAILS_BYTES = {'{', '"', 'p', '"', ':', '1', '0', '}', '\n'};
    inline constexpr std::array<uint8_t, 4> MSGPACK_ASK_DETAILS_BYTES = {0x81, 0xA1, 0x70, 0x0A};

    // --- ASK_STATE ---
    inline constexpr std::array<uint8_t, 9> JSON_ASK_STATE_BYTES = {'{', '"', 'p', '"', ':', '1', '1', '}', '\n'};
    inline constexpr std::array<uint8_t, 4> MSGPACK_ASK_STATE_BYTES = {0x81, 0xA1, 0x70, 0x0B};

    // --- GENERAL_FAILOVER ---
    inline constexpr std::array<uint8_t, 9> JSON_GENERAL_FAILOVER_BYTES = {'{', '"', 'p', '"', ':', '1', '5', '}', '\n'};
    inline constexpr std::array<uint8_t, 4> MSGPACK_GENERAL_FAILOVER_BYTES = {0x81, 0xA1, 0x70, 0x0F};

} // namespace constants::payloads

#endif // LSHESP_CONSTANTS_PAYLOADS_HPP
