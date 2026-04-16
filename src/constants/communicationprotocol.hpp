/**
 * @file Auto-generated from shared/lsh_protocol.json.
 * @brief Defines the communication protocol contract (JSON keys and command IDs).
 * @note Do not edit manually. Run tools/generate_lsh_protocol.py instead.
 */

#ifndef LSHESP_CONSTANTS_COMMUNICATIONPROTOCOL_HPP
#define LSHESP_CONSTANTS_COMMUNICATIONPROTOCOL_HPP

#include <stdint.h>

namespace LSH
{
    namespace protocol
    {
        inline constexpr uint32_t SPEC_REVISION = 2026041601U; //!< Code-only revision, never transmitted on wire.
        inline constexpr uint8_t WIRE_PROTOCOL_MAJOR = 3U; //!< Handshake-only protocol major, transmitted only in DEVICE_DETAILS.

        // === JSON KEYS ===
        inline constexpr char KEY_PAYLOAD[] = "p";
        inline constexpr char KEY_PROTOCOL_MAJOR[] = "v";
        inline constexpr char KEY_NAME[] = "n";
        inline constexpr char KEY_ACTUATORS_ARRAY[] = "a";
        inline constexpr char KEY_BUTTONS_ARRAY[] = "b";
        inline constexpr char KEY_CORRELATION_ID[] = "c";
        inline constexpr char KEY_ID[] = "i";
        inline constexpr char KEY_STATE[] = "s";
        inline constexpr char KEY_TYPE[] = "t";

        /**
         * @brief Valid command types for the 'p' (payload) key.
         */
        enum class Command : uint8_t
        {
            DEVICE_DETAILS = 1, //!< Device details payload with handshake-only protocol major used for wire compatibility checks.
            ACTUATORS_STATE = 2, //!< Bitpacked actuator state payload.
            NETWORK_CLICK_REQUEST = 3, //!< Network click request with correlation ID.
            BOOT = 4, //!< Controller boot notification and re-sync trigger. Does not carry version metadata.
            PING_ = 5, //!< Ping or heartbeat payload.
            REQUEST_DETAILS = 10, //!< Request device details.
            REQUEST_STATE = 11, //!< Request current state.
            SET_STATE = 12, //!< Set all actuators.
            SET_SINGLE_ACTUATOR = 13, //!< Set a single actuator.
            NETWORK_CLICK_ACK = 14, //!< Acknowledge a network click with correlation ID.
            FAILOVER = 15, //!< General failover signal.
            FAILOVER_CLICK = 16, //!< Failover for a specific click with correlation ID.
            NETWORK_CLICK_CONFIRM = 17, //!< Confirm a network click after ACK using the same correlation ID.
            SYSTEM_REBOOT = 254, //!< ESP system reboot command.
            SYSTEM_RESET = 255, //!< ESP system reset command.
        };

        /**
         * @brief Valid click types for the 't' (type) key.
         */
        enum class ProtocolClickType : uint8_t
        {
            LONG = 1,
            SUPER_LONG = 2,
        };

    } // namespace protocol
} // namespace LSH

#endif // LSHESP_CONSTANTS_COMMUNICATIONPROTOCOL_HPP
