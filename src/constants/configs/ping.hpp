#ifndef LSHESP_CONSTANTS_CONFIGS_PINGCONFIGS_HPP
#define LSHESP_CONSTANTS_CONFIGS_PINGCONFIGS_HPP

#include <cstdint>

/**
 * @brief Timers for ping and timeout.
 *
 */
namespace constants
{
        namespace ping
        {
#ifndef CONFIG_PING_INTERVAL_CONTROLLINO_MS
                static constexpr const std::uint16_t PING_INTERVAL_CONTROLLINO_MS = 10000U;
#else
                static constexpr const std::uint16_t PING_INTERVAL_CONTROLLINO_MS = CONFIG_PING_INTERVAL_CONTROLLINO_MS;
#endif // CONFIG_PING_INTERVAL_CONTROLLINO_MS

#ifndef CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS
                static constexpr const std::uint16_t CONNECTION_TIMEOUT_CONTROLLINO_MS = PING_INTERVAL_CONTROLLINO_MS + 200U;
#else
                static constexpr const std::uint16_t CONNECTION_TIMEOUT_CONTROLLINO_MS = CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS;
#endif    // CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS
        } // namespace pingConfigs
} // namespace constants

#endif // LSHESP_CONSTANTS_CONFIGS_PINGCONFIGS_HPP
