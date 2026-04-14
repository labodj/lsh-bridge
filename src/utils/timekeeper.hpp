#ifndef LSHESP_UTILS_TIMEKEEPER_HPP
#define LSHESP_UTILS_TIMEKEEPER_HPP

#include <cstdint>

#include <Arduino.h>

/**
 * @brief Time utility to keep track of time around the code.
 *
 */
namespace timeKeeper
{
    extern std::uint32_t now; //!< Stored time

    /**
     * @brief Get the time.
     *
     * @return std::uint32_t stored time (not real one).
     */
    inline auto getTime() -> std::uint32_t
    {
        return now;
    }

    /**
     * @brief Update stored time to real time
     *
     */
    inline void update()
    {
        now = millis();
    }

    /**
     * @brief Get the real time
     *
     * @return std::uint32_t real time
     */
    inline auto getRealTime() -> std::uint32_t
    {
        return millis();
    }
} // namespace timeKeeper

#endif // LSHARDUINO_TIMEKEEPER_HPP
