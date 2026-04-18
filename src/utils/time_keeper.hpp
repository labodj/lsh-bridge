/**
 * @file    time_keeper.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the cached time utility used across the bridge runtime.
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

#ifndef LSH_BRIDGE_UTILS_TIME_KEEPER_HPP
#define LSH_BRIDGE_UTILS_TIME_KEEPER_HPP

#include <cstdint>

#include <Arduino.h>

/**
 * @brief Time utility to keep track of time around the code.
 *
 */
namespace timeKeeper
{
extern std::uint32_t now;  //!< Cached copy of `millis()` refreshed by the main loop.

/**
 * @brief Get the time.
 *
 * @return std::uint32_t stored time (not real one).
 */
inline auto getTime() -> std::uint32_t
{
    // Reading the cached value keeps time comparisons inside hot code paths
    // consistent for the duration of one loop iteration.
    return now;
}

/**
 * @brief Update stored time to real time
 *
 */
inline void update()
{
    // The bridge refreshes this once per main-loop pass, so every component
    // that uses `getTime()` observes the same "current" tick inside that pass.
    now = millis();
}

/**
 * @brief Get the real time
 *
 * @return std::uint32_t real time
 */
inline auto getRealTime() -> std::uint32_t
{
    // Use the real hardware tick only when a path must measure the exact
    // instant "right now" instead of the loop-cached snapshot.
    return millis();
}
}  // namespace timeKeeper

#endif  // LSH_BRIDGE_UTILS_TIME_KEEPER_HPP
