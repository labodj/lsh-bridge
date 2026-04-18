/**
 * @file    debug.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Provides debug and non-debug serial helpers used across lsh-bridge.
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

#ifndef LSH_BRIDGE_DEBUG_DEBUG_HPP
#define LSH_BRIDGE_DEBUG_DEBUG_HPP

#include <Arduino.h>

// Shared flag that remembers whether this module has already opened `Serial`.
// Both debug and non-debug helpers respect it so repeated calls stay harmless.

inline bool serialIsActive = false;

#ifdef LSH_DEBUG

#include <ArduinoJson.h>

#include "constants/debug_strings.hpp"
#include "debug/va_print.hpp"

inline VaPrint debugPrinter;

/**
 * @brief Debug: Print
 *
 */
template <typename T, typename... Others> constexpr inline void DP(T first, Others... others)
{
    debugPrinter.print(first, others...);
}

/**
 * @brief Debug: Print Line
 *
 */
template <typename T, typename... Others> constexpr inline void DPL(T first, Others... others)
{
    debugPrinter.print(first, others...);
    Serial.println();
}

/**
 * @brief Debug: Serial Begin
 *
 */
inline void DSB()
{
    if (!serialIsActive)
    {
        // Debug logs use their own fast serial speed so verbose tracing does
        // not become the bottleneck while inspecting bridge behavior.
        Serial.begin(500000);
        serialIsActive = true;
    }
}

/**
 * @brief Non-Debug Serial Begin, not needed when in debug mode
 *
 */
#define NDSB

/**
 * @brief Debug: Print Json (even if CONFIG_MSG_PACK has been set)
 *
 */
template <typename T> constexpr inline void DPJ(const T &json)
{
    serializeJson(json, Serial);
    Serial.println();
}

// Prints the current C++ function signature, which is often the fastest way to
// understand where a debug line came from while reading a noisy serial log.
#define DP_CONTEXT() DPL(FPSTR(debug::METHOD_CALLED), __PRETTY_FUNCTION__)

#else  // Not in debug

// In release builds every debug macro becomes a no-op, so instrumentation can
// stay in the code without affecting runtime cost or serial output.
#define DP(...)
#define DPL(...)
#define DSB()
#define DPJ(x)
#define DP_CONTEXT()

/**
 * @brief Non-Debug Serial Begin, turn on Serial when not in debug mode
 *
 */
inline void NDSB()
{
    if (!serialIsActive)
    {
        // Outside debug builds the application may still need a regular Serial
        // console, so `NDSB()` keeps that path available explicitly.
        Serial.begin(500000);
        serialIsActive = true;
    }
}

#endif  // LSH_DEBUG

#endif  // LSH_BRIDGE_DEBUG_DEBUG_HPP
