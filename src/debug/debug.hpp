#ifndef LSHESP_DEBUG_DEBUG_HPP
#define LSHESP_DEBUG_DEBUG_HPP

#include <Arduino.h>

inline bool serialIsActive = false;

#ifdef LSH_DEBUG

#include <ArduinoJson.h>

#include "constants/debugstrings.hpp"
#include "debug/vaprint.hpp"

inline VaPrint vp;

/**
 * @brief Debug: Print
 *
 */
template <typename T, typename... Others>
constexpr inline void DP(T first, Others... others)
{
    vp.print(first, others...);
}

/**
 * @brief Debug: Print Line
 *
 */
template <typename T, typename... Others>
constexpr inline void DPL(T first, Others... others)
{
    vp.print(first, others...);
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
template <typename T>
constexpr inline void DPJ(const T &json)
{
    serializeJson(json, Serial);
    Serial.println();
}

/**
 * @brief Debug: Print Method/Function Context (auto-detects name).
 * @details Uses the compiler's __PRETTY_FUNCTION__ intrinsic to automatically
 *          print the full signature of the calling function.
 */
#define DP_CONTEXT() DPL(FPSTR(debug::METHOD_CALLED), __PRETTY_FUNCTION__)

#else // Not in debug

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
        Serial.begin(500000);
        serialIsActive = true;
    }
}

#endif // LSH_DEBUG

#endif // LSHESP_DEBUG_DEBUG_HPP
