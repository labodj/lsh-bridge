/**
 * @file    json_scalars.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Provides small JSON scalar validation helpers shared by bridge parsers.
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

#ifndef LSH_BRIDGE_UTILS_JSON_SCALARS_HPP
#define LSH_BRIDGE_UTILS_JSON_SCALARS_HPP

#include <cstdint>

#include <ArduinoJson.h>

namespace utils::json
{

/** @brief Validate one JSON scalar as an unsigned byte without going through floating-point conversion. */
[[nodiscard]] inline auto tryGetUint8Scalar(const JsonVariantConst value, std::uint8_t &outValue) -> bool
{
    if (value.isNull() || value.is<const char *>() || value.is<bool>() || value.is<JsonArrayConst>() || value.is<JsonObjectConst>())
    {
        return false;
    }

    if (value.is<std::uint8_t>())
    {
        outValue = value.as<std::uint8_t>();
        return true;
    }

    if (value.is<std::uint16_t>())
    {
        const auto rawValue = value.as<std::uint16_t>();
        if (rawValue > 0xFFU)
        {
            return false;
        }

        outValue = static_cast<std::uint8_t>(rawValue);
        return true;
    }

    if (value.is<std::uint32_t>())
    {
        const auto rawValue = value.as<std::uint32_t>();
        if (rawValue > 0xFFU)
        {
            return false;
        }

        outValue = static_cast<std::uint8_t>(rawValue);
        return true;
    }

    if (value.is<std::int8_t>())
    {
        const auto rawValue = value.as<std::int8_t>();
        if (rawValue < 0)
        {
            return false;
        }

        outValue = static_cast<std::uint8_t>(rawValue);
        return true;
    }

    if (value.is<std::int16_t>())
    {
        const auto rawValue = value.as<std::int16_t>();
        if (rawValue < 0 || rawValue > 0xFF)
        {
            return false;
        }

        outValue = static_cast<std::uint8_t>(rawValue);
        return true;
    }

    if (value.is<std::int32_t>())
    {
        const auto rawValue = value.as<std::int32_t>();
        if (rawValue < 0 || rawValue > 0xFF)
        {
            return false;
        }

        outValue = static_cast<std::uint8_t>(rawValue);
        return true;
    }

    return false;
}

/** @brief Validate one JSON scalar as a binary `0`/`1` state. */
[[nodiscard]] inline auto tryGetBinaryState(const JsonVariantConst value, bool &outState) -> bool
{
    std::uint8_t rawState = 0U;
    if (!tryGetUint8Scalar(value, rawState) || rawState > 1U)
    {
        return false;
    }

    outState = (rawState == 1U);
    return true;
}

}  // namespace utils::json

#endif  // LSH_BRIDGE_UTILS_JSON_SCALARS_HPP
