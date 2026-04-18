/**
 * @file    conversions.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Provides tiny literal-conversion helpers used by Homie-facing bridge code.
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

#ifndef LSH_BRIDGE_UTILS_CONVERSIONS_HPP
#define LSH_BRIDGE_UTILS_CONVERSIONS_HPP

#include "constants/homie.hpp"

namespace utils
{
namespace conversions
{
/**
 * @brief Gets the literal "true" or "false" C-style string at compile-time.
 * @details This branch-free function is highly efficient and can be resolved
 *          by the compiler during compilation if the input is known.
 * @param state The boolean state to convert.
 * @return A const char pointer to either "true" or "false".
 */
constexpr auto to_literal(bool state) -> const char *
{
    return state ? constants::homie::BOOL_TRUE_LITERAL : constants::homie::BOOL_FALSE_LITERAL;
}

}  // namespace conversions
}  // namespace utils

#endif  // LSH_BRIDGE_UTILS_CONVERSIONS_HPP
