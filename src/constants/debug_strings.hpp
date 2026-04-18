/**
 * @file    debug_strings.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Stores PROGMEM debug string literals reused by lsh-bridge logs.
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

#ifndef LSH_BRIDGE_CONSTANTS_DEBUG_STRINGS_HPP
#define LSH_BRIDGE_CONSTANTS_DEBUG_STRINGS_HPP

#ifdef LSH_DEBUG

#include <pgmspace.h>

namespace debug
{
/// @brief Single-space separator used by debug prints.
static constexpr const char SPACE[] PROGMEM = " ";  // NOLINT
/// @brief Colon-plus-space separator used in key/value debug messages.
static constexpr const char COLON_SPACE[] PROGMEM = ": ";  // NOLINT
/// @brief Dot separator used in compact debug output.
static constexpr const char POINT[] PROGMEM = ".";  // NOLINT
/// @brief Visual divider used to separate debug fields on one line.
static constexpr const char DIVIDER[] PROGMEM = "||";  // NOLINT

/// @brief Label printed before the current execution context.
static constexpr const char METHOD_CALLED[] PROGMEM = "Context:";  // NOLINT
/// @brief Verb appended after a function or method name in debug output.
static constexpr const char CALLED[] PROGMEM = "called";  // NOLINT
/// @brief Label used when printing remaining free memory.
static constexpr const char FREE_MEMORY[] PROGMEM = "Free memory";  // NOLINT
/// @brief Label used when printing the GCC toolchain version.
static constexpr const char COMPILED_BY_GCC[] PROGMEM = "Compiled by GCC";  // NOLINT
/// @brief Label used for execution-time measurements.
static constexpr const char EXEC_TIME[] PROGMEM = "Exec time";  // NOLINT
/// @brief Label used when printing connection state.
static constexpr const char IS_CONNECTED[] PROGMEM = "Is connected";  // NOLINT
/// @brief Literal label for a button identifier.
static constexpr const char BUTTON[] PROGMEM = "Button";  // NOLINT
/// @brief Literal label for a short click type.
static constexpr const char SHORT[] PROGMEM = "short";  // NOLINT
/// @brief Literal label for a long click type.
static constexpr const char LONG[] PROGMEM = "long";  // NOLINT
/// @brief Literal label for a super-long click type.
static constexpr const char SUPER_LONG[] PROGMEM = "super long";  // NOLINT
/// @brief Verb used after a button click label.
static constexpr const char CLICKED[] PROGMEM = "clicked";  // NOLINT
/// @brief Label used when printing an ESP-side exit code.
static constexpr const char ESP_EXIT_CODE[] PROGMEM = "ESP exit code";  // NOLINT
/// @brief Label used when printing the timestamp of a received message.
static constexpr const char MESSAGE_RECEIVED_AT_TIME[] PROGMEM = "Message received at time";  // NOLINT
/// @brief Literal label for an actuator identifier.
static constexpr const char ACTUATOR[] PROGMEM = "Actuator";  // NOLINT
/// @brief Message fragment used when a requested entity is missing.
static constexpr const char DOES_NOT_EXIST[] PROGMEM = "does not exist";  // NOLINT
/// @brief Error string printed when a JsonDocument pointer is null.
static constexpr const char JSON_DOC_IS_NULL[] PROGMEM = "JsonDoc is NULL";  // NOLINT
/// @brief Label used before dumping a received JSON payload.
static constexpr const char JSON_RECEIVED[] PROGMEM = "JSON received";  // NOLINT
/// @brief Label used before dumping a sent JSON payload.
static constexpr const char JSON_SENT[] PROGMEM = "JSON sent";  // NOLINT
/// @brief Message printed when sending a ping toward the controller.
static constexpr const char SENDING_PING_TO_ESP[] PROGMEM = "Sending ping to ESP";  // NOLINT
/// @brief Message printed when forwarding a boot notification toward the controller.
static constexpr const char SENDING_BOOT_TO_ESP[] PROGMEM = "Sending boot to ESP";  // NOLINT
/// @brief Label used for UUID values.
static constexpr const char UUID[] PROGMEM = "UUID";  // NOLINT
/// @brief Label used for numeric indexes.
static constexpr const char INDEX[] PROGMEM = "Index";  // NOLINT
/// @brief Label used when printing expiration state.
static constexpr const char EXPIRED[] PROGMEM = "Expired";  // NOLINT
/// @brief Message printed before checking cached network buttons.
static constexpr const char NET_BUTTONS_NOT_EMPTY[] PROGMEM = "Network buttons not empty, checking...";  // NOLINT
/// @brief Literal label for a click event.
static constexpr const char CLICK[] PROGMEM = "Click";  // NOLINT
/// @brief Label used for click type fields.
static constexpr const char TYPE[] PROGMEM = "type";  // NOLINT
/// @brief Glue word used in short debug sentences.
static constexpr const char FOR[] PROGMEM = "for";  // NOLINT
/// @brief Label used for loop or retry counters.
static constexpr const char ITERATIONS[] PROGMEM = "iterations";  // NOLINT
}  // namespace debug

#endif  // LSH_DEBUG

#endif  // LSH_BRIDGE_CONSTANTS_DEBUG_STRINGS_HPP
