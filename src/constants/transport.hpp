/**
 * @file    transport.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief Defines the serial MsgPack frame transport byte contract.
 * @note Do not edit manually. Run tools/generate_lsh_protocol.py instead.
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

#ifndef LSH_BRIDGE_CONSTANTS_TRANSPORT_HPP
#define LSH_BRIDGE_CONSTANTS_TRANSPORT_HPP

#include <stdint.h>

namespace lsh::bridge
{
namespace transport
{
inline constexpr uint8_t MSGPACK_FRAME_END = 0xC0U;             //!< Delimiter byte that starts and ends each framed MsgPack payload.
inline constexpr uint8_t MSGPACK_FRAME_ESCAPE = 0xDBU;          //!< Escape marker emitted before reserved payload bytes.
inline constexpr uint8_t MSGPACK_FRAME_ESCAPED_END = 0xDCU;     //!< Escaped representation of MSGPACK_FRAME_END.
inline constexpr uint8_t MSGPACK_FRAME_ESCAPED_ESCAPE = 0xDDU;  //!< Escaped representation of MSGPACK_FRAME_ESCAPE.

}  // namespace transport
}  // namespace lsh::bridge

#endif  // LSH_BRIDGE_CONSTANTS_TRANSPORT_HPP
