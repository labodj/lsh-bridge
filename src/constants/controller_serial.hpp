/**
 * @file    controller_serial.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines buffer and document sizes for the bridge-to-controller serial link.
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

#ifndef LSH_BRIDGE_CONSTANTS_CONTROLLER_SERIAL_HPP
#define LSH_BRIDGE_CONSTANTS_CONTROLLER_SERIAL_HPP

#include <bit>
#include <cstdint>

#include <ArduinoJson.h>

#include "constants/configs/virtual_device.hpp"

namespace constants
{
namespace controllerSerial
{
namespace detail
{
/** @brief Return the MsgPack size needed to encode one unsigned integer up to `maxValue`. */
constexpr auto msgPackUnsignedSize(std::uint16_t maxValue) -> std::uint16_t
{
    if (maxValue <= 0x7FU)
    {
        return 1U;
    }

    if (maxValue <= 0xFFU)
    {
        return 2U;
    }

    return 3U;
}

/** @brief Return the MsgPack prefix size needed for one array with `elementCount` elements. */
constexpr auto msgPackArrayPrefixSize(std::uint16_t elementCount) -> std::uint16_t
{
    return elementCount <= 15U ? 1U : 3U;
}

/** @brief Return the MsgPack size needed to encode one string of `stringLength` bytes. */
constexpr auto msgPackStringSize(std::uint16_t stringLength) -> std::uint16_t
{
    if (stringLength <= 31U)
    {
        return static_cast<std::uint16_t>(1U + stringLength);
    }

    if (stringLength <= 0xFFU)
    {
        return static_cast<std::uint16_t>(2U + stringLength);
    }

    return static_cast<std::uint16_t>(3U + stringLength);
}
}  // namespace detail

/**
 * @brief Worst-case raw JSON bytes for one complete controller `DEVICE_DETAILS` payload.
 * @details JSON serial builds assemble newline-delimited text frames from this
 *          shape:
 *          Example payload:
 *          `{"p":1,"v":2,"n":"name","a":[255,255,...],"b":[255,255,...]}`
 *          Formula: `34 + NAME_LEN + (MAX_ACT * 4) + (MAX_BTN * 4) + 2`.
 */
constexpr std::uint16_t JSON_SERIAL_MESSAGE_MAX_SIZE =
    std::bit_ceil(34U + virtualDevice::MAX_NAME_LENGTH + (virtualDevice::MAX_ACTUATORS * 4) + (virtualDevice::MAX_BUTTONS * 4) + 2U);

/**
 * @brief Worst-case deframed MsgPack bytes for one complete controller `DEVICE_DETAILS` payload.
 * @details The serial framing bytes are intentionally excluded because the
 *          bridge deframes incrementally and only stores the pure MsgPack
 *          payload. Keys are one-byte protocol strings and every numeric field
 *          accepted from the controller is bounded to `uint8_t`.
 */
constexpr std::uint16_t MSGPACK_SERIAL_MESSAGE_MAX_SIZE = std::bit_ceil(
    1U +                                 // root map with 5 key/value pairs
    (5U * 2U) +                          // one-byte string keys: "p", "v", "n", "a", "b"
    detail::msgPackUnsignedSize(255U) +  // payload ID
    detail::msgPackUnsignedSize(255U) +  // protocol major
    detail::msgPackStringSize(virtualDevice::MAX_NAME_LENGTH) + detail::msgPackArrayPrefixSize(virtualDevice::MAX_ACTUATORS) +
    (virtualDevice::MAX_ACTUATORS * detail::msgPackUnsignedSize(255U)) + detail::msgPackArrayPrefixSize(virtualDevice::MAX_BUTTONS) +
    (virtualDevice::MAX_BUTTONS * detail::msgPackUnsignedSize(255U)));

/**
 * @brief Buffer size reserved for one complete inbound controller payload.
 * @details JSON serial builds use a newline-terminated text buffer. MsgPack
 *          serial builds use the same storage for the deframed binary payload.
 */
#ifdef CONFIG_MSG_PACK_ARDUINO
constexpr std::uint16_t SERIAL_RX_BUFFER_SIZE = MSGPACK_SERIAL_MESSAGE_MAX_SIZE;
#else
constexpr std::uint16_t SERIAL_RX_BUFFER_SIZE = JSON_SERIAL_MESSAGE_MAX_SIZE;
#endif

#ifdef CONFIG_MSG_PACK_ARDUINO
/**
 * @brief Worst-case raw UART bytes occupied by one complete framed MsgPack payload.
 * @details The SLIP-like transport wraps the pure payload with an opening
 *          delimiter, a closing delimiter, and may escape every payload byte
 *          into a two-byte sequence.
 */
constexpr std::uint16_t SERIAL_RX_MAX_FRAMED_MESSAGE_SIZE = static_cast<std::uint16_t>(2U + (SERIAL_RX_BUFFER_SIZE * 2U));
#endif

/**
 * @brief Number of packed bytes in one inbound MQTT `SET_STATE` command.
 */
constexpr std::uint16_t MQTT_PACKED_STATE_BYTES = (virtualDevice::MAX_ACTUATORS + 7U) / 8U;

/**
 * @brief Worst-case raw JSON bytes for one supported inbound MQTT command.
 * @details The bridge accepts only shallow control payloads from MQTT. The
 *          largest supported shape is either the packed `SET_STATE` command
 *          or one click payload carrying `p`, `t`, `i` and `c`.
 */
constexpr std::uint16_t JSON_MQTT_COMMAND_MESSAGE_MAX_SIZE = std::bit_ceil(16U + (MQTT_PACKED_STATE_BYTES * 4U));

/**
 * @brief Worst-case raw JSON bytes for one inbound MQTT click command.
 * @details Covers payloads shaped like
 *          `{"p":255,"t":255,"i":255,"c":255}`.
 */
constexpr std::uint16_t JSON_MQTT_CLICK_MESSAGE_MAX_SIZE = 32U;

/**
 * @brief Worst-case raw MsgPack bytes for one supported inbound MQTT command.
 * @details The largest supported command remains the packed `SET_STATE`
 *          payload. Keys are the one-byte protocol strings `"p"` and `"s"`.
 */
constexpr std::uint16_t MSGPACK_MQTT_COMMAND_MESSAGE_MAX_SIZE =
    std::bit_ceil(1U +                                 // root map with 2 key/value pairs
                  (2U * 2U) +                          // one-byte string keys: "p", "s"
                  detail::msgPackUnsignedSize(255U) +  // payload ID
                  detail::msgPackArrayPrefixSize(MQTT_PACKED_STATE_BYTES) + (MQTT_PACKED_STATE_BYTES * detail::msgPackUnsignedSize(255U)));

/**
 * @brief Worst-case raw MsgPack bytes for one inbound MQTT click command.
 * @details Covers payloads shaped like
 *          `{"p":17,"t":255,"i":255,"c":255}` in logical form.
 */
constexpr std::uint16_t MSGPACK_MQTT_CLICK_MESSAGE_MAX_SIZE = std::bit_ceil(1U + (4U * 2U) + (4U * detail::msgPackUnsignedSize(255U)));

/**
 * @brief Buffer size reserved for one queued inbound MQTT command payload.
 * @details This queue never stores topology publishes. It only holds
 *          controller/service commands awaiting parsing in the main loop.
 */
constexpr std::uint16_t MQTT_COMMAND_MESSAGE_MAX_SIZE = []() constexpr -> std::uint16_t
{
    constexpr std::uint16_t jsonMax = JSON_MQTT_COMMAND_MESSAGE_MAX_SIZE < JSON_MQTT_CLICK_MESSAGE_MAX_SIZE
                                          ? JSON_MQTT_CLICK_MESSAGE_MAX_SIZE
                                          : JSON_MQTT_COMMAND_MESSAGE_MAX_SIZE;
    constexpr std::uint16_t msgPackMax = MSGPACK_MQTT_COMMAND_MESSAGE_MAX_SIZE < MSGPACK_MQTT_CLICK_MESSAGE_MAX_SIZE
                                             ? MSGPACK_MQTT_CLICK_MESSAGE_MAX_SIZE
                                             : MSGPACK_MQTT_COMMAND_MESSAGE_MAX_SIZE;
    return jsonMax < msgPackMax ? msgPackMax : jsonMax;
}();

/**
 * @brief Buffer size reserved for one outbound MQTT publish payload.
 * @details Outbound publishes include retained topology snapshots, so this
 *          buffer must still fit the worst-case `DEVICE_DETAILS` payload in the
 *          active MQTT codec.
 */
#ifdef CONFIG_MSG_PACK_MQTT
constexpr std::uint16_t MQTT_PUBLISH_MESSAGE_MAX_SIZE = MSGPACK_SERIAL_MESSAGE_MAX_SIZE;
#else
constexpr std::uint16_t MQTT_PUBLISH_MESSAGE_MAX_SIZE = JSON_SERIAL_MESSAGE_MAX_SIZE;
#endif

/**
 * @brief Maximum raw serial bytes drained by one `processSerialBuffer()` call.
 * @details This fairness budget bounds how much malformed, partial or noisy
 *          UART traffic one bridge loop iteration may consume before control
 *          returns to MQTT, Homie and bootstrap work.
 */
#ifdef CONFIG_ARDCOM_SERIAL_MAX_RX_BYTES_PER_LOOP
constexpr std::uint16_t SERIAL_MAX_RX_BYTES_PER_LOOP = CONFIG_ARDCOM_SERIAL_MAX_RX_BYTES_PER_LOOP;
#else
#ifdef CONFIG_MSG_PACK_ARDUINO
constexpr std::uint16_t SERIAL_MAX_RX_BYTES_PER_LOOP = SERIAL_RX_MAX_FRAMED_MESSAGE_SIZE;
#else
constexpr std::uint16_t SERIAL_MAX_RX_BYTES_PER_LOOP = SERIAL_RX_BUFFER_SIZE;
#endif
#endif
static_assert(SERIAL_MAX_RX_BYTES_PER_LOOP > 0U, "SERIAL_MAX_RX_BYTES_PER_LOOP must be greater than zero.");

/**
 * @brief Memory pool size for the persistent document that stores received device details.
 * @details Computed for a payload shaped like
 *          `{"p":1,"v":2,"n":"name","a":[255,255,...],"b":[255,255,...]}`.
 *          The estimate assumes that keys and string values are stored as
 *          `const char *`. The bare minimum is 81 bytes even when the
 *          device has no actuators, buttons or name.
 */
constexpr std::uint16_t JSON_RECEIVED_MAX_SIZE =
    std::bit_ceil(JSON_OBJECT_SIZE(5) + JSON_ARRAY_SIZE(virtualDevice::MAX_ACTUATORS) + JSON_ARRAY_SIZE(virtualDevice::MAX_BUTTONS) + 13U +
                  virtualDevice::MAX_NAME_LENGTH);
/**
 * @brief Memory pool size for an inbound MQTT `SET_STATE` command document.
 * @details Covers payloads shaped like
 *          `{"p":12,"s":[packedByte0,...]}` where the state is bitpacked.
 */
constexpr std::uint16_t MQTT_SET_STATE_DOC_SIZE =
    std::bit_ceil(JSON_ARRAY_SIZE((virtualDevice::MAX_ACTUATORS + 7U) / 8U) + JSON_OBJECT_SIZE(2) + 4U);
/**
 * @brief Memory pool size for an inbound MQTT `SET_SINGLE_ACTUATOR` command document.
 * @details Covers payloads shaped like `{"p":13,"i":255,"s":1}`.
 */
constexpr std::uint16_t MQTT_SET_SINGLE_ACTUATOR_DOC_SIZE = std::bit_ceil(JSON_OBJECT_SIZE(3) + 4U);
/**
 * @brief Memory pool size for inbound MQTT click command documents.
 * @details Covers payloads shaped like
 *          `{"p":17,"t":2,"i":255,"c":255}`.
 */
constexpr std::uint16_t MQTT_CLICK_DOC_SIZE = std::bit_ceil(JSON_OBJECT_SIZE(4) + 4U);
/**
 * @brief Memory pool size for the first-pass MQTT command-id decode.
 * @details Most bridge-local and controller-control commands only need the
 *          `p` field. A tiny filtered document keeps those hot paths from
 *          paying for the larger `SET_STATE`/click document pool.
 */
constexpr std::uint16_t MQTT_COMMAND_ID_DOC_SIZE = std::bit_ceil(JSON_OBJECT_SIZE(1) + 4U);
/**
 * @brief Maximum memory pool size needed for any supported inbound MQTT command.
 * @details The bridge fully deserializes supported MQTT control payloads,
 *          so the static document must fit the largest known command type.
 */
constexpr std::uint16_t MQTT_RECEIVED_DOC_MAX_SIZE = []() constexpr -> std::uint16_t
{
    std::uint16_t maxSize = MQTT_SET_STATE_DOC_SIZE;
    if (maxSize < MQTT_SET_SINGLE_ACTUATOR_DOC_SIZE)
    {
        maxSize = MQTT_SET_SINGLE_ACTUATOR_DOC_SIZE;
    }
    if (maxSize < MQTT_CLICK_DOC_SIZE)
    {
        maxSize = MQTT_CLICK_DOC_SIZE;
    }
    return maxSize;
}();
static_assert(MQTT_RECEIVED_DOC_MAX_SIZE >= JSON_OBJECT_SIZE(4), "MQTT_RECEIVED_DOC_MAX_SIZE must fit click payloads.");

}  // namespace controllerSerial
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONTROLLER_SERIAL_HPP
