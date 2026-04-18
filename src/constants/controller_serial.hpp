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
/**
 * @brief Stack buffer size reserved for one complete raw controller frame.
 * @details Sized for the longest supported DEVICE_DETAILS payload plus the
 *          terminators required by the text transport.
 *          Example payload:
 *          `{"p":1,"v":2,"n":"name","a":[255,255,...],"b":[255,255,...]}`
 *          Formula: `34 + NAME_LEN + (MAX_ACT * 4) + (MAX_BTN * 4) + 2`.
 */
constexpr std::uint16_t RAW_MESSAGE_MAX_SIZE =
    std::bit_ceil(34U + virtualDevice::MAX_NAME_LENGTH + (virtualDevice::MAX_ACTUATORS * 4) + (virtualDevice::MAX_BUTTONS * 4) + 2U);

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
 * @brief Memory pool size for inbound MQTT click command documents.
 * @details Covers payloads shaped like
 *          `{"p":17,"t":2,"i":255,"c":255}`.
 */
constexpr std::uint16_t MQTT_CLICK_DOC_SIZE = std::bit_ceil(JSON_OBJECT_SIZE(4) + 4U);
/**
 * @brief Maximum memory pool size needed for any supported inbound MQTT command.
 * @details The bridge fully deserializes supported MQTT control payloads,
 *          so the static document must fit the largest known command type.
 */
constexpr std::uint16_t MQTT_RECEIVED_DOC_MAX_SIZE =
    MQTT_SET_STATE_DOC_SIZE < MQTT_CLICK_DOC_SIZE ? MQTT_CLICK_DOC_SIZE : MQTT_SET_STATE_DOC_SIZE;
static_assert(MQTT_RECEIVED_DOC_MAX_SIZE >= JSON_OBJECT_SIZE(4), "MQTT_RECEIVED_DOC_MAX_SIZE must fit click payloads.");

}  // namespace controllerSerial
}  // namespace constants

#endif  // LSH_BRIDGE_CONSTANTS_CONTROLLER_SERIAL_HPP
