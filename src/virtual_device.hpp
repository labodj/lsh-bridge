/**
 * @file    virtual_device.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the cached bridge-side model of the attached controller device.
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

#ifndef LSH_BRIDGE_VIRTUAL_DEVICE_HPP
#define LSH_BRIDGE_VIRTUAL_DEVICE_HPP

#include <cstdint>

#include <ArduinoJson.h>
#include <etl/bitset.h>
#include <etl/string.h>
#include <etl/vector.h>

#include "constants/configs/virtual_device.hpp"

/**
 * @brief Plain validated snapshot of controller topology details.
 */
struct DeviceDetailsSnapshot
{
    etl::string<constants::virtualDevice::MAX_NAME_LENGTH> name{};                     //!< Validated controller name.
    etl::vector<std::uint8_t, constants::virtualDevice::MAX_ACTUATORS> actuatorIds{};  //!< Validated logical actuator IDs.
    etl::vector<std::uint8_t, constants::virtualDevice::MAX_BUTTONS> buttonIds{};      //!< Validated logical button IDs.

    /** @brief Reset the snapshot to an empty invalid state. */
    void clear() noexcept
    {
        name.clear();
        actuatorIds.clear();
        buttonIds.clear();
    }
};

/**
 * @brief Validated authoritative actuator-state snapshot materialized as bits.
 * @details The parser owns the responsibility of decoding and validating the
 *          inbound packed bytes. The runtime model only receives this canonical
 *          bitset form and applies it.
 */
using ActuatorStateBitset = etl::bitset<constants::virtualDevice::MAX_ACTUATORS>;

/**
 * @brief Holds the cached bridge-side model of the attached physical device.
 */
class VirtualDevice
{
private:
    // Device identity
    etl::string<constants::virtualDevice::MAX_NAME_LENGTH> name{};  //!< Device name, populated from the serial device.
    bool detailsCached = false;                                     //!< True after a validated details payload has been cached.

    // Actuators
    std::uint8_t totalActuators = 0U;  //!< Total number of actuators.

    // Stores the original actuator UUIDs. This is necessary for initializing Homie nodes,
    // building MQTT/Homie topology and mapping state indexes back to logical IDs.
    etl::vector<std::uint8_t, constants::virtualDevice::MAX_ACTUATORS> actuatorIds{};
    etl::vector<std::uint8_t, constants::virtualDevice::MAX_BUTTONS>
        buttonIds{};  //!< Stores original button IDs for cache-backed `DEVICE_DETAILS` replies.

    ActuatorStateBitset actuatorsState{};  //!< Real-time state of all actuators.
    ActuatorStateBitset dirtyActuators{};  //!< Union of actuator changes not yet mirrored to Homie since the last publish window.
    bool runtimeSynchronized = false;      //!< False until a fresh authoritative state arrives from the controller.
    bool fullStatePublishPending = true;   //!< Forces a full Homie snapshot after the next authoritative state sync.

public:
    VirtualDevice() noexcept {};
    ~VirtualDevice() = default;

    // --- Deleted copy/move constructors and assignment operators to prevent accidental copies ---
    VirtualDevice(const VirtualDevice &) = delete;
    VirtualDevice(VirtualDevice &&) = delete;
    auto operator=(const VirtualDevice &) -> VirtualDevice & = delete;
    auto operator=(VirtualDevice &&) -> VirtualDevice & = delete;

    // --- Setters ---

    void setDetails(const DeviceDetailsSnapshot &details);

    void applyAuthoritativeState(const ActuatorStateBitset &newState) noexcept;

    void invalidateRuntimeModel() noexcept;

    auto getName() const -> const etl::istring &;

    [[nodiscard]] auto hasCachedDetails() const noexcept -> bool;

    [[nodiscard]] auto populateDetailsDocument(JsonDocument &doc) const -> bool;

    [[nodiscard]] auto exportDetailsSnapshot(DeviceDetailsSnapshot &out) const noexcept -> bool;

    [[nodiscard]] auto matchesDetails(const DeviceDetailsSnapshot &details) const noexcept -> bool;

    auto getActuatorId(std::uint8_t index) const -> std::uint8_t;

    [[nodiscard]] auto tryGetActuatorIndex(std::uint8_t actuatorId, std::uint8_t &outIndex) const noexcept -> bool;

    auto getTotalActuators() const noexcept -> std::uint8_t;

    auto getTotalButtons() const noexcept -> std::uint8_t;

    auto getStateByIndex(std::uint8_t index) const noexcept -> bool;

    auto isActuatorDirty(std::uint8_t index) const noexcept -> bool;

    void clearDirtyActuators() noexcept;

    auto isRuntimeSynchronized() const noexcept -> bool;

    auto isFullStatePublishPending() const noexcept -> bool;

    void clearFullStatePublishPending() noexcept;
};

#endif  // LSH_BRIDGE_VIRTUAL_DEVICE_HPP
