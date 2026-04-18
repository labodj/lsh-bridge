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
 * @brief Holds the cached data model for the attached physical device.
 * @details The bridge stores the last validated topology snapshot (device name,
 *          actuator IDs and button IDs) plus the latest authoritative actuator
 *          state. Topology validity and runtime-state validity are tracked
 *          separately so stale state can be invalidated without losing the
 *          bootstrap details snapshot.
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
        buttonIds{};  //!< Stores original button IDs for cache-backed DEVICE_DETAILS replies.

    etl::bitset<constants::virtualDevice::MAX_ACTUATORS> actuatorsState{};  //!< Real-time state of all actuators.
    etl::bitset<constants::virtualDevice::MAX_ACTUATORS>
        dirtyActuators{};                 //!< Union of actuator changes not yet mirrored to Homie since the last publish window.
    bool runtimeSynchronized = false;     //!< False until a fresh authoritative state arrives from the controller.
    bool fullStatePublishPending = true;  //!< Forces a full Homie snapshot after the next authoritative state sync.

public:
    VirtualDevice() noexcept {};
    ~VirtualDevice() = default;

    // --- Deleted copy/move constructors and assignment operators to prevent accidental copies ---
    VirtualDevice(const VirtualDevice &) = delete;
    VirtualDevice(VirtualDevice &&) = delete;
    auto operator=(const VirtualDevice &) -> VirtualDevice & = delete;
    auto operator=(VirtualDevice &&) -> VirtualDevice & = delete;

    // --- Setters ---

    void setDetails(const char *deviceName,
                    const JsonArrayConst &actuatorIds,
                    const JsonArrayConst &buttonIds);  // Cache validated topology snapshot

    void setStateFromPackedBytes(const JsonArrayConst &packedBytes);  // Update authoritative actuator state from packed bytes

    void invalidateRuntimeModel() noexcept;  // Mark runtime state as stale but keep cached topology

    // --- Getters ---

    auto getName() const -> const etl::istring &;  // Get cached device name

    [[nodiscard]] auto hasCachedDetails() const noexcept -> bool;  // Return if validated details are cached

    [[nodiscard]] auto populateDetailsDocument(JsonDocument &doc) const -> bool;  // Populate a JsonDocument with cached details

    auto getActuatorId(std::uint8_t index) const -> std::uint8_t;  // Get actuator ID by index

    [[nodiscard]] auto tryGetActuatorIndex(std::uint8_t actuatorId, std::uint8_t &outIndex) const noexcept
        -> bool;  // Try to resolve an actuator index from its logical ID

    auto getTotalActuators() const noexcept -> std::uint8_t;  // Get total number of cached actuators

    auto getTotalButtons() const noexcept -> std::uint8_t;  // Get total number of cached buttons

    auto getStateByIndex(std::uint8_t index) const noexcept -> bool;  // Get actuator state by index

    auto isActuatorDirty(std::uint8_t index) const noexcept -> bool;  // Return if an actuator still needs a Homie state refresh

    void clearDirtyActuators() noexcept;  // Clear the dirty actuators set

    auto isRuntimeSynchronized() const noexcept -> bool;  // Return if runtime model is synchronized with the controller

    auto consumeFullStatePublishPending() noexcept -> bool;  // Return and clear the one-shot full-state publish flag
};

#endif  // LSH_BRIDGE_VIRTUAL_DEVICE_HPP
