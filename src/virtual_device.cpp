/**
 * @file    virtual_device.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the cached bridge-side controller model.
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

#include "virtual_device.hpp"

#include "constants/communication_protocol.hpp"
#include "constants/configs/virtual_device.hpp"
#include "debug/debug.hpp"

namespace
{
template <std::size_t Capacity> void copyValidatedIds(const JsonArrayConst &source, etl::vector<std::uint8_t, Capacity> &target)
{
    target.clear();
    for (const JsonVariantConst idVariant : source)
    {
        target.push_back(idVariant.as<std::uint8_t>());
    }
}

/**
 * @brief Lookup table for bit masks (8-bit).
 * @details - On AVR: Essential, avoids expensive O(i) shift operations (no barrel shifter)
 *          - On Xtensa/RISC-V (ESP32): Optional, but kept for code consistency
 *            ESP32 has barrel shifter so `1 << i` is also O(1)
 */
constexpr std::uint8_t BIT_MASK_8[8] = {0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x20U, 0x40U, 0x80U};
}  // namespace

/**
 * @brief Caches the validated bootstrap topology snapshot.
 * @details Details and runtime validity are separate concerns. Updating the
 *          cached topology invalidates the runtime state, because the next
 *          authoritative state frame must become the new baseline for that
 *          topology.
 */
void VirtualDevice::setDetails(const char *const deviceName, const JsonArrayConst &actuatorIds, const JsonArrayConst &buttonIds)
{
    DP_CONTEXT();
    DPL("↑ Name: ", deviceName);
    DPL("↑ Total actuators from JSON: ", actuatorIds.size());
    DPL("↑ Total buttons from JSON: ", buttonIds.size());

    this->name.assign(deviceName);
    this->detailsCached = true;
    this->totalActuators = static_cast<std::uint8_t>(actuatorIds.size());
    this->actuatorsState.reset();
    this->invalidateRuntimeModel();
    copyValidatedIds(actuatorIds, this->actuatorIds);
    copyValidatedIds(buttonIds, this->buttonIds);
}

/**
 * @brief Updates the state of all actuators from a bitpacked byte array.
 * @details The bridge keeps a compact authoritative bitset plus a cumulative
 *          dirty bitset. When multiple controller state frames arrive inside the
 *          same publish window, the dirty bitset keeps the union of changes so
 *          Homie still refreshes every actuator whose published state became stale.
 *
 * @param packedBytes A JsonArrayConst containing the packed bytes [byte0, byte1, ...].
 */
void VirtualDevice::setStateFromPackedBytes(const JsonArrayConst &packedBytes)
{
    DP_CONTEXT();
    // Build new state from packed bytes using optimized loop
    etl::bitset<constants::virtualDevice::MAX_ACTUATORS> newState;

    const std::uint8_t numBytes = static_cast<std::uint8_t>(packedBytes.size());
    std::uint8_t actuatorIndex = 0U;

    // Outer loop: iterate per byte (fewer iterations than per-actuator)
    for (std::uint8_t byteIndex = 0U; byteIndex < numBytes && actuatorIndex < this->totalActuators; ++byteIndex)
    {
        const std::uint8_t packedByte = packedBytes[byteIndex].as<std::uint8_t>();

        // Inner loop: unpack 8 bits from this byte
        // Using bit operations: i & 7 is equivalent to i % 8 but MUCH faster on AVR/ESP
        for (std::uint8_t bitIndex = 0U; bitIndex < 8U && actuatorIndex < this->totalActuators; ++bitIndex)
        {
            // LUT lookup: O(1), no shift needed
            newState[actuatorIndex] = (packedByte & BIT_MASK_8[bitIndex]) != 0U;
            ++actuatorIndex;
        }
    }

    const etl::bitset<constants::virtualDevice::MAX_ACTUATORS> changedBits = this->actuatorsState ^ newState;
    this->actuatorsState = newState;
    this->dirtyActuators |= changedBits;

    this->runtimeSynchronized = true;
    DPL(changedBits.count(), " actuators changed (from packed bytes)");
}

/**
 * @brief Marks the cached bridge-side model as needing a fresh authoritative sync.
 * @details The bridge keeps the last known topology and state in memory so Homie
 *          nodes remain addressable, but commands are blocked until a new full
 *          state frame confirms the model again.
 */
void VirtualDevice::invalidateRuntimeModel() noexcept
{
    this->runtimeSynchronized = false;
    this->fullStatePublishPending = true;
    this->dirtyActuators.reset();
}

/**
 * @brief Gets the device name.
 * @return A const reference to the device name string.
 */
auto VirtualDevice::getName() const -> const etl::istring &
{
    return this->name;
}

/**
 * @brief Returns whether the bridge already cached a validated details payload.
 *
 * @return true if cached details are available.
 * @return false if the topology snapshot is still missing.
 */
auto VirtualDevice::hasCachedDetails() const noexcept -> bool
{
    return this->detailsCached;
}

/**
 * @brief Build a DEVICE_DETAILS document from the cached topology snapshot.
 *
 * @param doc document to populate.
 * @return true if the document has been populated.
 * @return false if cached details are not available yet.
 */
auto VirtualDevice::populateDetailsDocument(JsonDocument &doc) const -> bool
{
    using namespace lsh::bridge::protocol;

    if (!this->detailsCached)
    {
        return false;
    }

    doc.clear();
    doc[KEY_PAYLOAD] = static_cast<std::uint8_t>(Command::DEVICE_DETAILS);
    doc[KEY_PROTOCOL_MAJOR] = WIRE_PROTOCOL_MAJOR;
    doc[KEY_NAME] = this->name.c_str();

    JsonArray actuatorIds = doc.createNestedArray(KEY_ACTUATORS_ARRAY);
    for (const std::uint8_t actuatorId : this->actuatorIds)
    {
        actuatorIds.add(actuatorId);
    }

    JsonArray buttonIds = doc.createNestedArray(KEY_BUTTONS_ARRAY);
    for (const std::uint8_t buttonId : this->buttonIds)
    {
        buttonIds.add(buttonId);
    }

    return true;
}

/**
 * @brief Gets the ID of an actuator by its index.
 * @param index The index of the actuator.
 * @return The numeric actuator ID.
 */
auto VirtualDevice::getActuatorId(std::uint8_t index) const -> std::uint8_t
{
    return this->actuatorIds[index];
}

/**
 * @brief Resolve one actuator index from its logical wire ID.
 *
 * @param actuatorId logical actuator ID.
 * @param outIndex resolved array index.
 * @return true if the actuator exists.
 * @return false if the actuator is not part of the cached topology.
 */
auto VirtualDevice::tryGetActuatorIndex(std::uint8_t actuatorId, std::uint8_t &outIndex) const noexcept -> bool
{
    for (std::uint8_t index = 0U; index < this->totalActuators; ++index)
    {
        if (this->actuatorIds[index] == actuatorId)
        {
            outIndex = index;
            return true;
        }
    }

    return false;
}

/**
 * @brief Gets the total number of configured actuators.
 * @return The total count of actuators.
 */
auto VirtualDevice::getTotalActuators() const noexcept -> std::uint8_t
{
    return this->totalActuators;
}

/**
 * @brief Gets the total number of configured buttons.
 * @return The total count of buttons.
 */
auto VirtualDevice::getTotalButtons() const noexcept -> std::uint8_t
{
    return static_cast<std::uint8_t>(this->buttonIds.size());
}

/**
 * @brief Gets the state of a single actuator by its index.
 * @param index The index of the actuator.
 * @return The boolean state of the actuator. Returns false if the index is out of bounds.
 */
auto VirtualDevice::getStateByIndex(std::uint8_t index) const noexcept -> bool
{
    if (index >= this->totalActuators)
    {
        return false;
    }
    return this->actuatorsState[index];
}

/**
 * @brief Returns whether an actuator still needs a Homie state refresh.
 */
auto VirtualDevice::isActuatorDirty(std::uint8_t index) const noexcept -> bool
{
    return index < this->totalActuators && this->dirtyActuators.test(index);
}

/**
 * @brief Clears the accumulated dirty-actuator set after a successful Homie refresh window.
 */
void VirtualDevice::clearDirtyActuators() noexcept
{
    this->dirtyActuators.reset();
}

/**
 * @brief Returns whether the cached device model is synchronized with the controller.
 */
auto VirtualDevice::isRuntimeSynchronized() const noexcept -> bool
{
    return this->runtimeSynchronized;
}

/**
 * @brief Returns and clears the one-shot flag that requests a full Homie state publish.
 */
auto VirtualDevice::consumeFullStatePublishPending() noexcept -> bool
{
    const bool pending = this->fullStatePublishPending;
    this->fullStatePublishPending = false;
    return pending;
}
