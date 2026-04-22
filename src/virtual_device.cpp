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
/**
 * @brief Copy one stored ETL ID vector into another ETL vector of the same capacity.
 *
 * @tparam Capacity ETL vector capacity.
 * @param source cached ID vector to copy from.
 * @param target destination vector to rewrite.
 */
template <std::size_t Capacity>
void copyStoredIds(const etl::vector<std::uint8_t, Capacity> &source, etl::vector<std::uint8_t, Capacity> &target)
{
    target.clear();
    for (const std::uint8_t id : source)
    {
        target.push_back(id);
    }
}
}  // namespace

/**
 * @brief Caches the validated bootstrap topology snapshot.
 * @details Details and runtime validity are separate concerns. Updating the
 *          cached topology invalidates the runtime state, because the next
 *          authoritative state frame must become the new baseline for that
 *          topology.
 *
 * @param details plain validated topology snapshot that becomes the new cached
 *        bridge-side baseline.
 */
void VirtualDevice::setDetails(const DeviceDetailsSnapshot &details)
{
    DP_CONTEXT();
    DPL("↑ Name: ", details.name.c_str());
    DPL("↑ Total actuators from snapshot: ", details.actuatorIds.size());
    DPL("↑ Total buttons from snapshot: ", details.buttonIds.size());

    this->name.assign(details.name.c_str());
    this->detailsCached = true;
    this->totalActuators = static_cast<std::uint8_t>(details.actuatorIds.size());
    this->actuatorsState.reset();
    this->invalidateRuntimeModel();
    copyStoredIds(details.actuatorIds, this->actuatorIds);
    copyStoredIds(details.buttonIds, this->buttonIds);

    // Cache whether logical actuator IDs already match the dense runtime order.
    // When true, later ID→index resolution becomes a trivial `id - 1` fast path.
    this->actuatorIdsAreDenseOrdered = true;
    for (std::size_t index = 0U; index < details.actuatorIds.size(); ++index)
    {
        const auto expectedId = static_cast<std::uint8_t>(index + 1U);
        if (details.actuatorIds[index] != expectedId)
        {
            this->actuatorIdsAreDenseOrdered = false;
            break;
        }
    }
}

/**
 * @brief Applies one already-decoded authoritative actuator-state snapshot.
 * @details The bridge keeps a compact authoritative bitset plus a cumulative
 *          dirty bitset. When multiple controller state frames arrive inside the
 *          same publish window, the dirty bitset keeps the union of changes so
 *          Homie refreshes every actuator whose published state became stale.
 *
 * @param newState validated authoritative actuator-state bitset.
 */
void VirtualDevice::applyAuthoritativeState(const ActuatorStateBitset &newState) noexcept
{
    DP_CONTEXT();
    const ActuatorStateBitset changedBits = this->actuatorsState ^ newState;
    this->actuatorsState = newState;
    this->dirtyActuators |= changedBits;

    this->runtimeSynchronized = true;
    DPL(changedBits.count(), " actuators changed (from decoded authoritative state)");
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
 * @return false if the topology snapshot is missing.
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
 * @brief Copies the cached topology into a plain bridge-side snapshot.
 * @details Used by the NVS cache layer and by topology-comparison logic that
 *          should not depend on temporary JSON views.
 *
 * @param out destination snapshot that receives the cached topology.
 * @return true if cached details were available and copied into `out`.
 * @return false if no validated topology is currently cached; in that case
 *         `out` is cleared for safety.
 */
auto VirtualDevice::exportDetailsSnapshot(DeviceDetailsSnapshot &out) const noexcept -> bool
{
    if (!this->detailsCached)
    {
        out.clear();
        return false;
    }

    out.name.assign(this->name.c_str());
    copyStoredIds(this->actuatorIds, out.actuatorIds);
    copyStoredIds(this->buttonIds, out.buttonIds);
    return true;
}

/**
 * @brief Returns whether a validated topology exactly matches the cached one.
 * @details The bridge uses exact matching because any topology drift, even a
 *          simple device-name change, can affect MQTT topic layout and Homie
 *          node identity.
 *
 * @param details validated topology snapshot to compare against the cached one.
 * @return true if name, actuator IDs and button IDs all match exactly.
 * @return false if no topology is cached yet or if any field differs.
 */
auto VirtualDevice::matchesDetails(const DeviceDetailsSnapshot &details) const noexcept -> bool
{
    if (!this->detailsCached)
    {
        return false;
    }

    if (this->name != details.name || this->actuatorIds.size() != details.actuatorIds.size() ||
        this->buttonIds.size() != details.buttonIds.size())
    {
        return false;
    }

    for (std::size_t index = 0U; index < this->actuatorIds.size(); ++index)
    {
        if (this->actuatorIds[index] != details.actuatorIds[index])
        {
            return false;
        }
    }

    for (std::size_t index = 0U; index < this->buttonIds.size(); ++index)
    {
        if (this->buttonIds[index] != details.buttonIds[index])
        {
            return false;
        }
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
    // Common reference-stack topology: logical actuator IDs are exactly 1..N in
    // the same order as the dense runtime arrays. Cache that fact once in
    // `setDetails()` so the hot lookup path stays O(1).
    if (this->actuatorIdsAreDenseOrdered && actuatorId != 0U && actuatorId <= this->totalActuators)
    {
        outIndex = static_cast<std::uint8_t>(actuatorId - 1U);
        return true;
    }

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
 * @brief Export one packed authoritative-state byte in wire order.
 *
 * @param byteIndex packed byte index inside the canonical `ACTUATORS_STATE` payload.
 * @return std::uint8_t packed byte, or zero when the index is out of range.
 */
auto VirtualDevice::getPackedStateByte(std::uint8_t byteIndex) const noexcept -> std::uint8_t
{
    const std::uint8_t byteCount = this->getPackedStateByteCount();
    if (byteIndex >= byteCount)
    {
        return 0U;
    }

    const std::size_t bitOffset = static_cast<std::size_t>(byteIndex) * 8U;
    const std::size_t remainingBits = static_cast<std::size_t>(this->totalActuators) - bitOffset;
    const std::size_t extractedBits = remainingBits < 8U ? remainingBits : 8U;
    // The authoritative state already lives as one dense bitset, so exporting
    // MQTT/controller wire bytes is just a bounded bit extraction.
    return this->actuatorsState.extract<std::uint8_t>(bitOffset, extractedBits);
}

/**
 * @brief Returns whether an actuator needs a Homie state refresh.
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
 * @brief Returns whether the next successful Homie refresh must publish every node.
 */
auto VirtualDevice::isFullStatePublishPending() const noexcept -> bool
{
    return this->fullStatePublishPending;
}

/**
 * @brief Clears the one-shot flag that requests a full Homie state publish.
 */
void VirtualDevice::clearFullStatePublishPending() noexcept
{
    this->fullStatePublishPending = false;
}
