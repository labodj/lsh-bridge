/**
 * @file    details_cache_store.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the small NVS-backed cache used to persist controller
 * topology details across bridge reboots.
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

#include "details_cache_store.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <Preferences.h>
#include <etl/bitset.h>

#include "constants/configs/virtual_device.hpp"

namespace
{
constexpr char kCacheNamespace[] = "lsh-bridge";   //!< Single NVS namespace reserved for the controller topology cache.
constexpr char kCacheRecordKey[] = "details";      //!< Single NVS key that holds the full serialized topology record.
constexpr std::uint16_t kCacheRecordVersion = 1U;  //!< Version of the serialized cache format stored in NVS.

/**
 * @brief Flat record stored as one NVS blob.
 * @details The bridge writes one blob per topology change. Using a single record
 *          keeps the code straightforward, keeps writes infrequent, and avoids
 *          partially updated multi-key state after resets.
 */
struct StoredDetailsRecord
{
    std::uint16_t version = kCacheRecordVersion;
    std::uint8_t nameLength = 0U;
    std::uint8_t actuatorCount = 0U;
    std::uint8_t buttonCount = 0U;
    char name[constants::virtualDevice::MAX_NAME_LENGTH + 1U]{};
    std::uint8_t actuatorIds[constants::virtualDevice::MAX_ACTUATORS]{};
    std::uint8_t buttonIds[constants::virtualDevice::MAX_BUTTONS]{};
    std::uint32_t checksum = 0U;
};

/**
 * @brief Computes a tiny deterministic checksum over the serialized cache record.
 * @details The record is trusted only after checksum verification. This is not
 *          meant to protect against malicious tampering; it simply rejects torn
 *          or corrupted NVS contents before they reach the runtime model.
 *
 * @param record flat serialized cache record, excluding any external metadata.
 * @return std::uint32_t deterministic checksum over every byte that precedes
 *         the `checksum` field inside `record`.
 */
[[nodiscard]] auto computeRecordChecksum(const StoredDetailsRecord &record) -> std::uint32_t
{
    constexpr std::uint32_t offsetBasis = 2166136261U;
    constexpr std::uint32_t prime = 16777619U;

    std::uint32_t checksum = offsetBasis;
    const auto *bytes = reinterpret_cast<const std::uint8_t *>(&record);
    for (std::size_t index = 0U; index < offsetof(StoredDetailsRecord, checksum); ++index)
    {
        checksum ^= bytes[index];
        checksum *= prime;
    }

    return checksum;
}

/**
 * @brief Validates that an ID list contains only unique positive IDs.
 *
 * @param ids fixed-size array that stores the serialized IDs.
 * @param count how many entries inside `ids` are logically used.
 * @return true if the logical prefix `[0, count)` contains only unique IDs
 *         greater than zero.
 * @return false if `count` exceeds the buffer capacity or if the logical prefix
 *         contains zero/duplicate IDs.
 */
template <std::size_t Capacity>
[[nodiscard]] auto validatePositiveUniqueIds(const std::uint8_t (&ids)[Capacity], std::uint8_t count) -> bool
{
    if (count > Capacity)
    {
        return false;
    }

    etl::bitset<256U> seenIds;
    for (std::uint8_t index = 0U; index < count; ++index)
    {
        const std::uint8_t id = ids[index];
        if (id == 0U || seenIds.test(id))
        {
            return false;
        }

        seenIds.set(id);
    }

    return true;
}

/**
 * @brief Validates one deserialized cache record before exposing it to runtime.
 *
 * @param record flat cache record loaded from NVS.
 * @return true if version, lengths, checksum and ID lists all look sane.
 * @return false if the NVS record looks torn, stale, corrupted or structurally invalid.
 */
[[nodiscard]] auto validateStoredRecord(const StoredDetailsRecord &record) -> bool
{
    if (record.version != kCacheRecordVersion)
    {
        return false;
    }

    if (record.nameLength == 0U || record.nameLength > constants::virtualDevice::MAX_NAME_LENGTH)
    {
        return false;
    }

    if (record.name[record.nameLength] != '\0')
    {
        return false;
    }

    if (record.actuatorCount > constants::virtualDevice::MAX_ACTUATORS || record.buttonCount > constants::virtualDevice::MAX_BUTTONS)
    {
        return false;
    }

    if (!validatePositiveUniqueIds(record.actuatorIds, record.actuatorCount) ||
        !validatePositiveUniqueIds(record.buttonIds, record.buttonCount))
    {
        return false;
    }

    return record.checksum == computeRecordChecksum(record);
}
}  // namespace

namespace DetailsCacheStore
{

/**
 * @brief Load the last saved controller topology snapshot from NVS.
 * @details A failed load is not fatal for the bridge: the runtime simply falls
 *          back to waiting for fresh `DEVICE_DETAILS` from the controller.
 *
 * @param outDetails destination snapshot that receives the validated cached
 *        topology on success and is cleared on failure.
 * @return true if a valid cached topology has been loaded into `outDetails`.
 * @return false if NVS is unavailable, the key is missing, or the stored data
 *         does not pass structural validation.
 */
auto load(DeviceDetailsSnapshot &outDetails) -> bool
{
    outDetails.clear();

    Preferences preferences;
    if (!preferences.begin(kCacheNamespace, true))
    {
        return false;
    }

    const std::size_t recordLength = preferences.getBytesLength(kCacheRecordKey);
    if (recordLength != sizeof(StoredDetailsRecord))
    {
        preferences.end();
        return false;
    }

    StoredDetailsRecord record{};
    const std::size_t bytesRead = preferences.getBytes(kCacheRecordKey, &record, sizeof(record));
    preferences.end();

    if (bytesRead != sizeof(record) || !validateStoredRecord(record))
    {
        return false;
    }

    outDetails.name.assign(record.name);
    for (std::uint8_t index = 0U; index < record.actuatorCount; ++index)
    {
        outDetails.actuatorIds.push_back(record.actuatorIds[index]);
    }

    for (std::uint8_t index = 0U; index < record.buttonCount; ++index)
    {
        outDetails.buttonIds.push_back(record.buttonIds[index]);
    }

    return true;
}

/**
 * @brief Save one validated controller topology snapshot to NVS.
 * @details The caller is expected to invoke this only after a real topology
 *          change. That policy keeps flash wear low and keeps the code honest:
 *          NVS here is a resilience optimization, not a second source of truth.
 *
 * @param details validated topology snapshot to serialize as the new cache baseline.
 * @return true if the snapshot passed defensive validation and the full record
 *         has been committed to NVS.
 * @return false if the snapshot is structurally invalid or if NVS refused the write.
 */
auto save(const DeviceDetailsSnapshot &details) -> bool
{
    if (details.name.empty() || details.actuatorIds.size() > constants::virtualDevice::MAX_ACTUATORS ||
        details.buttonIds.size() > constants::virtualDevice::MAX_BUTTONS)
    {
        return false;
    }

    StoredDetailsRecord record{};
    record.version = kCacheRecordVersion;
    record.nameLength = static_cast<std::uint8_t>(details.name.size());
    record.actuatorCount = static_cast<std::uint8_t>(details.actuatorIds.size());
    record.buttonCount = static_cast<std::uint8_t>(details.buttonIds.size());
    std::memcpy(record.name, details.name.c_str(), details.name.size());
    record.name[details.name.size()] = '\0';

    for (std::uint8_t index = 0U; index < record.actuatorCount; ++index)
    {
        record.actuatorIds[index] = details.actuatorIds[index];
    }

    for (std::uint8_t index = 0U; index < record.buttonCount; ++index)
    {
        record.buttonIds[index] = details.buttonIds[index];
    }

    if (!validatePositiveUniqueIds(record.actuatorIds, record.actuatorCount) ||
        !validatePositiveUniqueIds(record.buttonIds, record.buttonCount))
    {
        return false;
    }

    record.checksum = computeRecordChecksum(record);

    Preferences preferences;
    if (!preferences.begin(kCacheNamespace, false))
    {
        return false;
    }

    const std::size_t bytesWritten = preferences.putBytes(kCacheRecordKey, &record, sizeof(record));
    preferences.end();

    return bytesWritten == sizeof(record);
}

/**
 * @brief Remove the cached topology snapshot from NVS.
 * @details Mainly useful for diagnostics or for future explicit reset flows.
 *
 * @return true if the key existed and NVS reported a successful removal.
 * @return false if NVS could not be opened or if the key was not removed.
 */
auto clear() -> bool
{
    Preferences preferences;
    if (!preferences.begin(kCacheNamespace, false))
    {
        return false;
    }

    const bool removed = preferences.remove(kCacheRecordKey);
    preferences.end();
    return removed;
}

}  // namespace DetailsCacheStore
