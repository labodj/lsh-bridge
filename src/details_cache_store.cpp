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
constexpr char kCacheNamespace[] = "lsh-bridge";  //!< Single NVS namespace reserved for the controller topology cache.
constexpr char kCacheRecordKey[] = "details";     //!< Single NVS key that holds the full serialized topology record.
constexpr std::uint8_t kCacheRecordVersion = 2U;  //!< Version of the byte-stable serialized cache format stored in NVS.
constexpr std::uint8_t kCacheRecordMagic[] = {'L', 'S', 'H', 'D'};
constexpr std::size_t kCacheHeaderSize = sizeof(kCacheRecordMagic) + 4U;  // magic + version + name_len + actuator_count + button_count
constexpr std::size_t kCacheChecksumSize = sizeof(std::uint32_t);
constexpr std::size_t kCacheMinimumRecordSize = kCacheHeaderSize + kCacheChecksumSize;
constexpr std::size_t kCacheMaximumRecordSize = kCacheHeaderSize + constants::virtualDevice::MAX_NAME_LENGTH +
                                                constants::virtualDevice::MAX_ACTUATORS + constants::virtualDevice::MAX_BUTTONS +
                                                kCacheChecksumSize;

/**
 * @brief Computes a tiny deterministic checksum over serialized cache bytes.
 * @details The record is trusted only after checksum verification. This is not
 *          meant to protect against malicious tampering; it simply rejects torn
 *          or corrupted NVS contents before they reach the runtime model.
 *
 * @param bytes serialized record bytes excluding the trailing checksum.
 * @param length number of bytes to include in the checksum.
 * @return std::uint32_t deterministic checksum over the byte range.
 */
[[nodiscard]] auto computeRecordChecksum(const std::uint8_t *bytes, std::size_t length) -> std::uint32_t
{
    constexpr std::uint32_t offsetBasis = 2166136261U;
    constexpr std::uint32_t prime = 16777619U;

    std::uint32_t checksum = offsetBasis;
    for (std::size_t index = 0U; index < length; ++index)
    {
        checksum ^= bytes[index];
        checksum *= prime;
    }

    return checksum;
}

/**
 * @brief Append one byte to a bounded serialized record buffer.
 */
[[nodiscard]] auto appendByte(std::uint8_t *record, std::size_t capacity, std::size_t &offset, std::uint8_t value) -> bool
{
    if (record == nullptr || offset >= capacity)
    {
        return false;
    }

    record[offset] = value;
    ++offset;
    return true;
}

/**
 * @brief Append a byte range to a bounded serialized record buffer.
 */
[[nodiscard]] auto
appendBytes(std::uint8_t *record, std::size_t capacity, std::size_t &offset, const std::uint8_t *source, std::size_t sourceLength) -> bool
{
    if (sourceLength == 0U)
    {
        return true;
    }

    if (record == nullptr || source == nullptr || offset > capacity || sourceLength > (capacity - offset))
    {
        return false;
    }

    std::memcpy(record + offset, source, sourceLength);
    offset += sourceLength;
    return true;
}

/**
 * @brief Append one little-endian uint32 to a bounded serialized record buffer.
 */
[[nodiscard]] auto appendUint32Le(std::uint8_t *record, std::size_t capacity, std::size_t &offset, std::uint32_t value) -> bool
{
    return appendByte(record, capacity, offset, static_cast<std::uint8_t>(value & 0xFFU)) &&
           appendByte(record, capacity, offset, static_cast<std::uint8_t>((value >> 8U) & 0xFFU)) &&
           appendByte(record, capacity, offset, static_cast<std::uint8_t>((value >> 16U) & 0xFFU)) &&
           appendByte(record, capacity, offset, static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
}

/**
 * @brief Read one little-endian uint32 from the serialized record tail.
 */
[[nodiscard]] auto readUint32Le(const std::uint8_t *bytes) -> std::uint32_t
{
    return static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8U) |
           (static_cast<std::uint32_t>(bytes[2]) << 16U) | (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

/**
 * @brief Validate that a serialized ID list contains only unique positive IDs.
 */
[[nodiscard]] auto validatePositiveUniqueIds(const std::uint8_t *ids, std::uint8_t count) -> bool
{
    if (ids == nullptr && count != 0U)
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
 * @brief Validate that an ETL ID vector contains only unique positive IDs.
 */
template <typename TIdVector> [[nodiscard]] auto validatePositiveUniqueIds(const TIdVector &ids) -> bool
{
    etl::bitset<256U> seenIds;
    for (const std::uint8_t id : ids)
    {
        if (id == 0U || seenIds.test(id))
        {
            return false;
        }

        seenIds.set(id);
    }

    return true;
}

/**
 * @brief Decode a byte-stable NVS cache record into a topology snapshot.
 * @details Format v2 is deliberately independent from C++ object layout:
 *          magic[4], version, name_len, actuator_count, button_count, raw name
 *          bytes, actuator IDs, button IDs, little-endian FNV checksum. There is
 *          no padding, no null terminator on flash, and no compiler ABI coupling.
 */
[[nodiscard]] auto decodeStoredRecord(const std::uint8_t *record, std::size_t recordLength, DeviceDetailsSnapshot &outDetails) -> bool
{
    outDetails.clear();

    if (record == nullptr || recordLength < kCacheMinimumRecordSize || recordLength > kCacheMaximumRecordSize)
    {
        return false;
    }

    std::size_t offset = 0U;
    if (std::memcmp(record, kCacheRecordMagic, sizeof(kCacheRecordMagic)) != 0)
    {
        return false;
    }
    offset += sizeof(kCacheRecordMagic);

    const std::uint8_t version = record[offset++];
    const std::uint8_t nameLength = record[offset++];
    const std::uint8_t actuatorCount = record[offset++];
    const std::uint8_t buttonCount = record[offset++];

    if (version != kCacheRecordVersion || nameLength == 0U || nameLength > constants::virtualDevice::MAX_NAME_LENGTH ||
        actuatorCount > constants::virtualDevice::MAX_ACTUATORS || buttonCount > constants::virtualDevice::MAX_BUTTONS)
    {
        return false;
    }

    const std::size_t expectedLength =
        kCacheHeaderSize + nameLength + static_cast<std::size_t>(actuatorCount) + buttonCount + kCacheChecksumSize;
    if (recordLength != expectedLength)
    {
        return false;
    }

    const std::uint32_t storedChecksum = readUint32Le(record + recordLength - kCacheChecksumSize);
    if (storedChecksum != computeRecordChecksum(record, recordLength - kCacheChecksumSize))
    {
        return false;
    }

    char nameBuffer[constants::virtualDevice::MAX_NAME_LENGTH + 1U]{};
    std::memcpy(nameBuffer, record + offset, nameLength);
    offset += nameLength;

    const std::uint8_t *const actuatorIds = record + offset;
    offset += actuatorCount;
    const std::uint8_t *const buttonIds = record + offset;

    if (!validatePositiveUniqueIds(actuatorIds, actuatorCount) || !validatePositiveUniqueIds(buttonIds, buttonCount))
    {
        return false;
    }

    outDetails.name.assign(nameBuffer);
    for (std::uint8_t index = 0U; index < actuatorCount; ++index)
    {
        outDetails.actuatorIds.push_back(actuatorIds[index]);
    }

    for (std::uint8_t index = 0U; index < buttonCount; ++index)
    {
        outDetails.buttonIds.push_back(buttonIds[index]);
    }

    return true;
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
    if (recordLength < kCacheMinimumRecordSize || recordLength > kCacheMaximumRecordSize)
    {
        preferences.end();
        return false;
    }

    std::uint8_t record[kCacheMaximumRecordSize]{};
    const std::size_t bytesRead = preferences.getBytes(kCacheRecordKey, record, recordLength);
    preferences.end();

    if (bytesRead != recordLength)
    {
        return false;
    }

    return decodeStoredRecord(record, bytesRead, outDetails);
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

    if (!validatePositiveUniqueIds(details.actuatorIds) || !validatePositiveUniqueIds(details.buttonIds))
    {
        return false;
    }

    std::uint8_t record[kCacheMaximumRecordSize]{};
    std::size_t recordLength = 0U;
    const auto nameLength = static_cast<std::uint8_t>(details.name.size());
    const auto actuatorCount = static_cast<std::uint8_t>(details.actuatorIds.size());
    const auto buttonCount = static_cast<std::uint8_t>(details.buttonIds.size());

    if (!appendBytes(record, sizeof(record), recordLength, kCacheRecordMagic, sizeof(kCacheRecordMagic)) ||
        !appendByte(record, sizeof(record), recordLength, kCacheRecordVersion) ||
        !appendByte(record, sizeof(record), recordLength, nameLength) || !appendByte(record, sizeof(record), recordLength, actuatorCount) ||
        !appendByte(record, sizeof(record), recordLength, buttonCount) ||
        !appendBytes(record, sizeof(record), recordLength, reinterpret_cast<const std::uint8_t *>(details.name.c_str()), nameLength))
    {
        return false;
    }

    for (const std::uint8_t actuatorId : details.actuatorIds)
    {
        if (!appendByte(record, sizeof(record), recordLength, actuatorId))
        {
            return false;
        }
    }

    for (const std::uint8_t buttonId : details.buttonIds)
    {
        if (!appendByte(record, sizeof(record), recordLength, buttonId))
        {
            return false;
        }
    }

    const std::uint32_t checksum = computeRecordChecksum(record, recordLength);
    if (!appendUint32Le(record, sizeof(record), recordLength, checksum))
    {
        return false;
    }

    Preferences preferences;
    if (!preferences.begin(kCacheNamespace, false))
    {
        return false;
    }

    const std::size_t bytesWritten = preferences.putBytes(kCacheRecordKey, record, recordLength);
    preferences.end();

    return bytesWritten == recordLength;
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
