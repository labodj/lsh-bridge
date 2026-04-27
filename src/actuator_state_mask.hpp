/**
 * @file    actuator_state_mask.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Compact actuator-state bit container optimized for small static topologies.
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

#ifndef LSH_BRIDGE_ACTUATOR_STATE_MASK_HPP
#define LSH_BRIDGE_ACTUATOR_STATE_MASK_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <etl/bitset.h>

#include "constants/configs/virtual_device.hpp"

template <bool UseNativeMask> class ActuatorStateMaskImpl;

/**
 * @brief Native uint32_t implementation used when MAX_ACTUATORS <= 32.
 */
template <> class ActuatorStateMaskImpl<true>
{
public:
    using Storage = std::conditional_t<constants::virtualDevice::MAX_ACTUATORS <= 8U,
                                       std::uint8_t,
                                       std::conditional_t<constants::virtualDevice::MAX_ACTUATORS <= 16U, std::uint16_t, std::uint32_t>>;

    void reset() noexcept
    {
        bits = 0U;
    }

    void reset(std::uint8_t index) noexcept
    {
        if (index < constants::virtualDevice::MAX_ACTUATORS)
        {
            bits &= ~bitFor(index);
        }
    }

    void set(std::uint8_t index, bool value = true) noexcept
    {
        if (index >= constants::virtualDevice::MAX_ACTUATORS)
        {
            return;
        }

        const Storage bit = bitFor(index);
        if (value)
        {
            bits |= bit;
        }
        else
        {
            bits &= ~bit;
        }
    }

    [[nodiscard]] auto test(std::uint8_t index) const noexcept -> bool
    {
        return index < constants::virtualDevice::MAX_ACTUATORS && (bits & bitFor(index)) != 0U;
    }

    [[nodiscard]] auto operator[](std::uint8_t index) const noexcept -> bool
    {
        return test(index);
    }

    [[nodiscard]] auto any() const noexcept -> bool
    {
        return bits != 0U;
    }

    [[nodiscard]] auto count() const noexcept -> std::uint8_t
    {
        return static_cast<std::uint8_t>(__builtin_popcount(bits));
    }

    template <typename T> [[nodiscard]] auto extract(std::size_t bitOffset, std::size_t bitCount) const noexcept -> T
    {
        if (bitCount == 0U)
        {
            return T{};
        }

        const std::uint32_t shifted = bitOffset >= 32U ? 0U : (static_cast<std::uint32_t>(bits) >> bitOffset);
        const std::uint32_t mask = bitCount >= 32U ? 0xFFFFFFFFUL : ((1UL << bitCount) - 1UL);
        return static_cast<T>(shifted & mask);
    }

    auto operator|=(const ActuatorStateMaskImpl &other) noexcept -> ActuatorStateMaskImpl &
    {
        bits = static_cast<Storage>((bits | other.bits) & liveMask());
        return *this;
    }

    auto operator&=(const ActuatorStateMaskImpl &other) noexcept -> ActuatorStateMaskImpl &
    {
        bits &= other.bits;
        return *this;
    }

    [[nodiscard]] friend auto operator==(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept -> bool
    {
        return lhs.bits == rhs.bits;
    }

    [[nodiscard]] friend auto operator!=(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept -> bool
    {
        return !(lhs == rhs);
    }

    [[nodiscard]] friend auto operator^(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept
        -> ActuatorStateMaskImpl
    {
        ActuatorStateMaskImpl result{};
        result.bits = (lhs.bits ^ rhs.bits) & liveMask();
        return result;
    }

    [[nodiscard]] friend auto operator&(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept
        -> ActuatorStateMaskImpl
    {
        ActuatorStateMaskImpl result{};
        result.bits = lhs.bits & rhs.bits;
        return result;
    }

    [[nodiscard]] friend auto operator|(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept
        -> ActuatorStateMaskImpl
    {
        ActuatorStateMaskImpl result{};
        result.bits = (lhs.bits | rhs.bits) & liveMask();
        return result;
    }

    [[nodiscard]] friend auto operator~(const ActuatorStateMaskImpl &value) noexcept -> ActuatorStateMaskImpl
    {
        ActuatorStateMaskImpl result{};
        result.bits = (~value.bits) & liveMask();
        return result;
    }

private:
    Storage bits = 0U;

    [[nodiscard]] static constexpr auto bitFor(std::uint8_t index) noexcept -> Storage
    {
        return static_cast<Storage>(1UL << index);
    }

    [[nodiscard]] static constexpr auto liveMask() noexcept -> Storage
    {
        if constexpr (constants::virtualDevice::MAX_ACTUATORS == 0U)
        {
            return 0U;
        }
        else if constexpr (constants::virtualDevice::MAX_ACTUATORS >= (sizeof(Storage) * 8U))
        {
            return static_cast<Storage>(~static_cast<Storage>(0U));
        }
        else
        {
            return static_cast<Storage>((1UL << constants::virtualDevice::MAX_ACTUATORS) - 1UL);
        }
    }
};

/**
 * @brief ETL bitset fallback used when MAX_ACTUATORS is larger than 32.
 */
template <> class ActuatorStateMaskImpl<false>
{
public:
    void reset() noexcept
    {
        bits.reset();
    }

    void reset(std::uint8_t index) noexcept
    {
        if (index < constants::virtualDevice::MAX_ACTUATORS)
        {
            bits.reset(index);
        }
    }

    void set(std::uint8_t index, bool value = true) noexcept
    {
        if (index < constants::virtualDevice::MAX_ACTUATORS)
        {
            bits.set(index, value);
        }
    }

    [[nodiscard]] auto test(std::uint8_t index) const noexcept -> bool
    {
        return index < constants::virtualDevice::MAX_ACTUATORS && bits.test(index);
    }

    [[nodiscard]] auto operator[](std::uint8_t index) const noexcept -> bool
    {
        return test(index);
    }

    [[nodiscard]] auto any() const noexcept -> bool
    {
        return bits.any();
    }

    [[nodiscard]] auto count() const noexcept -> std::uint8_t
    {
        return static_cast<std::uint8_t>(bits.count());
    }

    template <typename T> [[nodiscard]] auto extract(std::size_t bitOffset, std::size_t bitCount) const noexcept -> T
    {
        return bits.template extract<T>(bitOffset, bitCount);
    }

    auto operator|=(const ActuatorStateMaskImpl &other) noexcept -> ActuatorStateMaskImpl &
    {
        bits |= other.bits;
        return *this;
    }

    auto operator&=(const ActuatorStateMaskImpl &other) noexcept -> ActuatorStateMaskImpl &
    {
        bits &= other.bits;
        return *this;
    }

    [[nodiscard]] friend auto operator==(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept -> bool
    {
        return lhs.bits == rhs.bits;
    }

    [[nodiscard]] friend auto operator!=(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept -> bool
    {
        return !(lhs == rhs);
    }

    [[nodiscard]] friend auto operator^(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept
        -> ActuatorStateMaskImpl
    {
        ActuatorStateMaskImpl result{};
        result.bits = lhs.bits ^ rhs.bits;
        return result;
    }

    [[nodiscard]] friend auto operator&(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept
        -> ActuatorStateMaskImpl
    {
        ActuatorStateMaskImpl result{};
        result.bits = lhs.bits & rhs.bits;
        return result;
    }

    [[nodiscard]] friend auto operator|(const ActuatorStateMaskImpl &lhs, const ActuatorStateMaskImpl &rhs) noexcept
        -> ActuatorStateMaskImpl
    {
        ActuatorStateMaskImpl result{};
        result.bits = lhs.bits | rhs.bits;
        return result;
    }

    [[nodiscard]] friend auto operator~(const ActuatorStateMaskImpl &value) noexcept -> ActuatorStateMaskImpl
    {
        ActuatorStateMaskImpl result{};
        result.bits = ~value.bits;
        return result;
    }

private:
    etl::bitset<constants::virtualDevice::MAX_ACTUATORS> bits{};
};

using ActuatorStateMask = ActuatorStateMaskImpl<(constants::virtualDevice::MAX_ACTUATORS <= 32U)>;

#endif  // LSH_BRIDGE_ACTUATOR_STATE_MASK_HPP
