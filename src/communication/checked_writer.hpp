/**
 * @file    checked_writer.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Writer adapters for one-pass serialization on serial and fixed buffers.
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

#ifndef LSH_BRIDGE_COMMUNICATION_CHECKED_WRITER_HPP
#define LSH_BRIDGE_COMMUNICATION_CHECKED_WRITER_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace lsh::bridge::communication
{
/**
 * @brief Tiny sink adapter that remembers whether any write was short or invalid.
 * @details Used by one-pass serialization paths that emit directly into UART or
 *          framed transport writers. The adapter lets the caller serialize once
 *          and then reject the whole payload if any sub-write was truncated.
 *
 * @tparam Sink concrete sink type exposing `write(uint8_t)` and
 *         `write(const uint8_t*, size_t)`.
 */
template <typename Sink> class CheckedWriter
{
public:
    /**
     * @brief Bind the adapter to one concrete destination sink.
     *
     * @param destination sink that will receive serialized bytes.
     */
    explicit CheckedWriter(Sink &destination) noexcept : sink(destination)
    {}

    /**
     * @brief Forward one byte to the sink and flag the adapter on short write.
     *
     * @param byte single byte to write.
     * @return std::size_t number of bytes accepted by the sink.
     */
    auto write(std::uint8_t byte) -> std::size_t
    {
        const auto writtenBytes = this->sink.write(byte);
        if (writtenBytes != 1U)
        {
            this->writeFailed = true;
        }
        return writtenBytes;
    }

    /**
     * @brief Forward a contiguous byte buffer to the sink.
     * @details Zero-length writes are harmless no-ops. A null pointer for a
     *          non-zero request marks the adapter as failed because that would
     *          represent a broken serialization call site.
     *
     * @param buffer source bytes to write.
     * @param size requested byte count.
     * @return std::size_t number of bytes accepted by the sink.
     */
    auto write(const std::uint8_t *buffer, std::size_t size) -> std::size_t
    {
        if (size == 0U)
        {
            return 0U;
        }

        if (buffer == nullptr)
        {
            this->writeFailed = true;
            return 0U;
        }

        const auto writtenBytes = this->sink.write(buffer, size);
        if (writtenBytes != size)
        {
            this->writeFailed = true;
        }
        return writtenBytes;
    }

    /**
     * @brief Return whether any previous write failed or was truncated.
     */
    [[nodiscard]] auto failed() const noexcept -> bool
    {
        return this->writeFailed;
    }

private:
    Sink &sink;
    bool writeFailed = false;
};

/**
 * @brief Bounded in-memory writer used for one-pass MQTT payload serialization.
 * @details The writer accumulates serialized bytes into a caller-owned fixed
 *          buffer and never overflows it. Once `overflowed()` becomes true the
 *          caller must discard the whole payload instead of publishing a partial
 *          buffer prefix.
 */
class FixedBufferWriter
{
public:
    /**
     * @brief Bind the writer to one caller-owned fixed buffer.
     *
     * @param destination mutable destination buffer.
     * @param destinationCapacity total writable capacity of `destination`.
     */
    FixedBufferWriter(char *destination, std::size_t destinationCapacity) noexcept : buffer(destination), capacity(destinationCapacity)
    {}

    /**
     * @brief Append one byte to the fixed buffer.
     *
     * @param byte single byte to append.
     * @return std::size_t number of bytes appended.
     */
    auto write(std::uint8_t byte) -> std::size_t
    {
        return this->write(&byte, 1U);
    }

    /**
     * @brief Append a contiguous byte range to the fixed buffer.
     * @details The writer refuses null pointers and out-of-capacity writes, then
     *          latches `overflowed()` so callers can treat the whole payload as
     *          invalid instead of publishing a truncated frame.
     *
     * @param source source bytes to append.
     * @param size requested byte count.
     * @return std::size_t number of bytes appended.
     */
    auto write(const std::uint8_t *source, std::size_t size) -> std::size_t
    {
        if (size == 0U)
        {
            return 0U;
        }

        if (source == nullptr || this->buffer == nullptr || this->length > this->capacity || size > (this->capacity - this->length))
        {
            this->overflowed_ = true;
            return 0U;
        }

        std::memcpy(this->buffer + this->length, source, size);
        this->length += size;
        return size;
    }

    /**
     * @brief Return the beginning of the accumulated byte range.
     */
    [[nodiscard]] auto data() const noexcept -> const char *
    {
        return this->buffer;
    }

    /**
     * @brief Return the number of valid bytes currently stored in the buffer.
     */
    [[nodiscard]] auto size() const noexcept -> std::size_t
    {
        return this->length;
    }

    /**
     * @brief Return whether any append request exceeded the available capacity.
     */
    [[nodiscard]] auto overflowed() const noexcept -> bool
    {
        return this->overflowed_;
    }

private:
    char *buffer = nullptr;
    std::size_t capacity = 0U;
    std::size_t length = 0U;
    bool overflowed_ = false;
};
}  // namespace lsh::bridge::communication

#endif  // LSH_BRIDGE_COMMUNICATION_CHECKED_WRITER_HPP
