#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace lsh::bridge
{
/**
 * One bounded MQTT command in flight. Homie serializes receive callbacks;
 * only that callback may touch this object. Reset on session change or a new
 * message, and enqueue the bytes before the next callback reuses the buffer.
 */
template <std::size_t Capacity> class MqttCommandAssembler
{
public:
    enum class Result : std::uint8_t
    {
        Partial,
        Complete,
        Invalid
    };

    void reset()
    {
        received = expected = 0U;
    }

    Result append(const std::uint8_t *payload, std::size_t length, std::size_t index, std::size_t total, bool serviceTopic)
    {
        if (index == 0U)
        {
            reset();
            expected = total;
            service = serviceTopic;
        }
        if (payload == nullptr || length == 0U || total == 0U || total > Capacity || total != expected || service != serviceTopic ||
            index != received || index > total || length > total - index)
        {
            reset();
            return Result::Invalid;
        }
        std::memcpy(buffer.data() + index, payload, length);
        received += length;
        if (received != total)
        {
            return Result::Partial;
        }
        reset();
        return Result::Complete;
    }

    const char *data() const
    {
        return reinterpret_cast<const char *>(buffer.data());
    }

private:
    std::array<std::uint8_t, Capacity> buffer{};
    std::size_t received = 0U;
    std::size_t expected = 0U;
    bool service = false;
};
}  // namespace lsh::bridge
