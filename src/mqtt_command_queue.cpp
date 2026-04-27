/**
 * @file    mqtt_command_queue.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the fixed MQTT command queue used by the bridge runtime.
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

#include "mqtt_command_queue.hpp"

#include <cstring>

namespace lsh::bridge
{
namespace
{
/**
 * @brief Advance one ring-buffer index without using `%`.
 * @details Queue capacity is a runtime-configurable compile-time constant, not
 *          necessarily a power of two. A compare-and-reset branch avoids the
 *          division emitted by modulo on small embedded targets while keeping
 *          support for every valid queue capacity.
 */
[[nodiscard]] auto nextQueueIndex(std::uint8_t index) noexcept -> std::uint8_t
{
    ++index;
    if (index >= constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY)
    {
        return 0U;
    }
    return index;
}

/**
 * @brief Compare two queue sequence numbers across uint16_t wraparound.
 */
[[nodiscard]] auto sequenceBefore(std::uint16_t lhs, std::uint16_t rhs) noexcept -> bool
{
    return static_cast<std::int16_t>(lhs - rhs) < 0;
}
}  // namespace

/**
 * @brief Copy one complete MQTT frame into the fixed bridge-side queue.
 * @details The MQTT callback may run concurrently with disconnect teardown.
 *          The queue reserves a slot under the lock, copies payload bytes
 *          outside the lock, then commits only if no clear() happened meanwhile.
 *
 * @param source Topic family that produced this command.
 * @param payload Raw MQTT payload bytes.
 * @param payloadLength Number of bytes available at `payload`.
 * @return true if the command has been copied into the queue.
 * @return false if the payload is invalid or the queue is full.
 */
auto MqttCommandQueue::enqueue(MqttCommandSource source, const char *payload, std::size_t payloadLength) -> bool
{
    if (payload == nullptr || payloadLength == 0U || payloadLength > constants::controllerSerial::MQTT_COMMAND_MESSAGE_MAX_SIZE)
    {
        return false;
    }

    std::uint8_t reservedIndex = 0U;
    std::uint16_t reservedGeneration = 0U;
    bool wasReserved = false;

    portENTER_CRITICAL(&this->queueMux);
    if (this->count < constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY)
    {
        std::uint8_t candidateIndex = this->tail;
        for (std::uint8_t visited = 0U; visited < constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY; ++visited)
        {
            auto &slot = this->queue[candidateIndex];
            if (slot.state == MqttCommandSlotState::Free)
            {
                reservedIndex = candidateIndex;
                reservedGeneration = this->queueGeneration;
                slot.state = MqttCommandSlotState::Writing;
                slot.generation = reservedGeneration;
                slot.sequence = this->nextSequence;
                slot.length = 0U;
                ++this->nextSequence;
                this->tail = nextQueueIndex(candidateIndex);
                ++this->count;
                wasReserved = true;
                break;
            }
            candidateIndex = nextQueueIndex(candidateIndex);
        }
    }
    portEXIT_CRITICAL(&this->queueMux);

    if (!wasReserved)
    {
        return false;
    }

    auto &slot = this->queue[reservedIndex];
    std::memcpy(slot.payload, payload, payloadLength);

    bool wasQueued = false;
    portENTER_CRITICAL(&this->queueMux);
    if (slot.state == MqttCommandSlotState::Writing && slot.generation == reservedGeneration &&
        reservedGeneration == this->queueGeneration)
    {
        slot.source = source;
        slot.length = static_cast<std::uint16_t>(payloadLength);
        slot.state = MqttCommandSlotState::Ready;
        wasQueued = true;
    }
    else if (slot.state == MqttCommandSlotState::Writing && slot.generation == reservedGeneration)
    {
        slot.length = 0U;
        slot.state = MqttCommandSlotState::Free;
        --this->count;
    }
    portEXIT_CRITICAL(&this->queueMux);

    return wasQueued;
}

/**
 * @brief Pop the oldest queued MQTT frame from the fixed ring buffer.
 * @details Slot metadata is protected by `queueMux` because the MQTT callback
 *          path and the main loop can touch the queue concurrently. The payload
 *          copy itself happens outside the lock after a short slot reservation,
 *          then the slot is released only if clear() did not race.
 *
 * @param outCommand Destination for the oldest queued frame.
 * @return true if one queued frame has been copied into `outCommand`.
 * @return false if the queue was empty.
 */
auto MqttCommandQueue::dequeue(QueuedMqttCommand &outCommand) -> bool
{
    std::uint8_t reservedIndex = 0U;
    std::uint16_t reservedGeneration = 0U;
    std::uint16_t reservedLength = 0U;
    MqttCommandSource reservedSource = MqttCommandSource::Device;
    bool wasReserved = false;

    portENTER_CRITICAL(&this->queueMux);
    if (this->count > 0U)
    {
        bool hasOldestSlot = false;
        std::uint8_t oldestIndex = 0U;
        std::uint16_t oldestSequence = 0U;
        for (std::uint8_t index = 0U; index < constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY; ++index)
        {
            const auto &slot = this->queue[index];
            if (slot.state != MqttCommandSlotState::Free &&
                (!hasOldestSlot || sequenceBefore(slot.sequence, oldestSequence)))
            {
                oldestIndex = index;
                oldestSequence = slot.sequence;
                hasOldestSlot = true;
            }
        }

        if (hasOldestSlot)
        {
            auto &slot = this->queue[oldestIndex];
            if (slot.state == MqttCommandSlotState::Ready)
            {
                reservedIndex = oldestIndex;
                reservedGeneration = slot.generation;
                reservedLength = slot.length;
                reservedSource = slot.source;
                slot.state = MqttCommandSlotState::Reading;
                wasReserved = true;
            }
        }
    }
    portEXIT_CRITICAL(&this->queueMux);

    if (!wasReserved)
    {
        return false;
    }

    outCommand.source = reservedSource;
    outCommand.length = reservedLength;
    std::memcpy(outCommand.payload, this->queue[reservedIndex].payload, reservedLength);

    bool wasDequeued = false;
    portENTER_CRITICAL(&this->queueMux);
    auto &slot = this->queue[reservedIndex];
    if (slot.state == MqttCommandSlotState::Reading && slot.generation == reservedGeneration)
    {
        wasDequeued = (reservedGeneration == this->queueGeneration);
        slot.state = MqttCommandSlotState::Free;
        slot.length = 0U;
        --this->count;
    }
    portEXIT_CRITICAL(&this->queueMux);

    return wasDequeued;
}

/**
 * @brief Drop every queued MQTT frame.
 * @details Used when MQTT disconnects so the bridge does not replay commands
 *          captured before the next MQTT session starts.
 */
void MqttCommandQueue::clear()
{
    portENTER_CRITICAL(&this->queueMux);
    ++this->queueGeneration;
    if (this->queueGeneration == 0U)
    {
        this->queueGeneration = 1U;
    }
    this->tail = 0U;
    this->count = 0U;
    for (auto &slot : this->queue)
    {
        if (slot.state == MqttCommandSlotState::Writing || slot.state == MqttCommandSlotState::Reading)
        {
            ++this->count;
        }
        else
        {
            slot.state = MqttCommandSlotState::Free;
            slot.length = 0U;
            slot.generation = this->queueGeneration;
        }
    }
    portEXIT_CRITICAL(&this->queueMux);
}

/**
 * @brief Aggregate one MQTT queue overflow occurrence.
 * @details The callback may drop many commands in quick succession. Instead of
 *          publishing one diagnostic per drop, the bridge keeps a small
 *          per-topic-family counter and publishes an aggregated event later
 *          from the main loop.
 *
 * @param source Topic family whose command could not be queued.
 */
void MqttCommandQueue::recordDroppedCommand(MqttCommandSource source)
{
    portENTER_CRITICAL(&this->queueMux);
    if (source == MqttCommandSource::Device)
    {
        if (this->droppedDeviceCommandCount < DIAGNOSTIC_COUNTER_MAX)
        {
            ++this->droppedDeviceCommandCount;
        }
    }
    else
    {
        if (this->droppedServiceCommandCount < DIAGNOSTIC_COUNTER_MAX)
        {
            ++this->droppedServiceCommandCount;
        }
    }
    portEXIT_CRITICAL(&this->queueMux);
}

/**
 * @brief Aggregate one MQTT command rejection observed either in the callback or during main-loop parse.
 *
 * @param reason stable reason why the MQTT callback rejected the command.
 */
void MqttCommandQueue::recordRejectedCommand(MqttRejectedCommandReason reason)
{
    portENTER_CRITICAL(&this->queueMux);
    switch (reason)
    {
    case MqttRejectedCommandReason::Retained:
        if (this->rejectedRetainedCommandCount < DIAGNOSTIC_COUNTER_MAX)
        {
            ++this->rejectedRetainedCommandCount;
        }
        break;

    case MqttRejectedCommandReason::Oversize:
        if (this->rejectedOversizeCommandCount < DIAGNOSTIC_COUNTER_MAX)
        {
            ++this->rejectedOversizeCommandCount;
        }
        break;

    case MqttRejectedCommandReason::Fragmented:
        if (this->rejectedFragmentedCommandCount < DIAGNOSTIC_COUNTER_MAX)
        {
            ++this->rejectedFragmentedCommandCount;
        }
        break;

    case MqttRejectedCommandReason::Malformed:
        if (this->rejectedMalformedCommandCount < DIAGNOSTIC_COUNTER_MAX)
        {
            ++this->rejectedMalformedCommandCount;
        }
        break;
    }
    portEXIT_CRITICAL(&this->queueMux);
}

/**
 * @brief Forget every queued-command overflow counter.
 */
void MqttCommandQueue::clearDroppedCounters()
{
    portENTER_CRITICAL(&this->queueMux);
    this->droppedDeviceCommandCount = 0U;
    this->droppedServiceCommandCount = 0U;
    portEXIT_CRITICAL(&this->queueMux);
}

/**
 * @brief Forget every rejected-command counter.
 */
void MqttCommandQueue::clearRejectedCounters()
{
    portENTER_CRITICAL(&this->queueMux);
    this->rejectedRetainedCommandCount = 0U;
    this->rejectedOversizeCommandCount = 0U;
    this->rejectedFragmentedCommandCount = 0U;
    this->rejectedMalformedCommandCount = 0U;
    portEXIT_CRITICAL(&this->queueMux);
}

/**
 * @brief Snapshot the current dropped-command counters without mutating them.
 *
 * @param outDroppedDeviceCommands Receives the number of dropped device-topic commands.
 * @param outDroppedServiceCommands Receives the number of dropped service-topic commands.
 */
void MqttCommandQueue::snapshotDroppedCounters(std::uint16_t &outDroppedDeviceCommands, std::uint16_t &outDroppedServiceCommands)
{
    portENTER_CRITICAL(&this->queueMux);
    outDroppedDeviceCommands = this->droppedDeviceCommandCount;
    outDroppedServiceCommands = this->droppedServiceCommandCount;
    portEXIT_CRITICAL(&this->queueMux);
}

/**
 * @brief Snapshot the current rejected-command counters without mutating them.
 *
 * @param outRejectedRetainedCommands Receives the number of retained commands rejected by policy.
 * @param outRejectedOversizeCommands Receives the number of oversize commands rejected before enqueue.
 * @param outRejectedFragmentedCommands Receives the number of fragmented commands rejected before enqueue.
 * @param outRejectedMalformedCommands Receives the number of malformed commands rejected during main-loop parse.
 */
void MqttCommandQueue::snapshotRejectedCounters(std::uint16_t &outRejectedRetainedCommands,
                                                std::uint16_t &outRejectedOversizeCommands,
                                                std::uint16_t &outRejectedFragmentedCommands,
                                                std::uint16_t &outRejectedMalformedCommands)
{
    portENTER_CRITICAL(&this->queueMux);
    outRejectedRetainedCommands = this->rejectedRetainedCommandCount;
    outRejectedOversizeCommands = this->rejectedOversizeCommandCount;
    outRejectedFragmentedCommands = this->rejectedFragmentedCommandCount;
    outRejectedMalformedCommands = this->rejectedMalformedCommandCount;
    portEXIT_CRITICAL(&this->queueMux);
}

/**
 * @brief Consume already-published dropped-command counters.
 * @details The counters are saturated accumulators. After the bridge publishes
 *          an aggregated diagnostic event, it subtracts only the counts that
 *          were successfully reported so concurrent new drops are preserved.
 *
 * @param droppedDeviceCommands Number of published device-topic drops to subtract.
 * @param droppedServiceCommands Number of published service-topic drops to subtract.
 */
void MqttCommandQueue::consumeDroppedCounters(std::uint16_t droppedDeviceCommands, std::uint16_t droppedServiceCommands)
{
    portENTER_CRITICAL(&this->queueMux);
    this->droppedDeviceCommandCount = (this->droppedDeviceCommandCount > droppedDeviceCommands)
                                          ? static_cast<std::uint16_t>(this->droppedDeviceCommandCount - droppedDeviceCommands)
                                          : 0U;
    this->droppedServiceCommandCount = (this->droppedServiceCommandCount > droppedServiceCommands)
                                           ? static_cast<std::uint16_t>(this->droppedServiceCommandCount - droppedServiceCommands)
                                           : 0U;
    portEXIT_CRITICAL(&this->queueMux);
}

/**
 * @brief Consume already-published rejected-command counters.
 *
 * @param rejectedRetainedCommands Number of published retained-command rejections to subtract.
 * @param rejectedOversizeCommands Number of published oversize-command rejections to subtract.
 * @param rejectedFragmentedCommands Number of published fragmented-command rejections to subtract.
 * @param rejectedMalformedCommands Number of published malformed-command rejections to subtract.
 */
void MqttCommandQueue::consumeRejectedCounters(std::uint16_t rejectedRetainedCommands,
                                               std::uint16_t rejectedOversizeCommands,
                                               std::uint16_t rejectedFragmentedCommands,
                                               std::uint16_t rejectedMalformedCommands)
{
    portENTER_CRITICAL(&this->queueMux);
    this->rejectedRetainedCommandCount = (this->rejectedRetainedCommandCount > rejectedRetainedCommands)
                                             ? static_cast<std::uint16_t>(this->rejectedRetainedCommandCount - rejectedRetainedCommands)
                                             : 0U;
    this->rejectedOversizeCommandCount = (this->rejectedOversizeCommandCount > rejectedOversizeCommands)
                                             ? static_cast<std::uint16_t>(this->rejectedOversizeCommandCount - rejectedOversizeCommands)
                                             : 0U;
    this->rejectedFragmentedCommandCount =
        (this->rejectedFragmentedCommandCount > rejectedFragmentedCommands)
            ? static_cast<std::uint16_t>(this->rejectedFragmentedCommandCount - rejectedFragmentedCommands)
            : 0U;
    this->rejectedMalformedCommandCount = (this->rejectedMalformedCommandCount > rejectedMalformedCommands)
                                              ? static_cast<std::uint16_t>(this->rejectedMalformedCommandCount - rejectedMalformedCommands)
                                              : 0U;
    portEXIT_CRITICAL(&this->queueMux);
}

}  // namespace lsh::bridge
