/**
 * @file    mqtt_command_queue.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the fixed MQTT command queue used by the bridge runtime.
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

#ifndef LSH_BRIDGE_MQTT_COMMAND_QUEUE_HPP
#define LSH_BRIDGE_MQTT_COMMAND_QUEUE_HPP

#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>

#include "constants/controller_serial.hpp"
#include "constants/configs/runtime.hpp"

namespace lsh::bridge
{

/**
 * @brief Identify which MQTT topic family produced one queued command.
 */
enum class MqttCommandSource : std::uint8_t
{
    Device,  //!< Command received from the device-scoped controller topic.
    Service  //!< Command received from the bridge-scoped service topic.
};

/**
 * @brief Classify why one inbound MQTT command was rejected before enqueue.
 */
enum class MqttRejectedCommandReason : std::uint8_t
{
    Retained,    //!< The broker replayed a retained command that the bridge must ignore.
    Oversize,    //!< The payload exceeded the fixed inbound MQTT command buffer.
    Fragmented,  //!< AsyncMqttClient delivered the command as fragments, which the bridge rejects by design.
    Malformed    //!< The payload reached the main loop but was not a valid bridge command document.
};

/**
 * @brief Internal lifecycle of one MQTT command queue slot.
 */
enum class MqttCommandSlotState : std::uint8_t
{
    Free,     //!< The slot can be reserved by a producer.
    Writing,  //!< A producer reserved the slot and is copying payload bytes outside the queue lock.
    Ready,    //!< The slot contains one complete command visible to the main loop.
    Reading   //!< The main loop reserved the slot and is copying payload bytes outside the queue lock.
};

/**
 * @brief Fixed-size storage slot for one queued MQTT command.
 */
struct QueuedMqttCommand
{
    MqttCommandSource source = MqttCommandSource::Device;  //!< Topic family that produced this command.
    std::uint16_t length = 0U;                             //!< Number of valid bytes currently stored in `payload`.
    MqttCommandSlotState state = MqttCommandSlotState::Free;  //!< Internal slot lifecycle, ignored for stack-local command copies.
    std::uint16_t generation = 0U;  //!< Queue generation that reserved this slot, used to reject stale commits after clear().
    std::uint16_t sequence = 0U;    //!< Reservation order, used to preserve command ordering while copies happen outside the lock.
    // Intentionally not zero-initialized for stack locals: dequeue() writes only
    // `length` live bytes, and parsers receive that exact length. This avoids a
    // full command-buffer memset before every queued MQTT command is processed.
    std::uint8_t payload[constants::controllerSerial::MQTT_COMMAND_MESSAGE_MAX_SIZE];  //!< Raw queued MQTT payload bytes.
};

/**
 * @brief Fixed ring buffer used to decouple the MQTT callback from the main loop.
 * @details The queue is intentionally small and fully static. The AsyncMqttClient
 *          callback only validates and copies complete frames into this buffer;
 *          parsing happens later in the main loop where it can be coordinated
 *          with controller serial traffic. Slot payload copies happen outside
 *          the queue lock; disconnect teardown advances the generation and
 *          only frees completed slots so in-flight copies cannot race with a
 *          later producer reusing the same storage.
 */
class MqttCommandQueue
{
private:
    static constexpr std::uint16_t DIAGNOSTIC_COUNTER_MAX = 0xFFFFU;  //!< Saturation limit for queued-drop diagnostics.

    QueuedMqttCommand queue[constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY]{};  //!< Fixed storage for queued MQTT commands.
    portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;  //!< Protects queue indices and drop counters across callback/main-loop access.
    std::uint8_t tail = 0U;                                //!< Ring-buffer index of the next free enqueue slot.
    std::uint8_t count = 0U;                               //!< Number of non-free slots, including in-flight Writing/Reading slots.
    std::uint16_t queueGeneration = 1U;                    //!< Incremented by clear() so stale unlocked copies cannot commit.
    std::uint16_t nextSequence = 1U;                       //!< Monotonic slot reservation sequence; wrap-safe while capacity stays small.
    std::uint16_t droppedDeviceCommandCount = 0U;          //!< Aggregated number of dropped device-topic commands.
    std::uint16_t droppedServiceCommandCount = 0U;         //!< Aggregated number of dropped service-topic commands.
    std::uint16_t rejectedRetainedCommandCount = 0U;       //!< Aggregated number of retained commands rejected before enqueue.
    std::uint16_t rejectedOversizeCommandCount = 0U;       //!< Aggregated number of oversize commands rejected before enqueue.
    std::uint16_t rejectedFragmentedCommandCount = 0U;     //!< Aggregated number of fragmented commands rejected before enqueue.
    std::uint16_t rejectedMalformedCommandCount = 0U;      //!< Aggregated number of malformed commands rejected during main-loop parse.

public:
    [[nodiscard]] auto enqueue(MqttCommandSource source, const char *payload, std::size_t payloadLength) -> bool;
    [[nodiscard]] auto dequeue(QueuedMqttCommand &outCommand) -> bool;
    void clear();
    void recordDroppedCommand(MqttCommandSource source);
    void recordRejectedCommand(MqttRejectedCommandReason reason);
    void clearDroppedCounters();
    void clearRejectedCounters();
    void snapshotDroppedCounters(std::uint16_t &outDroppedDeviceCommands, std::uint16_t &outDroppedServiceCommands);
    void snapshotRejectedCounters(std::uint16_t &outRejectedRetainedCommands,
                                  std::uint16_t &outRejectedOversizeCommands,
                                  std::uint16_t &outRejectedFragmentedCommands,
                                  std::uint16_t &outRejectedMalformedCommands);
    void consumeDroppedCounters(std::uint16_t droppedDeviceCommands, std::uint16_t droppedServiceCommands);
    void consumeRejectedCounters(std::uint16_t rejectedRetainedCommands,
                                 std::uint16_t rejectedOversizeCommands,
                                 std::uint16_t rejectedFragmentedCommands,
                                 std::uint16_t rejectedMalformedCommands);
};

}  // namespace lsh::bridge

#endif  // LSH_BRIDGE_MQTT_COMMAND_QUEUE_HPP
