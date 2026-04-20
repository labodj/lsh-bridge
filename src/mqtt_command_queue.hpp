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
    Retained,   //!< The broker replayed a retained command that the bridge must ignore.
    Oversize,   //!< The payload exceeded the fixed inbound MQTT command buffer.
    Fragmented  //!< AsyncMqttClient delivered the command as fragments, which the bridge rejects by design.
};

/**
 * @brief Fixed-size storage slot for one queued MQTT command.
 */
struct QueuedMqttCommand
{
    MqttCommandSource source = MqttCommandSource::Device;  //!< Topic family that produced this command.
    std::uint16_t length = 0U;                             //!< Number of valid bytes currently stored in `payload`.
    bool ready = false;                                    //!< True only after the producer finished copying `payload`.
    std::uint8_t payload[constants::controllerSerial::MQTT_COMMAND_MESSAGE_MAX_SIZE]{};  //!< Raw queued MQTT payload bytes.
};

/**
 * @brief Fixed ring buffer used to decouple the MQTT callback from the main loop.
 * @details The queue is intentionally small and fully static. The AsyncMqttClient
 *          callback only validates and copies complete frames into this buffer;
 *          parsing happens later in the main loop where it can be coordinated
 *          with controller serial traffic. Enqueue keeps the full slot commit
 *          under the queue lock so disconnect teardown cannot reuse a slot
 *          while an older callback is still finishing its copy.
 */
class MqttCommandQueue
{
private:
    static constexpr std::uint16_t DIAGNOSTIC_COUNTER_MAX = 0xFFFFU;  //!< Saturation limit for queued-drop diagnostics.

    QueuedMqttCommand queue[constants::runtime::MQTT_COMMAND_QUEUE_CAPACITY]{};  //!< Fixed storage for queued MQTT commands.
    portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;  //!< Protects queue indices and drop counters across callback/main-loop access.
    std::uint8_t head = 0U;                                //!< Ring-buffer index of the next command to dequeue.
    std::uint8_t tail = 0U;                                //!< Ring-buffer index of the next free enqueue slot.
    std::uint8_t count = 0U;                               //!< Number of commands currently stored in `queue`.
    std::uint16_t droppedDeviceCommandCount = 0U;          //!< Aggregated number of dropped device-topic commands.
    std::uint16_t droppedServiceCommandCount = 0U;         //!< Aggregated number of dropped service-topic commands.
    std::uint16_t rejectedRetainedCommandCount = 0U;       //!< Aggregated number of retained commands rejected before enqueue.
    std::uint16_t rejectedOversizeCommandCount = 0U;       //!< Aggregated number of oversize commands rejected before enqueue.
    std::uint16_t rejectedFragmentedCommandCount = 0U;     //!< Aggregated number of fragmented commands rejected before enqueue.

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
                                  std::uint16_t &outRejectedFragmentedCommands);
    void consumeDroppedCounters(std::uint16_t droppedDeviceCommands, std::uint16_t droppedServiceCommands);
    void consumeRejectedCounters(std::uint16_t rejectedRetainedCommands,
                                 std::uint16_t rejectedOversizeCommands,
                                 std::uint16_t rejectedFragmentedCommands);
};

}  // namespace lsh::bridge

#endif  // LSH_BRIDGE_MQTT_COMMAND_QUEUE_HPP
