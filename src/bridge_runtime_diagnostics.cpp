/**
 * @file    bridge_runtime_diagnostics.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements MQTT diagnostic helpers used by the bridge runtime.
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

#include "bridge_runtime_diagnostics.hpp"

#include <cstdint>

#include <ArduinoJson.h>

#include "communication/mqtt_publisher.hpp"

namespace
{

constexpr std::uint16_t BRIDGE_DIAGNOSTIC_DOC_SIZE = 192U;         //!< JsonDocument capacity used by every bridge-local diagnostic payload.
constexpr char BRIDGE_EVENT_KEY[] = "event";                       //!< Shared JSON key that stores the bridge event type.
constexpr char DIAGNOSTIC_EVENT[] = "diagnostic";                  //!< Event name used by generic bridge diagnostic payloads.
constexpr char SERVICE_PING_REPLY_EVENT[] = "service_ping_reply";  //!< Event name used by the bridge service-topic ping reply.
constexpr char BRIDGE_DIAGNOSTIC_KIND_KEY[] = "kind";              //!< JSON key that carries the diagnostic subtype identifier.
constexpr char PENDING_DURATION_MS_KEY[] = "pending_ms";           //!< JSON key that reports how long an actuator storm stayed pending.
constexpr char MUTATION_COUNT_KEY[] = "mutation_count";  //!< JSON key that reports how many actuator writes were coalesced and dropped.
constexpr char DROPPED_DEVICE_COMMANDS_KEY[] = "dropped_device_commands";    //!< JSON key that reports device-topic queue overflows.
constexpr char DROPPED_SERVICE_COMMANDS_KEY[] = "dropped_service_commands";  //!< JSON key that reports service-topic queue overflows.
constexpr char REJECTED_RETAINED_COMMANDS_KEY[] =
    "rejected_retained_commands";  //!< JSON key that reports retained commands rejected by policy.
constexpr char REJECTED_OVERSIZE_COMMANDS_KEY[] =
    "rejected_oversize_commands";  //!< JSON key that reports oversize commands rejected before enqueue.
constexpr char REJECTED_FRAGMENTED_COMMANDS_KEY[] =
    "rejected_fragmented_commands";  //!< JSON key that reports fragmented commands rejected before enqueue.
constexpr char REJECTED_MALFORMED_COMMANDS_KEY[] =
    "rejected_malformed_commands";                                   //!< JSON key that reports malformed commands rejected during parse.
constexpr char CONTROLLER_CONNECTED_KEY[] = "controller_connected";  //!< JSON key that exposes current controller-link liveness.
constexpr char RUNTIME_SYNCHRONIZED_KEY[] = "runtime_synchronized";  //!< JSON key that exposes bridge/runtime synchronization state.
constexpr char BOOTSTRAP_PHASE_KEY[] = "bootstrap_phase";            //!< JSON key that names the current high-level bootstrap phase.
constexpr char ACTUATOR_STORM_DROPPED_DIAGNOSTIC[] =
    "actuator_command_storm_dropped";  //!< Diagnostic subtype used when the controller-side actuator batch had to drop writes.
constexpr char MQTT_QUEUE_OVERFLOW_DIAGNOSTIC[] =
    "mqtt_queue_overflow";  //!< Diagnostic subtype used when the inbound MQTT queue overflowed.
constexpr char MQTT_COMMAND_REJECTED_DIAGNOSTIC[] =
    "mqtt_command_rejected";  //!< Diagnostic subtype used when the bridge rejects MQTT commands before routing them into the live runtime.

}  // namespace

namespace lsh::bridge::runtime
{

/**
 * @brief Forget every bridge-local diagnostic that belongs to the current MQTT session.
 *
 * @param controllerSerialLink Controller transport that stores the pending actuator-storm diagnostic.
 * @param mqttCommandQueue Bridge-side MQTT queue that accumulates dropped-command counters.
 */
void clearPendingBridgeDiagnostics(ControllerSerialLink &controllerSerialLink, MqttCommandQueue &mqttCommandQueue)
{
    controllerSerialLink.clearPendingActuatorStormDiagnostic();
    mqttCommandQueue.clearDroppedCounters();
    mqttCommandQueue.clearRejectedCounters();
}

/**
 * @brief Publish pending bridge-local diagnostics on the `bridge` topic.
 * @details Diagnostics are emitted from the main loop, not from callbacks, so
 *          the bridge never tries to publish MQTT while already executing the
 *          MQTT client's message callback.
 *
 * @param controllerSerialLink Controller transport that exposes pending actuator-storm diagnostics.
 * @param mqttCommandQueue Bridge-side MQTT queue that accumulates dropped-command counters.
 * @param homieConnected True when the bridge currently has a live Homie/MQTT session.
 * @param hasBridgeTopic True when the bridge can publish on its device-scoped `bridge` topic.
 * @param bridgeTopic Device-scoped MQTT topic used for bridge-local runtime events.
 */
void publishPendingBridgeDiagnostics(ControllerSerialLink &controllerSerialLink,
                                     MqttCommandQueue &mqttCommandQueue,
                                     bool homieConnected,
                                     bool hasBridgeTopic,
                                     const char *bridgeTopic)
{
    if (!homieConnected || !hasBridgeTopic || bridgeTopic == nullptr || bridgeTopic[0] == '\0')
    {
        return;
    }

    ControllerSerialLink::ActuatorStormDiagnostic actuatorStormDiagnostic{};
    if (controllerSerialLink.peekPendingActuatorStormDiagnostic(actuatorStormDiagnostic))
    {
        StaticJsonDocument<BRIDGE_DIAGNOSTIC_DOC_SIZE> diagnosticDoc;
        diagnosticDoc[BRIDGE_EVENT_KEY] = DIAGNOSTIC_EVENT;
        diagnosticDoc[BRIDGE_DIAGNOSTIC_KIND_KEY] = ACTUATOR_STORM_DROPPED_DIAGNOSTIC;
        diagnosticDoc[PENDING_DURATION_MS_KEY] = actuatorStormDiagnostic.pendingDurationMs;
        diagnosticDoc[MUTATION_COUNT_KEY] = actuatorStormDiagnostic.mutationCount;

        if (MqttPublisher::sendJson(diagnosticDoc, bridgeTopic, false, 1))
        {
            controllerSerialLink.clearPendingActuatorStormDiagnostic();
        }
    }

    std::uint16_t droppedDeviceCommands = 0U;
    std::uint16_t droppedServiceCommands = 0U;
    mqttCommandQueue.snapshotDroppedCounters(droppedDeviceCommands, droppedServiceCommands);

    StaticJsonDocument<BRIDGE_DIAGNOSTIC_DOC_SIZE> diagnosticDoc;
    if (droppedDeviceCommands > 0U || droppedServiceCommands > 0U)
    {
        diagnosticDoc[BRIDGE_EVENT_KEY] = DIAGNOSTIC_EVENT;
        diagnosticDoc[BRIDGE_DIAGNOSTIC_KIND_KEY] = MQTT_QUEUE_OVERFLOW_DIAGNOSTIC;
        if (droppedDeviceCommands > 0U)
        {
            diagnosticDoc[DROPPED_DEVICE_COMMANDS_KEY] = droppedDeviceCommands;
        }
        if (droppedServiceCommands > 0U)
        {
            diagnosticDoc[DROPPED_SERVICE_COMMANDS_KEY] = droppedServiceCommands;
        }

        if (MqttPublisher::sendJson(diagnosticDoc, bridgeTopic, false, 1))
        {
            mqttCommandQueue.consumeDroppedCounters(droppedDeviceCommands, droppedServiceCommands);
        }
    }

    std::uint16_t rejectedRetainedCommands = 0U;
    std::uint16_t rejectedOversizeCommands = 0U;
    std::uint16_t rejectedFragmentedCommands = 0U;
    std::uint16_t rejectedMalformedCommands = 0U;
    mqttCommandQueue.snapshotRejectedCounters(rejectedRetainedCommands, rejectedOversizeCommands, rejectedFragmentedCommands,
                                              rejectedMalformedCommands);

    if (rejectedRetainedCommands == 0U && rejectedOversizeCommands == 0U && rejectedFragmentedCommands == 0U &&
        rejectedMalformedCommands == 0U)
    {
        return;
    }

    diagnosticDoc.clear();
    diagnosticDoc[BRIDGE_EVENT_KEY] = DIAGNOSTIC_EVENT;
    diagnosticDoc[BRIDGE_DIAGNOSTIC_KIND_KEY] = MQTT_COMMAND_REJECTED_DIAGNOSTIC;
    if (rejectedRetainedCommands > 0U)
    {
        diagnosticDoc[REJECTED_RETAINED_COMMANDS_KEY] = rejectedRetainedCommands;
    }
    if (rejectedOversizeCommands > 0U)
    {
        diagnosticDoc[REJECTED_OVERSIZE_COMMANDS_KEY] = rejectedOversizeCommands;
    }
    if (rejectedFragmentedCommands > 0U)
    {
        diagnosticDoc[REJECTED_FRAGMENTED_COMMANDS_KEY] = rejectedFragmentedCommands;
    }
    if (rejectedMalformedCommands > 0U)
    {
        diagnosticDoc[REJECTED_MALFORMED_COMMANDS_KEY] = rejectedMalformedCommands;
    }

    if (MqttPublisher::sendJson(diagnosticDoc, bridgeTopic, false, 1))
    {
        mqttCommandQueue.consumeRejectedCounters(rejectedRetainedCommands, rejectedOversizeCommands, rejectedFragmentedCommands,
                                                 rejectedMalformedCommands);
    }
}

/**
 * @brief Publish one simple bridge-local diagnostic with no extra payload.
 *
 * @param homieConnected True when the bridge currently has a live Homie/MQTT session.
 * @param hasBridgeTopic True when the bridge can publish on its device-scoped `bridge` topic.
 * @param bridgeTopic Device-scoped MQTT topic used for bridge-local runtime events.
 * @param diagnosticKind Short stable identifier of the lifecycle event to publish.
 * @return true if the MQTT client accepted the diagnostic publish.
 * @return false if the diagnostic could not be published.
 */
auto publishSimpleBridgeDiagnostic(bool homieConnected, bool hasBridgeTopic, const char *bridgeTopic, const char *diagnosticKind) -> bool
{
    if (!homieConnected || !hasBridgeTopic || bridgeTopic == nullptr || bridgeTopic[0] == '\0' || diagnosticKind == nullptr ||
        diagnosticKind[0] == '\0')
    {
        return false;
    }

    StaticJsonDocument<BRIDGE_DIAGNOSTIC_DOC_SIZE> diagnosticDoc;
    diagnosticDoc[BRIDGE_EVENT_KEY] = DIAGNOSTIC_EVENT;
    diagnosticDoc[BRIDGE_DIAGNOSTIC_KIND_KEY] = diagnosticKind;
    return MqttPublisher::sendJson(diagnosticDoc, bridgeTopic, false, 1);
}

/**
 * @brief Publish the service-topic `PING` reply on the bridge-local topic.
 * @details This reply intentionally reports bridge/runtime health only. It
 *          never claims that the controller-backed device topic is usable
 *          unless both controller connectivity and runtime synchronization are
 *          currently true.
 *
 * @param homieConnected True when the bridge currently has a live Homie/MQTT session.
 * @param hasBridgeTopic True when the bridge can publish on its device-scoped `bridge` topic.
 * @param bridgeTopic Device-scoped MQTT topic used for bridge-local runtime events.
 * @param controllerConnected Current controller-link liveness bit.
 * @param runtimeSynchronized Current bridge-runtime synchronization bit.
 * @param bootstrapPhaseName Stable string that names the current bootstrap phase.
 * @return true if the MQTT client accepted the ping reply publish.
 * @return false if the ping reply could not be published.
 */
auto publishServicePingReply(bool homieConnected,
                             bool hasBridgeTopic,
                             const char *bridgeTopic,
                             bool controllerConnected,
                             bool runtimeSynchronized,
                             const char *bootstrapPhaseName) -> bool
{
    if (!homieConnected || !hasBridgeTopic || bridgeTopic == nullptr || bridgeTopic[0] == '\0' || bootstrapPhaseName == nullptr)
    {
        return false;
    }

    StaticJsonDocument<BRIDGE_DIAGNOSTIC_DOC_SIZE> bridgeDoc;
    bridgeDoc[BRIDGE_EVENT_KEY] = SERVICE_PING_REPLY_EVENT;
    bridgeDoc[CONTROLLER_CONNECTED_KEY] = controllerConnected;
    bridgeDoc[RUNTIME_SYNCHRONIZED_KEY] = runtimeSynchronized;
    bridgeDoc[BOOTSTRAP_PHASE_KEY] = bootstrapPhaseName;
    return MqttPublisher::sendJson(bridgeDoc, bridgeTopic, false, 1);
}

}  // namespace lsh::bridge::runtime
