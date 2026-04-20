/**
 * @file    bridge_runtime_diagnostics.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares MQTT diagnostic helpers used by the bridge runtime.
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

#ifndef LSH_BRIDGE_BRIDGE_RUNTIME_DIAGNOSTICS_HPP
#define LSH_BRIDGE_BRIDGE_RUNTIME_DIAGNOSTICS_HPP

#include "communication/controller_serial_link.hpp"
#include "mqtt_command_queue.hpp"

namespace lsh::bridge::runtime
{

void clearPendingBridgeDiagnostics(ControllerSerialLink &controllerSerialLink, MqttCommandQueue &mqttCommandQueue);

void publishPendingBridgeDiagnostics(ControllerSerialLink &controllerSerialLink,
                                     MqttCommandQueue &mqttCommandQueue,
                                     bool homieConnected,
                                     bool hasBridgeTopic,
                                     const char *bridgeTopic);

[[nodiscard]] auto
publishSimpleBridgeDiagnostic(bool homieConnected, bool hasBridgeTopic, const char *bridgeTopic, const char *diagnosticKind) -> bool;

[[nodiscard]] auto publishServicePingReply(bool homieConnected,
                                           bool hasBridgeTopic,
                                           const char *bridgeTopic,
                                           bool controllerConnected,
                                           bool runtimeSynchronized,
                                           const char *bootstrapPhaseName) -> bool;

}  // namespace lsh::bridge::runtime

#endif  // LSH_BRIDGE_BRIDGE_RUNTIME_DIAGNOSTICS_HPP
