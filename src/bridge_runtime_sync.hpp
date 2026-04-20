/**
 * @file    bridge_runtime_sync.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares bridge-runtime synchronization helpers.
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

#ifndef LSH_BRIDGE_BRIDGE_RUNTIME_SYNC_HPP
#define LSH_BRIDGE_BRIDGE_RUNTIME_SYNC_HPP

#include <cstdint>

#include "communication/controller_serial_link.hpp"
#include "virtual_device.hpp"

namespace lsh::bridge::runtime
{

/**
 * @brief Tracks the high-level controller synchronization stage of the bridge.
 */
enum class BootstrapPhase : std::uint8_t
{
    WAITING_FOR_DETAILS,               //!< The bridge still needs one authoritative `DEVICE_DETAILS` snapshot.
    WAITING_FOR_STATE,                 //!< Topology is known, but one fresh `STATE` frame is still required.
    SYNCED,                            //!< Topology and runtime state are both aligned with the controller.
    TOPOLOGY_MIGRATION_PENDING_REBOOT  //!< A new topology was staged and the bridge now waits for a controlled reboot.
};

void clearPendingRuntimeState(ControllerSerialLink &controllerSerialLink,
                              bool &isAuthoritativeStateDirty,
                              bool &canServeCachedStateRequests);

void scheduleBootstrapRequestNow(bool &bootstrapRequestDue, std::uint32_t &lastBootstrapRequestMs);

void enterWaitingForDetails(ControllerSerialLink &controllerSerialLink,
                            VirtualDevice &virtualDevice,
                            bool &isAuthoritativeStateDirty,
                            bool &canServeCachedStateRequests,
                            BootstrapPhase &bootstrapPhase,
                            bool &bootstrapRequestDue,
                            std::uint32_t &lastBootstrapRequestMs);

void enterWaitingForState(ControllerSerialLink &controllerSerialLink,
                          VirtualDevice &virtualDevice,
                          bool &isAuthoritativeStateDirty,
                          bool &canServeCachedStateRequests,
                          BootstrapPhase &bootstrapPhase,
                          bool &bootstrapRequestDue,
                          std::uint32_t &lastBootstrapRequestMs);

void stageTopologyMigration(ControllerSerialLink &controllerSerialLink,
                            VirtualDevice &virtualDevice,
                            DeviceDetailsSnapshot &pendingTopologyDetails,
                            bool &hasPendingTopologySave,
                            bool &isAuthoritativeStateDirty,
                            bool &canServeCachedStateRequests,
                            const DeviceDetailsSnapshot &details,
                            std::uint32_t &lastTopologySaveAttemptMs,
                            BootstrapPhase &bootstrapPhase);

void requestAuthoritativeStateRefresh(ControllerSerialLink &controllerSerialLink,
                                      VirtualDevice &virtualDevice,
                                      bool &isAuthoritativeStateDirty,
                                      bool &canServeCachedStateRequests,
                                      BootstrapPhase &bootstrapPhase,
                                      bool &bootstrapRequestDue,
                                      std::uint32_t &lastBootstrapRequestMs);

void refreshControllerConnectivity(bool isConnectedNow,
                                   bool &isControllerConnected,
                                   ControllerSerialLink &controllerSerialLink,
                                   VirtualDevice &virtualDevice,
                                   bool &isAuthoritativeStateDirty,
                                   bool &canServeCachedStateRequests,
                                   BootstrapPhase &bootstrapPhase,
                                   bool &bootstrapRequestDue,
                                   std::uint32_t &lastBootstrapRequestMs);

[[nodiscard]] auto bootstrapPhaseName(BootstrapPhase bootstrapPhase) -> const char *;

}  // namespace lsh::bridge::runtime

#endif  // LSH_BRIDGE_BRIDGE_RUNTIME_SYNC_HPP
