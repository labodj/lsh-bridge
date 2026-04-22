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

/**
 * @brief Hot bridge/runtime flags and timestamps that are touched repeatedly in the main loop.
 */
struct RuntimeHotState
{
    BootstrapPhase bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;  //!< Current high-level controller sync phase.
    bool bootstrapRequestDue = true;                                      //!< True when the next bootstrap request may be sent immediately.
    bool hasPendingTopologySave = false;        //!< True while a changed topology still has to be persisted to NVS.
    bool isAuthoritativeStateDirty = false;     //!< True after fresh controller state arrived but is not yet mirrored to MQTT/Homie.
    bool canServeCachedStateRequests = false;   //!< True only after this MQTT session has already seen one fresh controller-backed state.
    bool isControllerConnected = false;         //!< Last known controller connectivity bit, used to detect link transitions.
    std::uint32_t lastBootstrapRequestMs = 0U;  //!< Real-time timestamp of the last `ASK_DETAILS` or `ASK_STATE`.
    std::uint32_t lastTopologySaveAttemptMs = 0U;       //!< Real-time timestamp of the last deferred NVS save attempt.
    std::uint32_t lastAuthoritativeStateUpdateMs = 0U;  //!< Real-time timestamp of the latest accepted authoritative state frame.
};

/**
 * @brief Forget transient runtime state that must be rebuilt from fresh controller-backed data.
 */
void clearPendingRuntimeState(ControllerSerialLink &controllerSerialLink, RuntimeHotState &runtimeState);

/**
 * @brief Schedule the next bootstrap request for the earliest safe loop iteration.
 */
void scheduleBootstrapRequestNow(RuntimeHotState &runtimeState);

/**
 * @brief Enter the phase that waits for authoritative `DEVICE_DETAILS`.
 */
void enterWaitingForDetails(ControllerSerialLink &controllerSerialLink, VirtualDevice &virtualDevice, RuntimeHotState &runtimeState);

/**
 * @brief Enter the phase that waits for one fresh authoritative `STATE` frame.
 */
void enterWaitingForState(ControllerSerialLink &controllerSerialLink, VirtualDevice &virtualDevice, RuntimeHotState &runtimeState);

/**
 * @brief Stage one validated topology update that must be persisted before reboot.
 */
void stageTopologyMigration(ControllerSerialLink &controllerSerialLink,
                            VirtualDevice &virtualDevice,
                            RuntimeHotState &runtimeState,
                            DeviceDetailsSnapshot &pendingTopologyDetails,
                            const DeviceDetailsSnapshot &details);

/**
 * @brief Mark the runtime model stale and request one authoritative state refresh.
 */
void requestAuthoritativeStateRefresh(ControllerSerialLink &controllerSerialLink,
                                      VirtualDevice &virtualDevice,
                                      RuntimeHotState &runtimeState);

/**
 * @brief React to one controller-link connectivity transition.
 */
void refreshControllerConnectivity(bool isConnectedNow,
                                   ControllerSerialLink &controllerSerialLink,
                                   VirtualDevice &virtualDevice,
                                   RuntimeHotState &runtimeState);

[[nodiscard]] auto bootstrapPhaseName(BootstrapPhase bootstrapPhase) -> const char *;

}  // namespace lsh::bridge::runtime

#endif  // LSH_BRIDGE_BRIDGE_RUNTIME_SYNC_HPP
