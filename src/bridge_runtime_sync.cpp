/**
 * @file    bridge_runtime_sync.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements bridge/controller runtime synchronization helpers.
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

#include "bridge_runtime_sync.hpp"

namespace lsh::bridge::runtime
{

/**
 * @brief Forget transient runtime state that must be rebuilt from fresh
 * controller-backed data.
 * @details The topology cache stays intact. Only pending actuator writes and
 *          session-local state-serving flags are cleared so the bridge can
 *          request a clean authoritative refresh.
 *
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param isAuthoritativeStateDirty bridge flag that tracks delayed state publish work.
 * @param canServeCachedStateRequests bridge flag that allows answering `REQUEST_STATE`
 *        directly from the synchronized cache.
 */
void clearPendingRuntimeState(ControllerSerialLink &controllerSerialLink,
                              bool &isAuthoritativeStateDirty,
                              bool &canServeCachedStateRequests)
{
    controllerSerialLink.clearPendingActuatorBatch();
    isAuthoritativeStateDirty = false;
    canServeCachedStateRequests = false;
}

/**
 * @brief Schedule the next bootstrap request for the earliest safe loop iteration.
 * @details The caller still relies on the main loop to pace `ASK_DETAILS` and
 *          `ASK_STATE`. This helper only clears the wait timer so the next loop
 *          can retry immediately.
 *
 * @param bootstrapRequestDue flag consumed by the bootstrap state machine.
 * @param lastBootstrapRequestMs timestamp of the last accepted bootstrap request.
 */
void scheduleBootstrapRequestNow(bool &bootstrapRequestDue, std::uint32_t &lastBootstrapRequestMs)
{
    bootstrapRequestDue = true;
    lastBootstrapRequestMs = 0U;
}

/**
 * @brief Enter the phase that waits for authoritative `DEVICE_DETAILS`.
 * @details This is the safest recovery phase after controller `BOOT` or any
 *          topology mismatch because actuator IDs and button IDs cannot be
 *          trusted until a fresh details snapshot arrives.
 *
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that must forget its runtime state.
 * @param isAuthoritativeStateDirty bridge flag that tracks delayed state publish work.
 * @param canServeCachedStateRequests bridge flag that controls cached `REQUEST_STATE` replies.
 * @param bootstrapPhase high-level synchronization phase to update.
 * @param bootstrapRequestDue retry flag consumed by the bootstrap loop.
 * @param lastBootstrapRequestMs timestamp of the last accepted bootstrap request.
 */
void enterWaitingForDetails(ControllerSerialLink &controllerSerialLink,
                            VirtualDevice &virtualDevice,
                            bool &isAuthoritativeStateDirty,
                            bool &canServeCachedStateRequests,
                            BootstrapPhase &bootstrapPhase,
                            bool &bootstrapRequestDue,
                            std::uint32_t &lastBootstrapRequestMs)
{
    clearPendingRuntimeState(controllerSerialLink, isAuthoritativeStateDirty, canServeCachedStateRequests);
    virtualDevice.invalidateRuntimeModel();
    bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;
    scheduleBootstrapRequestNow(bootstrapRequestDue, lastBootstrapRequestMs);
}

/**
 * @brief Enter the phase that waits for one fresh authoritative `STATE` frame.
 * @details This phase is used when topology is still trusted but runtime state
 *          may be stale, for example after controller reconnect or after one
 *          malformed state frame.
 *
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that must forget its runtime state.
 * @param isAuthoritativeStateDirty bridge flag that tracks delayed state publish work.
 * @param canServeCachedStateRequests bridge flag that controls cached `REQUEST_STATE` replies.
 * @param bootstrapPhase high-level synchronization phase to update.
 * @param bootstrapRequestDue retry flag consumed by the bootstrap loop.
 * @param lastBootstrapRequestMs timestamp of the last accepted bootstrap request.
 */
void enterWaitingForState(ControllerSerialLink &controllerSerialLink,
                          VirtualDevice &virtualDevice,
                          bool &isAuthoritativeStateDirty,
                          bool &canServeCachedStateRequests,
                          BootstrapPhase &bootstrapPhase,
                          bool &bootstrapRequestDue,
                          std::uint32_t &lastBootstrapRequestMs)
{
    clearPendingRuntimeState(controllerSerialLink, isAuthoritativeStateDirty, canServeCachedStateRequests);
    virtualDevice.invalidateRuntimeModel();
    bootstrapPhase = BootstrapPhase::WAITING_FOR_STATE;
    scheduleBootstrapRequestNow(bootstrapRequestDue, lastBootstrapRequestMs);
}

/**
 * @brief Stage one validated topology update that must be persisted before reboot.
 * @details The bridge intentionally avoids hot-rebuilding Homie nodes and MQTT
 *          topic state in place. Persisting the new topology and rebooting once
 *          keeps startup as the single place that reconstructs the runtime model.
 *
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that must forget its runtime state.
 * @param pendingTopologyDetails storage for the validated topology waiting to be saved.
 * @param hasPendingTopologySave flag that tells the main loop to attempt the deferred save.
 * @param isAuthoritativeStateDirty bridge flag that tracks delayed state publish work.
 * @param canServeCachedStateRequests bridge flag that controls cached `REQUEST_STATE` replies.
 * @param details freshly received validated topology snapshot.
 * @param lastTopologySaveAttemptMs timestamp of the last deferred save attempt.
 * @param bootstrapPhase high-level synchronization phase to update.
 */
void stageTopologyMigration(ControllerSerialLink &controllerSerialLink,
                            VirtualDevice &virtualDevice,
                            DeviceDetailsSnapshot &pendingTopologyDetails,
                            bool &hasPendingTopologySave,
                            bool &isAuthoritativeStateDirty,
                            bool &canServeCachedStateRequests,
                            const DeviceDetailsSnapshot &details,
                            std::uint32_t &lastTopologySaveAttemptMs,
                            BootstrapPhase &bootstrapPhase)
{
    clearPendingRuntimeState(controllerSerialLink, isAuthoritativeStateDirty, canServeCachedStateRequests);
    virtualDevice.invalidateRuntimeModel();
    pendingTopologyDetails = details;
    hasPendingTopologySave = true;
    lastTopologySaveAttemptMs = 0U;
    bootstrapPhase = BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT;
}

/**
 * @brief Mark the runtime model stale and request one authoritative state refresh.
 * @details This is a convenience wrapper around `enterWaitingForState()` for
 *          call sites that care about intent more than the exact phase name.
 *
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that must forget its runtime state.
 * @param isAuthoritativeStateDirty bridge flag that tracks delayed state publish work.
 * @param canServeCachedStateRequests bridge flag that controls cached `REQUEST_STATE` replies.
 * @param bootstrapPhase high-level synchronization phase to update.
 * @param bootstrapRequestDue retry flag consumed by the bootstrap loop.
 * @param lastBootstrapRequestMs timestamp of the last accepted bootstrap request.
 */
void requestAuthoritativeStateRefresh(ControllerSerialLink &controllerSerialLink,
                                      VirtualDevice &virtualDevice,
                                      bool &isAuthoritativeStateDirty,
                                      bool &canServeCachedStateRequests,
                                      BootstrapPhase &bootstrapPhase,
                                      bool &bootstrapRequestDue,
                                      std::uint32_t &lastBootstrapRequestMs)
{
    enterWaitingForState(controllerSerialLink, virtualDevice, isAuthoritativeStateDirty, canServeCachedStateRequests, bootstrapPhase,
                         bootstrapRequestDue, lastBootstrapRequestMs);
}

/**
 * @brief React to one controller-link connectivity transition.
 * @details Runtime state is dropped on disconnect because it may already be
 *          stale. On reconnect the helper schedules only the minimum safe
 *          follow-up request for the current phase.
 *
 * @param isConnectedNow freshly observed controller connectivity bit.
 * @param isControllerConnected last known connectivity bit stored by the bridge.
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that may need to invalidate its runtime state.
 * @param isAuthoritativeStateDirty bridge flag that tracks delayed state publish work.
 * @param canServeCachedStateRequests bridge flag that controls cached `REQUEST_STATE` replies.
 * @param bootstrapPhase high-level synchronization phase to update.
 * @param bootstrapRequestDue retry flag consumed by the bootstrap loop.
 * @param lastBootstrapRequestMs timestamp of the last accepted bootstrap request.
 */
void refreshControllerConnectivity(bool isConnectedNow,
                                   bool &isControllerConnected,
                                   ControllerSerialLink &controllerSerialLink,
                                   VirtualDevice &virtualDevice,
                                   bool &isAuthoritativeStateDirty,
                                   bool &canServeCachedStateRequests,
                                   BootstrapPhase &bootstrapPhase,
                                   bool &bootstrapRequestDue,
                                   std::uint32_t &lastBootstrapRequestMs)
{
    if (isConnectedNow == isControllerConnected)
    {
        return;
    }

    isControllerConnected = isConnectedNow;
    if (!isConnectedNow)
    {
        clearPendingRuntimeState(controllerSerialLink, isAuthoritativeStateDirty, canServeCachedStateRequests);
        virtualDevice.invalidateRuntimeModel();
        if (bootstrapPhase != BootstrapPhase::WAITING_FOR_DETAILS && bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            bootstrapPhase = BootstrapPhase::WAITING_FOR_STATE;
            scheduleBootstrapRequestNow(bootstrapRequestDue, lastBootstrapRequestMs);
        }
        return;
    }

    if (bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
    {
        scheduleBootstrapRequestNow(bootstrapRequestDue, lastBootstrapRequestMs);
        return;
    }

    if (bootstrapPhase == BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
    {
        return;
    }

    if (!virtualDevice.isRuntimeSynchronized())
    {
        requestAuthoritativeStateRefresh(controllerSerialLink, virtualDevice, isAuthoritativeStateDirty, canServeCachedStateRequests,
                                         bootstrapPhase, bootstrapRequestDue, lastBootstrapRequestMs);
    }
}

/**
 * @brief Return the stable human-readable name of one bootstrap phase.
 *
 * @param bootstrapPhase high-level synchronization phase to stringify.
 * @return const char * stable string literal describing the supplied phase.
 */
[[nodiscard]] auto bootstrapPhaseName(BootstrapPhase bootstrapPhase) -> const char *
{
    switch (bootstrapPhase)
    {
    case BootstrapPhase::WAITING_FOR_DETAILS:
        return "waiting_details";

    case BootstrapPhase::WAITING_FOR_STATE:
        return "waiting_state";

    case BootstrapPhase::SYNCED:
        return "synced";

    case BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT:
        return "topology_migration_pending_reboot";

    default:
        return "waiting_details";
    }
}

}  // namespace lsh::bridge::runtime
