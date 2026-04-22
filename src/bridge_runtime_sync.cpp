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
 * @param runtimeState hot bridge-side runtime flags and timestamps that must be cleared.
 */
void clearPendingRuntimeState(ControllerSerialLink &controllerSerialLink, RuntimeHotState &runtimeState)
{
    controllerSerialLink.clearPendingActuatorBatch();
    runtimeState.isAuthoritativeStateDirty = false;
    runtimeState.canServeCachedStateRequests = false;
}

/**
 * @brief Schedule the next bootstrap request for the earliest safe loop iteration.
 * @details The caller still relies on the main loop to pace `ASK_DETAILS` and
 *          `ASK_STATE`. This helper only clears the wait timer so the next loop
 *          can retry immediately.
 *
 * @param runtimeState hot bridge-side runtime flags and timestamps to update.
 */
void scheduleBootstrapRequestNow(RuntimeHotState &runtimeState)
{
    runtimeState.bootstrapRequestDue = true;
    runtimeState.lastBootstrapRequestMs = 0U;
}

/**
 * @brief Enter the phase that waits for authoritative `DEVICE_DETAILS`.
 * @details This is the safest recovery phase after controller `BOOT` or any
 *          topology mismatch because actuator IDs and button IDs cannot be
 *          trusted until a fresh details snapshot arrives.
 *
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that must forget its runtime state.
 * @param runtimeState hot bridge-side runtime flags and timestamps to update.
 */
void enterWaitingForDetails(ControllerSerialLink &controllerSerialLink, VirtualDevice &virtualDevice, RuntimeHotState &runtimeState)
{
    clearPendingRuntimeState(controllerSerialLink, runtimeState);
    virtualDevice.invalidateRuntimeModel();
    runtimeState.bootstrapPhase = BootstrapPhase::WAITING_FOR_DETAILS;
    scheduleBootstrapRequestNow(runtimeState);
}

/**
 * @brief Enter the phase that waits for one fresh authoritative `STATE` frame.
 * @details This phase is used when topology is still trusted but runtime state
 *          may be stale, for example after controller reconnect or after one
 *          malformed state frame.
 *
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that must forget its runtime state.
 * @param runtimeState hot bridge-side runtime flags and timestamps to update.
 */
void enterWaitingForState(ControllerSerialLink &controllerSerialLink, VirtualDevice &virtualDevice, RuntimeHotState &runtimeState)
{
    clearPendingRuntimeState(controllerSerialLink, runtimeState);
    virtualDevice.invalidateRuntimeModel();
    runtimeState.bootstrapPhase = BootstrapPhase::WAITING_FOR_STATE;
    scheduleBootstrapRequestNow(runtimeState);
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
 * @param runtimeState hot bridge-side runtime flags and timestamps to update.
 * @param details freshly received validated topology snapshot.
 */
void stageTopologyMigration(ControllerSerialLink &controllerSerialLink,
                            VirtualDevice &virtualDevice,
                            RuntimeHotState &runtimeState,
                            DeviceDetailsSnapshot &pendingTopologyDetails,
                            const DeviceDetailsSnapshot &details)
{
    clearPendingRuntimeState(controllerSerialLink, runtimeState);
    virtualDevice.invalidateRuntimeModel();
    pendingTopologyDetails = details;
    runtimeState.hasPendingTopologySave = true;
    runtimeState.lastTopologySaveAttemptMs = 0U;
    runtimeState.bootstrapPhase = BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT;
}

/**
 * @brief Mark the runtime model stale and request one authoritative state refresh.
 * @details This is a convenience wrapper around `enterWaitingForState()` for
 *          call sites that care about intent more than the exact phase name.
 *
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that must forget its runtime state.
 * @param runtimeState hot bridge-side runtime flags and timestamps to update.
 */
void requestAuthoritativeStateRefresh(ControllerSerialLink &controllerSerialLink,
                                      VirtualDevice &virtualDevice,
                                      RuntimeHotState &runtimeState)
{
    enterWaitingForState(controllerSerialLink, virtualDevice, runtimeState);
}

/**
 * @brief React to one controller-link connectivity transition.
 * @details Runtime state is dropped on disconnect because it may already be
 *          stale. On reconnect the helper schedules only the minimum safe
 *          follow-up request for the current phase.
 *
 * @param isConnectedNow freshly observed controller connectivity bit.
 * @param controllerSerialLink controller link that owns the pending actuator batch.
 * @param virtualDevice in-memory controller model that may need to invalidate its runtime state.
 * @param runtimeState hot bridge-side runtime flags and timestamps that track the session lifecycle.
 */
void refreshControllerConnectivity(bool isConnectedNow,
                                   ControllerSerialLink &controllerSerialLink,
                                   VirtualDevice &virtualDevice,
                                   RuntimeHotState &runtimeState)
{
    if (isConnectedNow == runtimeState.isControllerConnected)
    {
        return;
    }

    runtimeState.isControllerConnected = isConnectedNow;
    if (!isConnectedNow)
    {
        clearPendingRuntimeState(controllerSerialLink, runtimeState);
        virtualDevice.invalidateRuntimeModel();
        if (runtimeState.bootstrapPhase != BootstrapPhase::WAITING_FOR_DETAILS &&
            runtimeState.bootstrapPhase != BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
        {
            runtimeState.bootstrapPhase = BootstrapPhase::WAITING_FOR_STATE;
            scheduleBootstrapRequestNow(runtimeState);
        }
        return;
    }

    if (runtimeState.bootstrapPhase == BootstrapPhase::WAITING_FOR_DETAILS)
    {
        scheduleBootstrapRequestNow(runtimeState);
        return;
    }

    if (runtimeState.bootstrapPhase == BootstrapPhase::TOPOLOGY_MIGRATION_PENDING_REBOOT)
    {
        return;
    }

    if (!virtualDevice.isRuntimeSynchronized())
    {
        requestAuthoritativeStateRefresh(controllerSerialLink, virtualDevice, runtimeState);
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
