/**
 * @file    lsh_node.cpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Implements the Homie node wrapper used for one cached controller
 * actuator.
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

#include "lsh_node.hpp"

#include <Homie.h>

#include "communication/controller_serial_link.hpp"
#include "constants/homie.hpp"
#include "utils/conversions.hpp"
#include "virtual_device.hpp"

/**
 * @brief Callback for MQTT messages received on the '/set' topic for this node.
 * @details This function is triggered by Homie when a command is sent to this node.
 *          It validates the command and updates the bridge-side desired-state shadow
 *          so a short coalescing window can emit a single SET_STATE toward the controller.
 * @param range The HomieRange object (not used here).
 * @param value The message payload, expected to be "true" or "false".
 */
void LSHNode::handleSetCommand(const HomieRange &range, const String &value)
{
    DP_CONTEXT();
    DPL("↑ ID: ", this->actuatorId, " | value: ", value.c_str());
    (void)range;

    using constants::homie::BOOL_FALSE_LITERAL;
    using constants::homie::BOOL_TRUE_LITERAL;

    // The bridge only stages Homie-originated actuator intent when its cached model is authoritative.
    // During re-sync Homie writes are ignored until a fresh controller state frame re-establishes
    // the baseline; raw LSH writes are handled separately by the MQTT input path.
    if (!this->virtualDevice.isRuntimeSynchronized())
    {
        this->controllerSerialLink.recordRejectedHomieCommand(ControllerSerialLink::HomieRejectedCommandReason::RuntimeDesynchronized);
        DPL("Bridge model is waiting for a fresh state sync. Ignoring command.");
        return;
    }

    // The lambda in the constructor always consumes this Homie callback. Invalid
    // values are ignored here so they cannot reach the controller as ambiguous state.
    if ((value != BOOL_TRUE_LITERAL) && (value != BOOL_FALSE_LITERAL))
    {
        this->controllerSerialLink.recordRejectedHomieCommand(ControllerSerialLink::HomieRejectedCommandReason::InvalidPayload);
        return;
    }

    const bool newState = (value == BOOL_TRUE_LITERAL);  // New state
    if (!this->controllerSerialLink.stageSingleActuatorCommand(this->actuatorId, newState))
    {
        this->controllerSerialLink.recordRejectedHomieCommand(ControllerSerialLink::HomieRejectedCommandReason::StageFailed);
        DPL("Failed to stage desired actuator state for this Homie node.");
        return;
    }

    Homie.getLogger() << "Command sent to turn light " << (newState ? "on" : "off") << endl;
}

/**
 * @brief Sends the current state of the actuator to its MQTT state topic.
 * @details This function is called to synchronize the MQTT state with the
 *          actuator's actual state held in `VirtualDevice`.
 *
 * @return true if Homie accepted the publish request.
 * @return false if Homie rejected the publish request.
 */
auto LSHNode::sendState() const -> bool
{
    DP_CONTEXT();
    DP("↑ ID: ", this->getId());

    const bool stateToSend = this->virtualDevice.getStateByIndexUnchecked(this->actuatorIndex);
    DPL(" | state: ", stateToSend);

    using constants::homie::HOMIE_PROPERTY_ADVERTISE;

    const auto packetId = this->setProperty(HOMIE_PROPERTY_ADVERTISE)
                              .overwriteSetter(false)
                              .setQos(1)
                              .send(utils::conversions::to_literal(stateToSend));

    if (packetId == 0U)
    {
        DPL("Homie publish rejected for actuator ID ", this->actuatorId, ".");
        return false;
    }

    return true;
}
