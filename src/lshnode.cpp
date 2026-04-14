#include "lshnode.hpp"

#include <Homie.h>

#include "communication/ardcom.hpp"
#include "constants/homie.hpp"
#include "debug/debug.hpp"
#include "utils/conversions.hpp"
#include "virtualdevice.hpp"

/**
 * @brief Callback for MQTT messages received on the '/set' topic for this node.
 * @details This function is triggered by Homie when a command is sent to this node.
 *          It validates the command and updates the bridge-side desired-state shadow
 *          so a short coalescing window can emit a single SET_STATE toward the controller.
 * @param range The HomieRange object (not used here).
 * @param value The message payload, expected to be "true" or "false".
 * @return true always, to indicate the event has been handled and should not be
 *         propagated to other handlers. Returning false would pass it to the
 *         global handler.
 */
auto LSHNode::callBack(const HomieRange &range, const String &value) -> bool
{
    DP_CONTEXT();
    DPL("↑ ID: ", this->actuatorId, " | value: ", value.c_str());
    (void)range;

    using constants::homie::BOOL_FALSE_LITERAL;
    using constants::homie::BOOL_TRUE_LITERAL;

    // The bridge only stages Homie-originated actuator intent when its cached model is authoritative.
    // During re-sync Homie writes are ignored until a fresh controller state frame re-establishes
    // the baseline; raw LSH writes are handled separately by the MQTT input path.
    if (!this->vDev.isRuntimeSynchronized())
    {
        DPL("Bridge model is waiting for a fresh state sync. Ignoring command.");
        return true;
    }

    // If value is not "true" or "false" then it's not valid, don't propagate the callback
    if ((value != BOOL_TRUE_LITERAL) && (value != BOOL_FALSE_LITERAL))
    {
        return true;
    }

    const bool newState = (value == BOOL_TRUE_LITERAL); // New state
    if (!this->com.stageSingleActuatorCommand(this->actuatorId, newState))
    {
        DPL("Failed to stage desired actuator state for this Homie node.");
        return true;
    }

    Homie.getLogger() << "Command sent to turn light " << (newState ? "on" : "off") << endl;
    return true;
}

/**
 * @brief Sends the current state of the actuator to its MQTT state topic.
 * @details This function is called to synchronize the MQTT state with the
 *          actuator's actual state held in VirtualDevice.
 */
void LSHNode::sendState() const
{
    DP_CONTEXT();
    DP("↑ ID: ", this->getId());

    const bool stateToSend = this->vDev.getStateByIndex(this->actuatorIndex);
    DPL(" | state: ", stateToSend);

    using constants::homie::HOMIE_PROPERTY_ADVERTISE;

    this->setProperty(HOMIE_PROPERTY_ADVERTISE)
        .overwriteSetter(false)
        .setQos(1)
        .setRetained(true)
        .send(utils::conversions::to_literal(stateToSend));
}
