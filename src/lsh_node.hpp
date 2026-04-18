/**
 * @file    lsh_node.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the Homie node wrapper used for cached controller actuators.
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

#ifndef LSH_BRIDGE_LSH_NODE_HPP
#define LSH_BRIDGE_LSH_NODE_HPP

#include <cstdint>

#include <HomieNode.hpp>
#include <etl/string.h>

#include "constants/homie.hpp"
#include "debug/debug.hpp"

class ControllerSerialLink;  //!< FORWARD DECLARATION
class VirtualDevice;         //!< FORWARD DECLARATION

/**
 * @brief Stores the decimal Homie node ID derived from a numeric actuator ID.
 * @details `HomieNode` needs C-style strings during base-class construction, so
 *          the converted ID string must already exist before the `HomieNode`
 *          constructor runs.
 */
struct HomieIdHolder
{
    etl::string<4> homieId;

    explicit HomieIdHolder(std::uint8_t numericId)
    {
        char buffer[4];
        itoa(numericId, buffer, 10);
        homieId.assign(buffer);
    }
};

/**
 * @brief Wraps one cached controller actuator as a Homie node.
 */
class LSHNode : private HomieIdHolder, public HomieNode
{
private:
    ControllerSerialLink &controllerSerialLink;  //!< Serial communication object
    VirtualDevice &virtualDevice;                //!< Reference to the main cached device model
    const std::uint8_t actuatorIndex;            //!< Index of this actuator for O(1) access
    const std::uint8_t actuatorId;               //!< Logical numeric actuator ID used on the wire

    [[nodiscard]] auto handleSetCommand(const HomieRange &range, const String &value) -> bool;  // Callback for MQTT `/set` commands

public:
    /**
     * @brief Construct a new LSHNode object.
     *
     * @param controllerSerialLink serial link used to send commands to the controller.
     * @param virtualDevice cached device model.
     * @param index actuator index.
     * @param numericId logical actuator ID.
     */
    LSHNode(ControllerSerialLink &controllerSerialLink, VirtualDevice &virtualDevice, std::uint8_t index, std::uint8_t numericId) :
        HomieIdHolder(numericId), HomieNode(homieId.c_str(), homieId.c_str(), constants::homie::HOMIE_ACTUATOR_TYPE),
        controllerSerialLink(controllerSerialLink), virtualDevice(virtualDevice), actuatorIndex(index), actuatorId(numericId)
    {
        DPL("Created new LSHNode | ID: ", this->homieId.c_str(), " | Index: ", this->actuatorIndex);

        this->advertise(constants::homie::HOMIE_PROPERTY_ADVERTISE)
            .setName(this->homieId.c_str())
            .setDatatype(constants::homie::HOMIE_PROPERTY_DATATYPE_BOOLEAN)
            .settable([this](const HomieRange &range, const String &value) -> bool { return this->handleSetCommand(range, value); });
    }

    void sendState() const;  // Send cached actuator state to Homie
};

#endif  // LSH_BRIDGE_LSH_NODE_HPP
