#ifndef LSHESP_LSH_NODE_HPP
#define LSHESP_LSH_NODE_HPP

#include <cstdint>

#include <HomieNode.hpp>
#include <etl/string.h>

#include "constants/homie.hpp"
#include "debug/debug.hpp"

class ArdCom;        // Forward declaration
class VirtualDevice; // Forward declaration

// This struct's only purpose is to convert a numeric ID to a string
// and hold it, so it can be used in LSHNode's base class initializer list.
struct HomieIdHolder
{
    etl::string<4> homieId;

    explicit HomieIdHolder(uint8_t numericId)
    {
        char buffer[4];
        itoa(numericId, buffer, 10);
        homieId.assign(buffer);
    }
};

/**
 * @brief Custom LSH Node to map Controllino actuators.
 *
 */
class LSHNode : private HomieIdHolder, public HomieNode
{
private:
    ArdCom &com;
    VirtualDevice &vDev; // Store a reference to the main device model
    // The ID is already stored in HomieNode, accessible via getId()
    const std::uint8_t actuatorIndex; // The index for this node for O(1) access
    const std::uint8_t actuatorId;    // The logical numeric ID for communication
    auto callBack(const HomieRange &range, const String &value) -> bool;

public:
    LSHNode(ArdCom &ardCom,
                               VirtualDevice &virtualDevice,
                               std::uint8_t index,
                               std::uint8_t numericId)
        : HomieIdHolder(numericId),
          HomieNode(homieId.c_str(), homieId.c_str(), constants::homie::HOMIE_ACTUATOR_TYPE),
          com(ardCom),
          vDev(virtualDevice),
          actuatorIndex(index),
          actuatorId(numericId)
    {
        DPL("Created new LSHNode | ID: ", this->homieId.c_str(), " | Index: ", actuatorIndex);

        this->advertise(constants::homie::HOMIE_PROPERTY_ADVERTISE)
            .setName(this->homieId.c_str())
            .setDatatype(constants::homie::HOMIE_PROPERTY_DATATYPE_BOOLEAN)
            .settable([this](const HomieRange &range, const String &value) -> bool
                      { return this->callBack(range, value); });
    }

    void sendState() const;
};

#endif // LSHESP_LSH_NODE_HPP
