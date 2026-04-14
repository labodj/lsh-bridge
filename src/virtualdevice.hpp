#ifndef LSHESP_VIRTUALDEVICE_HPP
#define LSHESP_VIRTUALDEVICE_HPP

#include <cstdint>

#include <ArduinoJson.h>
#include <etl/bitset.h>
#include <etl/string.h>
#include <etl/vector.h>

#include "constants/configs/vdev.hpp"

/**
 * @brief Holds the data model for the attached physical device.
 * @details The bridge only persists the subset of details it needs after the
 *          handshake: device name, actuator IDs and actuator state. Button IDs
 *          are still validated when `details` arrive, but they are not retained
 *          because the bridge never reads them after forwarding the raw details
 *          payload to MQTT.
 */
class VirtualDevice
{
private:
    // Device identity
    etl::string<constants::vDev::MAX_NAME_LENGTH> name{}; //!< Device name, populated from the serial device.

    // Actuators
    std::uint8_t totalActuators = 0U; //!< Total number of actuators.

    // Stores the original actuator UUIDs. This is necessary for initializing Homie nodes,
    // building MQTT/Homie topology and mapping state indexes back to logical IDs.
    etl::vector<uint8_t, constants::vDev::MAX_ACTUATORS> actuatorIds{};

    etl::bitset<constants::vDev::MAX_ACTUATORS> actuatorsState{}; //!< Real-time state of all actuators.
    etl::bitset<constants::vDev::MAX_ACTUATORS> dirtyActuators{}; //!< Union of actuator changes not yet mirrored to Homie since the last publish window.
    bool runtimeSynchronized = false; //!< False until a fresh authoritative state arrives from the controller.
    bool fullStatePublishPending = true; //!< Forces a full Homie snapshot after the next authoritative state sync.

public:
    VirtualDevice() noexcept {};
    ~VirtualDevice() = default;

    // --- Deleted copy/move constructors and assignment operators to prevent accidental copies ---
    VirtualDevice(const VirtualDevice &) = delete;
    VirtualDevice(VirtualDevice &&) = delete;
    auto operator=(const VirtualDevice &) -> VirtualDevice & = delete;
    auto operator=(VirtualDevice &&) -> VirtualDevice & = delete;

    // SETTERS
    // Name setters
    void setName(const char *deviceName);

    // Actuators Unique IDs setters
    void setActuatorsIds(const JsonArrayConst &ids);

    void setStateFromPackedBytes(const JsonArrayConst &packedBytes); // Actuator group state setter (bitpacked bytes: [90, 255, ...])
    void invalidateRuntimeModel() noexcept;

    // GETTERS
    auto getName() const -> const etl::istring &; // Name getter

    auto getActuatorId(uint8_t index) const -> std::uint8_t;
    [[nodiscard]] auto tryGetActuatorIndex(uint8_t actuatorId, std::uint8_t &outIndex) const noexcept -> bool;
    auto getTotalActuators() const noexcept -> std::uint8_t;    // Total actuators getter
    auto getStateByIndex(uint8_t index) const noexcept -> bool; // Get the state of a single actuator by its index.
    auto isActuatorDirty(uint8_t index) const noexcept -> bool;
    void clearDirtyActuators() noexcept;
    auto isRuntimeSynchronized() const noexcept -> bool;
    auto consumeFullStatePublishPending() noexcept -> bool;
};

#endif // LSHESP_VIRTUALDEVICE_HPP
