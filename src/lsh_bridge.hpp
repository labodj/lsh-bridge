/**
 * @file    lsh_bridge.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the public lsh::bridge facade and its runtime option types.
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

#ifndef LSH_BRIDGE_LSH_BRIDGE_HPP
#define LSH_BRIDGE_LSH_BRIDGE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

class HardwareSerial;
struct HomieEvent;                        //!< FORWARD DECLARATION
struct AsyncMqttClientMessageProperties;  //!< FORWARD DECLARATION

namespace lsh::bridge
{

/**
 * @brief Controls bridge runtime logging behavior.
 */
enum class LoggingMode : std::uint8_t
{
    AutoFromBuild,  //!< Enable logs only when the build configuration enables them.
    Enabled,        //!< Force runtime logging on.
    Disabled        //!< Force runtime logging off.
};

/**
 * @brief Collects the runtime dependencies and policy knobs for `LSHBridge`.
 */
struct BridgeOptions
{
    HardwareSerial *serial = nullptr;                      //!< UART used for the controller link. `nullptr` selects `Serial2`.
    bool disableLedFeedback = true;                        //!< Disable Homie LED feedback by default.
    LoggingMode loggingMode = LoggingMode::AutoFromBuild;  //!< Runtime logging policy.
};

/**
 * @brief Opinionated single-instance runtime bridge between LSH serial and Homie/MQTT.
 * @details The bridge preserves the current bridge runtime behavior while
 *          hiding the legacy internal types behind a small facade. The
 *          controller remains the authoritative source of topology and state.
 */
class LSHBridge
{
public:
    explicit LSHBridge(BridgeOptions options = {});  // Construct a new bridge object

    ~LSHBridge();  // Destroy the bridge object

    LSHBridge(const LSHBridge &) = delete;
    auto operator=(const LSHBridge &) -> LSHBridge & = delete;
    LSHBridge(LSHBridge &&) = delete;
    auto operator=(LSHBridge &&) -> LSHBridge & = delete;

    void begin();  // Initialize serial transport, bootstrap the controller model and start Homie

    void loop();  // Execute one bridge main-loop iteration

    [[nodiscard]] auto isRuntimeSynchronized() const noexcept -> bool;  // Return whether runtime state is synchronized

    [[nodiscard]] auto deviceName() const noexcept -> const char *;  // Return cached controller device name

private:
    class Impl;                            //!< FORWARD DECLARATION
    std::unique_ptr<Impl> implementation;  //!< PIMPL used to keep the public facade small.

    static LSHBridge *activeInstance;                         //!< Currently active bridge instance used by static callbacks.
    static void onHomieEventStatic(const HomieEvent &event);  //!< Static trampoline for Homie callbacks
    static void onMqttMessageStatic(char *topic,
                                    char *payload,
                                    AsyncMqttClientMessageProperties properties,
                                    std::size_t len,
                                    std::size_t index,
                                    std::size_t total);  //!< Static trampoline for AsyncMqttClient callbacks
};

}  // namespace lsh::bridge

#endif  // LSH_BRIDGE_LSH_BRIDGE_HPP
