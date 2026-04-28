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

class HardwareSerial;
struct HomieEvent;                        //!< Forward declaration of the Homie event payload.
struct AsyncMqttClientMessageProperties;  //!< Forward declaration of the AsyncMqttClient message metadata.

namespace lsh::bridge
{

namespace detail
{
#ifndef CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE
inline constexpr std::size_t BRIDGE_IMPL_STORAGE_SIZE = 3072U;
#else
static_assert(CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE > 0U, "CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE must be greater than zero.");
inline constexpr std::size_t BRIDGE_IMPL_STORAGE_SIZE = CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE;
#endif

#ifndef CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER
inline constexpr bool DEFAULT_DISABLE_RESET_TRIGGER = false;
#else
static_assert((CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER == 0) || (CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER == 1),
              "CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER must be 0 or 1.");
inline constexpr bool DEFAULT_DISABLE_RESET_TRIGGER = (CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER != 0);
#endif
}  // namespace detail

/**
 * @brief Controls bridge runtime logging behavior.
 */
enum class LoggingMode : std::uint8_t
{
    AutoFromBuild,  // Enable logs only when the build configuration enables them.
    Enabled,        // Force runtime logging on.
    Disabled        // Force runtime logging off.
};

/**
 * @brief Collects the runtime dependencies and policy knobs for `LSHBridge`.
 */
struct BridgeOptions
{
    HardwareSerial *serial = nullptr;                                  //!< UART used for the controller link. `nullptr` selects `Serial2`.
    bool disableLedFeedback = true;                                    //!< Disable Homie LED feedback by default.
    bool disableResetTrigger = detail::DEFAULT_DISABLE_RESET_TRIGGER;  //!< Disable Homie's physical reset trigger.
    bool disableWifiSleep = true;                                      //!< Disable ESP32 Wi-Fi modem sleep by default.
    LoggingMode loggingMode = LoggingMode::AutoFromBuild;              //!< Runtime logging policy.
};

/**
 * @brief Opinionated single-instance runtime bridge between LSH serial and Homie/MQTT.
 */
class LSHBridge
{
public:
    explicit LSHBridge(BridgeOptions options = {});

    ~LSHBridge();

    LSHBridge(const LSHBridge &) = delete;
    auto operator=(const LSHBridge &) -> LSHBridge & = delete;
    LSHBridge(LSHBridge &&) = delete;
    auto operator=(LSHBridge &&) -> LSHBridge & = delete;

    void begin();

    void loop();

    [[nodiscard]] auto isRuntimeSynchronized() const noexcept -> bool;

    [[nodiscard]] auto deviceName() const noexcept -> const char *;

private:
    class Impl;  //!< Forward declaration of the hidden runtime implementation.
    // The bridge keeps its implementation opaque without using heap allocation.
    // If a custom capacity profile ever makes Impl larger, the build fails with
    // a clear static_assert in lsh_bridge.cpp and the embedding firmware can
    // raise CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE explicitly.
    alignas(std::max_align_t) std::byte implementationStorage[detail::BRIDGE_IMPL_STORAGE_SIZE]{};
    Impl *implementation = nullptr;

    static LSHBridge *activeInstance;
    static void onHomieEventStatic(const HomieEvent &event);
    static void onMqttMessageStatic(char *topic,
                                    char *payload,
                                    AsyncMqttClientMessageProperties properties,
                                    std::size_t len,
                                    std::size_t index,
                                    std::size_t total);
};

}  // namespace lsh::bridge

#endif  // LSH_BRIDGE_LSH_BRIDGE_HPP
