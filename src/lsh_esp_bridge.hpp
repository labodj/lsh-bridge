#ifndef LSH_ESP_LIB_BRIDGE_HPP
#define LSH_ESP_LIB_BRIDGE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include <etl/string.h>

class HardwareSerial;
struct HomieEvent;
struct AsyncMqttClientMessageProperties;

namespace lsh::esp
{

inline constexpr std::size_t kMaxFirmwareNameLength = 32U;
inline constexpr std::size_t kMaxFirmwareVersionLength = 16U;
// Homie stores the brand in a 22-byte buffer including the null terminator.
inline constexpr std::size_t kMaxHomieBrandLength = 21U;

enum class LoggingMode : std::uint8_t
{
    AutoFromBuild,
    Enabled,
    Disabled
};

class BridgeIdentity
{
public:
    BridgeIdentity();
    BridgeIdentity(const char *firmwareName,
                   const char *firmwareVersion,
                   const char *homieBrand);

    void resetToDefaults();

    [[nodiscard]] auto setFirmwareName(const char *value) -> bool;
    [[nodiscard]] auto setFirmwareVersion(const char *value) -> bool;
    [[nodiscard]] auto setHomieBrand(const char *value) -> bool;

    [[nodiscard]] auto firmwareName() const noexcept -> const char *;
    [[nodiscard]] auto firmwareVersion() const noexcept -> const char *;
    [[nodiscard]] auto homieBrand() const noexcept -> const char *;

private:
    etl::string<kMaxFirmwareNameLength> firmwareName_{};
    etl::string<kMaxFirmwareVersionLength> firmwareVersion_{};
    etl::string<kMaxHomieBrandLength> homieBrand_{};
};

struct BridgeOptions
{
    HardwareSerial *serial = nullptr;
    BridgeIdentity identity{};
    bool disableLedFeedback = true;
    LoggingMode loggingMode = LoggingMode::AutoFromBuild;
};

/**
 * @brief Opinionated single-instance runtime bridge between LSH serial and Homie/MQTT.
 * @details The class preserves the behavior of the current `lsh-esp` firmware while hiding
 *          the internal runtime types behind a small facade. The bridge still performs the
 *          blocking bootstrap handshake and keeps the controller as the authoritative source
 *          of state.
 */
class LSHEspBridge
{
public:
    explicit LSHEspBridge(BridgeOptions options = {});
    ~LSHEspBridge();
    LSHEspBridge(const LSHEspBridge &) = delete;
    auto operator=(const LSHEspBridge &) -> LSHEspBridge & = delete;
    LSHEspBridge(LSHEspBridge &&) = delete;
    auto operator=(LSHEspBridge &&) -> LSHEspBridge & = delete;

    void begin();
    void loop();

    [[nodiscard]] auto isRuntimeSynchronized() const noexcept -> bool;
    [[nodiscard]] auto deviceName() const noexcept -> const char *;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    static LSHEspBridge *activeInstance_;
    static void onHomieEventStatic_(const HomieEvent &event);
    static void onMqttMessageStatic_(char *topic,
                                     char *payload,
                                     AsyncMqttClientMessageProperties properties,
                                     std::size_t len,
                                     std::size_t index,
                                     std::size_t total);
};

} // namespace lsh::esp

#endif
