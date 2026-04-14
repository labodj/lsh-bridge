#include <Arduino.h>

#include <lsh_esp_bridge.hpp>

namespace
{

lsh::esp::BridgeOptions makeBridgeOptions()
{
    lsh::esp::BridgeOptions options;
    options.serial = &Serial2;
    (void)options.identity.setFirmwareVersion("1.0.0");
    options.disableLedFeedback = true;
    options.loggingMode = lsh::esp::LoggingMode::AutoFromBuild;
    return options;
}

lsh::esp::LSHEspBridge bridge(makeBridgeOptions());

} // namespace

void setup()
{
    bridge.begin();
}

void loop()
{
    bridge.loop();
}
