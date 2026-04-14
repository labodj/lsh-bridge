#ifndef LSHESP_CONSTANTS_CONFIGS_ARDCOM_HPP
#define LSHESP_CONSTANTS_CONFIGS_ARDCOM_HPP

#include <cstdint>

#include <ArduinoJson.h>

#include "constants/configs/vdev.hpp"

namespace constants
{
        namespace ardCom
        {
#ifndef CONFIG_ARDCOM_SERIAL_RX_PIN
                static constexpr const std::uint8_t ARDCOM_SERIAL_RX_PIN = 16;
#else
                static constexpr const std::uint8_t ARDCOM_SERIAL_RX_PIN = CONFIG_ARDCOM_SERIAL_RX_PIN;
#endif

#ifndef CONFIG_ARDCOM_SERIAL_TX_PIN
                static constexpr const std::uint8_t ARDCOM_SERIAL_TX_PIN = 17;
#else
                static constexpr const std::uint8_t ARDCOM_SERIAL_TX_PIN = CONFIG_ARDCOM_SERIAL_TX_PIN;
#endif

#ifndef CONFIG_ARDCOM_SERIAL_BAUD
                static constexpr const std::uint32_t ARDCOM_SERIAL_BAUD = 250000U;
#else
                static constexpr const std::uint32_t ARDCOM_SERIAL_BAUD = CONFIG_ARDCOM_SERIAL_BAUD;
#endif // CONFIG_ARDCOM_SERIAL_BAUD

#ifndef CONFIG_ARDCOM_SERIAL_TIMEOUT_MS
                static constexpr const std::uint8_t ARDCOM_SERIAL_TIMEOUT_MS = 5U; //!< Default Serial connected with Arduino timeout
#else
                static constexpr const std::uint8_t ARDCOM_SERIAL_TIMEOUT_MS = CONFIG_ARDCOM_SERIAL_TIMEOUT_MS; //!< Serial connected with Arduino timeout
#endif // CONFIG_ARDCOM_SERIAL_TIMEOUT_MS
        } // namespace ardCom
} // namespace constants

#endif // LSHESP_CONSTANTS_CONFIGS_ARDCOM_HPP