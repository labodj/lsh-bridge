#ifndef LSHESP_CONSTANTS_CONFIGS_VDEVCONFIGS_HPP
#define LSHESP_CONSTANTS_CONFIGS_VDEVCONFIGS_HPP

#include <cstdint>

namespace constants
{
        namespace vDev
        {

#ifndef CONFIG_MAX_ACTUATORS
                static constexpr const std::uint8_t MAX_ACTUATORS = 12U;
#else
                static constexpr const std::uint8_t MAX_ACTUATORS = CONFIG_MAX_ACTUATORS;
#endif // CONFIG_MAX_ACTUATORS

#ifndef CONFIG_MAX_BUTTONS
                static constexpr const std::uint8_t MAX_BUTTONS = 12U;
#else
                static constexpr const std::uint8_t MAX_BUTTONS = CONFIG_MAX_BUTTONS;
#endif // CONFIG_MAX_BUTTONS

#ifndef CONFIG_MAX_NAME_LENGTH
                static constexpr const std::uint8_t MAX_NAME_LENGTH = 4U;
#else
                static constexpr const std::uint8_t MAX_NAME_LENGTH = CONFIG_MAX_NAME_LENGTH;
#endif // CONFIG_MAX_NAME_LENGTH
        } // namespace vDevConfigs

} // namespace constants

#endif // LSHESP_CONSTANTS_CONFIGS_VDEVCONFIGS_HPP
