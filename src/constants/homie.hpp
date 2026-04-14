#ifndef LSHESP_CONSTANTS_HOMIE_HPP
#define LSHESP_CONSTANTS_HOMIE_HPP

#include <WString.h>

/**
 * @brief Homie messages.
 *
 */
namespace constants
{
    namespace homie
    {
        static constexpr const char *BOOL_TRUE_LITERAL = "true";                  //!< Literal "true"
        static constexpr const char *BOOL_FALSE_LITERAL = "false";                //!< Literal "false"
        static constexpr const char *HOMIE_PROPERTY_DATATYPE_BOOLEAN = "boolean"; //!< Homie boolean data type
        static constexpr const char *HOMIE_ACTUATOR_TYPE = "switch";              //!< Homie actuator type
        static constexpr const char *HOMIE_PROPERTY_ADVERTISE = "state";          //!< Homie advertise
    } // namespace homie
} // namespace constants

#endif // LSHESP_CONSTANTS_HOMIE_HPP
