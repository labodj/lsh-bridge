#ifndef LSHESP_CONSTANTS_DESERIALIZE_EXIT_CODES_HPP
#define LSHESP_CONSTANTS_DESERIALIZE_EXIT_CODES_HPP

/**
 * @brief Namespace for constants.
 */
namespace constants
{
    /**
     * @brief Exit codes for json deserialization
     *
     */
    enum class DeserializeExitCode
    {
        OK,                                      //!< General use OK
        OK_DETAILS,                              //!< Details received OK
        OK_STATE,                                //!< State received OK
        OK_PING,                                 //!< Ping received OK
        OK_BOOT,                                 //!< Boot received OK
        OK_FORWARDED,                            //!< Json has been forwarded to MQTT broker
        OK_NETWORK_CLICK,                        //!< Network click received and stored OK
        OK_OTHER_PAYLOAD,                        //!< Other (unknown) payload received
        ERR,                                     //!< General use ERR
        ERR_INCOMPLETE,                          //!< Incomplete payload received
        ERR_JSON_ERROR,                          //!< Json is malformed, invalid input or serial timeout
        ERR_DOC_NULL,                            //!< Json is empty
        ERR_MISSING_KEY_PAYLOAD,                 //!< Payload content key is missing
        ERR_MISSING_KEY_STATE,                   //!< State key is missing or not an array
        ERR_UNKNOWN_PAYLOAD,                     //!< Payload unknown
        ERR_NOT_CONNECTED_FAILOVER_SENT,         //!< Network click received, cannot forward it, failover sent to controllino
        ERR_NOT_CONNECTED_GENERAL_FAILOVER_SENT, //!< Network click received, cannot forward it, general failover sent to controllino
        ERR_NO_NAME,                             //!< No device name received
        ERR_NAME_TOO_LONG,                       //!< Device name exceeds the compiled bridge limit
        ERR_NO_CLICK_TYPE,                       //!< No click type for long click network action
        ERR_UNKNOWN_CLICK_TYPE,                  //!< Click type is unknown
        ERR_LONG_CLICKED_BUTTON_IMPLAUSIBLE,     //!< Received long clicked button not in range (min=0, max=total buttons)
        ERR_NOT_FORWARDED_OTHER_PROBLEM,         //!< Message not forwarded due to another problem
        ERR_NOT_USEFUL_JSON,                     //!< Arrived Json is not useful right now (maybe we need to config something first)
        ERR_NO_ACTUATORS,                        //!< No actuators array received
        ERR_ACTUATORS_MISMATCH,                  //!< Received actuators state array size is different than stored total actuators
        ERR_STATE_VALUE_IMPLAUSIBLE,             //!< Received state payload contains non-byte entries
        ERR_MISSING_KEY_PROTOCOL_MAJOR,          //!< Missing handshake-only protocol major in details payload
        ERR_PROTOCOL_MAJOR_MISMATCH,             //!< Received protocol major is not compatible with this firmware
        ERR_MISSING_KEY_ACTUATORS_IDS,           //!< Missing actuators UUIDs key from Json
        ERR_MISSING_KEY_BUTTONS_IDS,             //!< Missing buttons UUIDs key from Json
        ERR_ACTUATORS_IDS_NUMBER_IMPLAUSIBLE,    //!< Total actuators UUIDs number is different than total actuators
        ERR_ACTUATOR_ID_IMPLAUSIBLE,             //!< Actuator IDs must be unique positive uint8_t values
        ERR_BUTTON_ID_IMPLAUSIBLE                //!< Button IDs must be unique positive uint8_t values
    }; // namespace DeserializeExitCode
} // namespace constants

#endif // LSHESP_CONSTANTS_DESERIALIZE_EXIT_CODES_HPP
