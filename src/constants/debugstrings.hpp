#ifndef LSHESP_CONSTANTS_DEBUGSTRINGS_HPP
#define LSHESP_CONSTANTS_DEBUGSTRINGS_HPP

#ifdef LSH_DEBUG

#include <pgmspace.h>

namespace debug
{
    static constexpr const char SPACE[] PROGMEM = " ";        // NOLINT
    static constexpr const char COLON_SPACE[] PROGMEM = ": "; // NOLINT
    static constexpr const char POINT[] PROGMEM = ".";        // NOLINT
    static constexpr const char DIVIDER[] PROGMEM = "||";     // NOLINT

    static constexpr const char METHOD_CALLED[] PROGMEM = "Context:";                                       // NOLINT
    static constexpr const char CALLED[] PROGMEM = "called";                                                // NOLINT
    static constexpr const char FREE_MEMORY[] PROGMEM = "Free memory";                                      // NOLINT
    static constexpr const char COMPILED_BY_GCC[] PROGMEM = "Compiled by GCC";                              // NOLINT
    static constexpr const char EXEC_TIME[] PROGMEM = "Exec time";                                          // NOLINT
    static constexpr const char IS_CONNECTED[] PROGMEM = "Is connected";                                    // NOLINT
    static constexpr const char BUTTON[] PROGMEM = "Button";                                                // NOLINT
    static constexpr const char SHORT[] PROGMEM = "short";                                                  // NOLINT
    static constexpr const char LONG[] PROGMEM = "long";                                                    // NOLINT
    static constexpr const char SUPER_LONG[] PROGMEM = "super long";                                        // NOLINT
    static constexpr const char CLICKED[] PROGMEM = "clicked";                                              // NOLINT
    static constexpr const char ESP_EXIT_CODE[] PROGMEM = "ESP exit code";                                  // NOLINT
    static constexpr const char MESSAGE_RECEIVED_AT_TIME[] PROGMEM = "Message received at time";            // NOLINT
    static constexpr const char ACTUATOR[] PROGMEM = "Actuator";                                            // NOLINT
    static constexpr const char DOES_NOT_EXIST[] PROGMEM = "does not exist";                                // NOLINT
    static constexpr const char JSON_DOC_IS_NULL[] PROGMEM = "JsonDoc is NULL";                             // NOLINT
    static constexpr const char JSON_RECEIVED[] PROGMEM = "JSON received";                                  // NOLINT
    static constexpr const char JSON_SENT[] PROGMEM = "JSON sent";                                          // NOLINT
    static constexpr const char SENDING_PING_TO_ESP[] PROGMEM = "Sending ping to ESP";                      // NOLINT
    static constexpr const char SENDING_BOOT_TO_ESP[] PROGMEM = "Sending boot to ESP";                      // NOLINT
    static constexpr const char UUID[] PROGMEM = "UUID";                                                    // NOLINT
    static constexpr const char INDEX[] PROGMEM = "Index";                                                  // NOLINT
    static constexpr const char EXPIRED[] PROGMEM = "Expired";                                              // NOLINT
    static constexpr const char NET_BUTTONS_NOT_EMPTY[] PROGMEM = "Network buttons not empty, checking..."; // NOLINT
    static constexpr const char CLICK[] PROGMEM = "Click";                                                  // NOLINT
    static constexpr const char TYPE[] PROGMEM = "type";                                                    // NOLINT
    static constexpr const char FOR[] PROGMEM = "for";                                                      // NOLINT
    static constexpr const char ITERATIONS[] PROGMEM = "iterations";                                        // NOLINT
} // namespace debug

#endif // LSH_DEBUG

#endif // LSHESP_CONSTANTS_DEBUGSTRINGS_HPP
