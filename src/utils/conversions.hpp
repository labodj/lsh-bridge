#ifndef LSHESP_UTILS_CONVERSIONS_HPP
#define LSHESP_UTILS_CONVERSIONS_HPP

#include "constants/homie.hpp"

namespace utils
{
    namespace conversions
    {
        /**
         * @brief Gets the literal "true" or "false" C-style string at compile-time.
         * @details This branch-free function is highly efficient and can be resolved
         *          by the compiler during compilation if the input is known.
         * @param state The boolean state to convert.
         * @return A const char pointer to either "true" or "false".
         */
        constexpr auto to_literal(bool state) -> const char *
        {
            return state ? constants::homie::BOOL_TRUE_LITERAL : constants::homie::BOOL_FALSE_LITERAL;
        }

    } // namespace conversions
} // namespace utils

#endif // LSHESP_UTILS_CONVERSIONS_HPP
