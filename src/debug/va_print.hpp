/**
 * @file    va_print.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares a tiny variadic print helper for debug builds.
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

#ifndef LSH_BRIDGE_DEBUG_VA_PRINT_HPP
#define LSH_BRIDGE_DEBUG_VA_PRINT_HPP

#include <cstdint>

#include <Arduino.h>

/**
 * @brief Small wrapper used to change the numeric base for following prints.
 *
 */
struct Base
{
    constexpr explicit Base(std::uint8_t b = 10U) : base(b)
    {}
    std::uint8_t base;  //!< Numeric base to use while printing integers
};

constexpr static const Base Bin(2U);   //!< Binary base helper
constexpr static const Base Oct(8U);   //!< Octal base helper
constexpr static const Base Dec(10U);  //!< Decimal base helper
constexpr static const Base Hex(16U);  //!< Hexadecimal base helper

/**
 * @brief Small wrapper used to change the precision for following float prints.
 *
 */
struct Prec
{
    constexpr explicit Prec(std::uint8_t p = 2U) : prec(p)
    {}
    std::uint8_t prec;  //!< Decimal precision to use while printing floating numbers
};

/**
 * @brief Tiny variadic print helper used by debug macros.
 *
 */
class VaPrint
{
private:
    Print *m_pr{&Serial};      //!< Output stream used by the helper
    std::uint8_t m_base{10U};  //!< Current numeric base
    std::uint8_t m_prec{2U};   //!< Current floating precision

public:
    constexpr VaPrint() = default;

    /**
     * @brief Print many values in sequence.
     *
     */
    template <typename T, typename... Rest> void print(T first, Rest... rest)
    {
        this->print(first);
        this->print(rest...);
    }

    /**
     * @brief Change numeric base used for following integer prints.
     *
     * @param b new numeric base wrapper.
     */
    void print(Base b)
    {
        this->m_base = b.base;
    }

    /**
     * @brief Change floating precision used for following decimal prints.
     *
     * @param p new precision wrapper.
     */
    void print(Prec p)
    {
        this->m_prec = p.prec;
    }

    /**
     * @brief Print an Arduino String.
     *
     * @param str string to print.
     */
    void print(const String &str)
    {
        this->m_pr->print(str);
    }

    /**
     * @brief Print one character.
     *
     * @param c character to print.
     */
    void print(const char c)
    {
        this->m_pr->print(c);
    }

    /**
     * @brief Print a mutable C string.
     *
     * @param str string to print.
     */
    void print(char *const str)
    {
        this->m_pr->print(str);
    }

    /**
     * @brief Print a constant C string.
     *
     * @param str string to print.
     */
    void print(const char *const str)
    {
        this->m_pr->print(str);
    }

    /**
     * @brief Print a flash-resident string.
     *
     * @param str flash string to print.
     */
    void print(const __FlashStringHelper *const str)
    {
        this->m_pr->print(str);
    }

    /**
     * @brief Print a float using the currently selected precision.
     *
     * @param f floating number to print.
     */
    void print(const float f)
    {
        this->print(static_cast<double>(f));
    }

    /**
     * @brief Print a double using the currently selected precision.
     *
     * @param d floating number to print.
     */
    void print(const double d)
    {
        this->m_pr->print(d, this->m_prec);
    }

    /**
     * @brief Print a generic integral-like value using the current base.
     *
     * @param arg value to print.
     */
    template <typename T> void print(T arg)
    {
        this->m_pr->print(arg, this->m_base);
    }

    /**
     * @brief Print a newline.
     *
     */
    void println()
    {
        this->m_pr->println();
    }
};

#endif  // LSH_BRIDGE_DEBUG_VA_PRINT_HPP
