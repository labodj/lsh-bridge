#ifndef LSHESP_DEBUG_VAPRINT_HPP
#define LSHESP_DEBUG_VAPRINT_HPP

#include <cstdint>

#include <Arduino.h>

struct Base
{
  constexpr explicit Base(uint8_t b = 10) : base(b) {}
  std::uint8_t base;
};
constexpr static const Base Bin(2);
constexpr static const Base Oct(8);
constexpr static const Base Dec(10);
constexpr static const Base Hex(16);

struct Prec
{
  constexpr explicit Prec(uint8_t p = 2) : prec(p) {}
  std::uint8_t prec;
};

class VaPrint
{

private:
  Print *m_pr{&Serial};
  std::uint8_t m_base{10};
  std::uint8_t m_prec{2};

public:
  constexpr VaPrint() = default;

  template <typename T, typename... Rest>
  void print(T first, Rest... rest)
  {
    this->print(first);
    this->print(rest...);
  }

  void print(Base b) { this->m_base = b.base; }
  void print(Prec p) { this->m_prec = p.prec; }

  void print(const String &str) { this->m_pr->print(str); }
  void print(const char c) { this->m_pr->print(c); }
  void print(char *const str) { this->m_pr->print(str); }
  void print(const char *const str) { this->m_pr->print(str); }
  void print(const __FlashStringHelper *const str) { m_pr->print(str); }
  void print(const float f) { this->print(static_cast<double>(f)); }
  void print(const double d) { this->m_pr->print(d, this->m_prec); }

  template <typename T>
  void print(T arg) { this->m_pr->print(arg, this->m_base); }

  void println() { this->m_pr->println(); }
};

#endif // LSHESP_DEBUG_VAPRINT_HPP