#ifndef LSHESP_UTILS_HASH_HPP
#define LSHESP_UTILS_HASH_HPP

#include <cstdint>

__attribute__((always_inline)) constexpr auto djb2_hash(const char *str) -> std::uint32_t
{
    if (str == nullptr || str[0] == '\0')
    {
        return 0U;
    }

    std::uint32_t hash = 5381U;
    int c = 0;

    while ((c = *str++))
    {
        hash = ((hash << 5) + hash) ^ c;
    }

    return hash;
}

#endif // LSHESP_UTILS_HASH_HPP