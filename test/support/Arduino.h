#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

// Native sink for exercising the production UART writer without Arduino hardware.
class Print
{
public:
    virtual std::size_t write(std::uint8_t byte) = 0;
    virtual std::size_t write(const std::uint8_t *bytes, std::size_t size) = 0;
    virtual ~Print() = default;
};

class HardwareSerial : public Print
{
public:
    std::vector<std::uint8_t> bytes;
    std::size_t write(std::uint8_t byte) override
    {
        bytes.push_back(byte);
        return 1;
    }
    std::size_t write(const std::uint8_t *data, std::size_t size) override
    {
        bytes.insert(bytes.end(), data, data + size);
        return size;
    }
};
