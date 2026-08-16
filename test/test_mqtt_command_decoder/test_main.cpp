#include <cstddef>
#include <cstdint>

#include <unity.h>

#include "mqtt_command_decoder.hpp"

namespace
{
using lsh::bridge::DecodedMqttCommand;
using lsh::bridge::DecodedMqttCommandShape;
using lsh::bridge::decodeMqttCommandShallow;
using lsh::bridge::protocol::Command;

void testValidSetSingleActuator()
{
#ifdef CONFIG_MSG_PACK_MQTT
    constexpr std::uint8_t payload[]{0x83U, 0xA1U, 'p', 13U, 0xA1U, 'i', 2U, 0xA1U, 's', 1U};
    const auto *bytes = reinterpret_cast<const char *>(payload);
    constexpr auto length = sizeof(payload);
#else
    constexpr char payload[] = R"({"p":13,"i":2,"s":1})";
    const auto *bytes = payload;
    constexpr auto length = sizeof(payload) - 1U;
#endif

    DecodedMqttCommand command{};
    TEST_ASSERT_TRUE(decodeMqttCommandShallow(bytes, length, command));
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(Command::SET_SINGLE_ACTUATOR), static_cast<std::uint8_t>(command.command));
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(DecodedMqttCommandShape::SetSingleActuator),
                            static_cast<std::uint8_t>(command.shape));
    TEST_ASSERT_EQUAL_UINT8(2U, command.actuatorId);
    TEST_ASSERT_TRUE(command.state);
}

void testRejectsDuplicateFields()
{
#ifdef CONFIG_MSG_PACK_MQTT
    constexpr std::uint8_t payload[]{0x84U, 0xA1U, 'p', 13U, 0xA1U, 'p', 13U, 0xA1U, 'i', 2U, 0xA1U, 's', 1U};
    const auto *bytes = reinterpret_cast<const char *>(payload);
    constexpr auto length = sizeof(payload);
#else
    constexpr char payload[] = R"({"p":13,"p":13,"i":2,"s":1})";
    const auto *bytes = payload;
    constexpr auto length = sizeof(payload) - 1U;
#endif

    DecodedMqttCommand command{};
    TEST_ASSERT_FALSE(decodeMqttCommandShallow(bytes, length, command));
}

}  // namespace

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(testValidSetSingleActuator);
    RUN_TEST(testRejectsDuplicateFields);
    return UNITY_END();
}
