#include <cstddef>
#include <cstdint>

#include <unity.h>

#include "mqtt_command_decoder.hpp"
#include "mqtt_command_assembler.hpp"
#include <array>
#include <limits>

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

void testCommandAssemblyEverySplit()
{
    using Assembler = lsh::bridge::MqttCommandAssembler<32>;
    std::array<std::uint8_t, 32> payload{};
    for (std::size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<std::uint8_t>(i);
    for (std::size_t split = 1; split < payload.size(); ++split)
    {
        Assembler assembler;
        TEST_ASSERT_TRUE(assembler.append(payload.data(), split, 0, payload.size(), false) == Assembler::Result::Partial);
        TEST_ASSERT_TRUE(assembler.append(payload.data() + split, payload.size() - split, split, payload.size(), false) ==
                         Assembler::Result::Complete);
        TEST_ASSERT_EQUAL_MEMORY(payload.data(), assembler.data(), payload.size());
    }
    Assembler assembler;
    for (std::size_t i = 0; i < payload.size(); ++i)
    {
        const auto expected = i + 1 == payload.size() ? Assembler::Result::Complete : Assembler::Result::Partial;
        TEST_ASSERT_TRUE(assembler.append(payload.data() + i, 1, i, payload.size(), true) == expected);
    }
    TEST_ASSERT_EQUAL_MEMORY(payload.data(), assembler.data(), payload.size());
}

void testCommandAssemblyRejectsInvalidFragments()
{
    using Assembler = lsh::bridge::MqttCommandAssembler<4>;
    const std::uint8_t bytes[]{1, 0, 2, 3};
    Assembler assembler;
    const auto partial = [&] { TEST_ASSERT_TRUE(assembler.append(bytes, 2, 0, 4, false) == Assembler::Result::Partial); };
    partial();
    TEST_ASSERT_TRUE(assembler.append(bytes, 1, 3, 4, false) == Assembler::Result::Invalid);  // Gap.
    partial();
    TEST_ASSERT_TRUE(assembler.append(bytes, 2, 1, 4, false) == Assembler::Result::Invalid);  // Overlap.
    partial();
    TEST_ASSERT_TRUE(assembler.append(bytes, 1, 2, 3, false) == Assembler::Result::Invalid);  // Changed total.
    partial();
    TEST_ASSERT_TRUE(assembler.append(bytes, 2, 2, 4, true) == Assembler::Result::Invalid);  // Changed topic family.
    partial();
    assembler.reset();  // Disconnect or rejected/intervening message.
    TEST_ASSERT_TRUE(assembler.append(bytes, 2, 2, 4, false) == Assembler::Result::Invalid);
    TEST_ASSERT_TRUE(assembler.append(bytes, 4, 0, 5, false) == Assembler::Result::Invalid);
    TEST_ASSERT_TRUE(assembler.append(nullptr, 1, 0, 1, false) == Assembler::Result::Invalid);
    TEST_ASSERT_TRUE(assembler.append(bytes, 0, 0, 0, false) == Assembler::Result::Invalid);
    TEST_ASSERT_TRUE(assembler.append(bytes, 4, std::numeric_limits<std::size_t>::max(), 4, false) == Assembler::Result::Invalid);
    partial();
    TEST_ASSERT_TRUE(assembler.append(bytes, 4, 0, 4, true) == Assembler::Result::Complete);  // Fresh message supersedes partial.
    TEST_ASSERT_EQUAL_MEMORY(bytes, assembler.data(), sizeof(bytes));
    TEST_ASSERT_TRUE(assembler.append(bytes, 2, 2, 4, true) == Assembler::Result::Invalid);  // Duplicate final fragment.
}

}  // namespace

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(testValidSetSingleActuator);
    RUN_TEST(testRejectsDuplicateFields);
    RUN_TEST(testCommandAssemblyEverySplit);
    RUN_TEST(testCommandAssemblyRejectsInvalidFragments);
    return UNITY_END();
}
