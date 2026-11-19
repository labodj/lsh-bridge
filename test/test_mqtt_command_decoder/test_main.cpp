#include <cstddef>
#include <cstdint>

#include <unity.h>
#include <ArduinoJson.h>
#include <fstream>
#include <string>
#include "communication/msgpack_commands.hpp"
#include "communication/msgpack_serial_framing.hpp"
#include "utils/json_scalars.hpp"

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

std::vector<std::uint8_t> fromHex(const std::string &hex)
{
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i < hex.size(); i += 2)
        bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    return bytes;
}

void testSharedV3WireVectors()
{
    std::ifstream input("vendor/lsh-protocol/shared/lsh_protocol_golden_payloads.json");
    DynamicJsonDocument document(32768);
    TEST_ASSERT_FALSE(deserializeJson(document, input));
    const auto vectors = document["wireVectors"].as<JsonObjectConst>();
    TEST_ASSERT_GREATER_THAN(0, vectors.size());
    for (const auto entry : vectors)
    {
        const auto payload = entry.value()["payload"].as<JsonObjectConst>();
        const auto raw = fromHex(entry.value()["msgpack"].as<std::string>());
        const auto expected = fromHex(entry.value()["uart"].as<std::string>());
        HardwareSerial serial;
        lsh::bridge::transport::MsgPackFrameWriter writer(serial);
        TEST_ASSERT_TRUE(writer.beginFrame());
        const auto command = static_cast<Command>(payload["p"].as<std::uint8_t>());
        if (command == Command::NETWORK_CLICK_ACK || command == Command::FAILOVER_CLICK)
            lsh::bridge::communication::writeClick(writer, command, payload["t"], payload["i"], payload["c"]);
        else if (command == Command::SET_STATE)
        {
            const auto packed = payload["s"].as<JsonArrayConst>();
            lsh::bridge::communication::writePackedState(writer, packed.size(),
                                                         [&](std::uint8_t i) { return packed[i].as<std::uint8_t>(); });
        }
        else
            serializeMsgPack(payload, writer);
        TEST_ASSERT_TRUE(writer.endFrame());
        TEST_ASSERT_EQUAL_UINT(expected.size(), serial.bytes.size());
        TEST_ASSERT_EQUAL_MEMORY_MESSAGE(expected.data(), serial.bytes.data(), expected.size(), entry.key().c_str());
        if (command == Command::DEVICE_DETAILS || command == Command::ACTUATORS_STATE)
            continue;
        DecodedMqttCommand decoded{};
#ifdef CONFIG_MSG_PACK_MQTT
        TEST_ASSERT_TRUE(decodeMqttCommandShallow(reinterpret_cast<const char *>(raw.data()), raw.size(), decoded));
#else
        std::string json;
        serializeJson(payload, json);
        TEST_ASSERT_TRUE(decodeMqttCommandShallow(json.data(), json.size(), decoded));
#endif
        TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(command), static_cast<std::uint8_t>(decoded.command));
        if (command == Command::SET_SINGLE_ACTUATOR)
        {
            TEST_ASSERT_EQUAL_UINT8(payload["i"].as<std::uint8_t>(), decoded.actuatorId);
            TEST_ASSERT_EQUAL(payload["s"].as<int>(), decoded.state);
        }
        if (command == Command::NETWORK_CLICK_ACK || command == Command::FAILOVER_CLICK)
        {
            TEST_ASSERT_EQUAL_UINT8(payload["i"].as<std::uint8_t>(), decoded.clickableId);
            TEST_ASSERT_EQUAL_UINT8(payload["c"].as<std::uint8_t>(), decoded.correlationId);
            TEST_ASSERT_EQUAL_UINT8(payload["t"].as<std::uint8_t>(), decoded.clickType);
        }
        if (command == Command::SET_STATE)
        {
            TEST_ASSERT_EQUAL_UINT(payload["s"].size(), decoded.packedStateLength);
            for (std::uint8_t i = 0; i < decoded.packedStateLength; ++i)
                TEST_ASSERT_EQUAL_UINT8(payload["s"][i].as<std::uint8_t>(), decoded.packedStateBytes[i]);
        }
    }
}

void testLshRelayStateRejectsBooleans()
{
    for (bool value : {false, true})
    {
        StaticJsonDocument<128> payload;
        payload["p"] = 13;
        payload["i"] = 1;
        payload["s"] = value;
        bool state = false;
        TEST_ASSERT_FALSE(utils::json::tryGetBinaryState(payload["s"], state));
        std::string encoded;
#ifdef CONFIG_MSG_PACK_MQTT
        serializeMsgPack(payload, encoded);
#else
        serializeJson(payload, encoded);
#endif
        DecodedMqttCommand command{};
        TEST_ASSERT_FALSE(decodeMqttCommandShallow(encoded.data(), encoded.size(), command));
    }
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
    RUN_TEST(testSharedV3WireVectors);
    RUN_TEST(testLshRelayStateRejectsBooleans);
    RUN_TEST(testRejectsDuplicateFields);
    RUN_TEST(testCommandAssemblyEverySplit);
    RUN_TEST(testCommandAssemblyRejectsInvalidFragments);
    return UNITY_END();
}
