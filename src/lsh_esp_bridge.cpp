#include "lsh_esp_bridge.hpp"

#include <cstdint>
#include <cstring>
#include <utility>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <Homie.h>
#include <etl/bitset.h>
#include <etl/vector.h>

#include "communication/ardcom.hpp"
#include "communication/mqtttopicsbuilder.hpp"
#include "communication/nodecom.hpp"
#include "constants/communicationprotocol.hpp"
#include "constants/configs/mqtt.hpp"
#include "constants/configs/vdev.hpp"
#include "constants/deserializeexitcodes.hpp"
#include "constants/payloads.hpp"
#include "lshnode.hpp"
#include "utils/timekeeper.hpp"
#include "virtualdevice.hpp"

namespace
{

using constants::DeserializeExitCode;

constexpr std::uint8_t BIT_MASK_8[8] = {
    0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x20U, 0x40U, 0x80U};

constexpr char FLAG_PREFIX_FIRMWARE_NAME[] = "\xbf\x84\xe4\x13\x54";
constexpr char FLAG_SUFFIX_FIRMWARE_NAME[] = "\x93\x44\x6b\xa7\x75";
constexpr char FLAG_PREFIX_FIRMWARE_VERSION[] = "\x6a\x3f\x3e\x0e\xe1";
constexpr char FLAG_SUFFIX_FIRMWARE_VERSION[] = "\xb0\x30\x48\xd4\x1a";
constexpr char FLAG_PREFIX_BRAND[] = "\xfb\x2a\xf5\x68\xc0";
constexpr char FLAG_SUFFIX_BRAND[] = "\x6e\x2f\x0f\xeb\x2d";

[[nodiscard]] auto resolveSerial(HardwareSerial *serial) -> HardwareSerial *
{
    return serial != nullptr ? serial : &Serial2;
}

template <std::size_t N>
[[nodiscard]] auto assignBounded(etl::string<N> &target, const char *value) -> bool
{
    if (value == nullptr)
    {
        return false;
    }

    const auto length = std::strlen(value);
    if (length == 0U || length > N)
    {
        return false;
    }

    target.assign(value, length);
    return true;
}

template <std::size_t N>
[[nodiscard]] auto buildFlaggedString(const char *input,
                                      const char (&prefix)[6],
                                      const char (&suffix)[6],
                                      char (&output)[N]) -> bool
{
    if (input == nullptr)
    {
        return false;
    }

    const auto inputLength = std::strlen(input);
    if (inputLength == 0U || (inputLength + 11U) > N)
    {
        return false;
    }

    std::memcpy(output, prefix, 5U);
    std::memcpy(output + 5U, input, inputLength);
    std::memcpy(output + 5U + inputLength, suffix, 5U);
    output[5U + inputLength + 5U] = '\0';
    return true;
}

[[nodiscard]] auto shouldDisableLogging(lsh::esp::LoggingMode mode) -> bool
{
    switch (mode)
    {
    case lsh::esp::LoggingMode::Enabled:
        return false;

    case lsh::esp::LoggingMode::Disabled:
        return true;

    case lsh::esp::LoggingMode::AutoFromBuild:
    default:
#ifdef LSH_DEBUG
        return false;
#else
        return true;
#endif
    }
}

[[nodiscard]] auto tryGetUint8Scalar(const JsonVariantConst value, std::uint8_t &out) -> bool
{
    if (value.isNull() || value.is<const char *>() || value.is<bool>() ||
        value.is<JsonArrayConst>() || value.is<JsonObjectConst>())
    {
        return false;
    }

    if (value.is<std::uint8_t>())
    {
        out = value.as<std::uint8_t>();
        return true;
    }

    const double rawValue = value.as<double>();
    if (rawValue < 0.0 || rawValue > 255.0)
    {
        return false;
    }

    const auto coercedValue = static_cast<std::uint8_t>(rawValue);
    if (static_cast<double>(coercedValue) != rawValue)
    {
        return false;
    }

    out = coercedValue;
    return true;
}

[[nodiscard]] auto tryGetBinaryState(const JsonVariantConst value, bool &out) -> bool
{
    std::uint8_t rawState = 0U;
    if (!tryGetUint8Scalar(value, rawState) || rawState > 1U)
    {
        return false;
    }

    out = rawState == 1U;
    return true;
}

} // namespace

namespace lsh::esp
{

class LSHEspBridge::Impl
{
public:
    enum class MqttCommandSource : std::uint8_t
    {
        Device,
        Service
    };

    enum class HandshakeState
    {
        IDLE,
        WAITING_FOR_DETAILS,
        WAITING_FOR_STATE,
        COMPLETE,
        FAIL_RESTART
    };

    static constexpr std::uint32_t DEVICE_INFO_REQUEST_INTERVAL_MS = 500U;
    static constexpr std::uint32_t STATE_PUBLISH_SETTLE_INTERVAL_MS = 40U;
    static constexpr std::uint8_t MQTT_COMMAND_QUEUE_CAPACITY = 8U;

    struct QueuedMqttCommand
    {
        MqttCommandSource source = MqttCommandSource::Device;
        std::uint16_t length = 0U;
        std::uint8_t payload[constants::ardCom::RAW_MESSAGE_MAX_SIZE]{};
    };

    explicit Impl(BridgeOptions bridgeOptions)
        : options(std::move(bridgeOptions)),
          serial(resolveSerial(options.serial)),
          ardCom(serial, virtualDevice)
    {
        options.serial = serial;
    }

    BridgeOptions options;
    bool started = false;
    HandshakeState handshakeState = HandshakeState::IDLE;
    bool authoritativeStateDirty = false;
    bool cachedStateRepliesEnabled = false;
    bool controllerConnected = false;
    std::uint32_t lastAuthoritativeStateUpdate_ms = 0U;
    etl::vector<LSHNode, constants::vDev::MAX_ACTUATORS> nodes{};
    VirtualDevice virtualDevice{};
    HardwareSerial *const serial;
    ArdCom ardCom;
    AsyncMqttClient *mqttClient = nullptr;
    AsyncMqttClient *mqttMessageBoundClient = nullptr;
    QueuedMqttCommand mqttCommandQueue[MQTT_COMMAND_QUEUE_CAPACITY]{};
    portMUX_TYPE mqttCommandQueueMux = portMUX_INITIALIZER_UNLOCKED;
    std::uint8_t mqttCommandQueueHead = 0U;
    std::uint8_t mqttCommandQueueTail = 0U;
    std::uint8_t mqttCommandQueueCount = 0U;

    void configureHomie() const
    {
        BridgeIdentity fallbackIdentity;

        char flaggedFirmwareName[HomieInternals::MAX_FIRMWARE_NAME_LENGTH + 11U]{};
        char flaggedFirmwareVersion[HomieInternals::MAX_FIRMWARE_VERSION_LENGTH + 11U]{};
        char flaggedBrand[HomieInternals::MAX_BRAND_LENGTH + 11U]{};

        const char *firmwareName = options.identity.firmwareName();
        const char *firmwareVersion = options.identity.firmwareVersion();
        const char *brand = options.identity.homieBrand();

        if (!buildFlaggedString(firmwareName, FLAG_PREFIX_FIRMWARE_NAME, FLAG_SUFFIX_FIRMWARE_NAME, flaggedFirmwareName))
        {
            const bool fallbackOk = buildFlaggedString(fallbackIdentity.firmwareName(), FLAG_PREFIX_FIRMWARE_NAME, FLAG_SUFFIX_FIRMWARE_NAME, flaggedFirmwareName);
            (void)fallbackOk;
        }

        if (!buildFlaggedString(firmwareVersion, FLAG_PREFIX_FIRMWARE_VERSION, FLAG_SUFFIX_FIRMWARE_VERSION, flaggedFirmwareVersion))
        {
            const bool fallbackOk = buildFlaggedString(fallbackIdentity.firmwareVersion(), FLAG_PREFIX_FIRMWARE_VERSION, FLAG_SUFFIX_FIRMWARE_VERSION, flaggedFirmwareVersion);
            (void)fallbackOk;
        }

        if (!buildFlaggedString(brand, FLAG_PREFIX_BRAND, FLAG_SUFFIX_BRAND, flaggedBrand))
        {
            const bool fallbackOk = buildFlaggedString(fallbackIdentity.homieBrand(), FLAG_PREFIX_BRAND, FLAG_SUFFIX_BRAND, flaggedBrand);
            (void)fallbackOk;
        }

        Homie.__setFirmware(flaggedFirmwareName, flaggedFirmwareVersion);
        Homie.__setBrand(flaggedBrand);

        if (options.disableLedFeedback)
        {
            Homie.disableLedFeedback();
        }

        if (shouldDisableLogging(options.loggingMode))
        {
            Homie.disableLogging();
        }
    }

    void initializeNodesFromModel()
    {
        nodes.clear();
        MqttTopicsBuilder::updateMqttTopics(virtualDevice.getName().c_str());

        for (std::uint8_t index = 0U; index < virtualDevice.getTotalActuators(); ++index)
        {
            const auto actuatorId = virtualDevice.getActuatorId(index);
            nodes.emplace_back(ardCom, virtualDevice, index, actuatorId);
        }
    }

    void clearPendingRuntimeState()
    {
        ardCom.clearPendingActuatorBatch();
        authoritativeStateDirty = false;
        cachedStateRepliesEnabled = false;
    }

    [[nodiscard]] auto publishCachedDetails() -> bool
    {
        if (!virtualDevice.hasCachedDetails())
        {
            return false;
        }

        StaticJsonDocument<constants::ardCom::JSON_RECEIVED_MAX_SIZE> doc;
        if (!virtualDevice.populateDetailsDocument(doc))
        {
            return false;
        }

        return NodeCom::sendJson(doc, MqttTopicsBuilder::mqttOutConfTopic.c_str(), true, 1);
    }

    void requestAuthoritativeStateRefresh()
    {
        clearPendingRuntimeState();
        ardCom.sendJson(constants::payloads::StaticType::ASK_STATE);
    }

    void softResyncMqttPeer()
    {
        if (!virtualDevice.hasCachedDetails())
        {
            Homie.reboot();
            return;
        }

        if (!publishCachedDetails())
        {
            cachedStateRepliesEnabled = false;
            return;
        }

        const bool controllerStateUsable = virtualDevice.isRuntimeSynchronized() && ardCom.isConnected();
        if (controllerStateUsable)
        {
            cachedStateRepliesEnabled = publishAuthoritativeLshState();
            return;
        }

        cachedStateRepliesEnabled = false;
        virtualDevice.invalidateRuntimeModel();

        if (ardCom.isConnected())
        {
            requestAuthoritativeStateRefresh();
        }
    }

    void refreshControllerConnectivity()
    {
        const bool connectedNow = ardCom.isConnected();
        if (connectedNow == controllerConnected)
        {
            return;
        }

        controllerConnected = connectedNow;
        if (!connectedNow)
        {
            clearPendingRuntimeState();
            virtualDevice.invalidateRuntimeModel();
            return;
        }

        if (!virtualDevice.isRuntimeSynchronized())
        {
            requestAuthoritativeStateRefresh();
        }
    }

    [[nodiscard]] auto enqueueMqttCommand(MqttCommandSource source,
                                          const char *payload,
                                          std::size_t length) -> bool
    {
        if (payload == nullptr || length == 0U || length > constants::ardCom::RAW_MESSAGE_MAX_SIZE)
        {
            return false;
        }

        bool queued = false;
        portENTER_CRITICAL(&mqttCommandQueueMux);
        if (mqttCommandQueueCount < MQTT_COMMAND_QUEUE_CAPACITY)
        {
            auto &slot = mqttCommandQueue[mqttCommandQueueTail];
            slot.source = source;
            std::memcpy(slot.payload, payload, length);
            slot.length = static_cast<std::uint16_t>(length);
            mqttCommandQueueTail = static_cast<std::uint8_t>((mqttCommandQueueTail + 1U) % MQTT_COMMAND_QUEUE_CAPACITY);
            ++mqttCommandQueueCount;
            queued = true;
        }
        portEXIT_CRITICAL(&mqttCommandQueueMux);

        return queued;
    }

    [[nodiscard]] auto dequeueMqttCommand(QueuedMqttCommand &outCommand) -> bool
    {
        bool dequeued = false;
        portENTER_CRITICAL(&mqttCommandQueueMux);
        if (mqttCommandQueueCount > 0U)
        {
            outCommand = mqttCommandQueue[mqttCommandQueueHead];
            mqttCommandQueueHead = static_cast<std::uint8_t>((mqttCommandQueueHead + 1U) % MQTT_COMMAND_QUEUE_CAPACITY);
            --mqttCommandQueueCount;
            dequeued = true;
        }
        portEXIT_CRITICAL(&mqttCommandQueueMux);

        return dequeued;
    }

    void clearQueuedMqttCommands()
    {
        portENTER_CRITICAL(&mqttCommandQueueMux);
        mqttCommandQueueHead = 0U;
        mqttCommandQueueTail = 0U;
        mqttCommandQueueCount = 0U;
        portEXIT_CRITICAL(&mqttCommandQueueMux);
    }

    void handleDisconnect()
    {
        mqttClient = nullptr;
        clearPendingRuntimeState();
        clearQueuedMqttCommands();
    }

    void bootstrapDevice()
    {
        auto handshakeDelegate = ArdCom::MessageCallback::create<Impl, &Impl::handleHandshakeMessage>(*this);
        ardCom.onMessage(handshakeDelegate);

        while (handshakeState != HandshakeState::COMPLETE)
        {
            handshakeState = HandshakeState::WAITING_FOR_DETAILS;
            std::uint32_t lastRequest_ms = 0U;
            bool requestDue = true;

            while (handshakeState != HandshakeState::COMPLETE && handshakeState != HandshakeState::FAIL_RESTART)
            {
                const auto now = timeKeeper::getRealTime();

                if (requestDue || (now - lastRequest_ms) > DEVICE_INFO_REQUEST_INTERVAL_MS)
                {
                    if (handshakeState == HandshakeState::WAITING_FOR_DETAILS)
                    {
                        ardCom.sendJson(constants::payloads::StaticType::ASK_DETAILS);
                    }
                    else
                    {
                        ardCom.sendJson(constants::payloads::StaticType::ASK_STATE);
                    }
                    lastRequest_ms = now;
                    requestDue = false;
                }

                if (serial->available())
                {
                    ardCom.processSerialBuffer();
                }

                yield();
            }
        }

        ardCom.onMessage(ArdCom::MessageCallback());
    }

    void handleHandshakeMessage(constants::DeserializeExitCode code, const JsonDocument &doc)
    {
        (void)doc;

        if (code == DeserializeExitCode::OK_BOOT)
        {
            if (handshakeState == HandshakeState::WAITING_FOR_STATE)
            {
                handshakeState = HandshakeState::FAIL_RESTART;
            }
            return;
        }

        switch (handshakeState)
        {
        case HandshakeState::WAITING_FOR_DETAILS:
            if (code == DeserializeExitCode::OK_DETAILS)
            {
                if (ardCom.storeDetailsFromReceived() == DeserializeExitCode::OK_DETAILS)
                {
                    handshakeState = HandshakeState::WAITING_FOR_STATE;
                    // Request the authoritative actuator snapshot immediately
                    // once the topology is known. The bootstrap loop still keeps
                    // its normal retry interval, so this only removes the first
                    // avoidable 500 ms gap between DETAILS and STATE.
                    ardCom.sendJson(constants::payloads::StaticType::ASK_STATE);
                }
                else
                {
                    handshakeState = HandshakeState::FAIL_RESTART;
                }
            }
            break;

        case HandshakeState::WAITING_FOR_STATE:
            if (code == DeserializeExitCode::OK_STATE)
            {
                if (ardCom.storeStateFromReceived() == DeserializeExitCode::OK_STATE)
                {
                    handshakeState = HandshakeState::COMPLETE;
                }
                else
                {
                    handshakeState = HandshakeState::FAIL_RESTART;
                }
            }
            break;

        default:
            break;
        }
    }

    void markAuthoritativeStateDirty()
    {
        authoritativeStateDirty = true;
        cachedStateRepliesEnabled = false;
        lastAuthoritativeStateUpdate_ms = timeKeeper::getRealTime();
    }

    [[nodiscard]] auto publishAuthoritativeLshState() -> bool
    {
        using namespace LSH::protocol;

        StaticJsonDocument<constants::ardCom::MQTT_SET_STATE_DOC_SIZE> doc;
        doc[KEY_PAYLOAD] = static_cast<std::uint8_t>(Command::ACTUATORS_STATE);
        JsonArray packedStates = doc.createNestedArray(KEY_STATE);

        const auto totalActuators = virtualDevice.getTotalActuators();
        std::uint8_t packedByte = 0U;
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < totalActuators; ++actuatorIndex)
        {
            if (virtualDevice.getStateByIndex(actuatorIndex))
            {
                packedByte |= BIT_MASK_8[actuatorIndex & 0x07U];
            }

            const bool endOfPackedByte = ((actuatorIndex & 0x07U) == 0x07U);
            const bool lastActuator = actuatorIndex == (totalActuators - 1U);
            if (endOfPackedByte || lastActuator)
            {
                packedStates.add(packedByte);
                packedByte = 0U;
            }
        }

        return NodeCom::sendJson(doc, MqttTopicsBuilder::mqttOutStateTopic.c_str(), true, 1);
    }

    [[nodiscard]] auto tryServeCachedStateRequest() -> bool
    {
        // The fast-path is only safe after the bridge has already published a
        // fresh controller-backed state snapshot in the current MQTT session.
        // Otherwise Node-RED's paired REQUEST_DETAILS + REQUEST_STATE recovery
        // cycle can observe state before details and later discard that state
        // when the fresh details arrive.
        if (!cachedStateRepliesEnabled || !virtualDevice.isRuntimeSynchronized() || !ardCom.isConnected())
        {
            return false;
        }

        DPL("Serving REQUEST_STATE directly from the synchronized bridge cache.");
        return publishAuthoritativeLshState();
    }

    void publishAllHomieNodeStates()
    {
        for (const auto &node : nodes)
        {
            node.sendState();
        }
    }

    void publishChangedHomieNodeStates()
    {
        for (std::uint8_t actuatorIndex = 0U; actuatorIndex < virtualDevice.getTotalActuators(); ++actuatorIndex)
        {
            if (virtualDevice.isActuatorDirty(actuatorIndex))
            {
                nodes[actuatorIndex].sendState();
            }
        }
    }

    void processPendingStatePublishes()
    {
        if (!authoritativeStateDirty)
        {
            return;
        }

        const auto now = timeKeeper::getRealTime();
        if ((now - lastAuthoritativeStateUpdate_ms) < STATE_PUBLISH_SETTLE_INTERVAL_MS)
        {
            return;
        }

        if (!publishAuthoritativeLshState())
        {
            return;
        }

        if (virtualDevice.consumeFullStatePublishPending())
        {
            publishAllHomieNodeStates();
        }
        else
        {
            publishChangedHomieNodeStates();
        }

        virtualDevice.clearDirtyActuators();
        authoritativeStateDirty = false;
        cachedStateRepliesEnabled = true;
    }

    void handleArdComMessage(constants::DeserializeExitCode code, const JsonDocument &doc)
    {
        switch (code)
        {
        case DeserializeExitCode::OK_BOOT:
            Homie.reboot();
            break;

        case DeserializeExitCode::OK_STATE:
            if (ardCom.storeStateFromReceived() == DeserializeExitCode::OK_STATE)
            {
                ardCom.reconcileDesiredActuatorStatesFromAuthoritative();
                markAuthoritativeStateDirty();
            }
            else
            {
                Homie.reboot();
            }
            break;

        case DeserializeExitCode::OK_DETAILS:
            // After bootstrap the bridge treats device topology as immutable.
            // Runtime DEVICE_DETAILS frames are ignored: configuration drift is
            // recovered through BOOT-driven bridge restarts, while transient
            // MQTT resyncs reuse the cached details snapshot directly.
            break;

        case DeserializeExitCode::OK_NETWORK_CLICK:
            if (Homie.isConnected())
            {
                NodeCom::sendJson(doc, MqttTopicsBuilder::mqttOutMiscTopic.c_str(), false, 2);
            }
            else
            {
                ardCom.triggerFailoverFromReceivedClick();
            }
            break;

        case DeserializeExitCode::OK_OTHER_PAYLOAD:
            if (Homie.isConnected())
            {
                NodeCom::sendJson(doc, MqttTopicsBuilder::mqttOutMiscTopic.c_str(), false, 2);
            }
            break;

        default:
            break;
        }
    }

    void handleHomieEvent(const HomieEvent &event)
    {
        switch (event.type)
        {
        case HomieEventType::WIFI_DISCONNECTED:
        case HomieEventType::MQTT_DISCONNECTED:
            handleDisconnect();
            break;

        case HomieEventType::MQTT_READY:
            if (updateMqttClient() && subscribeMqttTopics())
            {
                softResyncMqttPeer();
            }
            break;

        default:
            break;
        }
    }

    [[nodiscard]] auto updateMqttClient() -> bool
    {
        auto *nextClient = &(Homie.getMqttClient());
        if (nextClient == nullptr)
        {
            return false;
        }
        if (nextClient == mqttClient)
        {
            return true;
        }

        mqttClient = nextClient;
        NodeCom::setMqttClient(mqttClient);
        // AsyncMqttClient stores every onMessage callback that is registered.
        // The Homie MQTT client object survives reconnects, so blindly adding the
        // bridge handler on every MQTT_READY would make each inbound MQTT frame
        // execute the bridge logic multiple times after reconnects.
        if (mqttMessageBoundClient != nextClient)
        {
            mqttClient->onMessage(LSHEspBridge::onMqttMessageStatic_);
            mqttMessageBoundClient = nextClient;
        }
        return true;
    }

    [[nodiscard]] auto subscribeMqttTopics() -> bool
    {
        using constants::mqtt::MQTT_TOPIC_SERVICE;

        mqttClient->unsubscribe(MqttTopicsBuilder::mqttInTopic.c_str());
        mqttClient->unsubscribe(MQTT_TOPIC_SERVICE);

        const std::uint16_t packetIdIn = mqttClient->subscribe(MqttTopicsBuilder::mqttInTopic.c_str(), 2);
        if (packetIdIn == 0U)
        {
            return false;
        }

        const std::uint16_t packetIdSrv = mqttClient->subscribe(MQTT_TOPIC_SERVICE, 1);
        if (packetIdSrv == 0U)
        {
            mqttClient->unsubscribe(MqttTopicsBuilder::mqttInTopic.c_str());
            return false;
        }

        return true;
    }

    [[nodiscard]] auto forwardTypedControllerCommand(const JsonDocument &doc) -> bool
    {
        using namespace LSH::protocol;

        const auto cmd = static_cast<Command>(doc[KEY_PAYLOAD].as<std::uint8_t>());

        switch (cmd)
        {
        case Command::REQUEST_DETAILS:
            if (!virtualDevice.hasCachedDetails())
            {
                Homie.reboot();
            }
            else
            {
                (void)publishCachedDetails();
            }
            return true;

        case Command::REQUEST_STATE:
            if (!tryServeCachedStateRequest())
            {
                if (ardCom.isConnected())
                {
                    requestAuthoritativeStateRefresh();
                }
            }
            return true;

        case Command::FAILOVER:
            ardCom.sendJson(constants::payloads::StaticType::GENERAL_FAILOVER);
            return true;

        case Command::SET_SINGLE_ACTUATOR:
        {
            std::uint8_t actuatorId = 0U;
            bool requestedState = false;
            if (!tryGetUint8Scalar(doc[KEY_ID], actuatorId) || !tryGetBinaryState(doc[KEY_STATE], requestedState))
            {
                return false;
            }
            return ardCom.stageSingleActuatorCommand(actuatorId, requestedState);
        }

        case Command::SET_STATE:
            return ardCom.stageDesiredPackedState(doc[KEY_STATE].as<JsonArrayConst>());

        case Command::NETWORK_CLICK_ACK:
        case Command::FAILOVER_CLICK:
        case Command::NETWORK_CLICK_CONFIRM:
        {
            StaticJsonDocument<constants::ardCom::MQTT_RECEIVED_DOC_MAX_SIZE> serialDoc;
            std::uint8_t clickType = 0U;
            std::uint8_t clickableId = 0U;
            std::uint8_t correlationId = 0U;
            if (!tryGetUint8Scalar(doc[KEY_TYPE], clickType) ||
                !tryGetUint8Scalar(doc[KEY_ID], clickableId) ||
                !tryGetUint8Scalar(doc[KEY_CORRELATION_ID], correlationId))
            {
                return false;
            }

            serialDoc[KEY_PAYLOAD] = static_cast<std::uint8_t>(cmd);
            serialDoc[KEY_TYPE] = clickType;
            serialDoc[KEY_ID] = clickableId;
            serialDoc[KEY_CORRELATION_ID] = correlationId;
            ardCom.sendJson(serialDoc);
            return true;
        }

        default:
            return false;
        }
    }

    void forwardOrTranslateToController(const char *payload,
                                        std::size_t length,
                                        const JsonDocument &doc)
    {
#if (defined(CONFIG_MSG_PACK_MQTT) && defined(CONFIG_MSG_PACK_ARDUINO)) || \
    (!defined(CONFIG_MSG_PACK_MQTT) && !defined(CONFIG_MSG_PACK_ARDUINO))
        ardCom.sendJson(payload, length);
#else
        ardCom.sendJson(doc);
#endif
    }

    void processDeviceTopicCommand(LSH::protocol::Command cmd,
                                   const char *payload,
                                   std::size_t length,
                                   const JsonDocument &doc)
    {
        using namespace LSH::protocol;

        switch (cmd)
        {
        case Command::SYSTEM_RESET:
            Homie.reset();
            Homie.setIdle(true);
            return;

        case Command::SYSTEM_REBOOT:
            Homie.reboot();
            return;

        case Command::PING_:
            NodeCom::sendJson(constants::payloads::StaticType::PING_);
            return;

        case Command::SET_SINGLE_ACTUATOR:
        case Command::SET_STATE:
            if (!virtualDevice.isRuntimeSynchronized())
            {
                forwardOrTranslateToController(payload, length, doc);
                return;
            }
            break;

        default:
            break;
        }

        if (!forwardTypedControllerCommand(doc))
        {
            forwardOrTranslateToController(payload, length, doc);
        }
    }

    void processServiceTopicCommand(LSH::protocol::Command cmd)
    {
        using namespace LSH::protocol;

        switch (cmd)
        {
        case Command::BOOT:
            softResyncMqttPeer();
            return;

        case Command::SYSTEM_RESET:
            Homie.reset();
            Homie.setIdle(true);
            return;

        case Command::SYSTEM_REBOOT:
            Homie.reboot();
            return;

        case Command::PING_:
            NodeCom::sendJson(constants::payloads::StaticType::PING_);
            return;

        default:
            return;
        }
    }

    void processInboundMqttCommand(MqttCommandSource source, const char *payload, std::size_t length)
    {
        using namespace LSH::protocol;

        StaticJsonDocument<constants::ardCom::MQTT_RECEIVED_DOC_MAX_SIZE> doc;

#ifdef CONFIG_MSG_PACK_MQTT
        const DeserializationError err = deserializeMsgPack(doc,
                                                            reinterpret_cast<const std::uint8_t *>(payload),
                                                            length,
                                                            DeserializationOption::NestingLimit(2));
#else
        const DeserializationError err = deserializeJson(doc,
                                                         payload,
                                                         length,
                                                         DeserializationOption::NestingLimit(2));
#endif

        if (err != DeserializationError::Ok)
        {
            return;
        }

        std::uint8_t rawCommand = 0U;
        if (!tryGetUint8Scalar(doc[KEY_PAYLOAD], rawCommand))
        {
            return;
        }

        const auto cmd = static_cast<Command>(rawCommand);
        if (source == MqttCommandSource::Service)
        {
            processServiceTopicCommand(cmd);
            return;
        }

        processDeviceTopicCommand(cmd, payload, length, doc);
    }

    void processQueuedMqttCommands()
    {
        while (!serial->available())
        {
            QueuedMqttCommand queuedCommand{};
            if (!dequeueMqttCommand(queuedCommand))
            {
                return;
            }

            processInboundMqttCommand(queuedCommand.source,
                                      reinterpret_cast<const char *>(queuedCommand.payload),
                                      queuedCommand.length);
        }
    }

    void handleMqttMessage(char *topic,
                           char *payload,
                           AsyncMqttClientMessageProperties properties,
                           std::size_t len,
                           std::size_t index,
                           std::size_t total)
    {
        const auto topicHash = djb2_hash(topic);
        MqttCommandSource source = MqttCommandSource::Device;
        if (topicHash == MqttTopicsBuilder::mqttInTopicHash)
        {
            source = MqttCommandSource::Device;
        }
        else if (topicHash == constants::mqtt::MQTT_TOPIC_SERVICE_HASH)
        {
            source = MqttCommandSource::Service;
        }
        else
        {
            return;
        }

        if (total == 0U || len == 0U || properties.retain)
        {
            return;
        }

        if (total > constants::ardCom::RAW_MESSAGE_MAX_SIZE)
        {
            return;
        }

        if (index != 0U || len != total)
        {
            return;
        }

        (void)enqueueMqttCommand(source, payload, total);
    }
};

LSHEspBridge *LSHEspBridge::activeInstance_ = nullptr;

BridgeIdentity::BridgeIdentity()
{
    resetToDefaults();
}

BridgeIdentity::BridgeIdentity(const char *firmwareName,
                               const char *firmwareVersion,
                               const char *homieBrand)
{
    resetToDefaults();
    (void)setFirmwareName(firmwareName);
    (void)setFirmwareVersion(firmwareVersion);
    (void)setHomieBrand(homieBrand);
}

void BridgeIdentity::resetToDefaults()
{
    firmwareName_.assign("lsh-homie");
        firmwareVersion_.assign("1.0.2");
    homieBrand_.assign("LaboSmartHome");
}

auto BridgeIdentity::setFirmwareName(const char *value) -> bool
{
    return assignBounded(firmwareName_, value);
}

auto BridgeIdentity::setFirmwareVersion(const char *value) -> bool
{
    return assignBounded(firmwareVersion_, value);
}

auto BridgeIdentity::setHomieBrand(const char *value) -> bool
{
    return assignBounded(homieBrand_, value);
}

auto BridgeIdentity::firmwareName() const noexcept -> const char *
{
    return firmwareName_.c_str();
}

auto BridgeIdentity::firmwareVersion() const noexcept -> const char *
{
    return firmwareVersion_.c_str();
}

auto BridgeIdentity::homieBrand() const noexcept -> const char *
{
    return homieBrand_.c_str();
}

LSHEspBridge::LSHEspBridge(BridgeOptions options)
    : impl_(std::make_unique<Impl>(std::move(options)))
{
}

LSHEspBridge::~LSHEspBridge()
{
    if (activeInstance_ == this)
    {
        NodeCom::setMqttClient(nullptr);
        activeInstance_ = nullptr;
    }
}

void LSHEspBridge::begin()
{
    if (impl_->started)
    {
        return;
    }

    activeInstance_ = this;
    timeKeeper::update();
    impl_->ardCom.begin();
    impl_->configureHomie();
    Homie.onEvent(onHomieEventStatic_);
    impl_->bootstrapDevice();
    impl_->controllerConnected = impl_->ardCom.isConnected();

    auto mainDelegate = ArdCom::MessageCallback::create<Impl, &Impl::handleArdComMessage>(*impl_);
    impl_->ardCom.onMessage(mainDelegate);

    impl_->initializeNodesFromModel();
    Homie.setup();
    impl_->started = true;

#ifdef HOMIE_RESET
    Homie.reset();
    Homie.setIdle(true);
#endif
}

void LSHEspBridge::loop()
{
    if (!impl_->started)
    {
        return;
    }

    Homie.loop();
    timeKeeper::update();

    if (impl_->serial->available())
    {
        impl_->ardCom.processSerialBuffer();
    }

    impl_->processQueuedMqttCommands();
    impl_->refreshControllerConnectivity();
    impl_->ardCom.processPendingActuatorBatch();
    impl_->processPendingStatePublishes();
    impl_->ardCom.sendJson(constants::payloads::StaticType::PING_);
}

auto LSHEspBridge::isRuntimeSynchronized() const noexcept -> bool
{
    return impl_->virtualDevice.isRuntimeSynchronized();
}

auto LSHEspBridge::deviceName() const noexcept -> const char *
{
    return impl_->virtualDevice.getName().c_str();
}

void LSHEspBridge::onHomieEventStatic_(const HomieEvent &event)
{
    if (activeInstance_ != nullptr && activeInstance_->impl_ != nullptr)
    {
        activeInstance_->impl_->handleHomieEvent(event);
    }
}

void LSHEspBridge::onMqttMessageStatic_(char *topic,
                                        char *payload,
                                        AsyncMqttClientMessageProperties properties,
                                        std::size_t len,
                                        std::size_t index,
                                        std::size_t total)
{
    if (activeInstance_ != nullptr && activeInstance_->impl_ != nullptr)
    {
        activeInstance_->impl_->handleMqttMessage(topic, payload, properties, len, index, total);
    }
}

} // namespace lsh::esp
