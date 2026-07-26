#include "MQTT.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "ServiceEnvelope.h"
#include "configuration.h"
#include "main.h"
#include "mesh/Channels.h"
#include "mesh/Router.h"
#include "mesh/generated/meshtastic/mqtt.pb.h"
#include "mesh/generated/meshtastic/telemetry.pb.h"
#include "modules/RoutingModule.h"
#if defined(ARCH_ESP32)
#include "../mesh/generated/meshtastic/paxcount.pb.h"
#endif
#include "mesh/generated/meshtastic/remote_hardware.pb.h"
#include "sleep.h"
#if HAS_WIFI
#include "mesh/wifi/WiFiAPClient.h"
#include <WiFi.h>
#endif
#if HAS_ETHERNET && defined(USE_WS5500)
#include <ETHClass2.h>
#define ETH ETH2
#elif HAS_ETHERNET && defined(USE_CH390D)
#include "ESP32_CH390.h"
#define ETH CH390
#endif // HAS_ETHERNET
#include "Default.h"
#if !defined(ARCH_NRF52) || NRF52_USE_JSON
#include "serialization/JSON.h"
#include "serialization/MeshPacketSerializer.h"
#include <pb_decode.h>
#endif
#include <Throttle.h>
#include <assert.h>
#include <utility>
#include "modules/victron.pb.h" // En-tête généré pour VictronData
#include "IPAddress.h"
#if defined(ARCH_PORTDUINO)
#include <netinet/in.h>
#elif !defined(ntohl)
#include <machine/endian.h>
#define ntohl __ntohl
#endif
#include <RTC.h>

MQTT *mqtt;

namespace
{
constexpr int reconnectMax = 5;

static uint8_t bytes[meshtastic_MqttClientProxyMessage_size + 30];

static bool isMqttServerAddressPrivate = false;
static bool isConnected = false;

static uint32_t lastPositionUnavailableWarning = 0;
static const uint32_t POSITION_UNAVAILABLE_WARNING_INTERVAL_MS = 15000;

inline void onReceiveProto(char *topic, byte *payload, size_t length)
{
    const DecodedServiceEnvelope e(payload, length);
    if (!e.validDecode || e.channel_id == NULL || e.gateway_id == NULL || e.packet == NULL) {
        LOG_ERROR("Invalid MQTT service envelope, topic %s, len %u!", topic, length);
        return;
    }

    const meshtastic_Channel &ch = channels.getByName(e.channel_id);
    if (!(strcmp(e.channel_id, "PKI") == 0 ||
          (strcmp(e.channel_id, channels.getGlobalId(ch.index)) == 0 && ch.settings.downlink_enabled))) {
        return;
    }

    bool anyChannelHasDownlink = false;
    size_t numChan = channels.getNumChannels();
    for (size_t i = 0; i < numChan; ++i) {
        const auto &c = channels.getByIndex(i);
        if (c.settings.downlink_enabled) {
            anyChannelHasDownlink = true;
            break;
        }
    }

    if (strcmp(e.channel_id, "PKI") == 0 && !anyChannelHasDownlink) {
        return;
    }
    
    std::string nodeId = nodeDB->getNodeId();
    if (strcmp(e.gateway_id, nodeId.c_str()) == 0) {
        if (isFromUs(e.packet)) {
            auto pAck = routingModule->allocAckNak(meshtastic_Routing_Error_NONE, getFrom(e.packet), e.packet->id, ch.index);
            pAck->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MQTT;
            router->sendLocal(pAck);
        } else {
            LOG_INFO("Ignore downlink message we originally sent");
        }
        return;
    }
    if (isFromUs(e.packet)) {
        LOG_INFO("Ignore downlink message we originally sent");
        return;
    }

    LOG_INFO("Received MQTT topic %s, len=%u", topic, length);
    if (e.packet->hop_limit > HOP_MAX || e.packet->hop_start > HOP_MAX) {
        LOG_INFO("Invalid hop_limit(%u) or hop_start(%u)", e.packet->hop_limit, e.packet->hop_start);
        return;
    }

    UniquePacketPoolPacket p = packetPool.allocUniqueZeroed();
    p->from = e.packet->from;
    p->to = e.packet->to;
    p->id = e.packet->id;
    p->channel = e.packet->channel;
    p->hop_limit = e.packet->hop_limit;
    p->hop_start = e.packet->hop_start;
    p->want_ack = e.packet->want_ack;
    p->via_mqtt = true;
    p->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MQTT;
    p->which_payload_variant = e.packet->which_payload_variant;
    memcpy(&p->decoded, &e.packet->decoded, std::max(sizeof(p->decoded), sizeof(p->encrypted)));

    if (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        if (moduleConfig.mqtt.encryption_enabled) {
            LOG_INFO("Ignore decoded message on MQTT, encryption is enabled");
            return;
        }
        if (p->decoded.portnum == meshtastic_PortNum_ADMIN_APP) {
            LOG_INFO("Ignore decoded admin packet");
            return;
        }
        p->channel = ch.index;
    }

    if (router && p->which_payload_variant == meshtastic_MeshPacket_encrypted_tag && strcmp(e.channel_id, "PKI") == 0) {
        const meshtastic_NodeInfoLite *tx = nodeDB->getMeshNode(getFrom(p.get()));
        const meshtastic_NodeInfoLite *rx = nodeDB->getMeshNode(p->to);
        if (isToUs(p.get()) || (tx && tx->has_user && rx && rx->has_user))
            router->enqueueReceivedMessage(p.release());
    } else if (router && perhapsDecode(p.get()) == DecodeState::DECODE_SUCCESS)
        router->enqueueReceivedMessage(p.release());
}

#if !defined(ARCH_NRF52) || NRF52_USE_JSON
inline bool isValidJsonEnvelope(JSONObject &json)
{
    std::string nodeId = nodeDB->getNodeId();
    return (json.find("sender") != json.end() ? (json["sender"]->AsString().compare(nodeId) != 0) : true) &&
           (json.find("hopLimit") != json.end() ? json["hopLimit"]->IsNumber() : true) &&
           (json.find("from") != json.end()) && json["from"]->IsNumber() &&
           (json.find("type") != json.end()) && json["type"]->IsString() &&
           (json.find("payload") != json.end());
}

inline void onReceiveJson(byte *payload, size_t length)
{
    char payloadStr[length + 1];
    memcpy(payloadStr, payload, length);
    payloadStr[length] = 0;
    std::unique_ptr<JSONValue> json_value(JSON::Parse(payloadStr));
    if (json_value == nullptr) {
        LOG_ERROR("JSON received payload on MQTT but not a valid JSON");
        return;
    }

    JSONObject json = json_value->AsObject();

    if (!isValidJsonEnvelope(json)) {
        LOG_ERROR("JSON received payload on MQTT but not a valid envelope");
        return;
    }

    if (json["type"]->AsString().compare("sendtext") == 0 && json["payload"]->IsString()) {
        std::string jsonPayloadStr = json["payload"]->AsString();
        LOG_INFO("JSON payload %s, length %u", jsonPayloadStr.c_str(), jsonPayloadStr.length());

        meshtastic_MeshPacket *p = router->allocForSending();
        p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        if (json.find("channel") != json.end() && json["channel"]->IsNumber() &&
            (json["channel"]->AsNumber() < channels.getNumChannels()))
            p->channel = json["channel"]->AsNumber();
        if (json.find("to") != json.end() && json["to"]->IsNumber())
            p->to = json["to"]->AsNumber();
        if (json.find("hopLimit") != json.end() && json["hopLimit"]->IsNumber())
            p->hop_limit = json["hopLimit"]->AsNumber();
        if (jsonPayloadStr.length() <= sizeof(p->decoded.payload.bytes)) {
            memcpy(p->decoded.payload.bytes, jsonPayloadStr.c_str(), jsonPayloadStr.length());
            p->decoded.payload.size = jsonPayloadStr.length();
            service->sendToMesh(p, RX_SRC_LOCAL);
        } else {
            LOG_WARN("Received MQTT json payload too long, drop");
        }
    } else if (json["type"]->AsString().compare("sendposition") == 0 && json["payload"]->IsObject()) {
        JSONObject posit = json["payload"]->AsObject();
        meshtastic_Position pos = meshtastic_Position_init_default;
        if (posit.find("latitude_i") != posit.end() && posit["latitude_i"]->IsNumber())
            pos.latitude_i = posit["latitude_i"]->AsNumber();
        if (posit.find("longitude_i") != posit.end() && posit["longitude_i"]->IsNumber())
            pos.longitude_i = posit["longitude_i"]->AsNumber();
        if (posit.find("altitude") != posit.end() && posit["altitude"]->IsNumber())
            pos.altitude = posit["altitude"]->AsNumber();
        if (posit.find("time") != posit.end() && posit["time"]->IsNumber())
            pos.time = posit["time"]->AsNumber();

        meshtastic_MeshPacket *p = router->allocForSending();
        p->decoded.portnum = meshtastic_PortNum_POSITION_APP;
        if (json.find("channel") != json.end() && json["channel"]->IsNumber() &&
            (json["channel"]->AsNumber() < channels.getNumChannels()))
            p->channel = json["channel"]->AsNumber();
        if (json.find("to") != json.end() && json["to"]->IsNumber())
            p->to = json["to"]->AsNumber();
        if (json.find("hopLimit") != json.end() && json["hopLimit"]->IsNumber())
            p->hop_limit = json["hopLimit"]->AsNumber();
        p->decoded.payload.size =
            pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Position_msg, &pos);
        service->sendToMesh(p, RX_SRC_LOCAL);
    } else {
        LOG_DEBUG("JSON ignore downlink message with unsupported type");
    }
}
#endif

bool isPrivateIpAddress(const IPAddress &ip)
{
    constexpr struct {
        uint32_t network;
        uint32_t mask;
    } privateCidrRanges[] = {
        {.network = 192u << 24 | 168 << 16, .mask = 0xffff0000},
        {.network = 172u << 24 | 16 << 16, .mask = 0xfff00000},
        {.network = 169u << 24 | 254 << 16, .mask = 0xffff0000},
        {.network = 10u << 24, .mask = 0xff000000},
        {.network = 127u << 24 | 1, .mask = 0xffffffff},
        {.network = 100u << 24 | 64 << 16, .mask = 0xffc00000},
    };
    const uint32_t addr = ntohl(ip);
    for (const auto &cidrRange : privateCidrRanges) {
        if (cidrRange.network == (addr & cidrRange.mask)) {
            LOG_INFO("MQTT server on a private IP");
            return true;
        }
    }
    return false;
}

std::pair<String, uint16_t> parseHostAndPort(String server, uint16_t port = 0)
{
    const int delimIndex = server.indexOf(':');
    if (delimIndex > 0) {
        const long parsedPort = server.substring(delimIndex + 1, server.length()).toInt();
        if (parsedPort < 1 || parsedPort > UINT16_MAX) {
            LOG_WARN("Invalid MQTT port %d: %s", parsedPort, server.c_str());
        } else {
            port = parsedPort;
        }
        server[delimIndex] = 0;
    }
    return std::make_pair(std::move(server), port);
}

bool isDefaultServer(const String &host)
{
    return host.length() == 0 || host == default_mqtt_address;
}

bool isDefaultRootTopic(const String &root)
{
    return root.length() == 0 || root == default_mqtt_root;
}

struct PubSubConfig {
    explicit PubSubConfig(const meshtastic_ModuleConfig_MQTTConfig &config)
    {
        if (*config.address) {
            serverAddr = config.address;
            mqttUsername = config.username;
            mqttPassword = config.password;
        }
        if (config.tls_enabled) {
            serverPort = 8883;
        }
        auto [parsedServerAddr, parsedServerPort] = parseHostAndPort(serverAddr.c_str(), serverPort);
        serverAddr = std::move(parsedServerAddr);
        serverPort = parsedServerPort;
    }

    static constexpr uint16_t defaultPort = 1883;
    static constexpr uint16_t defaultPortTls = 8883;

    uint16_t serverPort = defaultPort;
    String serverAddr = default_mqtt_address;
    const char *mqttUsername = default_mqtt_username;
    const char *mqttPassword = default_mqtt_password;
};

#if HAS_NETWORKING
bool connectPubSub(const PubSubConfig &config, PubSubClient &pubSub, Client &client)
{
    pubSub.setBufferSize(1024, 1024);
    pubSub.setClient(client);
    pubSub.setServer(config.serverAddr.c_str(), config.serverPort);

    LOG_INFO("Connecting directly to MQTT server %s, port: %d, username: %s, password: %s", config.serverAddr.c_str(),
             config.serverPort, config.mqttUsername, config.mqttPassword);

    std::string nodeId = nodeDB->getNodeId();
    const bool connected = pubSub.connect(nodeId.c_str(), config.mqttUsername, config.mqttPassword);
    if (connected) {
        isConnected = true;
        LOG_INFO("MQTT connected");
    } else {
        isConnected = false;
        LOG_WARN("Failed to connect to MQTT server");
    }
    return connected;
}
#endif

inline bool isConnectedToNetwork()
{
#ifdef USE_WS5500
    if (ETH.connected())
        return true;
#elif defined(USE_CH390D)
    if (ETH.isConnected())
        return true;
#endif

#if HAS_WIFI
    return WiFi.isConnected();
#elif HAS_ETHERNET
    return Ethernet.linkStatus() == LinkON;
#else
    return false;
#endif
}

bool wantsLink()
{
    const bool hasChannelorMapReport =
        moduleConfig.mqtt.enabled && (moduleConfig.mqtt.map_reporting_enabled || channels.anyMqttEnabled());
    return hasChannelorMapReport && (moduleConfig.mqtt.proxy_to_client_enabled || isConnectedToNetwork());
}
} // namespace

void MQTT::mqttCallback(char *topic, byte *payload, unsigned int length)
{
    mqtt->onReceive(topic, payload, length);
}

void MQTT::onClientProxyReceive(meshtastic_MqttClientProxyMessage msg)
{
    onReceive(msg.topic, msg.payload_variant.data.bytes, msg.payload_variant.data.size);
}

void MQTT::onReceive(char *topic, byte *payload, size_t length)
{
    if (length == 0) {
        LOG_WARN("Empty MQTT payload received, topic %s!", topic);
        return;
    }

    if (moduleConfig.mqtt.json_enabled && (strncmp(topic, jsonTopic.c_str(), jsonTopic.length()) == 0)) {
#if !defined(ARCH_NRF52) || NRF52_USE_JSON
        char *channelName = topic + jsonTopic.length();
        channelName = strtok(channelName, "/") ? strtok(channelName, "/") : channelName;
        const meshtastic_Channel &sendChannel = channels.getByName(channelName);
        if (!(strncasecmp(channels.getGlobalId(sendChannel.index), Channels::mqttChannel, strlen(Channels::mqttChannel)) == 0 &&
              sendChannel.settings.downlink_enabled)) {
            LOG_WARN("JSON downlink received on channel not called 'mqtt' or without downlink enabled");
            return;
        }
        onReceiveJson(payload, length);
#endif
        return;
    }

    onReceiveProto(topic, payload, length);
}

void mqttInit()
{
    new MQTT();
}

#if HAS_NETWORKING
MQTT::MQTT() : MQTT(std::unique_ptr<MQTTClient>(new MQTTClient())) {}
MQTT::MQTT(std::unique_ptr<MQTTClient> _mqttClient)
    : concurrency::OSThread("mqtt"), mqttQueue(MAX_MQTT_QUEUE), mqttClient(std::move(_mqttClient)), pubSub(*mqttClient)
#else
MQTT::MQTT() : concurrency::OSThread("mqtt"), mqttQueue(MAX_MQTT_QUEUE)
#endif
{
    if (moduleConfig.mqtt.enabled) {
        LOG_DEBUG("Init MQTT");

        assert(!mqtt);
        mqtt = this;

        if (*moduleConfig.mqtt.root) {
            cryptTopic = moduleConfig.mqtt.root + cryptTopic;
            jsonTopic = moduleConfig.mqtt.root + jsonTopic;
            mapTopic = moduleConfig.mqtt.root + mapTopic;
            isConfiguredForDefaultRootTopic = isDefaultRootTopic(moduleConfig.mqtt.root);
        } else {
            cryptTopic = "msh" + cryptTopic;
            jsonTopic = "msh" + jsonTopic;
            mapTopic = "msh" + mapTopic;
            isConfiguredForDefaultRootTopic = true;
        }

        if (moduleConfig.mqtt.map_reporting_enabled && moduleConfig.mqtt.has_map_report_settings) {
            map_position_precision = Default::getConfiguredOrDefault(moduleConfig.mqtt.map_report_settings.position_precision,
                                                                     default_map_position_precision);
            map_publish_interval_msecs = Default::getConfiguredOrDefaultMs(
                moduleConfig.mqtt.map_report_settings.publish_interval_secs, default_map_publish_interval_secs);
        }

        auto [host, parsedPort] = parseHostAndPort(moduleConfig.mqtt.address);
        (void)parsedPort;
        isConfiguredForDefaultServer = isDefaultServer(host);
        IPAddress ip;
        isMqttServerAddressPrivate = ip.fromString(host.c_str()) && isPrivateIpAddress(ip);

#if HAS_NETWORKING
        if (!moduleConfig.mqtt.proxy_to_client_enabled)
            pubSub.setCallback(mqttCallback);
#endif

        if (moduleConfig.mqtt.proxy_to_client_enabled) {
            LOG_INFO("MQTT configured to use client proxy");
            enabled = true;
            runASAP = true;
            reconnectCount = 0;
#if !IS_RUNNING_TESTS
            publishNodeInfo();
#endif
        }
    } else {
        disable();
    }
}

bool MQTT::isConnectedDirectly()
{
#if HAS_NETWORKING
    return pubSub.connected();
#else
    return false;
#endif
}

bool MQTT::publish(const char *topic, const char *payload, bool retained)
{
    if (moduleConfig.mqtt.proxy_to_client_enabled) {
        meshtastic_MqttClientProxyMessage *msg = mqttClientProxyMessagePool.allocZeroed();
        msg->which_payload_variant = meshtastic_MqttClientProxyMessage_text_tag;
        strncpy(msg->topic, topic, sizeof(msg->topic));
        msg->topic[sizeof(msg->topic) - 1] = '\0';
        strncpy(msg->payload_variant.text, payload, sizeof(msg->payload_variant.text));
        msg->payload_variant.text[sizeof(msg->payload_variant.text) - 1] = '\0';
        msg->retained = retained;
        service->sendMqttMessageToClientProxy(msg);
        return true;
    }
#if HAS_NETWORKING
    else if (isConnectedDirectly()) {
        return pubSub.publish(topic, payload, retained);
    }
#endif
    return false;
}

bool MQTT::publish(const char *topic, const uint8_t *payload, size_t length, bool retained)
{
    if (moduleConfig.mqtt.proxy_to_client_enabled) {
        meshtastic_MqttClientProxyMessage *msg = mqttClientProxyMessagePool.allocZeroed();
        msg->which_payload_variant = meshtastic_MqttClientProxyMessage_data_tag;
        strncpy(msg->topic, topic, sizeof(msg->topic));
        msg->topic[sizeof(msg->topic) - 1] = '\0';
        if (length > sizeof(msg->payload_variant.data.bytes))
            length = sizeof(msg->payload_variant.data.bytes);
        msg->payload_variant.data.size = length;
        memcpy(msg->payload_variant.data.bytes, payload, length);
        msg->retained = retained;
        service->sendMqttMessageToClientProxy(msg);
        return true;
    }
#if HAS_NETWORKING
    else if (isConnectedDirectly()) {
        return pubSub.publish(topic, payload, length, retained);
    }
#endif
    return false;
}

void MQTT::reconnect()
{
    isConnected = false;
    if (wantsLink()) {
        if (moduleConfig.mqtt.proxy_to_client_enabled) {
            LOG_INFO("MQTT connect via client proxy instead");
            enabled = true;
            runASAP = true;
            reconnectCount = 0;

            publishNodeInfo();
            return;
        }
#if HAS_NETWORKING
        const PubSubConfig ps_config(moduleConfig.mqtt);
        MQTTClient *clientConnection = mqttClient.get();
#if MQTT_SUPPORTS_TLS
        if (moduleConfig.mqtt.tls_enabled) {
            mqttClientTLS.setInsecure();
            LOG_INFO("Use TLS-encrypted session");
            clientConnection = &mqttClientTLS;
        } else {
            LOG_INFO("Use non-TLS-encrypted session");
        }
#endif
        if (connectPubSub(ps_config, pubSub, *clientConnection)) {
            enabled = true;
            runASAP = true;
            reconnectCount = 0;
            isMqttServerAddressPrivate = isPrivateIpAddress(clientConnection->remoteIP());
            isConnected = true;
            publishNodeInfo();
            sendSubscriptions();
        } else {
#if HAS_WIFI && !defined(ARCH_PORTDUINO)
            reconnectCount++;
            LOG_ERROR("Failed to contact MQTT server directly (%d/%d)", reconnectCount, reconnectMax);
            if (reconnectCount >= reconnectMax) {
                needReconnect = true;
                wifiReconnect->setIntervalFromNow(0);
                reconnectCount = 0;
            }
#endif
        }
#endif
    }
}

void MQTT::sendSubscriptions()
{
#if HAS_NETWORKING
    bool hasDownlink = false;
    size_t numChan = channels.getNumChannels();
    for (size_t i = 0; i < numChan; i++) {
        const auto &ch = channels.getByIndex(i);
        if (ch.settings.downlink_enabled) {
            hasDownlink = true;
            std::string topic = cryptTopic + channels.getGlobalId(i) + "/+";
            LOG_INFO("Subscribe to %s", topic.c_str());
            pubSub.subscribe(topic.c_str(), 1);
#if !defined(ARCH_NRF52) || defined(NRF52_USE_JSON)
            if (moduleConfig.mqtt.json_enabled == true) {
                std::string topicDecoded = jsonTopic + channels.getGlobalId(i) + "/+";
                LOG_INFO("Subscribe to %s", topicDecoded.c_str());
                pubSub.subscribe(topicDecoded.c_str(), 1);
            }
#endif
        }
    }
#if !MESHTASTIC_EXCLUDE_PKI
    if (hasDownlink) {
        std::string topic = cryptTopic + "PKI/+";
        LOG_INFO("Subscribe to %s", topic.c_str());
        pubSub.subscribe(topic.c_str(), 1);
    }
#endif
#endif
}

int32_t MQTT::runOnce()
{
    if (!moduleConfig.mqtt.enabled || !(moduleConfig.mqtt.map_reporting_enabled || channels.anyMqttEnabled()))
        return disable();
    bool wantConnection = wantsLink();

    perhapsReportToMap();

    if (moduleConfig.mqtt.proxy_to_client_enabled) {
        publishQueuedMessages();
        return 200;
    }
#if HAS_NETWORKING
    else if (!pubSub.loop()) {
        if (!wantConnection)
            return 5000;
        else {
            reconnect();
            if (isConnectedDirectly()) {
                publishQueuedMessages();
                return 200;
            } else
                return 30000;
        }
    } else {
        if (!wantConnection) {
            LOG_INFO("MQTT link not needed, drop");
            pubSub.disconnect();
        }

        powerFSM.trigger(EVENT_CONTACT_FROM_PHONE);
        return 20;
    }
#else
    return 30000;
#endif
}

bool MQTT::isValidConfig(const meshtastic_ModuleConfig_MQTTConfig &config, MQTTClient *client)
{
    const PubSubConfig parsed(config);

    if (config.enabled && !config.proxy_to_client_enabled) {
#if HAS_NETWORKING
        if (config.tls_enabled) {
#if !MQTT_SUPPORTS_TLS
            LOG_ERROR("Invalid MQTT config: tls_enabled is not supported on this node");
            return false;
#endif
        }
        if (isConnectedToNetwork()) {
            MQTTClient testClient;
            if (!testClient.connect(parsed.serverAddr.c_str(), parsed.serverPort)) {
                const char *warning = "Could not reach the MQTT server. Settings will be saved, but please verify the server "
                                      "address and credentials.";
                LOG_WARN(warning);
#if !IS_RUNNING_TESTS
                meshtastic_ClientNotification *cn = clientNotificationPool.allocZeroed();
                if (cn) {
                    cn->level = meshtastic_LogRecord_Level_WARNING;
                    cn->time = getValidTime(RTCQualityFromNet);
                    strncpy(cn->message, warning, sizeof(cn->message) - 1);
                    cn->message[sizeof(cn->message) - 1] = '\0';
                    service->sendClientNotification(cn);
                }
#endif
            }
            testClient.stop();
        }
#else
        const char *warning = "Invalid MQTT config: proxy_to_client_enabled must be enabled on nodes that do not have a network";
        LOG_ERROR(warning);
#if !IS_RUNNING_TESTS
        meshtastic_ClientNotification *cn = clientNotificationPool.allocZeroed();
        cn->level = meshtastic_LogRecord_Level_ERROR;
        cn->time = getValidTime(RTCQualityFromNet);
        strncpy(cn->message, warning, sizeof(cn->message) - 1);
        cn->message[sizeof(cn->message) - 1] = '\0';
        service->sendClientNotification(cn);
#endif
        return false;
#endif
    }

    const bool defaultServer = isDefaultServer(parsed.serverAddr);
    if (defaultServer && !IS_ONE_OF(parsed.serverPort, PubSubConfig::defaultPort, PubSubConfig::defaultPortTls)) {
        const char *warning = "Invalid MQTT config: default server address must not have a port specified";
        LOG_ERROR(warning);
#if !IS_RUNNING_TESTS
        meshtastic_ClientNotification *cn = clientNotificationPool.allocZeroed();
        cn->level = meshtastic_LogRecord_Level_ERROR;
        cn->time = getValidTime(RTCQualityFromNet);
        strncpy(cn->message, warning, sizeof(cn->message) - 1);
        cn->message[sizeof(cn->message) - 1] = '\0';
        service->sendClientNotification(cn);
#endif
        return false;
    }
    return true;
}

void MQTT::publishNodeInfo()
{
}

void MQTT::publishQueuedMessages()
{
    if (mqttQueue.isEmpty())
        return;

    if (!moduleConfig.mqtt.proxy_to_client_enabled && !isConnected)
        return;

    LOG_DEBUG("Publish enqueued MQTT message");
    const std::unique_ptr<QueueEntry> entry(mqttQueue.dequeuePtr(0));
    LOG_INFO("publish %s, %u bytes from queue", entry->topic.c_str(), entry->envBytes.size());
    publish(entry->topic.c_str(), entry->envBytes.data(), entry->envBytes.size(), false);

#if !defined(ARCH_NRF52) || defined(NRF52_USE_JSON)
    if (!moduleConfig.mqtt.json_enabled)
        return;

    const DecodedServiceEnvelope mp_decoded(entry->envBytes.data(), entry->envBytes.size());
    if (!mp_decoded.validDecode || mp_decoded.packet == NULL || mp_decoded.channel_id == NULL)
        return;

    std::string jsonString = MeshPacketSerializer::JsonSerialize(mp_decoded.packet);

    if (jsonString.length() == 0)
        return;

    std::string nodeId = nodeDB->getNodeId();

    std::string topicJson;
    if (mp_decoded.packet->pki_encrypted)
        topicJson = jsonTopic + "PKI/" + mp_decoded.gateway_id;
    else
        topicJson = jsonTopic + mp_decoded.channel_id + "/" + mp_decoded.gateway_id;
    LOG_INFO("JSON publish message to %s, %u bytes: %s", topicJson.c_str(), jsonString.length(), jsonString.c_str());
    publish(topicJson.c_str(), jsonString.c_str(), false);
#endif
}

void MQTT::onSend(const meshtastic_MeshPacket &mp_encrypted, const meshtastic_MeshPacket &mp_decoded, ChannelIndex chIndex)
{
    if (mp_encrypted.via_mqtt)
        return;
    bool uplinkEnabled = false;
    for (int i = 0; i <= 7; i++) {
        if (channels.getByIndex(i).settings.uplink_enabled)
            uplinkEnabled = true;
    }
    if (!uplinkEnabled)
        return;
    auto &ch = channels.getByIndex(chIndex);

    if (mp_decoded.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        bool dontUplink = !mp_decoded.decoded.has_bitfield || !(mp_decoded.decoded.bitfield & BITFIELD_OK_TO_MQTT_MASK);
        if (!isFromUs(&mp_decoded) && !isMqttServerAddressPrivate && dontUplink) {
            LOG_INFO("MQTT onSend - Not forwarding packet due to DontMqttMeBro flag");
            return;
        }

        if (isConfiguredForDefaultServer && (mp_decoded.decoded.portnum == meshtastic_PortNum_RANGE_TEST_APP ||
                                             mp_decoded.decoded.portnum == meshtastic_PortNum_DETECTION_SENSOR_APP)) {
            LOG_DEBUG("MQTT onSend - Ignoring range test or detection sensor message on public mqtt");
            return;
        }
    }
    bool isPKIEncrypted = mp_encrypted.pki_encrypted || mp_decoded.pki_encrypted;
    if (!(ch.settings.uplink_enabled || isPKIEncrypted))
        return;
    const char *channelId = isPKIEncrypted ? "PKI" : channels.getGlobalId(chIndex);
    const meshtastic_MeshPacket *p = moduleConfig.mqtt.encryption_enabled ? &mp_encrypted : &mp_decoded;

    std::string nodeId = nodeDB->getNodeId();

    const meshtastic_ServiceEnvelope env = {.packet = const_cast<meshtastic_MeshPacket *>(p),
                                            .channel_id = const_cast<char *>(channelId),
                                            .gateway_id = const_cast<char *>(nodeId.c_str())};
    size_t numBytes = pb_encode_to_bytes(bytes, sizeof(bytes), &meshtastic_ServiceEnvelope_msg, &env);
    std::string topic = cryptTopic + channelId + "/" + nodeId;

    if (moduleConfig.mqtt.proxy_to_client_enabled || this->isConnectedDirectly()) {
        LOG_DEBUG("MQTT Publish %s, %u bytes", topic.c_str(), numBytes);
        publish(topic.c_str(), bytes, numBytes, false);

#if !defined(ARCH_NRF52) || defined(NRF52_USE_JSON)
        if (!moduleConfig.mqtt.json_enabled)
            return;

        std::string jsonString = MeshPacketSerializer::JsonSerialize(&mp_decoded);

        if (jsonString.length() == 0)
            return;

        std::string nodeIdForJson = nodeDB->getNodeId();
        std::string topicJson = jsonTopic + channelId + "/" + nodeIdForJson;
        LOG_INFO("JSON publish message to %s, %u bytes: %s", topicJson.c_str(), jsonString.length(), jsonString.c_str());
        publish(topicJson.c_str(), jsonString.c_str(), false);
#endif
    } else {
        LOG_INFO("MQTT not connected, queue packet");
        QueueEntry *entry;
        if (mqttQueue.numFree() == 0) {
            LOG_WARN("MQTT queue is full, discard oldest");
            entry = mqttQueue.dequeuePtr(0);
        } else {
            entry = new QueueEntry;
        }
        entry->topic = std::move(topic);
        entry->envBytes.assign(bytes, numBytes);
        if (mqttQueue.enqueue(entry, 0) == false) {
            LOG_CRIT("Failed to add a message to mqttQueue!");
            abort();
        }
    }
}

void MQTT::perhapsReportToMap()
{
    if (!moduleConfig.mqtt.map_reporting_enabled || !moduleConfig.mqtt.map_report_settings.should_report_location ||
        !(moduleConfig.mqtt.proxy_to_client_enabled || isConnectedDirectly()))
        return;

    if (map_position_precision < 12 || map_position_precision > 15) {
        LOG_WARN("MQTT Map report position precision %u is out of range, using default %u", map_position_precision,
                 default_map_position_precision);
        map_position_precision = default_map_position_precision;
    }

    if (Throttle::isWithinTimespanMs(last_report_to_map, map_publish_interval_msecs) && last_report_to_map != 0)
        return;

    if (localPosition.latitude_i == 0 && localPosition.longitude_i == 0) {
        if (Throttle::isWithinTimespanMs(lastPositionUnavailableWarning, POSITION_UNAVAILABLE_WARNING_INTERVAL_MS) == false) {
            LOG_WARN("MQTT Map report enabled, but no position available");
            lastPositionUnavailableWarning = millis();
        }
        return;
    }

    meshtastic_MeshPacket *mp = packetPool.allocZeroed();
    mp->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    mp->from = nodeDB->getNodeNum();
    mp->to = NODENUM_BROADCAST;
    mp->decoded.portnum = meshtastic_PortNum_MAP_REPORT_APP;

    meshtastic_MapReport mapReport = meshtastic_MapReport_init_default;
    memcpy(mapReport.long_name, owner.long_name, sizeof(owner.long_name));
    memcpy(mapReport.short_name, owner.short_name, sizeof(owner.short_name));
    mapReport.role = config.device.role;
    mapReport.hw_model = owner.hw_model;
    strncpy(mapReport.firmware_version, optstr(APP_VERSION), sizeof(mapReport.firmware_version));
    mapReport.region = config.lora.region;
    mapReport.modem_preset = config.lora.modem_preset;
    mapReport.has_default_channel = channels.hasDefaultChannel();
    mapReport.has_opted_report_location = true;

    mapReport.latitude_i = localPosition.latitude_i & (UINT32_MAX << (32 - map_position_precision));
    mapReport.longitude_i = localPosition.longitude_i & (UINT32_MAX << (32 - map_position_precision));
    mapReport.latitude_i += (1 << (31 - map_position_precision));
    mapReport.longitude_i += (1 << (31 - map_position_precision));

    mapReport.altitude = localPosition.altitude;
    mapReport.position_precision = map_position_precision;

    mapReport.num_online_local_nodes = nodeDB->getNumOnlineMeshNodes(true);

    mp->decoded.payload.size =
        pb_encode_to_bytes(mp->decoded.payload.bytes, sizeof(mp->decoded.payload.bytes), &meshtastic_MapReport_msg, &mapReport);

    std::string nodeId = nodeDB->getNodeId();

    const meshtastic_ServiceEnvelope se = {
        .packet = mp,
        .channel_id = (char *)channels.getGlobalId(channels.getPrimaryIndex()),
        .gateway_id = const_cast<char *>(nodeId.c_str())};
    size_t numBytes = pb_encode_to_bytes(bytes, sizeof(bytes), &meshtastic_ServiceEnvelope_msg, &se);

    LOG_INFO("MQTT Publish map report to %s", mapTopic.c_str());
    publish(mapTopic.c_str(), bytes, numBytes, false);

    packetPool.release(mp);

    last_report_to_map = millis();
}