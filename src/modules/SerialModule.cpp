#include "SerialModule.h"

#include "GeoCoord.h"
#include "MeshService.h"
#include "NMEAWPL.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "configuration.h"
#include "modules/victron.pb.h"

#include <pb_encode.h>
#include <Arduino.h>
#include <Throttle.h>

#ifdef HELTEC_MESH_SOLAR
#include "meshSolarApp.h"
#endif

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52) || defined(ARCH_RP2040) || defined(ARCH_STM32WL)) && \
    !defined(CONFIG_IDF_TARGET_ESP32S2) && \
    !defined(CONFIG_IDF_TARGET_ESP32C3)

/* --- Configuration Générale Serial --- */
#define RX_BUFFER 256
#define TIMEOUT 250
#define BAUD 38400
#define ACK 1

#define SERIAL_CONNECTION_TIMEOUT (15 * 60 * 1000UL)
#define SERIAL_GENERIC_DEFAULT_INTERVAL_MS 60000UL

/* --- Configuration Parser VE.Direct --- */
#define SERIAL_GENERIC_MAX_FIELDS 32
#define SERIAL_GENERIC_KEY_SIZE 32
#define SERIAL_GENERIC_VALUE_SIZE 64
#define SERIAL_GENERIC_LINE_SIZE (SERIAL_GENERIC_KEY_SIZE + SERIAL_GENERIC_VALUE_SIZE + 8)
#define SERIAL_GENERIC_MAX_FRAME_SIZE 2048

// Timeout augmenté à 1500 ms pour éviter les faux timeouts sous charge CPU
#define SERIAL_GENERIC_FRAME_TIMEOUT_MS 1500

/* --- Buffers & Variables Globales --- */
char serialBytes[512];
size_t serialPayloadSize = 0;

enum GenericSerialParserState {
    SERIAL_PARSER_WAITING_FOR_FRAME = 0,
    SERIAL_PARSER_RECEIVING_LINE,
    SERIAL_PARSER_RECEIVING_CHECKSUM
};

static GenericSerialParserState serialParserState = SERIAL_PARSER_WAITING_FOR_FRAME;

SerialModule *serialModule = nullptr;
SerialModuleRadio *serialModuleRadio = nullptr;

#ifndef SERIAL_PRINT_PORT
#define SERIAL_PRINT_PORT 2
#endif

#if SERIAL_PRINT_PORT == 0
#define SERIAL_PRINT_OBJECT Serial
#elif SERIAL_PRINT_PORT == 1
#define SERIAL_PRINT_OBJECT Serial1
#elif SERIAL_PRINT_PORT == 2
#define SERIAL_PRINT_OBJECT Serial2
#else
#error "Unsupported SERIAL_PRINT_PORT value. Allowed values are 0, 1, or 2."
#endif

SerialModule::SerialModule() : StreamAPI(&SERIAL_PRINT_OBJECT), concurrency::OSThread("Serial") {
    api_type = TYPE_SERIAL;
}

static Print *serialPrint = &SERIAL_PRINT_OBJECT;

struct GenericSerialField {
    char key[SERIAL_GENERIC_KEY_SIZE];
    char value[SERIAL_GENERIC_VALUE_SIZE];
};

static GenericSerialField serialFields[SERIAL_GENERIC_MAX_FIELDS];
static size_t serialFieldCount = 0;

static GenericSerialField serialSnapshot[SERIAL_GENERIC_MAX_FIELDS];
static size_t serialSnapshotFieldCount = 0;
static bool serialFrameComplete = false;

static unsigned long lastSerialSnapshot = 0;
static char serialLineBuffer[SERIAL_GENERIC_LINE_SIZE];
static size_t serialLineLength = 0;

static uint8_t serialChecksum = 0;
static unsigned long serialLastByteMillis = 0;
static uint32_t serialFrameBytes = 0;

// Statistiques
static uint32_t serialTotalBytes = 0;
static uint32_t serialTotalLines = 0;
static uint32_t serialTotalFrames = 0;
static uint32_t serialInvalidFrames = 0;
static uint32_t serialInvalidLines = 0;
static uint32_t serialResyncCount = 0;
static uint32_t serialOverflowCount = 0;

/* --- Validation Configuration --- */
bool SerialModule::isValidConfig(const meshtastic_ModuleConfig_SerialConfig &config) {
    if (config.override_console_serial_port &&
        !IS_ONE_OF(config.mode, 
                   meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA,
                   meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO,
                   meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MS_CONFIG)) {

        const char *warning = "Invalid Serial config: override console serial port is only supported in NMEA and CalTopo output-only modes.";
        LOG_ERROR(warning);

#if !IS_RUNNING_TESTS
        meshtastic_ClientNotification *cn = clientNotificationPool.allocZeroed();
        if (cn != nullptr) {
            cn->level = meshtastic_LogRecord_Level_ERROR;
            cn->time = getValidTime(RTCQualityFromNet);
            snprintf(cn->message, sizeof(cn->message), "%s", warning);
            service->sendClientNotification(cn);
        }
#endif
        return false;
    }
    return true;
}

/* --- Radio Module --- */
SerialModuleRadio::SerialModuleRadio() : MeshModule("SerialModuleRadio") {
    switch (moduleConfig.serial.mode) {
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_TEXTMSG:
        ourPortNum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        break;
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA:
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO:
        ourPortNum = meshtastic_PortNum_POSITION_APP;
        break;
    default:
        ourPortNum = meshtastic_PortNum_SERIAL_APP;
        boundChannel = Channels::serialChannel;
        break;
    }
}

bool SerialModule::checkIsConnected() {
    return Throttle::isWithinTimespanMs(lastContactMsec, SERIAL_CONNECTION_TIMEOUT);
}

/* --- Boucle Principale --- */
int32_t SerialModule::runOnce() {
    if (!moduleConfig.serial.enabled) {
        return disable();
    }

    if (moduleConfig.serial.override_console_serial_port || (moduleConfig.serial.rxd && moduleConfig.serial.txd)) {
        if (firstTime) {
            LOG_INFO("Init serial peripheral interface");
            uint32_t baud = getBaudRate();

            if (moduleConfig.serial.override_console_serial_port) {
#ifdef RP2040_SLOW_CLOCK
                Serial2.flush();
                serialPrint = &Serial2;
#else
                Serial.flush();
                serialPrint = &Serial;
#endif
                delay(10);
            }

#if defined(CONFIG_IDF_TARGET_ESP32C6)
            if (moduleConfig.serial.rxd && moduleConfig.serial.txd) {
                Serial1.setRxBufferSize(RX_BUFFER);
                Serial1.begin(baud, SERIAL_8N1, moduleConfig.serial.rxd, moduleConfig.serial.txd);
            } else {
                Serial.begin(baud);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            }
#elif defined(ARCH_STM32WL)
#ifndef RAK3172
            HardwareSerial *serialInstance = &Serial2;
#else
            HardwareSerial *serialInstance = &Serial1;
#endif
            if (moduleConfig.serial.rxd && moduleConfig.serial.txd) {
                serialInstance->setTx(moduleConfig.serial.txd);
                serialInstance->setRx(moduleConfig.serial.rxd);
            }
            serialInstance->begin(baud);
            serialInstance->setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);

#elif defined(ARCH_ESP32)
            if (moduleConfig.serial.rxd && moduleConfig.serial.txd) {
                Serial2.setRxBufferSize(RX_BUFFER);
                Serial2.begin(baud, SERIAL_8N1, moduleConfig.serial.rxd, moduleConfig.serial.txd);
            } else {
                Serial.begin(baud);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            }
#elif SERIAL_PRINT_PORT != 0
            if (moduleConfig.serial.rxd && moduleConfig.serial.txd) {
#ifdef ARCH_RP2040
                Serial2.setFIFOSize(RX_BUFFER);
                Serial2.setPinout(moduleConfig.serial.txd, moduleConfig.serial.rxd);
#else
                Serial2.setPins(moduleConfig.serial.rxd, moduleConfig.serial.txd);
#endif
                Serial2.begin(baud, SERIAL_8N1);
                Serial2.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            } else {
#ifdef RP2040_SLOW_CLOCK
                Serial2.begin(baud, SERIAL_8N1);
                Serial2.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#else
                Serial.begin(baud, SERIAL_8N1);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#endif
            }
#else
            Serial.begin(baud, SERIAL_8N1);
            Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#endif

            if (serialModuleRadio == nullptr) {
                serialModuleRadio = new SerialModuleRadio();
            }

            resetSerialParser();
            serialFrameComplete = false;
            serialSnapshotFieldCount = 0;
            serialTotalBytes = serialTotalLines = serialTotalFrames = 0;
            serialInvalidFrames = serialInvalidLines = serialResyncCount = serialOverflowCount = 0;

            lastSerialSnapshot = millis();
            firstTime = false;

            if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO) {
                emitRebooted();
            }
        } else {
            if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO) {
                return runOncePart();
            }

            if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA) && HAS_GPS) {
                if (!Throttle::isWithinTimespanMs(lastNmeaTime, 2000)) {
                    lastNmeaTime = millis();
                    printGGA(outbuf, sizeof(outbuf), localPosition);
                    serialPrint->printf("%s", outbuf);
                }
            } else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO) && HAS_GPS) {
                if (!Throttle::isWithinTimespanMs(lastNmeaTime, 10000)) {
                    lastNmeaTime = millis();
                    uint32_t readIndex = 0;
                    const meshtastic_NodeInfoLite *tempNodeInfo = nodeDB->readNextMeshNode(readIndex);
                    while (tempNodeInfo != nullptr) {
                        if (tempNodeInfo->has_user && nodeDB->hasValidPosition(tempNodeInfo)) {
                            printWPL(outbuf, sizeof(outbuf), tempNodeInfo->position, tempNodeInfo->user.long_name, true);
                            serialPrint->printf("%s", outbuf);
                        }
                        tempNodeInfo = nodeDB->readNextMeshNode(readIndex);
                    }
                }
            }
#if SERIAL_PRINT_PORT != 0
            else if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_WS85) {
                processWXSerial();
            }
#if defined(HELTEC_MESH_SOLAR)
            else if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MS_CONFIG) {
                serialPayloadSize = Serial.readBytes(serialBytes, sizeof(serialBytes) - 1);
                if (serialPayloadSize > 0 && meshSolarCmdHandle(serialBytes) != 0) {
                    return runOncePart(serialBytes, serialPayloadSize);
                }
            }
#endif
            else if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_VE_DIRECT) {
                processSerialGeneric();
            } else {
#if defined(CONFIG_IDF_TARGET_ESP32C6)
                while (Serial1.available()) {
                    serialPayloadSize = Serial1.readBytes(serialBytes, meshtastic_Constants_DATA_PAYLOAD_LEN);
                    if (serialPayloadSize > 0) serialModuleRadio->sendPayload();
                }
#else
#ifndef RAK3172
                HardwareSerial *serialInstance = &Serial2;
#else
                HardwareSerial *serialInstance = &Serial1;
#endif
                while (serialInstance->available()) {
                    serialPayloadSize = serialInstance->readBytes(serialBytes, meshtastic_Constants_DATA_PAYLOAD_LEN);
                    if (serialPayloadSize > 0) serialModuleRadio->sendPayload();
                }
#endif
            }
#endif
        }
        return 10;
    } else {
        return disable();
    }
}

/* --- Télémétrie Meshtastic --- */
void SerialModule::sendTelemetry(meshtastic_Telemetry m) {
    meshtastic_MeshPacket *p = router->allocForSending();
    if (p == nullptr) {
        LOG_WARN("Serial telemetry allocation failed");
        return;
    }

    p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    p->decoded.payload.size = pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Telemetry_msg, &m);
    p->to = NODENUM_BROADCAST;
    p->decoded.want_response = false;

    if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR) {
        p->want_ack = true;
        p->priority = meshtastic_MeshPacket_Priority_HIGH;
    } else {
        p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    }

    service->sendToMesh(p, RX_SRC_LOCAL, true);
}

/* --- Gestion du Parser VE.Direct --- */
void SerialModule::resetSerialFrame() {
    serialFieldCount = 0;
    serialLineLength = 0;
    serialLineBuffer[0] = '\0';
    serialChecksum = 0;
    serialFrameBytes = 0;
    serialLastByteMillis = millis();
    memset(serialFields, 0, sizeof(serialFields));
}

void SerialModule::resetSerialParser() {
    serialParserState = SERIAL_PARSER_WAITING_FOR_FRAME;
    resetSerialFrame();
}

void SerialModule::updateSerialField(const char *key, const char *value) {
    if (key == nullptr || value == nullptr || key[0] == '\0') return;

    for (size_t i = 0; i < serialFieldCount; i++) {
        if (strcmp(serialFields[i].key, key) == 0) {
            strlcpy(serialFields[i].value, value, sizeof(serialFields[i].value));
            LOG_DEBUG("VE.DIRECT FIELD UPDATE: %s=%s", key, value);
            return;
        }
    }

    if (serialFieldCount < SERIAL_GENERIC_MAX_FIELDS) {
        strlcpy(serialFields[serialFieldCount].key, key, sizeof(serialFields[serialFieldCount].key));
        strlcpy(serialFields[serialFieldCount].value, value, sizeof(serialFields[serialFieldCount].value));
        serialFieldCount++;
        LOG_DEBUG("VE.DIRECT FIELD: %s=%s", key, value);
    } else {
        LOG_WARN("VE.Direct field limit reached, ignoring key=%s", key);
    }
}

void SerialModule::processSerialFrame() {
    if (serialFieldCount == 0) {
        LOG_WARN("VE.Direct checksum-valid frame is empty");
        return;
    }

    memcpy(serialSnapshot, serialFields, sizeof(serialFields));
    serialSnapshotFieldCount = serialFieldCount;
    serialFrameComplete = true;
    serialTotalFrames++;

    LOG_INFO("VE.Direct VALID FRAME: fields=%u totalValid=%lu", (unsigned int)serialSnapshotFieldCount, (unsigned long)serialTotalFrames);

    uint32_t intervalSec = moduleConfig.serial.timeout;
    if (intervalSec == 0) intervalSec = 300;
    uint32_t intervalMs = intervalSec * 1000UL;

    if (!Throttle::isWithinTimespanMs(lastSerialSnapshot, intervalMs)) {
        if (serialModuleRadio != nullptr) {
            bool sent = sendSerialSnapshot();
            if (sent) {
                lastSerialSnapshot = millis();
                LOG_INFO("VE.DIRECT SNAPSHOT SENT: fields=%u", (unsigned int)serialSnapshotFieldCount);
            } else {
                LOG_WARN("VE.DIRECT SNAPSHOT SEND FAILED");
            }
        }
    } else {
        LOG_DEBUG("VE.DIRECT snapshot updated in RAM (Interval %u s not reached)", (unsigned int)intervalSec);
    }
}

void SerialModule::processSerialGenericLine() {
    if (serialLineLength == 0) return;

    serialLineBuffer[serialLineLength] = '\0';
    serialTotalLines++;

    LOG_DEBUG("VE.DIRECT LINE: %s", serialLineBuffer);

    char *separator = strchr(serialLineBuffer, '\t');
    if (separator == nullptr) separator = strchr(serialLineBuffer, '#');

    if (separator == nullptr) {
        serialInvalidLines++;
        LOG_WARN("VE.DIRECT INVALID LINE: %s", serialLineBuffer);
        serialLineLength = 0;
        serialLineBuffer[0] = '\0';
        return;
    }

    *separator = '\0';
    const char *key = serialLineBuffer;
    const char *value = separator + 1;

    if (key[0] == '\0') {
        serialInvalidLines++;
        serialLineLength = 0;
        serialLineBuffer[0] = '\0';
        return;
    }

    if (strcmp(key, "Checksum") == 0) {
        serialLineLength = 0;
        serialLineBuffer[0] = '\0';
        return;
    }

    updateSerialField(key, value);

    serialLineLength = 0;
    serialLineBuffer[0] = '\0';
}

void SerialModule::processSerialGenericByte(uint8_t byte) {
    serialTotalBytes++;
    serialLastByteMillis = millis();

    // État : Réception du Checksum binaire
    if (serialParserState == SERIAL_PARSER_RECEIVING_CHECKSUM) {
        serialChecksum = static_cast<uint8_t>(serialChecksum + byte);
        serialFrameBytes++;

        if (serialChecksum == 0) {
            LOG_INFO("VE.DIRECT CHECKSUM VALID: bytes=%lu", (unsigned long)serialFrameBytes);
            processSerialFrame();
        } else {
            serialInvalidFrames++;
            LOG_WARN("VE.DIRECT CHECKSUM INVALID: checksum=0x%02X invalidFrames=%lu", (unsigned int)serialChecksum, (unsigned long)serialInvalidFrames);
        }

        resetSerialParser();
        return;
    }

    // État : Début d'une nouvelle trame
    if (serialParserState == SERIAL_PARSER_WAITING_FOR_FRAME) {
        resetSerialFrame();
        serialParserState = SERIAL_PARSER_RECEIVING_LINE;
    }

    serialChecksum = static_cast<uint8_t>(serialChecksum + byte);
    serialFrameBytes++;

    if (serialFrameBytes > SERIAL_GENERIC_MAX_FRAME_SIZE) {
        serialOverflowCount++;
        LOG_WARN("VE.Direct frame overflow");
        resetSerialParser();
        serialResyncCount++;
        return;
    }

    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\t' &&
        serialParserState == SERIAL_PARSER_RECEIVING_LINE) {
        serialLineBuffer[serialLineLength] = '\0';
        if (strcmp(serialLineBuffer, "Checksum") == 0) {
            serialParserState =
                SERIAL_PARSER_RECEIVING_CHECKSUM;
            serialLineLength = 0;
            serialLineBuffer[0] = '\0';
            return;
        }
    }
    
    if (byte == '\r') return;

    if (byte == '\n') {
        if (serialLineLength > 0) {
            processSerialGenericLine();
        }
        serialLineLength = 0;
        serialLineBuffer[0] = '\0';
        return;
    }

    if (serialLineLength < sizeof(serialLineBuffer) - 1) {
        serialLineBuffer[serialLineLength++] = static_cast<char>(byte);
    } else {
        serialOverflowCount++;
        LOG_WARN("VE.Direct line overflow, resynchronizing");
        resetSerialParser();
        serialResyncCount++;
    }
}

void SerialModule::processSerialGeneric() {
#if SERIAL_PRINT_PORT != 0 && !defined(ARCH_STM32WL) && !defined(CONFIG_IDF_TARGET_ESP32C6)

    // Vérification du timeout si la trame est coupée en cours de route
    if (serialParserState != SERIAL_PARSER_WAITING_FOR_FRAME) {
        if (millis() - serialLastByteMillis > SERIAL_GENERIC_FRAME_TIMEOUT_MS) {
            LOG_WARN("VE.Direct frame timeout, resynchronizing");
            serialResyncCount++;
            resetSerialParser();
        }
    }

    while (Serial2.available()) {
        int value = Serial2.read();
        if (value < 0) break;
        processSerialGenericByte(static_cast<uint8_t>(value));
    }

    static uint32_t lastStatsLog = 0;
    if (!Throttle::isWithinTimespanMs(lastStatsLog, 10000)) {
        lastStatsLog = millis();
        LOG_DEBUG("VE.DIRECT RX: bytes=%lu lines=%lu valid=%lu invalidFrames=%lu invalidLines=%lu resync=%lu overflow=%lu state=%d fields=%u frameBytes=%lu checksum=0x%02X snapshotFields=%u",
                  (unsigned long)serialTotalBytes, (unsigned long)serialTotalLines, (unsigned long)serialTotalFrames,
                  (unsigned long)serialInvalidFrames, (unsigned long)serialInvalidLines, (unsigned long)serialResyncCount,
                  (unsigned long)serialOverflowCount, (int)serialParserState, (unsigned int)serialFieldCount,
                  (unsigned long)serialFrameBytes, serialChecksum, (unsigned int)serialSnapshotFieldCount);
    }
#endif
}

/* --- Émission Protobuf du Snapshot --- */
bool SerialModule::sendSerialSnapshot() {
    if (!serialFrameComplete || serialSnapshotFieldCount == 0) {
        LOG_WARN("VE.DIRECT PROTOBUF TX skipped: no valid snapshot");
        return false;
    }

    if (serialModuleRadio == nullptr) {
        LOG_WARN("VE.DIRECT PROTOBUF TX skipped: SerialModuleRadio unavailable");
        return false;
    }

    VictronData message = VictronData_init_zero;
    const size_t maxFields = sizeof(message.fields) / sizeof(message.fields[0]);
    size_t fieldCount = serialSnapshotFieldCount;

    if (fieldCount > maxFields) {
        LOG_WARN("VE.DIRECT PROTOBUF TX: field count truncated %u -> %u", (unsigned int)fieldCount, (unsigned int)maxFields);
        fieldCount = maxFields;
    }

    for (size_t i = 0; i < fieldCount; i++) {
        strlcpy(message.fields[i].key, serialSnapshot[i].key, sizeof(message.fields[i].key));
        strlcpy(message.fields[i].value, serialSnapshot[i].value, sizeof(message.fields[i].value));
    }
    message.fields_count = fieldCount;

    pb_ostream_t stream = pb_ostream_from_buffer(reinterpret_cast<pb_byte_t *>(serialBytes), sizeof(serialBytes));

    if (!pb_encode(&stream, VictronData_fields, &message)) {
        LOG_WARN("VE.DIRECT PROTOBUF TX encode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }

    serialPayloadSize = stream.bytes_written;

    if (serialPayloadSize == 0) {
        LOG_WARN("VE.DIRECT PROTOBUF TX: empty payload");
        return false;
    }

    if (serialPayloadSize > sizeof(serialBytes) || serialPayloadSize > meshtastic_Constants_DATA_PAYLOAD_LEN) {
        LOG_WARN("VE.DIRECT PROTOBUF TX too large: %u bytes", (unsigned int)serialPayloadSize);
        return false;
    }

    LOG_INFO("VE.DIRECT PROTOBUF TX: bytes=%u fields=%u", (unsigned int)serialPayloadSize, (unsigned int)fieldCount);

    if (serialModuleRadio->sendPayload()) {
        LOG_INFO("VE.DIRECT PROTOBUF TX SUBMITTED: bytes=%u fields=%u", (unsigned int)serialPayloadSize, (unsigned int)fieldCount);
        return true;
    }

    LOG_WARN("VE.DIRECT PROTOBUF TX SUBMIT FAILED");
    return false;
}

/* --- Routage Radio Packet --- */
meshtastic_MeshPacket *SerialModuleRadio::allocReply() {
    return allocDataPacket();
}

bool SerialModuleRadio::sendPayload(NodeNum dest, bool wantReplies) {
    if (serialPayloadSize == 0 || serialPayloadSize > meshtastic_Constants_DATA_PAYLOAD_LEN) {
        LOG_WARN("SerialModuleRadio TX skipped: invalid payload size %u", (unsigned int)serialPayloadSize);
        return false;
    }

    const meshtastic_Channel *ch = (boundChannel != nullptr) ? &channels.getByName(boundChannel) : nullptr;
    meshtastic_MeshPacket *p = allocReply();

    if (p == nullptr) {
        LOG_WARN("SerialModuleRadio TX failed: packet allocation failed");
        return false;
    }

    p->to = dest;
    if (ch != nullptr) {
        p->channel = ch->index;
        LOG_DEBUG("SerialModuleRadio TX: channel=%u port=SERIAL_APP", (unsigned int)ch->index);
    } else {
        LOG_WARN("SerialModuleRadio TX: no bound Serial channel");
    }

    p->decoded.want_response = wantReplies;
    p->want_ack = ACK;
    p->decoded.payload.size = serialPayloadSize;
    memcpy(p->decoded.payload.bytes, serialBytes, p->decoded.payload.size);

    LOG_INFO("SerialModuleRadio TX SEND: bytes=%u dest=0x%08lX channel=%u", (unsigned int)p->decoded.payload.size, (unsigned long)p->to, (unsigned int)p->channel);

    service->sendToMesh(p);
    return true;
}

ProcessMessage SerialModuleRadio::handleReceived(const meshtastic_MeshPacket &mp) {
    if (mp.decoded.portnum != ourPortNum) {
        return ProcessMessage::CONTINUE;
    }
    if (!moduleConfig.serial.enabled || moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO) {
        return ProcessMessage::CONTINUE;
    }

    auto &p = mp.decoded;

    if (isFromUs(&mp)) {
        if (moduleConfig.serial.echo && lastRxID != mp.id) {
            lastRxID = mp.id;
            serialPrint->write(p.payload.bytes, p.payload.size);
        }
        return ProcessMessage::CONTINUE;
    }

    if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_DEFAULT ||
        moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_SIMPLE) {
        serialPrint->write(p.payload.bytes, p.payload.size);
    } else if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_TEXTMSG) {
        meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(getFrom(&mp));
        const char *sender = (node && node->has_user) ? node->user.short_name : "???";
        serialPrint->println();
        serialPrint->printf("%s: %s", sender, p.payload.bytes);
        serialPrint->println();
    } else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA ||
                moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO) && HAS_GPS) {
        if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag && mp.decoded.portnum == ourPortNum) {
            meshtastic_Position scratch = meshtastic_Position_init_zero;
            if (pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Position_msg, &scratch)) {
                meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(getFrom(&mp));
                if (node && node->has_user) {
                    printWPL(outbuf, sizeof(outbuf), scratch, node->user.long_name, moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO);
                    serialPrint->printf("%s", outbuf);
                }
            }
        }
    }

    return ProcessMessage::CONTINUE;
}

uint32_t SerialModule::getBaudRate() {
    switch (moduleConfig.serial.baud) {
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_110: return 110;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_300: return 300;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_600: return 600;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_1200: return 1200;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_2400: return 2400;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_4800: return 4800;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_9600: return 9600;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_19200: return 19200;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_38400: return 38400;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_57600: return 57600;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_115200: return 115200;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_230400: return 230400;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_460800: return 460800;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_576000: return 576000;
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_921600: return 921600;
        default: return BAUD;
    }
}

/* --- Station Météo --- */
struct ParsedLine {
    char name[64];
    char value[128];
};

ParsedLine parseLine(const char *line) {
    ParsedLine result = {"", ""};
    const char *equals = strchr(line, '=');
    if (!equals) return result;

    char nameBuf[64];
    size_t nameLen = equals - line;
    if (nameLen >= sizeof(nameBuf)) nameLen = sizeof(nameBuf) - 1;
    strncpy(nameBuf, line, nameLen);
    nameBuf[nameLen] = '\0';

    char *nameStart = nameBuf;
    while (*nameStart && isspace(*nameStart)) nameStart++;
    char *nameEnd = nameStart + strlen(nameStart) - 1;
    while (nameEnd > nameStart && isspace(*nameEnd)) *nameEnd-- = '\0';

    strncpy(result.name, nameStart, sizeof(result.name) - 1);
    result.name[sizeof(result.name) - 1] = '\0';

    const char *valueStart = equals + 1;
    while (*valueStart && isspace(*valueStart)) valueStart++;
    strncpy(result.value, valueStart, sizeof(result.value) - 1);
    result.value[sizeof(result.value) - 1] = '\0';

    char *valueEnd = result.value + strlen(result.value) - 1;
    while (valueEnd > result.value && isspace(*valueEnd)) *valueEnd-- = '\0';

    return result;
}

void SerialModule::processWXSerial() {
#if SERIAL_PRINT_PORT != 0 && !defined(ARCH_STM32WL) && !defined(CONFIG_IDF_TARGET_ESP32C6)
    static unsigned int lastAveraged = 0;
    static unsigned int averageIntervalMillis = 300000;
    static double dir_sum_sin = 0, dir_sum_cos = 0;
    static float velSum = 0, gust = 0, lull = -1;
    static int velCount = 0, dirCount = 0;
    static char windDir[4] = "xxx", windVel[5] = "xx.x", windGust[5] = "xx.x";
    static char batVoltage[5] = "0.0V", capVoltage[5] = "0.0V", temperature[5] = "00.0";
    static float batVoltageF = 0, capVoltageF = 0, temperatureF = 0;
    static char rainStr[] = "5780860000";
    static int rainSum = 0;
    static float rain = 0;
    bool gotwind = false;

    while (Serial2.available()) {
        memset(serialBytes, '\0', sizeof(serialBytes));
        serialPayloadSize = Serial2.readBytes(serialBytes, sizeof(serialBytes) - 1);

        if (serialPayloadSize > 0) {
            int lineStart = 0, lineEnd = -1;
            for (size_t i = 0; i < serialPayloadSize; i++) {
                if (serialBytes[i] == '\n') {
                    lineEnd = i;
                    char line[meshtastic_Constants_DATA_PAYLOAD_LEN] = {0};

                    if ((size_t)(lineEnd - lineStart) < sizeof(line) - 1) {
                        memcpy(line, &serialBytes[lineStart], lineEnd - lineStart);
                        ParsedLine parsed = parseLine(line);

                        if (strlen(parsed.name) > 0) {
                            if (strcmp(parsed.name, "WindDir") == 0) {
                                strlcpy(windDir, parsed.value, sizeof(windDir));
                                double radians = GeoCoord::toRadians(strtof(windDir, nullptr));
                                dir_sum_sin += sin(radians);
                                dir_sum_cos += cos(radians);
                                dirCount++;
                                gotwind = true;
                            } else if (strcmp(parsed.name, "WindSpeed") == 0) {
                                strlcpy(windVel, parsed.value, sizeof(windVel));
                                float newv = strtof(windVel, nullptr);
                                velSum += newv;
                                velCount++;
                                if (newv < lull || lull == -1) lull = newv;
                                gotwind = true;
                            } else if (strcmp(parsed.name, "WindGust") == 0) {
                                strlcpy(windGust, parsed.value, sizeof(windGust));
                                float newg = strtof(windGust, nullptr);
                                if (newg > gust) gust = newg;
                                gotwind = true;
                            } else if (strcmp(parsed.name, "BatVoltage") == 0) {
                                strlcpy(batVoltage, parsed.value, sizeof(batVoltage));
                                batVoltageF = strtof(batVoltage, nullptr);
                            } else if (strcmp(parsed.name, "CapVoltage") == 0) {
                                strlcpy(capVoltage, parsed.value, sizeof(capVoltage));
                                capVoltageF = strtof(capVoltage, nullptr);
                            } else if (strcmp(parsed.name, "GXTS04Temp") == 0 || strcmp(parsed.name, "Temperature") == 0) {
                                strlcpy(temperature, parsed.value, sizeof(temperature));
                                temperatureF = strtof(temperature, nullptr);
                            } else if (strcmp(parsed.name, "RainIntSum") == 0) {
                                strlcpy(rainStr, parsed.value, sizeof(rainStr));
                                rainSum = int(strtof(rainStr, nullptr));
                            } else if (strcmp(parsed.name, "Rain") == 0) {
                                strlcpy(rainStr, parsed.value, sizeof(rainStr));
                                rain = strtof(rainStr, nullptr);
                            }
                        }
                        lineStart = lineEnd + 1;
                    }
                }
            }
            break;
        }
    }

    if (gotwind) {
        LOG_INFO("WS8X : %i %.1fg%.1f %.1fv %.1fv %.1fC rain: %.1f, %i sum", atoi(windDir), strtof(windVel, nullptr), strtof(windGust, nullptr), batVoltageF, capVoltageF, temperatureF, rain, rainSum);
    }

    if (gotwind && !Throttle::isWithinTimespanMs(lastAveraged, averageIntervalMillis)) {
        float velAvg = 1.0 * velSum / velCount;
        double avgSin = dir_sum_sin / dirCount;
        double avgCos = dir_sum_cos / dirCount;
        double avgRadians = atan2(avgSin, avgCos);
        float dirAvg = GeoCoord::toDegrees(avgRadians);
        if (dirAvg < 0) dirAvg += 360.0;

        lastAveraged = millis();

        meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
        m.which_variant = meshtastic_Telemetry_environment_metrics_tag;
        m.variant.environment_metrics.wind_speed = velAvg;
        m.variant.environment_metrics.has_wind_speed = true;
        m.variant.environment_metrics.wind_direction = dirAvg;
        m.variant.environment_metrics.has_wind_direction = true;
        m.variant.environment_metrics.temperature = temperatureF;
        m.variant.environment_metrics.has_temperature = true;
        m.variant.environment_metrics.voltage = capVoltageF > batVoltageF ? capVoltageF : batVoltageF;
        m.variant.environment_metrics.has_voltage = true;
        m.variant.environment_metrics.wind_gust = gust;
        m.variant.environment_metrics.has_wind_gust = true;
        m.variant.environment_metrics.rainfall_24h = rainSum;
        m.variant.environment_metrics.has_rainfall_24h = true;
        m.variant.environment_metrics.rainfall_1h = rain;
        m.variant.environment_metrics.has_rainfall_1h = true;
        if (lull == -1) lull = 0;
        m.variant.environment_metrics.wind_lull = lull;
        m.variant.environment_metrics.has_wind_lull = true;

        LOG_INFO("WS8X Transmit speed=%fm/s, direction=%d, lull=%f, gust=%f, voltage=%f temperature=%f",
                 m.variant.environment_metrics.wind_speed, m.variant.environment_metrics.wind_direction,
                 m.variant.environment_metrics.wind_lull, m.variant.environment_metrics.wind_gust,
                 m.variant.environment_metrics.voltage, m.variant.environment_metrics.temperature);

        sendTelemetry(m);

        velSum = 0; velCount = 0; dirCount = 0;
        dir_sum_sin = 0; dir_sum_cos = 0;
        gust = 0; lull = -1;
    }
#endif
}

#endif