#pragma once

#include "MeshModule.h"
#include "Router.h"
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "configuration.h"

#include <Arduino.h>
#include <functional>

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52) || defined(ARCH_RP2040) || defined(ARCH_STM32WL)) && \
    !defined(CONFIG_IDF_TARGET_ESP32S2) && \
    !defined(CONFIG_IDF_TARGET_ESP32C3)

class SerialModule : public StreamAPI, private concurrency::OSThread
{
    bool firstTime = true;

    unsigned long lastNmeaTime = 0;

    char outbuf[90] = "";

public:

    SerialModule();

    static bool isValidConfig(
        const meshtastic_ModuleConfig_SerialConfig &config);

protected:

    virtual int32_t runOnce() override;

    virtual bool checkIsConnected() override;

private:

    /*
     * --------------------------------------------------------------
     * Generic serial
     * --------------------------------------------------------------
     */

    uint32_t getBaudRate();

    void sendTelemetry(
        meshtastic_Telemetry m);

    void processWXSerial();

    /*
     * --------------------------------------------------------------
     * VE.Direct parser
     * --------------------------------------------------------------
     */

    /*
     * Read all available bytes from Serial2
     * and feed them to the VE.Direct parser.
     */
    void processSerialGeneric();

    /*
     * Process a single raw byte.
     */
    void processSerialGenericByte(
        uint8_t byte);

    /*
     * Process a complete VE.Direct line.
     *
     * Supported formats:
     *
     *     KEY<TAB>VALUE
     *     KEY#VALUE
     *
     * Special case:
     *
     *     Checksum<TAB>
     *
     * The next byte received is the binary
     * VE.Direct checksum byte.
     */
    void processSerialGenericLine();

    /*
     * Commit a checksum-valid VE.Direct frame.
     */
    void processSerialFrame();

    /*
     * Add or update a field in the current frame.
     */
    void updateSerialField(
        const char *key,
        const char *value);

    /*
     * Reset the current frame.
     */
    void resetSerialFrame();

    /*
     * Reset the parser completely.
     */
    void resetSerialParser();

    /*
     * Send the latest valid snapshot.
     *
     * The snapshot is sent using:
     *
     *     PortNum = SERIAL_APP
     *     Channel  = Serial channel
     */
    bool sendSerialSnapshot();
};


/*
 * Global SerialModule instance.
 */
extern SerialModule *serialModule;


/*
 * ----------------------------------------------------------------------
 * SerialModuleRadio
 * ----------------------------------------------------------------------
 *
 * Handles the radio side of the Serial module.
 *
 * VE.Direct:
 *
 *     Victron
 *        |
 *        v
 *     Serial2
 *        |
 *        v
 *     SerialModule
 *        |
 *        v
 *     SerialModuleRadio
 *        |
 *        +--> SERIAL_APP
 *        |
 *        +--> Serial channel
 *        |
 *        v
 *     Meshtastic LoRa
 *
 */

class SerialModuleRadio : public MeshModule
{
    uint32_t lastRxID = 0;

    char outbuf[90] = "";

public:

    SerialModuleRadio();

    /*
     * Send the payload currently stored in serialBytes.
     *
     * dest:
     *     Destination node.
     *
     * wantReplies:
     *     Whether a response is requested.
     *
     * Returns true if the packet was successfully
     * allocated and submitted to MeshService.
     */
    bool sendPayload(
        NodeNum dest = NODENUM_BROADCAST,
        bool wantReplies = false);

protected:

    virtual meshtastic_MeshPacket *allocReply() override;

    virtual ProcessMessage handleReceived(
        const meshtastic_MeshPacket &mp) override;

    /*
     * Port used by this module.
     */
    meshtastic_PortNum ourPortNum;

    /*
     * Receive only packets matching our port.
     */
    virtual bool wantPacket(
        const meshtastic_MeshPacket *p) override
    {
        if (p == nullptr) {
            return false;
        }

        return p->decoded.portnum == ourPortNum;
    }

private:

    /*
     * Allocate a packet for the Serial module.
     *
     * The packet is configured with:
     *
     *     SERIAL_APP
     */
    meshtastic_MeshPacket *allocDataPacket()
    {
        meshtastic_MeshPacket *p =
            router->allocForSending();

        if (p == nullptr) {
            return nullptr;
        }

        p->decoded.portnum =
            ourPortNum;

        return p;
    }
};


/*
 * Global SerialModuleRadio instance.
 */
extern SerialModuleRadio *serialModuleRadio;

#endif
