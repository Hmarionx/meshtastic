#pragma once

#include "TelemetrySensor.h"
#include <Wire.h>

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

class MS5837Sensor : public TelemetrySensor {
public:
    MS5837Sensor();

    // Seules ces 3 méthodes dérivent de TelemetrySensor
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;

private:
    bool readProm();
    uint8_t crc4(uint16_t prom[]);
    bool readRaw(uint32_t &D1, uint32_t &D2);

    // Stockage local du bus I2C fourni par Meshtastic lors de l'init
    TwoWire *i2cBus = nullptr;
    uint8_t address = 0x76;

    uint16_t C[8];
    float temperatureC = 0.0f;
    float pressureMbar = 0.0f;
    float waterLevelMm = 0.0f;

    const float EMPTY_PRESSURE_MBAR = 1013.25f;
};

#endif