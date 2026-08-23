#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#pragma once

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"

class MS5837Sensor : public TelemetrySensor
{
  private:
    TwoWire *i2cBus = nullptr;
    uint8_t address = 0x76;

    uint16_t C[8] = {0};

    float temperatureC = 0.0f;
    float pressureMbar = 0.0f;
    float waterLevelMm = 0.0f;

    bool readProm();
    bool readRaw(uint32_t &D1, uint32_t &D2);

    uint8_t crc4(uint16_t prom[]);

    // ============================================================
    // CALIBRATION
    //
    // À remplacer par la pression réellement mesurée lorsque
    // la cuve est complètement vide.
    // ============================================================
    static constexpr float EMPTY_PRESSURE_MBAR = 1013.25f;

  public:
    MS5837Sensor();

    virtual bool getMetrics(
        meshtastic_Telemetry *measurement) override;

    virtual bool initDevice(
        TwoWire *bus,
        ScanI2C::FoundDevice *dev) override;

    virtual int32_t runOnce() override;
};

#endif