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

    // Pression MS5837 BRUTE (sans aucune correction d'altitude). Sert exclusivement au calcul du
    // niveau d'eau dans getMetrics() (comparaison brute-à-brute avec rawAirPressureHpa côté BME280),
    // afin que les variations naturelles de pression atmosphérique n'affectent jamais la hauteur mesurée.
    float rawWaterPressureMbar = 0.0f;

    // Pression MS5837 "affichage", cohérente avec le BME280 (= rawWaterPressureMbar + ALTITUDE_CORRECTION_HPA,
    // calculée dans runOnce()). Ne jamais utiliser cette valeur pour calculer le niveau d'eau : l'offset
    // d'altitude s'annulerait de toute façon dans la soustraction, autant ne jamais l'y mélanger.
    float pressureMbar = 0.0f;

    float waterLevelMm = 0.0f;
};

#endif