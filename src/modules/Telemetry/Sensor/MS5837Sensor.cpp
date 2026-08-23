#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "MS5837Sensor.h"

#include <Wire.h>

MS5837Sensor::MS5837Sensor()
    : TelemetrySensor(
          meshtastic_TelemetrySensorType_SENSOR_UNSET,
          "MS5837")
{
}

bool MS5837Sensor::initDevice(
    TwoWire *bus,
    ScanI2C::FoundDevice *dev)
{
    LOG_INFO("Init sensor: %s", sensorName);

    i2cBus = bus;
    address = dev->address.address;

    // Reset du capteur
    i2cBus->beginTransmission(address);
    i2cBus->write(0x1E);

    if (i2cBus->endTransmission() != 0) {
        LOG_WARN("MS5837 reset failed");
        return false;
    }

    delay(10);

    if (!readProm()) {
        LOG_WARN("MS5837 PROM read failed");
        return false;
    }

    LOG_INFO("MS5837 PROM:");
    for (int i = 0; i < 8; i++) {
        LOG_INFO("  C[%d] = 0x%04X (%u)",
                 i,
                 C[i],
                 C[i]);
    }
    
    uint8_t crcRead = C[0] >> 12;

    uint16_t promCopy[8];

    for (int i = 0; i < 8; i++) {
        promCopy[i] = C[i];
    }

    promCopy[0] &= 0x0FFF;

    uint8_t crcCalculated = crc4(promCopy);

    if (crcCalculated != crcRead) {
        LOG_WARN(
            "MS5837 CRC failed: read=%u calculated=%u",
            crcRead,
            crcCalculated);

        return false;
    }

    status = 1;

    initI2CSensor();

    LOG_INFO(
        "MS5837 detected at 0x%02X",
        address);

    return true;
}

bool MS5837Sensor::readProm()
{
    for (uint8_t i = 0; i < 8; i++) {

        i2cBus->beginTransmission(address);

        // PROM read command:
        // 0xA0, 0xA2, 0xA4 ... 0xAE
        i2cBus->write(0xA0 + (i * 2));

        if (i2cBus->endTransmission() != 0) {
            return false;
        }

        if (i2cBus->requestFrom(address, (uint8_t)2) != 2) {
            return false;
        }

        C[i] =
            ((uint16_t)i2cBus->read() << 8) |
            i2cBus->read();
    }

    return true;
}

uint8_t MS5837Sensor::crc4(uint16_t prom[])
{
    uint16_t n_rem = 0;

    uint16_t promRead[8];

    for (uint8_t i = 0; i < 8; i++) {
        promRead[i] = prom[i];
    }

    promRead[0] &= 0x0FFF;

    for (uint8_t cnt = 0; cnt < 16; cnt++) {

        if (cnt % 2 == 1) {
            n_rem ^= (uint16_t)(promRead[cnt >> 1] & 0x00FF);
        } else {
            n_rem ^= (uint16_t)(promRead[cnt >> 1] >> 8);
        }

        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {

            if (n_rem & 0x8000) {
                n_rem =
                    (n_rem << 1) ^
                    0x3000;
            } else {
                n_rem =
                    n_rem << 1;
            }
        }
    }

    n_rem =
        (n_rem >> 12) &
        0x000F;

    return n_rem;
}

bool MS5837Sensor::readRaw(
    uint32_t &D1,
    uint32_t &D2)
{
    uint8_t buffer[3];

    // ============================================================
    // Conversion pression
    // OSR = 8192
    // ============================================================

    i2cBus->beginTransmission(address);
    i2cBus->write(0x4A);

    if (i2cBus->endTransmission() != 0) {
        return false;
    }

    delay(20);

    i2cBus->beginTransmission(address);
    i2cBus->write(0x00);

    if (i2cBus->endTransmission() != 0) {
        return false;
    }

    if (i2cBus->requestFrom(address, (uint8_t)3) != 3) {
        return false;
    }

    buffer[0] = i2cBus->read();
    buffer[1] = i2cBus->read();
    buffer[2] = i2cBus->read();

    D1 =
        ((uint32_t)buffer[0] << 16) |
        ((uint32_t)buffer[1] << 8) |
        buffer[2];

    // ============================================================
    // Conversion température
    // OSR = 8192
    // ============================================================

    i2cBus->beginTransmission(address);
    i2cBus->write(0x5A);

    if (i2cBus->endTransmission() != 0) {
        return false;
    }

    delay(20);

    i2cBus->beginTransmission(address);
    i2cBus->write(0x00);

    if (i2cBus->endTransmission() != 0) {
        return false;
    }

    if (i2cBus->requestFrom(address, (uint8_t)3) != 3) {
        return false;
    }

    buffer[0] = i2cBus->read();
    buffer[1] = i2cBus->read();
    buffer[2] = i2cBus->read();

    D2 =
        ((uint32_t)buffer[0] << 16) |
        ((uint32_t)buffer[1] << 8) |
        buffer[2];

    return true;
}

int32_t MS5837Sensor::runOnce()
{
    LOG_INFO("MS5837 runOnce()");
    uint32_t D1;
    uint32_t D2;

    if (!readRaw(D1, D2)) {

        LOG_WARN("MS5837 read failed");

        return
            DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }

    // ============================================================
    // Calcul spécifique MS5837-02BA
    // ============================================================

    int32_t dT =
        (int32_t)D2 -
        ((int32_t)C[5] << 8);

    int32_t TEMP =
        2000 +
        (((int64_t)dT * C[6]) >> 23);
    
    LOG_INFO(
        "MS5837 RAW: D1=%lu D2=%lu dT=%ld TEMP=%ld",
        (unsigned long)D1,
        (unsigned long)D2,
        (long)dT,
        (long)TEMP);

    int64_t OFF =
        ((int64_t)C[2] << 17) +
        (((int64_t)C[4] * dT) >> 6);

    int64_t SENS =
        ((int64_t)C[1] << 16) +
        (((int64_t)C[3] * dT) >> 7);

    int64_t Ti = 0;
    int64_t OFFi = 0;
    int64_t SENSi = 0;

    // ============================================================
    // Compensation température
    // ============================================================

    if (TEMP < 2000) {

        Ti =
            (11LL * dT * dT) >>
            35;

        OFFi =
            (31LL *
             (TEMP - 2000) *
             (TEMP - 2000)) >>
            3;

        SENSi =
            (63LL *
             (TEMP - 2000) *
             (TEMP - 2000)) >>
            5;
    }

    TEMP -= Ti;

    OFF -= OFFi;
    SENS -= SENSi;

    int32_t P =
        (((D1 * SENS) >> 21) -
         OFF) >>
        15;

    // ============================================================
    // MS5837-02BA
    //
    // P est en unités de 0.01 mbar
    // ============================================================

    temperatureC =
        TEMP / 100.0f;

    pressureMbar =
        P / 100.0f;

    // ============================================================
    // Calcul du niveau d'eau
    //
    // 1 mbar ≈ 10.19716 mm d'eau
    // ============================================================

    waterLevelMm =
        (pressureMbar -
         EMPTY_PRESSURE_MBAR) *
        10.19716f;

    if (waterLevelMm < 0.0f) {
        waterLevelMm = 0.0f;
    }

    LOG_DEBUG(
        "MS5837: "
        "T=%.2f C "
        "P=%.2f mbar "
        "Level=%.1f mm",
        temperatureC,
        pressureMbar,
        waterLevelMm);

    return
        DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
}

bool MS5837Sensor::getMetrics(
    meshtastic_Telemetry *measurement)
{
    measurement->variant
        .environment_metrics
        .has_temperature = true;

    measurement->variant
        .environment_metrics
        .temperature =
        temperatureC;

    measurement->variant
        .environment_metrics
        .has_barometric_pressure = true;

    measurement->variant
        .environment_metrics
        .barometric_pressure =
        pressureMbar;

    measurement->variant
        .environment_metrics
        .distance =
        waterLevelMm;

    return true;
}

#endif