#include "MS5837Sensor.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

MS5837Sensor::MS5837Sensor()
    : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "MS5837")
{
}

bool MS5837Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
    LOG_INFO("Init sensor: %s", sensorName);

    // Sauvegarde du bus I2C (Wire ou Wire1) transmis par le scanner Meshtastic
    i2cBus = bus;
    if (dev) {
        address = dev->address.address;
    }

    if (!i2cBus) {
        LOG_WARN("MS5837: bus I2C invalide");
        return false;
    }

    LOG_INFO("MS5837 init sur bus I2C, adresse=0x%02X", address);

    // Reset du capteur MS5837
    i2cBus->beginTransmission(address);
    i2cBus->write(0x1E);

    if (i2cBus->endTransmission(true) != 0) {
        LOG_WARN("MS5837 reset failed");
        return false;
    }

    delay(40);

    // Lecture PROM
    if (!readProm()) {
        LOG_WARN("MS5837 PROM read failed");
        return false;
    }

LOG_INFO("MS5837 PROM:");
    for (int i = 0; i < 7; i++) { // Changé : i < 7
        LOG_INFO("  C[%d] = 0x%04X (%u)", i, C[i], C[i]);
    }

    // Vérification CRC4
    uint8_t crcRead = C[0] >> 12;
    uint16_t promCopy[8];
    for (int i = 0; i < 7; i++) { // Changé : i < 7
        promCopy[i] = C[i];
    }
    promCopy[7] = 0; // Nécessaire pour l'algorithme CRC
    promCopy[0] &= 0x0FFF;

    uint8_t crcCalculated = crc4(promCopy);

    if (crcCalculated != crcRead) {
        LOG_WARN("MS5837 CRC failed: read=%u calculated=%u", crcRead, crcCalculated);
        return false;
    }

    LOG_INFO("MS5837 CRC OK: %u", crcCalculated);

    status = 1;
    initI2CSensor();

    LOG_INFO("MS5837 detected at 0x%02X", address);
    return true;
}

bool MS5837Sensor::readProm()
{
    // Le MS5837 ne contient QUE 7 mots en PROM (0 à 6)
    for (uint8_t i = 0; i < 7; i++) { 
        i2cBus->beginTransmission(address);
        i2cBus->write(0xA0 + (i * 2));

        if (i2cBus->endTransmission(false) != 0) {
            LOG_WARN("MS5837: Failed to send PROM read cmd for C[%d]", i);
            return false;
        }

        delay(3);

        if (i2cBus->requestFrom((int)address, 2, (int)true) != 2) {
            LOG_WARN("MS5837: Failed to request 2 bytes for C[%d]", i);
            return false;
        }

        C[i] = ((uint16_t)i2cBus->read() << 8) | i2cBus->read();
    }
    
    // Le 8ème mot virtuel doit être à 0 pour le calcul du CRC
    C[7] = 0; 
    
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
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = n_rem << 1;
            }
        }
    }

    n_rem = (n_rem >> 12) & 0x000F;
    return n_rem;
}

bool MS5837Sensor::readRaw(uint32_t &D1, uint32_t &D2)
{
    uint8_t buffer[3];

    // Pression D1
    i2cBus->beginTransmission(address);
    i2cBus->write(0x4A);
    if (i2cBus->endTransmission(true) != 0) {
        return false;
    }

    delay(20);

    i2cBus->beginTransmission(address);
    i2cBus->write(0x00);
    if (i2cBus->endTransmission(true) != 0) {
        return false;
    }

    if (i2cBus->requestFrom((int)address, 3, (int)true) != 3) {
        return false;
    }

    buffer[0] = i2cBus->read();
    buffer[1] = i2cBus->read();
    buffer[2] = i2cBus->read();

    D1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

    // Température D2
    i2cBus->beginTransmission(address);
    i2cBus->write(0x5A);
    if (i2cBus->endTransmission(true) != 0) {
        return false;
    }

    delay(20);

    i2cBus->beginTransmission(address);
    i2cBus->write(0x00);
    if (i2cBus->endTransmission(true) != 0) {
        return false;
    }

    if (i2cBus->requestFrom((int)address, 3, (int)true) != 3) {
        return false;
    }

    buffer[0] = i2cBus->read();
    buffer[1] = i2cBus->read();
    buffer[2] = i2cBus->read();

    D2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

    return true;
}

int32_t MS5837Sensor::runOnce()
{
    LOG_INFO("MS5837 runOnce()");

    uint32_t D1;
    uint32_t D2;

    if (!readRaw(D1, D2)) {
        LOG_WARN("MS5837 read failed");
        return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }

    // --- Calculs Mathématiques MS5837 ---
    int32_t dT = (int32_t)D2 - ((int32_t)C[5] << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * C[6]) >> 23);

    int64_t OFF = ((int64_t)C[2] << 17) + (((int64_t)C[4] * dT) >> 6);
    int64_t SENS = ((int64_t)C[1] << 16) + (((int64_t)C[3] * dT) >> 7);

    int64_t Ti = 0, OFFi = 0, SENSi = 0;
    if (TEMP < 2000) {
        Ti = (11LL * dT * dT) >> 35;
        OFFi = (31LL * (TEMP - 2000) * (TEMP - 2000)) >> 3;
        SENSi = (63LL * (TEMP - 2000) * (TEMP - 2000)) >> 5;
    }

    TEMP -= Ti;
    OFF -= OFFi;
    SENS -= SENSi;

    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;

    temperatureC = TEMP / 100.0f;
    pressureMbar = P / 100.0f;

    // --- LOGIQUE DE TARE AU BOOT ---
    if (!isTared && pressureMbar > 300.0f) { // Sécurité : on s'assure que la mesure est valide
        emptyPressureMbar = pressureMbar;
        isTared = true;
        LOG_INFO("MS5837 TARE effectuée au boot: P_ref = %.2f mbar", emptyPressureMbar);
    }

    // Calcul de la hauteur relative d'eau
    float rawLevel = (pressureMbar - emptyPressureMbar) * 10.19716f;
    
    waterLevelMm = rawLevel;
    if (waterLevelMm < 0.0f) {
        waterLevelMm = 0.0f; // Bloque si bruit de fond ou légère baisse barométrique
    }

    LOG_INFO("MS5837 measurement: T=%.2f C P=%.2f mbar Level=%.1f mm (raw=%.1f)",
             temperatureC, pressureMbar, waterLevelMm, rawLevel);

    return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
}

bool MS5837Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    if (measurement == nullptr) {
        return false;
    }

    auto &env = measurement->variant.environment_metrics;

    // 1. Température de l'eau (°C) -> Détournement sur dew_point (Point de rosée)
    env.has_lux = true;
    env.lux = temperatureC;

    // 2. Pression d'eau (mbar) -> Détournement sur weight (Poids)
    env.has_weight = true;
    env.weight = pressureMbar;

    // 3. Hauteur d'eau (mm) -> Champ standard distance
    env.has_distance = true;
    env.distance = waterLevelMm;

    LOG_INFO("MS5837 metrics: temperature=%.2f pressure=%.2f distance=%.1f",
             temperatureC, pressureMbar, waterLevelMm);

    return true;
}

#endif