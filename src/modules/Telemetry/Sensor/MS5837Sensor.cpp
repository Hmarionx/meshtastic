#include "MS5837Sensor.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

// Variables globales gérées par le BME280 (voir BME280Sensor.cpp)
extern const float ALTITUDE_CORRECTION_HPA; // constante d'altitude, SOURCE UNIQUE côté BME280
extern float rawAirPressureHpa;             // pression atmosphérique BRUTE, jamais touchée par l'altitude

// Calage à zéro déterminé une fois, cuve vide (ou à un niveau de référence connu) :
//   1. Décommente temporairement le LOG_INFO en fin de getMetrics()
//   2. Relève la valeur moyenne de "rawWaterPressureMbar - rawAirPressureHpa" sur cuve vide
//   3. Reporte cette valeur ci-dessous, puis recompile
// Recalage à partir d'un point de référence fiable : cuve physiquement VIDE (remplace
// l'ancienne valeur +1.14, issue d'une estimation visuelle à mi-niveau, moins précise).
// À cuve vide : rawWaterPressureMbar ≈ 983.81, rawAirPressureHpa ≈ 988.02
// -> OFFSET = rawWaterPressureMbar - rawAirPressureHpa ≈ -4.21
// À RE-VALIDER avec un second point (cuve remplie à un niveau connu, mesuré à la règle depuis
// le même point de référence que le capteur) pour confirmer qu'il s'agit bien d'un pur décalage
// constant et non d'un écart d'échelle — voir la discussion précédente à ce sujet.
static constexpr float WATER_LEVEL_ZERO_OFFSET_MBAR = -4.21f; // <-- à reconfirmer sur un relevé 100% sûr

// Nombre d'échecs consécutifs de lecture avant de tenter une récupération du bus. 5 cycles à
// ~100-200ms chacun laisse le temps à un échec ponctuel (glitch isolé) de se résoudre tout seul
// sans déclencher de récupération inutile, tout en réagissant en quelques secondes à un vrai
// blocage (le cas vu en log : échecs en continu pendant 7+ secondes).
static constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 5;

MS5837Sensor::MS5837Sensor()
    : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "MS5837")
{
}

bool MS5837Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
    LOG_INFO("Init sensor: %s", sensorName);
    i2cBus = bus;

    if (dev) {
        address = dev->address.address;
        // Le MS5837 est exclusivement géré sur l'adresse 0x76.
        // Évite tout conflit avec le BME280/680 positionné sur 0x77.
        if (address != 0x76) {
            LOG_INFO("MS5837: adresse 0x%02X ignorée (attendue: 0x76)", address);
            return false;
        }
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

    // Lecture de la PROM
    if (!readProm()) {
        LOG_WARN("MS5837 PROM read failed");
        return false;
    }

    // Vérification du CRC4
    uint8_t crcRead = C[0] >> 12;
    uint16_t promCopy[8];
    for (int i = 0; i < 7; i++) {
        promCopy[i] = C[i];
    }
    promCopy[7] = 0; // Nécessaire pour le calcul complet du CRC
    promCopy[0] &= 0x0FFF;

    uint8_t crcCalculated = crc4(promCopy);
    if (crcCalculated != crcRead) {
        LOG_WARN("MS5837 CRC failed: read=%u calculated=%u", crcRead, crcCalculated);
        return false;
    }

    status = 1;
    initI2CSensor();
    LOG_INFO("MS5837 detected at 0x%02X", address);
    return true;
}

bool MS5837Sensor::readProm()
{
    // Le MS5837 contient 7 mots en PROM (mots 0 à 6)
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
    // 8ème mot virtuel pour la compatibilité d'algorithme CRC
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

void MS5837Sensor::attemptBusRecovery()
{
    // Broches confirmées pour ce bus I2C sur la Heltec V4 (partagé avec BME280 + OLED SSD1306).
    constexpr uint8_t I2C_SDA_PIN = 17;
    constexpr uint8_t I2C_SCL_PIN = 18;

    LOG_WARN("MS5837: %u échecs I2C consécutifs, tentative de récupération du bus (SDA=%d, SCL=%d)...",
             MAX_CONSECUTIVE_FAILURES, I2C_SDA_PIN, I2C_SCL_PIN);

    // 1. Libérer le périphérique I2C matériel pour piloter les broches en GPIO classique.
    i2cBus->end();

    pinMode(I2C_SCL_PIN, OUTPUT);
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(10);

    // 2. Jusqu'à 9 impulsions d'horloge : un esclave bloqué en train d'envoyer un octet relâche
    //    SDA dès qu'il a reçu assez de fronts d'horloge pour terminer sa transmission en cours.
    //    On s'arrête dès que SDA repasse HIGH (bus débloqué), inutile d'aller jusqu'à 9 sinon.
    bool released = false;
    for (uint8_t i = 0; i < 9; i++) {
        if (digitalRead(I2C_SDA_PIN) == HIGH) {
            released = true;
            break;
        }
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(5);
    }

    // 3. Forcer une condition STOP manuelle : SDA passe de LOW à HIGH pendant que SCL est HIGH.
    //    Ça remet tout esclave présent sur le bus dans un état "prêt", qu'il ait été débloqué
    //    par les pulses ci-dessus ou qu'il n'ait jamais été bloqué.
    pinMode(I2C_SDA_PIN, OUTPUT);
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(5);

    // 4. Réinitialiser le périphérique I2C matériel avec les bonnes broches.
    i2cBus->begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(10);

    LOG_INFO("MS5837: récupération du bus terminée (SDA %s avant STOP)", released ? "libérée" : "forcée");
}

bool MS5837Sensor::readRaw(uint32_t &D1, uint32_t &D2)
{
    uint8_t buffer[3];

    // Pression D1 (OSR = 8192)
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

    // Température D2 (OSR = 8192)
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
    uint32_t D1, D2;
    if (!readRaw(D1, D2)) {
        LOG_WARN("MS5837 read failed");
        consecutiveReadFailures++;
        if (consecutiveReadFailures >= MAX_CONSECUTIVE_FAILURES) {
            attemptBusRecovery();
            consecutiveReadFailures = 0;
        }
        return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }
    consecutiveReadFailures = 0;

    // --- Calculs Mathématiques MS5837 (datasheet, second ordre) ---
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

    float newTemperatureC = TEMP / 100.0f;
    float newRawWaterPressureMbar = P / 100.0f;

    // Une lecture I2C corrompue (bus partagé avec le BME280, glitch électrique) peut produire des
    // valeurs "cohérentes en apparence" mais physiquement impossibles. On valide donc avant de
    // mettre à jour l'état du capteur ; en cas d'échec, on conserve la dernière valeur connue
    // plutôt que de laisser une lecture aberrante se propager jusqu'au calcul du niveau d'eau.
    //   - Température : plage du capteur (datasheet MS5837-30BA : -20°C à +85°C)
    //   - Pression : jamais sous la pression atmosphérique typique, jamais au-dessus de ce que
    //     la cuve peut physiquement produire. 3000 hPa (~20m de colonne d'eau) est volontairement
    //     large ; resserre cette borne haute à ta profondeur max réelle pour un filtrage plus strict.
    bool temperatureValid = (newTemperatureC > -20.0f && newTemperatureC < 85.0f);
    bool pressureValid = (newRawWaterPressureMbar > 500.0f && newRawWaterPressureMbar < 3000.0f);

    if (temperatureValid && pressureValid) {
        temperatureC = newTemperatureC;

        // Pression BRUTE du MS5837, jamais touchée par l'altitude : c'est cette valeur (comparée
        // à rawAirPressureHpa, elle aussi brute) qui sert au calcul du niveau d'eau dans getMetrics().
        rawWaterPressureMbar = newRawWaterPressureMbar;

        // Pression "affichage", cohérente avec le BME280 (barometric_pressure). Utilisée
        // uniquement pour la métrique env.weight — jamais mélangée au calcul du niveau d'eau.
        pressureMbar = rawWaterPressureMbar + ALTITUDE_CORRECTION_HPA;
    } else {
        LOG_WARN("MS5837 lecture aberrante ignorée (T=%.2f, P=%.2f)", newTemperatureC, newRawWaterPressureMbar);
        // temperatureC / rawWaterPressureMbar / pressureMbar conservent leur dernière valeur valide
    }

    return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS * 10;
}

bool MS5837Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    if (measurement == nullptr) {
        return false;
    }

    auto &env = measurement->variant.environment_metrics;

    // 1. Température de l'eau
    env.has_lux = true;
    env.lux = temperatureC;

    // 2. Pression MS5837 "affichage" — cohérente avec le BME280 (objectif : cohérence atmosphérique)
    env.has_weight = true;
    env.weight = pressureMbar; // = rawWaterPressureMbar + ALTITUDE_CORRECTION_HPA (calculé dans runOnce)

    // 3. Hauteur d'eau — comparaison BRUTE contre BRUTE, insensible aux variations météo naturelles.
    //    L'altitude ne doit JAMAIS entrer dans ce calcul : elle s'annulerait de toute façon dans la
    //    soustraction, autant ne jamais la mélanger à la physique et éviter tout risque de désync.
    if (rawAirPressureHpa > 500.0f) {
        float deltaPressure = rawWaterPressureMbar - rawAirPressureHpa - WATER_LEVEL_ZERO_OFFSET_MBAR;
        waterLevelMm = (deltaPressure > 0.0f) ? deltaPressure * 10.19716f : 0.0f;
    }
    // sinon : le BME280 n'a pas encore fourni de lecture valide -> on conserve la dernière valeur
    // connue de waterLevelMm plutôt que d'afficher un faux "0" (cuve vide).

    env.has_distance = true;
    env.distance = waterLevelMm;

    //LOG_INFO("MS5837: P_eau_brute=%.2f, P_air_brute=%.2f -> Distance=%.1f mm",
    //         rawWaterPressureMbar, rawAirPressureHpa, waterLevelMm);

    return true;
}

#endif
