#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<Adafruit_BME280.h>)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "BME280Sensor.h"
#include "TelemetrySensor.h"
#include <Adafruit_BME280.h>
#include <typeinfo>

// --- ESPACE GLOBAL : Seule la déclaration de type est autorisée ---

// Constante d'altitude : SOURCE UNIQUE, utilisée ici et dans MS5837Sensor.cpp pour que les deux
// capteurs affichent une pression cohérente ramenée au niveau de la mer (objectif "cohérence
// atmosphérique"). Ne JAMAIS l'utiliser dans un calcul de différence de pression (niveau d'eau) :
// elle sert uniquement à l'affichage.
// NOTE C++ : le mot-clé "extern" est obligatoire ici (et pas seulement dans MS5837Sensor.cpp) car
// une variable "const" au niveau fichier a une liaison interne par défaut en C++. Sans ce "extern"
// sur la définition, MS5837Sensor.cpp ne pourrait pas la voir (erreur de lien à la compilation).
extern const float ALTITUDE_CORRECTION_HPA = 38.8f;

// Pression atmosphérique BRUTE (sans aucune correction d'altitude), mise à jour à chaque lecture
// du BME280 et consommée par le MS5837 pour calculer la hauteur d'eau indépendamment de la météo.
float rawAirPressureHpa = 0.0f;

BME280Sensor::BME280Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_BME280, "BME280") {}

bool BME280Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
    LOG_INFO("Init sensor: %s", sensorName);
    status = bme280.begin(dev->address.address, bus);
    if (!status) {
        return status;
    }
    bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_1000);

    initI2CSensor();
    return status;
}

bool BME280Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    if (measurement == nullptr)
        return false;

    bme280.takeForcedMeasurement();

    // 1. Pression BRUTE réelle de l'air ambiant (jamais touchée par l'altitude)
    float rawPressure = bme280.readPressure() / 100.0F;
    if (rawPressure > 500.0F) {
        rawAirPressureHpa = rawPressure; // Sauvegarde de la pression BRUTE pour le MS5837
    }

    // 2. Métriques réseau
    measurement->variant.environment_metrics.has_temperature = true;
    measurement->variant.environment_metrics.temperature = bme280.readTemperature();
    measurement->variant.environment_metrics.has_relative_humidity = true;
    measurement->variant.environment_metrics.relative_humidity = bme280.readHumidity();

    // Pression ramenée au niveau de la mer pour l'affichage / la météo (cohérence entre capteurs)
    measurement->variant.environment_metrics.has_barometric_pressure = true;
    measurement->variant.environment_metrics.barometric_pressure = rawPressure + ALTITUDE_CORRECTION_HPA;

    return true;
}
#endif
