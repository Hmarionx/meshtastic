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
    auto &env = measurement->variant.environment_metrics;

    // Une lecture I2C corrompue (bus partagé avec le MS5837, glitch électrique) ressort de la
    // formule de compensation Bosch sous forme de valeurs "cohérentes en apparence" mais
    // physiquement impossibles (pression négative, température à 180°C...). On valide donc
    // chaque mesure contre sa plage physique plausible avant de la publier ; en cas d'échec, le
    // champ est simplement absent du message (has_X = false) plutôt que de publier du garbage.

    // Température : plage du capteur (datasheet Bosch BME280 : -40°C à +85°C)
    float temp = bme280.readTemperature();
    bool temperatureValid = (temp > -40.0f && temp < 85.0f);
    if (temperatureValid) {
        env.has_temperature = true;
        env.temperature = temp;
    }

    // Humidité : physiquement 0-100%. Une valeur collée pile à 100.0 combinée à une température
    // hors plage est le signe d'un calcul corrompu, pas d'une vraie mesure de saturation.
    float hum = bme280.readHumidity();
    if (hum >= 0.0f && hum <= 100.0f && temperatureValid) {
        env.has_relative_humidity = true;
        env.relative_humidity = hum;
    }

    // Pression : plage physique plausible (300-1100 hPa couvre du sommet de l'Everest à un
    // cyclone). Seule une lecture valide met à jour rawAirPressureHpa, consommée par le MS5837.
    float rawPressure = bme280.readPressure() / 100.0F;
    if (rawPressure > 300.0f && rawPressure < 1100.0f) {
        rawAirPressureHpa = rawPressure; // Sauvegarde de la pression BRUTE pour le MS5837
        env.has_barometric_pressure = true;
        env.barometric_pressure = rawPressure + ALTITUDE_CORRECTION_HPA;
    }

    return true;
}
#endif
