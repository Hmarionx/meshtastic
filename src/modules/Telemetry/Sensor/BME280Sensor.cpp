#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<Adafruit_BME280.h>)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "BME280Sensor.h"
#include "TelemetrySensor.h"
#include <Adafruit_BME280.h>
#include <typeinfo>

// --- ESPACE GLOBAL : Seule la déclaration de type est autorisée ---
float correctedAirPressureHpa = 0.0f; 
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

    // 1. Pression BRUTE réelle de l'air ambiant
    float rawPressure = bme280.readPressure() / 100.0F;

    if (rawPressure > 500.0F) {
        rawAirPressureHpa = rawPressure; // Sauvegarde de la pression BRUTE pour le MS5837
    }

    // 2. Métriques réseau
    measurement->variant.environment_metrics.has_temperature = true;
    measurement->variant.environment_metrics.temperature = bme280.readTemperature();

    measurement->variant.environment_metrics.has_relative_humidity = true;
    measurement->variant.environment_metrics.relative_humidity = bme280.readHumidity();

    // Envoi de la valeur corrigée d'altitude (+38.8 hPa) pour la météo
    measurement->variant.environment_metrics.has_barometric_pressure = true;
    measurement->variant.environment_metrics.barometric_pressure = rawPressure + 38.8F;

    return true;
}

#endif