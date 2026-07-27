#if !defined(ARCH_NRF52) || defined(NRF52_USE_JSON)
#include "MeshPacketSerializer.h"
#include "JSON.h"
#include "NodeDB.h"
#include "mesh/generated/meshtastic/mqtt.pb.h"
#include "mesh/generated/meshtastic/telemetry.pb.h"
#include "modules/RoutingModule.h"
#include <DebugConfiguration.h>
#include <mesh-pb-constants.h>
#if defined(ARCH_ESP32)
#include "../mesh/generated/meshtastic/paxcount.pb.h"
#endif
#include "mesh/generated/meshtastic/remote_hardware.pb.h"
#include <sys/types.h>
#include <pb.h>
#include <pb_decode.h>
#include "modules/victron.pb.h" // Ingestion de tes structures Nanopb Victron

static const char *errStr = "Error decoding proto for %s message!";

std::string MeshPacketSerializer::JsonSerialize(const meshtastic_MeshPacket *mp, bool shouldLog)
{
    std::string msgType;
    JSONObject jsonObj;

    if (mp->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        switch (mp->decoded.portnum) {
        case meshtastic_PortNum_REMOTE_HARDWARE_APP: {
            JSONObject msgPayload;
            msgType = "remote_hardware";

            meshtastic_HardwareMessage scratch;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(
                mp->decoded.payload.bytes,
                mp->decoded.payload.size,
                meshtastic_HardwareMessage_fields,
                &scratch)) {

                // 2. Extraire les masques et valeurs GPIO
                msgPayload["gpio_value"] = new JSONValue((double)scratch.gpio_value);
                msgPayload["gpio_mask"]  = new JSONValue((double)scratch.gpio_mask);

                // 3. Mapper l'action Hardware vers le type d'opération
                const char *actionType = "unset";
                switch (scratch.type) {
                case meshtastic_HardwareMessage_Type_WRITE_GPIOS:
                    actionType = "gpios_write";
                    break;
                case meshtastic_HardwareMessage_Type_READ_GPIOS:
                    actionType = "gpios_read";
                    break;
                case meshtastic_HardwareMessage_Type_READ_GPIOS_REPLY:
                    actionType = "gpios_read_reply";
                    break;
                case meshtastic_HardwareMessage_Type_WATCH_GPIOS:
                    actionType = "gpios_watch";
                    break;
                case meshtastic_HardwareMessage_Type_GPIOS_CHANGED:
                    actionType = "gpios_changed";
                    break;
                default:
                    actionType = "unset";
                    break;
                }
                msgPayload["type"] = new JSONValue(actionType);

                // 4. Attacher le payload au JSON global
                jsonObj["payload"] = new JSONValue(msgPayload);

            } else if (shouldLog) {
                LOG_ERROR(errStr, "RemoteHardware");
            }
            break;
        }

        case meshtastic_PortNum_SERIAL_APP: {
            JSONObject msgPayload;
            msgType = "serial";

            // 1. Instancier la structure VictronData de Nanopb
            VictronData victronData = VictronData_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(
                mp->decoded.payload.bytes, 
                mp->decoded.payload.size
            );

            // 2. Décodage du Protobuf binaire
            if (pb_decode(&stream, VictronData_fields, &victronData)) {
                // DÉCODAGE RÉUSSI : Mappe ici les champs de ta structure VictronData
                for (pb_size_t i = 0; i < victronData.fields_count; ++i) {
                    const VictronField &field = victronData.fields[i];
                    if (field.key[0] != '\0' && field.value[0] != '\0') {
                        msgPayload[field.key] = new JSONValue(field.value);
                    }
                }
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else {
                // FALLBACK : Si le décodage Protobuf échoue, c'est du texte brut Série standard
                char payloadStr[(mp->decoded.payload.size) + 1];
                memcpy(payloadStr, mp->decoded.payload.bytes, mp->decoded.payload.size);
                payloadStr[mp->decoded.payload.size] = 0;

                msgPayload["text"] = new JSONValue(payloadStr);
                jsonObj["payload"] = new JSONValue(msgPayload);
            }
            break;
        }

        case meshtastic_PortNum_TEXT_MESSAGE_APP: {
            JSONObject msgPayload;
            msgType = "text";
            if (shouldLog)
                LOG_DEBUG("got text message of size %u", mp->decoded.payload.size);

            char payloadStr[(mp->decoded.payload.size) + 1];
            memcpy(payloadStr, mp->decoded.payload.bytes, mp->decoded.payload.size);
            payloadStr[mp->decoded.payload.size] = 0;

            JSONValue *json_value = JSON::Parse(payloadStr);
            if (json_value != NULL) {
                if (shouldLog)
                    LOG_INFO("text message payload is of type json");
                jsonObj["payload"] = json_value;
            } else {
                if (shouldLog)
                    LOG_INFO("text message payload is of type plaintext");
                msgPayload["text"] = new JSONValue(payloadStr);
                jsonObj["payload"] = new JSONValue(msgPayload);
            }
            break;
        }
        case meshtastic_PortNum_TELEMETRY_APP: {
            JSONObject msgPayload; // <--- AJOUTÉ : Déclaration manquante
            msgType = "telemetry";
            meshtastic_Telemetry scratch;
            meshtastic_Telemetry *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, meshtastic_Telemetry_fields, &scratch)) {
                decoded = &scratch;
                if (decoded->which_variant == meshtastic_Telemetry_device_metrics_tag) {
                    if (decoded->variant.device_metrics.has_battery_level) {
                        msgPayload["battery_level"] = new JSONValue((int)decoded->variant.device_metrics.battery_level);
                    }
                    msgPayload["voltage"] = new JSONValue(decoded->variant.device_metrics.voltage);
                    msgPayload["channel_utilization"] = new JSONValue(decoded->variant.device_metrics.channel_utilization);
                    msgPayload["air_util_tx"] = new JSONValue(decoded->variant.device_metrics.air_util_tx);
                    msgPayload["uptime_seconds"] = new JSONValue((unsigned int)decoded->variant.device_metrics.uptime_seconds);
                } else if (decoded->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
                    if (decoded->variant.environment_metrics.has_temperature) {
                        msgPayload["temperature"] = new JSONValue(decoded->variant.environment_metrics.temperature);
                    }
                    if (decoded->variant.environment_metrics.has_relative_humidity) {
                        msgPayload["relative_humidity"] = new JSONValue(decoded->variant.environment_metrics.relative_humidity);
                    }
                    if (decoded->variant.environment_metrics.has_barometric_pressure) {
                        msgPayload["barometric_pressure"] = new JSONValue(decoded->variant.environment_metrics.barometric_pressure);
                    }
                    if (decoded->variant.environment_metrics.has_gas_resistance) {
                        msgPayload["gas_resistance"] = new JSONValue(decoded->variant.environment_metrics.gas_resistance);
                    }
                    if (decoded->variant.environment_metrics.has_voltage) {
                        msgPayload["voltage"] = new JSONValue(decoded->variant.environment_metrics.voltage);
                    }
                    if (decoded->variant.environment_metrics.has_current) {
                        msgPayload["current"] = new JSONValue(decoded->variant.environment_metrics.current);
                    }
                    if (decoded->variant.environment_metrics.has_lux) {
                        msgPayload["lux"] = new JSONValue(decoded->variant.environment_metrics.lux);
                    }
                    if (decoded->variant.environment_metrics.has_white_lux) {
                        msgPayload["white_lux"] = new JSONValue(decoded->variant.environment_metrics.white_lux);
                    }
                    if (decoded->variant.environment_metrics.has_iaq) {
                        msgPayload["iaq"] = new JSONValue((unsigned int)decoded->variant.environment_metrics.iaq);
                    }
                    if (decoded->variant.environment_metrics.has_distance) {
                        msgPayload["distance"] = new JSONValue(decoded->variant.environment_metrics.distance);
                    }
                    if (decoded->variant.environment_metrics.has_wind_speed) {
                        msgPayload["wind_speed"] = new JSONValue(decoded->variant.environment_metrics.wind_speed);
                    }
                    if (decoded->variant.environment_metrics.has_wind_direction) {
                        msgPayload["wind_direction"] = new JSONValue((unsigned int)decoded->variant.environment_metrics.wind_direction);
                    }
                    if (decoded->variant.environment_metrics.has_wind_gust) {
                        msgPayload["wind_gust"] = new JSONValue(decoded->variant.environment_metrics.wind_gust);
                    }
                    if (decoded->variant.environment_metrics.has_wind_lull) {
                        msgPayload["wind_lull"] = new JSONValue(decoded->variant.environment_metrics.wind_lull);
                    }
                    if (decoded->variant.environment_metrics.has_radiation) {
                        msgPayload["radiation"] = new JSONValue(decoded->variant.environment_metrics.radiation);
                    }
                    if (decoded->variant.environment_metrics.has_ir_lux) {
                        msgPayload["ir_lux"] = new JSONValue(decoded->variant.environment_metrics.ir_lux);
                    }
                    if (decoded->variant.environment_metrics.has_uv_lux) {
                        msgPayload["uv_lux"] = new JSONValue(decoded->variant.environment_metrics.uv_lux);
                    }
                    if (decoded->variant.environment_metrics.has_weight) {
                        msgPayload["weight"] = new JSONValue(decoded->variant.environment_metrics.weight);
                    }
                    if (decoded->variant.environment_metrics.has_rainfall_1h) {
                        msgPayload["rainfall_1h"] = new JSONValue(decoded->variant.environment_metrics.rainfall_1h);
                    }
                    if (decoded->variant.environment_metrics.has_rainfall_24h) {
                        msgPayload["rainfall_24h"] = new JSONValue(decoded->variant.environment_metrics.rainfall_24h);
                    }
                    if (decoded->variant.environment_metrics.has_soil_moisture) {
                        msgPayload["soil_moisture"] = new JSONValue((unsigned int)decoded->variant.environment_metrics.soil_moisture);
                    }
                    if (decoded->variant.environment_metrics.has_soil_temperature) {
                        msgPayload["soil_temperature"] = new JSONValue(decoded->variant.environment_metrics.soil_temperature);
                    }
                } else if (decoded->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
                    if (decoded->variant.air_quality_metrics.has_pm10_standard) {
                        msgPayload["pm10"] = new JSONValue((unsigned int)decoded->variant.air_quality_metrics.pm10_standard);
                    }
                    if (decoded->variant.air_quality_metrics.has_pm25_standard) {
                        msgPayload["pm25"] = new JSONValue((unsigned int)decoded->variant.air_quality_metrics.pm25_standard);
                    }
                    if (decoded->variant.air_quality_metrics.has_pm100_standard) {
                        msgPayload["pm100"] = new JSONValue((unsigned int)decoded->variant.air_quality_metrics.pm100_standard);
                    }
                    if (decoded->variant.air_quality_metrics.has_co2) {
                        msgPayload["co2"] = new JSONValue((unsigned int)decoded->variant.air_quality_metrics.co2);
                    }
                    if (decoded->variant.air_quality_metrics.has_co2_temperature) {
                        msgPayload["co2_temperature"] = new JSONValue(decoded->variant.air_quality_metrics.co2_temperature);
                    }
                    if (decoded->variant.air_quality_metrics.has_co2_humidity) {
                        msgPayload["co2_humidity"] = new JSONValue(decoded->variant.air_quality_metrics.co2_humidity);
                    }
                    if (decoded->variant.air_quality_metrics.has_form_formaldehyde) {
                        msgPayload["form_formaldehyde"] = new JSONValue(decoded->variant.air_quality_metrics.form_formaldehyde);
                    }
                    if (decoded->variant.air_quality_metrics.has_form_temperature) {
                        msgPayload["form_temperature"] = new JSONValue(decoded->variant.air_quality_metrics.form_temperature);
                    }
                    if (decoded->variant.air_quality_metrics.has_form_humidity) {
                        msgPayload["form_humidity"] = new JSONValue(decoded->variant.air_quality_metrics.form_humidity);
                    }
                } else if (decoded->which_variant == meshtastic_Telemetry_power_metrics_tag) {
                    if (decoded->variant.power_metrics.has_ch1_voltage) {
                        msgPayload["voltage_ch1"] = new JSONValue(decoded->variant.power_metrics.ch1_voltage);
                    }
                    if (decoded->variant.power_metrics.has_ch1_current) {
                        msgPayload["current_ch1"] = new JSONValue(decoded->variant.power_metrics.ch1_current);
                    }
                    if (decoded->variant.power_metrics.has_ch2_voltage) {
                        msgPayload["voltage_ch2"] = new JSONValue(decoded->variant.power_metrics.ch2_voltage);
                    }
                    if (decoded->variant.power_metrics.has_ch2_current) {
                        msgPayload["current_ch2"] = new JSONValue(decoded->variant.power_metrics.ch2_current);
                    }
                    if (decoded->variant.power_metrics.has_ch3_voltage) {
                        msgPayload["voltage_ch3"] = new JSONValue(decoded->variant.power_metrics.ch3_voltage);
                    }
                    if (decoded->variant.power_metrics.has_ch3_current) {
                        msgPayload["current_ch3"] = new JSONValue(decoded->variant.power_metrics.ch3_current);
                    }
                }
                jsonObj["payload"] = new JSONValue(msgPayload); // <--- CORRIGÉ : Instanciation JSONValue
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }
        case meshtastic_PortNum_NODEINFO_APP: {
            JSONObject msgPayload;
            msgType = "nodeinfo";
            meshtastic_User scratch;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, meshtastic_User_fields, &scratch)) {
                msgPayload["id"] = new JSONValue(scratch.id);
                msgPayload["longname"] = new JSONValue(scratch.long_name);
                msgPayload["shortname"] = new JSONValue(scratch.short_name);
                msgPayload["hardware"] = new JSONValue(scratch.hw_model);
                msgPayload["role"] = new JSONValue((int)scratch.role);
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }

        case meshtastic_PortNum_POSITION_APP: {
            JSONObject msgPayload;
            msgType = "position";
            meshtastic_Position scratch;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, meshtastic_Position_fields, &scratch)) {
                if ((int)scratch.time) {
                    msgPayload["time"] = new JSONValue((unsigned int)scratch.time);
                }
                msgPayload["latitude_i"] = new JSONValue((int)scratch.latitude_i);
                msgPayload["longitude_i"] = new JSONValue((int)scratch.longitude_i);
                if ((int)scratch.altitude) {
                    msgPayload["altitude"] = new JSONValue((int)scratch.altitude);
                }
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }

        default:
            if (mp->decoded.portnum == meshtastic_PortNum_ROUTING_APP) {
                msgType = "routing";
            } else {
                msgType = "unknown_app";
            }
            break;
        }
    } else if (shouldLog) {
        LOG_WARN("Couldn't convert encrypted payload of MeshPacket to JSON");
    }

    // Métadonnées standard du message
    jsonObj["id"] = new JSONValue((unsigned int)mp->id);
    jsonObj["timestamp"] = new JSONValue((unsigned int)mp->rx_time);
    jsonObj["to"] = new JSONValue((unsigned int)mp->to);
    jsonObj["from"] = new JSONValue((unsigned int)mp->from);
    jsonObj["channel"] = new JSONValue((unsigned int)mp->channel);
    jsonObj["type"] = new JSONValue(msgType.c_str());
    jsonObj["sender"] = new JSONValue(nodeDB->getNodeId().c_str());

    if (mp->rx_rssi != 0)
        jsonObj["rssi"] = new JSONValue((int)mp->rx_rssi);
    if (mp->rx_snr != 0)
        jsonObj["snr"] = new JSONValue((float)mp->rx_snr);

    const int8_t hopsAway = getHopsAway(*mp);
    if (hopsAway >= 0) {
        jsonObj["hops_away"] = new JSONValue((unsigned int)(hopsAway));
        jsonObj["hop_start"] = new JSONValue((unsigned int)(mp->hop_start));
    }

    // Sérialisation finale
    JSONValue *value = new JSONValue(jsonObj);
    std::string jsonStr;
    if (value) {
        jsonStr = value->Stringify();
        delete value;
    }

    if (shouldLog)
        LOG_INFO("serialized json message: %s", jsonStr.c_str());

    return jsonStr;
}

std::string MeshPacketSerializer::JsonSerializeEncrypted(const meshtastic_MeshPacket *mp)
{
    JSONObject jsonObj;

    jsonObj["id"] = new JSONValue((unsigned int)mp->id);
    jsonObj["time_ms"] = new JSONValue((double)millis());
    jsonObj["timestamp"] = new JSONValue((unsigned int)mp->rx_time);
    jsonObj["to"] = new JSONValue((unsigned int)mp->to);
    jsonObj["from"] = new JSONValue((unsigned int)mp->from);
    jsonObj["channel"] = new JSONValue((unsigned int)mp->channel);
    jsonObj["want_ack"] = new JSONValue(mp->want_ack);

    if (mp->rx_rssi != 0)
        jsonObj["rssi"] = new JSONValue((int)mp->rx_rssi);
    if (mp->rx_snr != 0)
        jsonObj["snr"] = new JSONValue((float)mp->rx_snr);

    const int8_t hopsAway = getHopsAway(*mp);
    if (hopsAway >= 0) {
        jsonObj["hops_away"] = new JSONValue((unsigned int)(hopsAway));
        jsonObj["hop_start"] = new JSONValue((unsigned int)(mp->hop_start));
    }
    jsonObj["size"] = new JSONValue((unsigned int)mp->encrypted.size);
    auto encryptedStr = bytesToHex(mp->encrypted.bytes, mp->encrypted.size);
    jsonObj["bytes"] = new JSONValue(encryptedStr.c_str());

    JSONValue *value = new JSONValue(jsonObj);
    std::string jsonStr = value->Stringify();

    delete value;
    return jsonStr;
}
#endif