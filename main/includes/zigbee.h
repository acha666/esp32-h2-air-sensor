#ifndef __ZIGBEE_H
#define __ZIGBEE_H

#ifdef __cplusplus
extern "C"
{
#endif

    void ZigbeeTask(void *pvParameters);
    esp_err_t writeAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value);
    esp_err_t reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID);

    void app_zb_report_temperature(float temperature_c);
    void app_zb_report_humidity(float humidity_percent);
    void app_zb_report_pressure(float pressure_hpa);

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE false /* enable the install code policy for security */
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 3000                                               /* 3000 millisecond */
#define HA_SENSOR_ENDPOINT 10                                         /* esp light bulb device endpoint, used to process light controlling commands */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

#define ESP_ZB_ZED_CONFIG()                               \
    {                                                     \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,             \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zed_cfg = {                              \
            .ed_timeout = ED_AGING_TIMEOUT,               \
            .keep_alive = ED_KEEP_ALIVE,                  \
        },                                                \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()    \
    {                                    \
        .radio_mode = RADIO_MODE_NATIVE, \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                       \
    {                                                      \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE, \
    }

#ifdef __cplusplus
}
#endif

#endif