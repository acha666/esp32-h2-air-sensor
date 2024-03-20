#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_log.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_core.h"
#include "driver/gpio.h"
#include "string.h"

#include <zigbee.h>
#include <main.h>

void app_zb_report_temperature(float temperature_c)
{
    int16_t temperature_int = temperature_c * 100 + 0.5;
    writeAttribute(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature_int);
    reportAttribute(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
    return;
}

void app_zb_report_humidity(float humidity_percent)
{
    uint16_t humidity_int = humidity_percent * 100 + 0.5;
    writeAttribute(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity_int);
    reportAttribute(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);
    return;
}

void app_zb_report_pressure(float pressure_hpa)
{
    int16_t pressure_hpa_int = pressure_hpa + 0.5;
    int16_t pressure_0_1_hpa_int = pressure_hpa * 10 + 0.5;

    writeAttribute(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pressure_hpa_int); // basic
    reportAttribute(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID);

    writeAttribute(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALED_VALUE_ID, &pressure_0_1_hpa_int); // PRESSURE_MEASUREMENT_SCALED_VALUE is not reportable
    return;
}