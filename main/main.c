#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "main.h"
#include "sensor.h"
#include "power.h"
#include "zigbee.h"
#include "display.h"

QueueHandle_t sensorDataQueue;
QueueHandle_t powerDataQueue;
QueueHandle_t displayMainDataQueue;
TaskHandle_t tempSensorTaskHandle = NULL;
TaskHandle_t powerTaskHandle = NULL;

static const char *TAG = "Main";

void main_task(void *pvParameters)
{
    sensor_data_t sensor_data;
    power_data_t power_data;
    while (1)
    {
        display_main_data_t display_main_data;
        xQueueReset(sensorDataQueue);
        xQueueReset(powerDataQueue);
        xTaskNotifyGive(tempSensorTaskHandle);
        xTaskNotifyGive(powerTaskHandle);

        if (xQueueReceive(sensorDataQueue, &sensor_data, pdMS_TO_TICKS(5000)) == pdPASS)
        {
            ESP_LOGI(TAG, "Hum: %.2f Tmp: %.2f Pre: %.2f", sensor_data.humidity, sensor_data.temperature, sensor_data.pressure);

            display_main_data.humidity = sensor_data.humidity;
            display_main_data.temperature = sensor_data.temperature;
            display_main_data.pressure = sensor_data.pressure;

            app_zb_report_temperature(sensor_data.temperature);
            app_zb_report_humidity(sensor_data.humidity);
            app_zb_report_pressure(sensor_data.pressure);
        }
        else
            ESP_LOGE(TAG, "No data from sensor");

        if (xQueueReceive(powerDataQueue, &power_data, pdMS_TO_TICKS(5000)) == pdPASS)
        {
            if (power_data.gauge_fault)
                display_main_data.battery_state = 0;

            display_main_data.battery_soc = power_data.gauge_battery_soc;
            display_main_data.battery_voltage = power_data.gauge_battery_voltage;
            if (power_data.gauge_fc)
                display_main_data.battery_state = 3;
            else if (power_data.gauge_battery_soc < 20)
                display_main_data.battery_state = 1;
            else
                display_main_data.battery_state = 2;

            if (power_data.charger_charging)
                display_main_data.battery_charging_state = 1;
            else
                display_main_data.battery_charging_state = 0;
        }
        else
            ESP_LOGE(TAG, "No data from power");

        display_main_data.timestamp = esp_timer_get_time() / 1000;
        xQueueSend(displayMainDataQueue, &display_main_data, 0);

        vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
    }
}

void i2c_scan_task(int i2c_gpio_sda, int i2c_gpio_scl)
{
    i2c_master_bus_handle_t tool_bus_handle;
    i2c_port_t i2c_port = I2C_NUM_0;

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = i2c_gpio_scl,
        .sda_io_num = i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &tool_bus_handle) != ESP_OK);

    printf("Starting scanning I2C bus, SCL=IO%d; SDA=IO%d\r\n", i2c_gpio_scl, i2c_gpio_sda);
    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16)
    {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++)
        {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(tool_bus_handle, address, 50);
            if (ret == ESP_OK)

                printf("%02x ", address);

            else if (ret == ESP_ERR_TIMEOUT)

                printf("UU ");

            else

                printf("-- ");
        }
        printf("\r\n");
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(tool_bus_handle));

    // vTaskDelete(NULL);
}

void app_main(void)
{
    i2c_scan_task(2, 3);
    i2c_scan_task(26, 27);

    sensorDataQueue = xQueueCreate(3, sizeof(sensor_data_t));
    powerDataQueue = xQueueCreate(3, sizeof(power_data_t));
    displayMainDataQueue = xQueueCreate(3, sizeof(display_main_data_t));
    assert(sensorDataQueue != NULL && powerDataQueue != NULL && displayMainDataQueue != NULL);

    xTaskCreate(sensor_init_task, "Temp_Sensor_Init_Task", 4096, NULL, 5, NULL);
    xTaskCreate(display_init_task, "Display_Task", 4096, NULL, 5, NULL);
    xTaskCreate(power_task, "Power_Task", 4096, NULL, 5, &powerTaskHandle);

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* load Zigbee light_bulb platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    /* hardware related and device init */

    xTaskCreate(ZigbeeTask, "Zigbee_Task", 4096, NULL, 5, NULL);
}