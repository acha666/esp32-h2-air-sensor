#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "main.h"
#include "sensor.h"
#include "power.h"
#include "zigbee.h"

QueueHandle_t sensorDataQueue;
TaskHandle_t tempSensorTaskHandle = NULL;
SemaphoreHandle_t xI2CSemaphore = NULL;

static const char *TAG = "Main";

void MainTask(void *pvParameters)
{
    static sensorData_t data;
    while (1)
    {
        xQueueReset(sensorDataQueue);
        xTaskNotifyGive(tempSensorTaskHandle);

        if (xQueueReceive(sensorDataQueue, &data, pdMS_TO_TICKS(5000)) == pdPASS)
        {
            ESP_LOGI(TAG, "Hum: %.2f Tmp: %.2f Pre: %.2f", data.humidity, data.temperature, data.pressure);

            app_zb_report_temperature(data.temperature);
            app_zb_report_humidity(data.humidity);
            app_zb_report_pressure(data.pressure);
        }
        else
        {
            ESP_LOGE(TAG, "No data from sensor");
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void I2CScanTask(int i2c_gpio_sda, int i2c_gpio_scl)
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
    while(1);
    // I2CScanTask(26,27);
    // // I2CScanTask(2,3);

    // sensorDataQueue = xQueueCreate(10, sizeof(sensorData_t));
    // xI2CSemaphore = xSemaphoreCreateMutex();
    // assert(sensorDataQueue != NULL);

    // esp_zb_platform_config_t config = {
    //     .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    //     .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    // };
    // ESP_ERROR_CHECK(nvs_flash_init());
    // /* load Zigbee light_bulb platform config to initialization */
    // ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    // /* hardware related and device init */

    // xTaskCreate(ZigbeeTask, "Zigbee_Task", 4096, NULL, 5, NULL);
    // xTaskCreate(SensorInitTask, "Temp_Sensor_Init_Task", 4096, NULL, 5, NULL);
    // xTaskCreate(PowerTask, "Power_Task", 4096, NULL, 5, NULL);
}