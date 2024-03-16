#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "main.h"
#include "sensor.h"
#include "zigbee.h"

QueueHandle_t sensorDataQueue;
TaskHandle_t tempSensorTaskHandle = NULL;
static const char *TAG = "DEMO";

void button_task(void *pvParameters)
{
    uint8_t last_state = 0;
    while (1)
    {
        uint8_t button_state = gpio_get_level(GPIO_NUM_12);
        if (button_state != last_state)
        {
            ESP_LOGI(TAG, "Button changed: %d", button_state);
            last_state = button_state;
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &button_state, 1);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void MainTask(void *pvParameters)
{
    static const char TAG[] = "MainTask";
    static sensorData_t data;
    while (1)
    {
        xQueueReset(sensorDataQueue);
        xTaskNotifyGive(tempSensorTaskHandle);

        if (xQueueReceive(sensorDataQueue, &data, pdMS_TO_TICKS(5000)) == pdPASS)
        {
            ESP_LOGI(TAG, "Hum: %.2f Tmp: %.2f Pre: %.2f", data.humidity, data.temperature, data.pressure);
            uint16_t temperature_int = data.temperature * 100;
            uint16_t humidity_int = data.humidity * 100;

            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature_int, 2);
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity_int, 2);

            int16_t pressure_hpa_int = data.pressure;
            int16_t pressure_0_1_hpa_int = data.pressure * 10;
            uint8_t pressure_scale = 1;

            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pressure_hpa_int, 2); // basic

            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALED_VALUE_ID, &pressure_0_1_hpa_int, 2);
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALE_ID, &pressure_scale, 1);
        }
        else
        {
            ESP_LOGE(TAG, "No data from temperature sensor");
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void I2CScanTask()
{
    i2c_master_bus_handle_t tool_bus_handle;
    i2c_port_t i2c_port = I2C_NUM_0;
    // int i2c_gpio_sda = 2;
    // int i2c_gpio_scl = 3;
    int i2c_gpio_sda = 26;
    int i2c_gpio_scl = 27;

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

    vTaskDelete(NULL);
}

void app_main(void)
{
    // xTaskCreate(I2CScanTask, "I2C_Scan_Task", 4096, NULL, 5, NULL);

    sensorDataQueue = xQueueCreate(10, sizeof(sensorData_t));
    assert(sensorDataQueue != NULL);

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* load Zigbee light_bulb platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    /* hardware related and device init */

    xTaskCreate(ZigbeeTask, "Zigbee_Task", 4096, NULL, 5, NULL);
    xTaskCreate(SensorInitTask, "Temp_Sensor_Init_Task", 4096, NULL, 5, NULL);

    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);
}