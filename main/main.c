#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#endif

#include "main.h"
#include "sensor.h"
#include "power.h"
#include "zigbee.h"
#include "display.h"

EventGroupHandle_t xZigbeeEvents;
QueueHandle_t sensorDataQueue;
QueueHandle_t powerDataQueue;
QueueHandle_t displayMainDataQueue;
QueueHandle_t zigbeeReportDataQueue;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t powerTaskHandle = NULL;

static const char *TAG = "Main";

void main_task(void *pvParameters)
{
    bool zigbee_init_success = false;
    EventBits_t uxBits;
    uxBits = xEventGroupWaitBits(
        xZigbeeEvents,
        ZIGBEE_INIT_SUCCESS | ZIGBEE_INIT_FAILED,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(10 * 1000));

    if (!(uxBits & ZIGBEE_INIT_SUCCESS) && !(uxBits & ZIGBEE_INIT_FAILED))
    {
        ESP_LOGE(TAG, "Zigbee init timeout");
        vTaskDelete(NULL);
    }
    else if (uxBits & ZIGBEE_INIT_SUCCESS)
    {
        ESP_LOGI(TAG, "Zigbee init success");
        zigbee_init_success = true;
    }

    while (1)
    {
        sensor_data_t sensor_data;
        power_data_t power_data;
        display_main_data_t display_main_data;
        zigbee_report_data_t zigbee_report_data;

        xQueueReset(sensorDataQueue);
        xQueueReset(powerDataQueue);

        xTaskNotifyGive(sensorTaskHandle);
        xTaskNotifyGive(powerTaskHandle);

        if (xQueueReceive(sensorDataQueue, &sensor_data, pdMS_TO_TICKS(5000)) == pdPASS)
        {
            ESP_LOGI(TAG, "Hum: %.2f Tmp: %.2f Pre: %.2f", sensor_data.humidity, sensor_data.temperature, sensor_data.pressure);

            display_main_data.humidity = sensor_data.humidity;
            display_main_data.temperature = sensor_data.temperature;
            display_main_data.pressure = sensor_data.pressure;
            zigbee_report_data.temperature = sensor_data.temperature;
            zigbee_report_data.humidity = sensor_data.humidity;
            zigbee_report_data.pressure = sensor_data.pressure;
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
        if (zigbee_init_success)
            xQueueSend(zigbeeReportDataQueue, &zigbee_report_data, 0);

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
        .flags.enable_internal_pullup = false,
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

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* esp zigbee light sleep initialization*/
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    /* load Zigbee platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    /* hardware related and device init */

    i2c_scan_task(2, 3);
    i2c_scan_task(22, 25);
    ESP_LOGI(TAG, "Starting main task");

    xZigbeeEvents = xEventGroupCreate();
    sensorDataQueue = xQueueCreate(3, sizeof(sensor_data_t));
    powerDataQueue = xQueueCreate(3, sizeof(power_data_t));
    displayMainDataQueue = xQueueCreate(3, sizeof(display_main_data_t));
    zigbeeReportDataQueue = xQueueCreate(3, sizeof(zigbee_report_data_t));
    assert(xZigbeeEvents != NULL);
    assert(sensorDataQueue != NULL);
    assert(powerDataQueue != NULL);
    assert(displayMainDataQueue != NULL);
    assert(zigbeeReportDataQueue != NULL);

    xTaskCreate(main_task, "Main_Task", 4096, NULL, 5, NULL);

    xTaskCreate(sensor_task, "Sensor_Task", 4096, NULL, 4, &sensorTaskHandle);
    // xTaskCreate(display_init_task, "Display_Task", 4096, NULL, 3, NULL);
    xTaskCreate(power_task, "Power_Task", 4096, NULL, 2, &powerTaskHandle);

    xTaskCreate(zigbee_task, "Zigbee_Task", 4096, NULL, 1, NULL);
    
}