#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "main.h"
#include "sensor.h"

#include "sht4x.h"
#include "register.h"

#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x
Adafruit_SHT4x sht4x;
#endif

static void sensor_i2c_init(void);
static void sht4x_init(void);

i2c_master_bus_handle_t sensor_i2c_master_bus_handle;

extern "C" void TempSensorInitTask(void *pvParameters)
{
    sensor_i2c_init();

#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x
    sht4x_init();
#elif CONFIG_PRJ_TEMP_SENSOR_SHT3x
    // todo
#elif CONFIG_PRJ_TEMP_SENSOR_HDC302x
    // todo
#endif

    xTaskCreate(TempSensorTask, "Temp_Sensor_Task", 4096, NULL, 5, &tempSensorTaskHandle);
    assert(tempSensorTaskHandle != NULL);

    vTaskDelete(NULL);
}

extern "C" void TempSensorTask(void *pvParameters)
{
    static const char *TAG = "Temp Sensor Task";
    static TempSensorData_t data;
    static esp_err_t ret;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x
        ret = sht4x.read(&data.humidity, &data.temperature);
        if (ret == ESP_ERR_INVALID_CRC)
        {
            ESP_LOGE(TAG, "CRC Error");
            continue;
        }
        else
        {
            ESP_ERROR_CHECK(ret);
        }
#elif CONFIG_PRJ_TEMP_SENSOR_SHT3x
        // todo
#elif CONFIG_PRJ_TEMP_SENSOR_HDC302x
        // todo
#endif

        if (xQueueSend(tempSensorDataQueue, &data, portMAX_DELAY) != pdPASS)
            assert(0);
    }
}

static void sensor_i2c_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_26,
        .scl_io_num = GPIO_NUM_27,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &sensor_i2c_master_bus_handle));

    return;
}

static void sht4x_init(void)
{
    static const char *TAG = "SHT4x Init";
    uint8_t sht4x_addr = CONFIG_PRJ_TEMP_SENSOR_SHT4x_ADDR;

    ESP_ERROR_CHECK(sht4x.begin(sensor_i2c_master_bus_handle, sht4x_addr));
    uint32_t serial = sht4x.readSerial();
    ESP_LOGI(TAG, "SHT4x Serial: %lu", serial);

#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x_NO_HEATER
    sht4x.setHeater(SHT4X_NO_HEATER);
#elif CONFIG_PRJ_TEMP_SENSOR_SHT4x_LOW_HEATER_100MS
    sht4x.setHeater(SHT4X_LOW_HEATER_100MS);
#elif CONFIG_PRJ_TEMP_SENSOR_SHT4x_MED_HEATER_100MS
    sht4x.setHeater(SHT4X_MED_HEATER_100MS);
#elif CONFIG_PRJ_TEMP_SENSOR_SHT4x_HIGH_HEATER_100MS
    sht4x.setHeater(SHT4X_HIGH_HEATER_100MS);
#endif

#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x_HIGH_PRECISION
    sht4x.setPrecision(SHT4X_HIGH_PRECISION);
#elif CONFIG_PRJ_TEMP_SENSOR_SHT4x_MED_PRECISION
    sht4x.setPrecision(SHT4X_MED_PRECISION);
#elif CONFIG_PRJ_TEMP_SENSOR_SHT4x_LOW_PRECISION
    sht4x.setPrecision(SHT4X_LOW_PRECISION);
#endif
}