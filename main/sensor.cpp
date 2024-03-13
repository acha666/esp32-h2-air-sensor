#include "main.h"
#include "sensor.h"

#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sht4x.h"

#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x
Adafruit_SHT4x sht4x;
#endif

extern "C" void TempSensorInitTask(void)
{
#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x
    sht4x_init();
#elif CONFIG_PRJ_TEMP_SENSOR_SHT3x
    // todo
#elif CONFIG_PRJ_TEMP_SENSOR_HDC302x
    // todo
#endif
}

extern "C" void TempSensorTask(void)
{
    TempSensorData_t data;

    for (;;)
    {
        // 等待来自主线程的通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x
        ESP_ERROR_CHECK(sht4x.read(&data.humidity, &data.temperature));
#elif CONFIG_PRJ_TEMP_SENSOR_SHT3x
        // todoF
#elif CONFIG_PRJ_TEMP_SENSOR_HDC302x
        // todo
#endif

        // 将数据发送到队列
        if (xQueueSend(tempSensorDataQueue, &data, portMAX_DELAY) != pdPASS)
        {
            // 处理错误情况
        }
    }
}

void sht4x_init(void)
{
    static const char *TAG = "SHT4x Init";
    uint8_t sht4x_addr = CONFIG_PRJ_TEMP_SENSOR_SHT4x_ADDR;
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_2,
        .scl_io_num = GPIO_NUM_1,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    ESP_ERROR_CHECK(sht4x.begin(&i2c_mst_config, sht4x_addr));
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