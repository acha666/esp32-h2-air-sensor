#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string>
#include <format>

#include "adafruit_lps2x.h"
#include "adafruit_sht4x.h"
#include "i2c_register.h"

#include "main.h"
#include "sensor.h"

Adafruit_SHT4x sht4x;
Adafruit_LPS22 lps22;

static void sensor_i2c_init(void);
static void sht4x_init(void);
static void lps22_init(void);

static const char *TAG = "Sensor";

Adafruit_Sensor *temp_sensor = NULL;
Adafruit_Sensor *humidity_sensor = NULL;
Adafruit_Sensor *pressure_temp_sensor = NULL;
Adafruit_Sensor *pressure_sensor = NULL;

i2c_master_bus_handle_t sensor_i2c_master_bus_handle;

extern "C" void sensor_init_task(void *pvParameters)
{
    sensor_i2c_init();

#ifdef CONFIG_PRJ_TEMP_SENSOR_SHT4x
    sht4x = Adafruit_SHT4x();
    sht4x_init();
#elif CONFIG_PRJ_TEMP_SENSOR_SHT3x
    // todo
#elif CONFIG_PRJ_TEMP_SENSOR_HDC302x
    // todo
#endif

#ifdef CONFIG_PRJ_PRESSURE_SENSOR_LPS22HH
    lps22 = Adafruit_LPS22();
    lps22_init();
#endif

    xTaskCreate(sensor_task, "Temp_Sensor_Task", 4096, NULL, 5, &tempSensorTaskHandle);
    assert(tempSensorTaskHandle != NULL);

    vTaskDelete(NULL);
}

extern "C" void sensor_task(void *pvParameters)
{
    sensors_event_t humidity_event, temp_event;
    sensors_event_t pressure_temp_event, pressure_event;
    static sensor_data_t data;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        temp_sensor->getEvent(&temp_event);
        humidity_sensor->getEvent(&humidity_event);
        pressure_sensor->getEvent(&pressure_event);

        data.temperature = temp_event.temperature;
        data.humidity = humidity_event.relative_humidity;
        data.pressure = pressure_event.pressure;

        if (xQueueSend(sensorDataQueue, &data, portMAX_DELAY) != pdPASS)
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

    temp_sensor = sht4x.getTemperatureSensor();
    humidity_sensor = sht4x.getHumiditySensor();
}

static void lps22_init(void)
{
    uint8_t lps22_addr = CONFIG_PRJ_PRESSURE_SENSOR_LPS22_ADDR;

    ESP_ERROR_CHECK(lps22.begin_I2C(sensor_i2c_master_bus_handle, lps22_addr));
    lps22.setDataRate(LPS22_RATE_10_HZ);

    pressure_temp_sensor = lps22.getTemperatureSensor();
    pressure_sensor = lps22.getPressureSensor();
}