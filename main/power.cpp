#include "esp_log.h"
#include "driver/i2c_master.h"
#include "main.h"
#include "bq27426.h"

static void power_i2c_init(void);

static char *TAG = "Power";
static i2c_master_bus_handle_t power_i2c_master_bus_handle;
BQ27426 battery_gauge;

extern "C" void PowerTask(void *pvParameters)
{
    power_i2c_init();
    assert(xSemaphoreTake(xI2CSemaphore, 5000 / portTICK_PERIOD_MS) == pdTRUE);
    ESP_ERROR_CHECK(battery_gauge.begin(power_i2c_master_bus_handle, 0x55U));
    xSemaphoreGive(xI2CSemaphore);

    vTaskDelete(NULL);
}

static void power_i2c_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = GPIO_NUM_2,
        .scl_io_num = GPIO_NUM_3,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    // i2c_mst_config.flags.enable_internal_pullup = true; // to be deleted

    assert(xSemaphoreTake(xI2CSemaphore, 5000 / portTICK_PERIOD_MS) == pdTRUE);
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &power_i2c_master_bus_handle));
    esp_err_t ret;
    ret = i2c_master_probe(power_i2c_master_bus_handle, 0X55, 500); // check if the device is connected
    ESP_LOGI(TAG, "Probe result: %d", ret);
    xSemaphoreGive(xI2CSemaphore);

    return;
}

extern "C" void BQScanTask()
{
    i2c_master_bus_handle_t tool_bus_handle;
    i2c_port_t i2c_port = I2C_NUM_0;

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = GPIO_NUM_2,
        .scl_io_num = GPIO_NUM_3,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &tool_bus_handle) != ESP_OK);

    esp_err_t ret = i2c_master_probe(tool_bus_handle, 0x55, 50);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "BQ27441 detected");
    }
    else
    {
        ESP_LOGE(TAG, "BQ27441 not detected");
    }

    BQ27426 battery_gauge;
    battery_gauge.begin(tool_bus_handle, 0x55);

    ESP_ERROR_CHECK(i2c_del_master_bus(tool_bus_handle));
}