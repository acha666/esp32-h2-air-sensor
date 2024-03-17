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
    i2c_mst_config.flags.enable_internal_pullup = true; // to be deleted

    assert(xSemaphoreTake(xI2CSemaphore, 5000 / portTICK_PERIOD_MS) == pdTRUE);
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &power_i2c_master_bus_handle));
    xSemaphoreGive(xI2CSemaphore);

    return;
}