#include "esp_log.h"
#include "driver/i2c_master.h"
#include "main.h"
#include "bq27426.h"
#include "bq2562x.h"
#include "bq2562x_defs.h"

static void power_i2c_init(void);

static const char *TAG = "Power";
static i2c_master_bus_handle_t power_i2c_master_bus_handle;

BQ27426 *BatteryGauge;
BQ2562x *Charger;

extern "C" void PowerTask(void *pvParameters)
{
    power_i2c_init();
    assert(xSemaphoreTake(xI2CSemaphore, 5000 / portTICK_PERIOD_MS) == pdTRUE);
    Charger = new BQ2562x(power_i2c_master_bus_handle, 0x6a);
    BatteryGauge = new BQ27426(power_i2c_master_bus_handle);

    uint32_t val = Charger->getChargeCurrent();
    ESP_LOGI(TAG, "Charge Current: %ld mA", val);

    val = Charger->getChargerControl();
    ESP_LOGI(TAG, "Charger Control: 0x%08lx", val);

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

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &power_i2c_master_bus_handle));

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

    ESP_ERROR_CHECK(i2c_del_master_bus(tool_bus_handle));
}