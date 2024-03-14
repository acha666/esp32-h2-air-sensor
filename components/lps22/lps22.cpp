#include "lps22.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

void LPS22::LPS22(void) {}

void LPS22::~LPS22(void)
{
    i2c_master_bus_rm_device(i2c_dev);
}

esp_err_t LPS22::begin(i2c_master_bus_handle_t bus_handle, uint8_t lps22_addr)
{
    esp_err_t ret;

    i2c_bus = bus_handle;

    ret = i2c_master_probe(i2c_bus, lps22_addr, default_timeout / portTICK_PERIOD_MS); // check if the device is connected
    if (ret != ESP_OK)
    {
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = lps22_addr,
        .scl_speed_hz = 100000,
    };

    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}