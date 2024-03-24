#include "bq27426.h"
#include "bq27426_defs.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "string.h"
#include <iostream>

using std::runtime_error;

#define TAG "BQ27426"

BQ27426::BQ27426(i2c_master_bus_handle_t bus_handle, uint8_t address)
{
    esp_err_t ret;

    _i2cBus = bus_handle;

    ret = i2c_master_probe(_i2cBus, address, 500); // check if the device is connected
    if (ret != ESP_OK)
        throw runtime_error("BQ27426 not found");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    ret = i2c_master_bus_add_device(_i2cBus, &dev_cfg, &_i2cDevice);
    if (ret != ESP_OK)
        throw runtime_error("Failed to add BQ27426 to I2C bus");

    _initRegisters();

    uint16_t deviceType = getDeviceType();
    ESP_LOGI(TAG, "Found Device Type: 0x%04x", deviceType);
}

void BQ27426::_initRegisters()
{
    using namespace BQ27426_DEFS;
    _commandControl = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_CONTROL, 2);
}

uint16_t BQ27426::getDeviceType()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::DEVICE_TYPE);
    return _commandControl.read();
}