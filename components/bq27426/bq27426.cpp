#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "string.h"
#include <stdexcept>

#include "bq27426.h"
#include "bq27426_defs.h"

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

BQ27426::~BQ27426()
{
    i2c_master_bus_rm_device(_i2cDevice);
}

// ------------------ Control Commands ------------------

BQ27426_DEFS::CONTROL_STATUS BQ27426::getControlStatus()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::STATUS);
    return BQ27426_DEFS::CONTROL_STATUS::from_value(_commandControl.read());
}

uint16_t BQ27426::getDeviceType()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::DEVICE_TYPE);
    return _commandControl.read();
}

uint16_t BQ27426::getFwVersion()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::FW_VERSION);
    return _commandControl.read();
}

uint16_t BQ27426::getChemId()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::CHEM_ID);
    return _commandControl.read();
}

void BQ27426::batteryInsert()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::BAT_INSERT);
}

void BQ27426::batteryRemove()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::BAT_REMOVE);
}

void BQ27426::setCfgUpdate()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::SET_CFGUPDATE);
}

void BQ27426::reset()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::RESET);
}

void BQ27426::softReset()
{
    _commandControl.write(BQ27426_DEFS::CONTROL_SUBCOMMANDS::SOFT_RESET);
}

// ------------------ Standard Commands ------------------

uint16_t BQ27426::getTemperature()
{
    return _commandTemperture.read();
}

uint16_t BQ27426::getVoltage()
{
    return _commandVoltage.read();
}

BQ27426_DEFS::FLAGS BQ27426::getFlags()
{
    return BQ27426_DEFS::FLAGS::from_value(_commandFlags.read());
}

uint16_t BQ27426::getNominalCapacity()
{
    return _commandNomCapacity.read();
}

uint16_t BQ27426::getFullAvailableCapacity()
{
    return _commandAvailCapacity.read();
}

uint16_t BQ27426::getRemainingCapacity()
{
    return _commandRemCapacity.read();
}

uint16_t BQ27426::getFullChargeCapacity()
{
    return _commandFullCapacity.read();
}

uint16_t BQ27426::getAverageCurrent()
{
    return _commandAvgCurrent.read();
}

uint16_t BQ27426::getAveragePower()
{
    return _commandAvgPower.read();
}

uint16_t BQ27426::getStateOfCharge()
{
    return _commandSOC.read();
}

uint16_t BQ27426::getInternalTemperature()
{
    return _commandIntTemp.read();
}

uint16_t BQ27426::getStateOfHealth()
{
    return _commandSOH.read();
}

uint16_t BQ27426::getRemainingCapacityUnfiltered()
{
    return _commandRemCapUnfil.read();
}

uint16_t BQ27426::getRemainingCapacityFiltered()
{
    return _commandRemCapFil.read();
}

uint16_t BQ27426::getFullChargeCapacityUnfiltered()
{
    return _commandFullCapUnfil.read();
}

// ------------------ Private Methods ------------------

void BQ27426::_initRegisters()
{
    using namespace BQ27426_DEFS;
    _commandControl = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_CONTROL, 2);
    _commandTemperture = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_TEMP, 2);
    _commandVoltage = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_VOLTAGE, 2);
    _commandFlags = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_FLAGS, 2);
    _commandNomCapacity = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_NOM_CAPACITY, 2);
    _commandAvailCapacity = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_AVAIL_CAPACITY, 2);
    _commandRemCapacity = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_REM_CAPACITY, 2);
    _commandFullCapacity = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_FULL_CAPACITY, 2);
    _commandAvgCurrent = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_AVG_CURRENT, 2);
    _commandAvgPower = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_AVG_POWER, 2);
    _commandSOC = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_SOC, 2);
    _commandIntTemp = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_INT_TEMP, 2);
    _commandSOH = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_SOH, 2);
    _commandRemCapUnfil = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_REM_CAP_UNFIL, 2);
    _commandRemCapFil = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_REM_CAP_FIL, 2);
    _commandFullCapUnfil = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_FULL_CAP_UNFIL, 2);
    _commandFullCapFil = I2C_Register(_i2cDevice, STANDARD_COMMANDS::BQ27426_COMMAND_FULL_CAP_FIL, 2);
}