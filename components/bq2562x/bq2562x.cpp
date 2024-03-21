/**
 *                           POWERFEATHER 4-CLAUSE LICENSE
 *
 *  Copyright (C) 2023, PowerFeather.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *  3. Neither the name of PowerFeather nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 *  4. This software, with or without modification, must only be run on official
 *      PowerFeather boards.
 *
 *  THIS SOFTWARE IS PROVIDED BY POWERFEATHER “AS IS” AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL POWERFEATHER OR CONTRIBUTORS BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "bq2562x.h"
#include "bq2562x_defs.h"
#include "i2c_register.h"
#include "iostream"
#include <memory>

using std::runtime_error;

constexpr const char *TAG = "BQ2562x";
constexpr bool DEBUG_OUTPUT_EN = true;

BQ2562x::BQ2562x(i2c_master_bus_handle_t bus_handle, uint8_t address)
{
    esp_err_t ret;

    _i2cBus = bus_handle;

    ret = i2c_master_probe(_i2cBus, address, 500); // check if the device is connected
    if (ret != ESP_OK)
        throw runtime_error("BQ2562x not found");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    ret = i2c_master_bus_add_device(_i2cBus, &dev_cfg, &_i2cDevice);
    if (ret != ESP_OK)
        throw runtime_error("Failed to add BQ2562x to I2C bus");

    _initRegisters();

    resetRegister();
    uint8_t pn = getPartNumber(), rev = getDeviceRevision();

    ESP_LOGI(TAG, "Found Part ID: 0x%02x, Revision: 0x%02x", pn, rev);
}

BQ2562x::~BQ2562x()
{
    i2c_master_bus_rm_device(_i2cDevice);
}

void BQ2562x::_initRegisters()
{
    using namespace BQ2562X_DEFS;
    _regChargeCurrentLimit = I2C_Register(_i2cDevice, REG_CHARGE_CURRENT_LIMIT_ADDR, 2);
    _regChargeVoltageLimit = I2C_Register(_i2cDevice, REG_CHARGE_VOLTAGE_LIMIT_ADDR, 2);
    _regInputCurrentLimit = I2C_Register(_i2cDevice, REG_INPUT_CURRENT_LIMIT_ADDR, 2);
    _regInputVoltageLimit = I2C_Register(_i2cDevice, REG_INPUT_VOLTAGE_LIMIT_ADDR, 2);
    _regMinimalSystemVoltage = I2C_Register(_i2cDevice, REG_MINIMAL_SYSTEM_VOLTAGE_ADDR, 2);
    _regPreChargeControl = I2C_Register(_i2cDevice, REG_PRE_CHARGE_CONTROL_ADDR, 2);
    _regTerminationControl = I2C_Register(_i2cDevice, REG_TERMINATION_CONTROL_ADDR, 2);
    _regChargeControl = I2C_Register(_i2cDevice, REG_CHARGE_CONTROL_ADDR, 1);
    _regChargeTimerControl = I2C_Register(_i2cDevice, REG_CHARGE_TIMER_CONTROL_ADDR, 1);
    _regChargerControl = I2C_Register(_i2cDevice, REG_CHARGER_CONTROL_0_ADDR, 4);
    _regNTCControl = I2C_Register(_i2cDevice, REG_NTC_CONTROL_0_ADDR, 3);
    _regChargerStatus = I2C_Register(_i2cDevice, REG_CHARGER_STATUS_0_ADDR, 2);
    _regFaultStatus0 = I2C_Register(_i2cDevice, REG_FAULT_STATUS_0_ADDR, 1);
    _regChargerFlag = I2C_Register(_i2cDevice, REG_CHARGER_FLAG_0_ADDR, 2);
    _regFaultFlag0 = I2C_Register(_i2cDevice, REG_FAULT_FLAG_0_ADDR, 1);
    _regChargerMask = I2C_Register(_i2cDevice, REG_CHARGER_MASK_0_ADDR, 2);
    _regFaultMask0 = I2C_Register(_i2cDevice, REG_FAULT_MASK_0_ADDR, 1);
    _regADCControl = I2C_Register(_i2cDevice, REG_ADC_CONTROL_ADDR, 1);
    _regADCFunctionDisable0 = I2C_Register(_i2cDevice, REG_ADC_FUNCTION_DISABLE_0_ADDR, 1);
    _regIBUS_ADC = I2C_Register(_i2cDevice, REG_IBUS_ADC_ADDR, 2);
    _regIBAT_ADC = I2C_Register(_i2cDevice, REG_IBAT_ADC_ADDR, 2);
    _regVBUS_ADC = I2C_Register(_i2cDevice, REG_VBUS_ADC_ADDR, 2);
    _regVPMI_DADC = I2C_Register(_i2cDevice, REG_VPMID_ADC_ADDR, 2);
    _regVBAT_ADC = I2C_Register(_i2cDevice, REG_VBAT_ADC_ADDR, 2);
    _regVSYS_ADC = I2C_Register(_i2cDevice, REG_VSYS_ADC_ADDR, 2);
    _regTS_ADC = I2C_Register(_i2cDevice, REG_TS_ADC_ADDR, 2);
    _regTDIE_ADC = I2C_Register(_i2cDevice, REG_TDIE_ADC_ADDR, 2);
    _regPartInformation = I2C_Register(_i2cDevice, REG_PART_INFORMATION_ADDR, 1);
}

/**
 * @brief
 *
 * @param current in mA, 40mA oer step
 * @return esp_err_t
 */
void BQ2562x::setChargeCurrent(uint16_t current)
{
    current /= 40;
    if (current >= 0x1 && current <= 0x32)
    {
        auto reg_bits = I2C_RegisterBits(&_regChargeCurrentLimit, 6, 4);
        reg_bits.write(current);
    }
    else
        throw std::runtime_error("Invalid charge current");
}

void BQ2562x::setWatchDog(WatchDogTimerConf timer)
{
    auto reg_bits = I2C_RegisterBits(&_regChargerControl, 2, 24);
    reg_bits.write(static_cast<uint8_t>(timer));
}

BQ2562x::WatchDogTimerConf BQ2562x::getWatchDog()
{
    auto reg_bits = I2C_RegisterBits(&_regChargerControl, 2, 24);
    return static_cast<WatchDogTimerConf>(reg_bits.read());
}

void BQ2562x::resetRegister()
{
    auto reg_bits = I2C_RegisterBits(&_regChargerControl, 1, 23);
    reg_bits.write(1);
}

uint8_t BQ2562x::getPartNumber()
{
    auto reg_bits = I2C_RegisterBits(&_regPartInformation, 3, 3);
    return reg_bits.read();
}

uint8_t BQ2562x::getDeviceRevision()
{
    auto reg_bits = I2C_RegisterBits(&_regPartInformation, 3, 0);
    return reg_bits.read();
}

// const char *partNumberToString(uint8_t part_number)
// {
//     switch (part_number)
//     {
//     case 0x02:
//         return "BQ25628";
//     case 0x04:
//         return "BQ25628E";
//     case 0x06:
//         return "BQ25629";
//     default:
//         throw std::runtime_error("Invalid part number");
//     }
// }
