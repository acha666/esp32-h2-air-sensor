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
#include <stdexcept>
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
        .scl_speed_hz = 100 * 1000,
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
    _regChargerControl = I2C_Register(_i2cDevice, REG_CHARGER_CONTROL_0_ADDR, 4, MSBFIRST);
    _regNTCControl = I2C_Register(_i2cDevice, REG_NTC_CONTROL_0_ADDR, 3, MSBFIRST);
    _regChargerStatus = I2C_Register(_i2cDevice, REG_CHARGER_STATUS_0_ADDR, 2, MSBFIRST);
    _regFaultStatus0 = I2C_Register(_i2cDevice, REG_FAULT_STATUS_0_ADDR, 1);
    _regChargerFlag = I2C_Register(_i2cDevice, REG_CHARGER_FLAG_0_ADDR, 2, MSBFIRST);
    _regFaultFlag0 = I2C_Register(_i2cDevice, REG_FAULT_FLAG_0_ADDR, 1);
    _regChargerMask = I2C_Register(_i2cDevice, REG_CHARGER_MASK_0_ADDR, 2, MSBFIRST);
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
 * @brief aka ICHG, POR=320mA
 * @note When Q4_FULLON=1, this register has a minimum value of 80mA
 *
 * @param current in mA, 40-2000mA, 40mA per step
 */
void BQ2562x::setChargeCurrent(uint16_t current)
{
    current /= 40;
    if (current >= 0x1 && current <= 0x32)
    {
        auto reg_bits = I2C_RegisterBits(&_regChargeCurrentLimit, 6, 5);
        reg_bits.write(current);
    }
    else
        throw std::runtime_error("Invalid charge current");
}

uint16_t BQ2562x::getChargeCurrent()
{
    // ESP_LOGD(TAG, "raw: 0x%04lx", _regChargeCurrentLimit.read());
    auto reg_bits = I2C_RegisterBits(&_regChargeCurrentLimit, 6, 5);
    return reg_bits.read() * 40;
}

/**
 * @brief aka VREG, POR=4200mV
 *
 * @param voltage in mV, 3500-4800mV, 10mV per step
 */
void BQ2562x::setChargeVoltage(uint16_t voltage)
{
    voltage /= 10;
    if (voltage >= 0x15e && voltage <= 0x1e0)
    {
        auto reg_bits = I2C_RegisterBits(&_regChargeVoltageLimit, 9, 3);
        reg_bits.write(voltage);
    }
    else
        throw std::runtime_error("Invalid charge voltage");
}

uint16_t BQ2562x::getChargeVoltage()
{
    auto reg_bits = I2C_RegisterBits(&_regChargeVoltageLimit, 9, 3);
    return reg_bits.read() * 10;
}

/**
 * @brief aka IINDPM, POR=3200mA
 * @note When the adapter is removed, IINDPM is reset to its POR value of 3.2 A.
 *
 * @param current in mA, 100-3200mA, 20mA per step
 */
void BQ2562x::setInputCurrent(uint16_t current)
{
    current /= 20;
    if (current >= 0x5 && current <= 0xA0)
    {
        auto reg_bits = I2C_RegisterBits(&_regInputCurrentLimit, 8, 4);
        reg_bits.write(current);
    }
    else
        throw std::runtime_error("Invalid input current");
}

uint16_t BQ2562x::getInputCurrent()
{
    auto reg_bits = I2C_RegisterBits(&_regInputCurrentLimit, 8, 4);
    return reg_bits.read() * 20;
}

/**
 * @brief aka VINDPM, POR=4600mV
 *
 * @param voltage in mV, 3800-16800mV, 40mV per step
 */
void BQ2562x::setInputVoltage(uint16_t voltage)
{
    voltage /= 40;
    if (voltage >= 0x5f && voltage <= 0x1a4)
    {
        auto reg_bits = I2C_RegisterBits(&_regInputVoltageLimit, 9, 5);
        reg_bits.write(voltage);
    }
    else
        throw std::runtime_error("Invalid input voltage");
}

uint16_t BQ2562x::getInputVoltage()
{
    auto reg_bits = I2C_RegisterBits(&_regInputVoltageLimit, 9, 5);
    return reg_bits.read() * 40;
}

/**
 * @brief aka VSYSMIN, POR=3520mV
 *
 * @param voltage in mV, 2560-3840mV, 80mV per step
 */
void BQ2562x::setMinimalSystemVoltage(uint16_t voltage)
{
    voltage /= 80;
    if (voltage >= 0x20 && voltage <= 0x30)
    {
        auto reg_bits = I2C_RegisterBits(&_regMinimalSystemVoltage, 6, 6);
        reg_bits.write(voltage);
    }
    else
        throw std::runtime_error("Invalid minimal system voltage");
}

uint16_t BQ2562x::getMinimalSystemVoltage()
{
    auto reg_bits = I2C_RegisterBits(&_regMinimalSystemVoltage, 6, 6);
    return reg_bits.read() * 80;
}

/**
 * @brief aka IPRECHG, POR=30mA
 * @note When Q4_FULLON=1, this register has a minimum value of 80mA, so Reset value becomes 80mA in this case
 *
 * @param current in mA, 10-310mA, 10mA per step
 */
void BQ2562x::setPreChargeCurrent(uint8_t current)
{
    current /= 10;
    if (current >= 0x1 && current <= 0x1F)
    {
        auto reg_bits = I2C_RegisterBits(&_regPreChargeControl, 5, 3);
        reg_bits.write(current);
    }
    else
        throw std::runtime_error("Invalid pre-charge current");
}

uint8_t BQ2562x::getPreChargeCurrent()
{
    auto reg_bits = I2C_RegisterBits(&_regPreChargeControl, 5, 3);
    return reg_bits.read() * 10;
}

/**
 * @brief aka ITERM, POR=20mA
 * @note When Q4_FULLON=1, this register has a minimum value of 60mA, so Reset value becomes 60mA in this case
 *
 * @param current in mA, 5-310mA, 5mA per step
 */
void BQ2562x::setTerminationCurrent(uint8_t current)
{
    current /= 5;
    if (current >= 0x1 && current <= 0x3E)
    {
        auto reg_bits = I2C_RegisterBits(&_regTerminationControl, 6, 2);
        reg_bits.write(current);
    }
    else
        throw std::runtime_error("Invalid termination current");
}

uint8_t BQ2562x::getTerminationCurrent()
{
    auto reg_bits = I2C_RegisterBits(&_regTerminationControl, 6, 2);
    return reg_bits.read() * 5;
}

/********************** REG0x14_Charge_Control **********************/
void BQ2562x::setChargeControl(BQ2562X_DEFS::CHARGE_CONTROL_REG control)
{
    uint8_t reg_value = _regChargeControl.read();
    reg_value = control.to_byte(reg_value);
    _regChargeControl.write(reg_value);
}

BQ2562X_DEFS::CHARGE_CONTROL_REG BQ2562x::getChargeControl()
{
    uint8_t reg_value = _regChargeControl.read();
    return BQ2562X_DEFS::CHARGE_CONTROL_REG::from_byte(reg_value);
}

uint8_t BQ2562x::getChargeControlRaw()
{
    return _regChargeControl.read();
}

/********************** REG0x15_Charge_Timer_Control **********************/
void BQ2562x::setChargeTimerControl(BQ2562X_DEFS::CHARGE_TIMER_CONTROL_REG control)
{
    uint8_t reg_value = _regChargeTimerControl.read();
    reg_value = control.to_byte(reg_value);
    _regChargeTimerControl.write(reg_value);
}

BQ2562X_DEFS::CHARGE_TIMER_CONTROL_REG BQ2562x::getChargeTimerControl()
{
    uint8_t reg_value = _regChargeTimerControl.read();
    return BQ2562X_DEFS::CHARGE_TIMER_CONTROL_REG::from_byte(reg_value);
}

uint8_t BQ2562x::getChargeTimerControlRaw()
{
    return _regChargeTimerControl.read();
}

/********************** REG0x16-0x19_Charger_Control **********************/
void BQ2562x::setChargerControl(BQ2562X_DEFS::CHARGER_CONTROL_REG control)
{
    uint32_t reg_value = _regChargerControl.read();
    reg_value = control.to_value(reg_value);
    _regChargerControl.write(reg_value);
}

BQ2562X_DEFS::CHARGER_CONTROL_REG BQ2562x::getChargerControl()
{
    uint32_t reg_value = _regChargerControl.read();
    return BQ2562X_DEFS::CHARGER_CONTROL_REG::from_value(reg_value);
}

uint32_t BQ2562x::getChargerControlRaw()
{
    return _regChargerControl.read();
}

/********************** REG0x1A-0x1C_NTC_Control **********************/
void BQ2562x::setNTCControl(BQ2562X_DEFS::NTC_CONTROL_REG control)
{
    uint32_t reg_value = _regNTCControl.read();
    reg_value = control.to_value(reg_value);
    _regNTCControl.write(reg_value);
}

BQ2562X_DEFS::NTC_CONTROL_REG BQ2562x::getNTCControl()
{
    uint32_t reg_value = _regNTCControl.read();
    return BQ2562X_DEFS::NTC_CONTROL_REG::from_value(reg_value);
}

uint32_t BQ2562x::getNTCControlRaw()
{
    return _regNTCControl.read();
}

/********************** REG0x1D-0x1E_Charger_Status **********************/
BQ2562X_DEFS::CHARGER_STATUS_REG BQ2562x::getChargerStatus()
{
    uint32_t reg_value = _regChargerStatus.read();
    return BQ2562X_DEFS::CHARGER_STATUS_REG::from_value(reg_value);
}

uint32_t BQ2562x::getChargerStatusRaw()
{
    return _regChargerStatus.read();
}

/********************** REG0x1F_FAULT_Status_0 **********************/
BQ2562X_DEFS::FAULT_STATUS_REG BQ2562x::getFaultStatus()
{
    uint8_t reg_value = _regFaultStatus0.read();
    return BQ2562X_DEFS::FAULT_STATUS_REG::from_byte(reg_value);
}

uint8_t BQ2562x::getFaultStatusRaw()
{
    return _regFaultStatus0.read();
}

/********************** REG0x20-0x21_Charger_Flag **********************/
BQ2562X_DEFS::CHARGER_FLAG_REG BQ2562x::getChargerFlag()
{
    uint32_t reg_value = _regChargerFlag.read();
    return BQ2562X_DEFS::CHARGER_FLAG_REG::from_value(reg_value);
}

uint8_t BQ2562x::getChargerFlagRaw()
{
    return _regChargerFlag.read();
}

/********************** REG0x22_FAULT_Flag_0 **********************/
BQ2562X_DEFS::FAULT_FLAG_REG BQ2562x::getFaultFlag()
{
    uint8_t reg_value = _regFaultFlag0.read();
    return BQ2562X_DEFS::FAULT_FLAG_REG::from_byte(reg_value);
}

uint8_t BQ2562x::getFaultFlagRaw()
{
    return _regFaultFlag0.read();
}

/********************** REG0x23-0x24_Charger_Mask **********************/
void BQ2562x::setChargerMask(BQ2562X_DEFS::CHARGER_MASK_REG mask)
{
    uint32_t reg_value = _regChargerMask.read();
    reg_value = mask.to_value(reg_value);
    _regChargerMask.write(reg_value);
}

BQ2562X_DEFS::CHARGER_MASK_REG BQ2562x::getChargerMask()
{
    uint32_t reg_value = _regChargerMask.read();
    return BQ2562X_DEFS::CHARGER_MASK_REG::from_value(reg_value);
}

uint8_t BQ2562x::getChargerMaskRaw()
{
    return _regChargerMask.read();
}

/********************** REG0x25_FAULT_Mask_0 **********************/
void BQ2562x::setFaultMask(BQ2562X_DEFS::FAULT_MASK_REG mask)
{
    uint8_t reg_value = _regFaultMask0.read();
    reg_value = mask.to_byte(reg_value);
    _regFaultMask0.write(reg_value);
}

BQ2562X_DEFS::FAULT_MASK_REG BQ2562x::getFaultMask()
{
    uint8_t reg_value = _regFaultMask0.read();
    return BQ2562X_DEFS::FAULT_MASK_REG::from_byte(reg_value);
}

uint8_t BQ2562x::getFaultMaskRaw()
{
    return _regFaultMask0.read();
}

/********************** REG0x26_ADC_Control **********************/
void BQ2562x::setADCControl(BQ2562X_DEFS::ADC_CONTROL_REG control)
{
    uint8_t reg_value = _regADCControl.read();
    reg_value = control.to_byte(reg_value);
    _regADCControl.write(reg_value);
}

BQ2562X_DEFS::ADC_CONTROL_REG BQ2562x::getADCControl()
{
    uint8_t reg_value = _regADCControl.read();
    return BQ2562X_DEFS::ADC_CONTROL_REG::from_byte(reg_value);
}

uint8_t BQ2562x::getADCControlRaw()
{
    return _regADCControl.read();
}

/********************** REG0x27_ADC_Function_Disable_0 **********************/
void BQ2562x::setADCFunctionDisable(BQ2562X_DEFS::ADC_FUNCTION_DISABLE_REG reg)
{
    uint8_t reg_value = _regADCFunctionDisable0.read();
    reg_value = reg.to_byte(reg_value);
    _regADCFunctionDisable0.write(reg_value);
}

BQ2562X_DEFS::ADC_FUNCTION_DISABLE_REG BQ2562x::getADCFunctionDisable()
{
    uint8_t reg_value = _regADCFunctionDisable0.read();
    return BQ2562X_DEFS::ADC_FUNCTION_DISABLE_REG::from_byte(reg_value);
}

uint8_t BQ2562x::getADCFunctionDisableRaw()
{
    return _regADCFunctionDisable0.read();
}

void BQ2562x::resetWatchDog()
{
    auto reg_bits = I2C_RegisterBits(&_regChargerControl, 1, 2 + 8 * 3);
    reg_bits.write(1);
}

void BQ2562x::resetRegister()
{
    auto reg_bits = I2C_RegisterBits(&_regChargerControl, 1, 7 + 8 * 2);
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
