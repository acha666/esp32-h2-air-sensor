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

constexpr char *TAG = "BQ2562x";
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

    ret = reset();
    if (ret != ESP_OK)
        throw runtime_error("Failed to reset BQ2562x");
}

BQ2562x::~BQ2562x()
{
    i2c_master_bus_rm_device(_i2cDevice);
}
void BQ2562x::_initRegisters()
{
    using namespace BQ2562X_DEFS;
    _regChargeCurrentLimit = std::make_unique<I2C_Register>(_i2cDevice, REG_CHARGE_CURRENT_LIMIT_ADDR, 2);
    _regChargeVoltageLimit = std::make_unique<I2C_Register>(_i2cDevice, REG_CHARGE_VOLTAGE_LIMIT_ADDR, 2);
    _regInputCurrentLimit = std::make_unique<I2C_Register>(_i2cDevice, REG_INPUT_CURRENT_LIMIT_ADDR, 2);
    _regInputVoltageLimit = std::make_unique<I2C_Register>(_i2cDevice, REG_INPUT_VOLTAGE_LIMIT_ADDR, 2);
    _regMinimalSystemVoltage = std::make_unique<I2C_Register>(_i2cDevice, REG_MINIMAL_SYSTEM_VOLTAGE_ADDR, 2);
    _regPreChargeControl = std::make_unique<I2C_Register>(_i2cDevice, REG_PRE_CHARGE_CONTROL_ADDR, 2);
    _regTerminationControl = std::make_unique<I2C_Register>(_i2cDevice, REG_TERMINATION_CONTROL_ADDR, 2);
    _regChargeControl = std::make_unique<I2C_Register>(_i2cDevice, REG_CHARGE_CONTROL_ADDR, 1);
    _regChargeTimerControl = std::make_unique<I2C_Register>(_i2cDevice, REG_CHARGE_TIMER_CONTROL_ADDR, 1);
    _regChargerControl = std::make_unique<I2C_Register>(_i2cDevice, REG_CHARGER_CONTROL_0_ADDR, 4);
    _regNTCControl = std::make_unique<I2C_Register>(_i2cDevice, REG_NTC_CONTROL_0_ADDR, 3);
    _regChargerStatus = std::make_unique<I2C_Register>(_i2cDevice, REG_CHARGER_STATUS_0_ADDR, 2);
    _regFaultStatus0 = std::make_unique<I2C_Register>(_i2cDevice, REG_FAULT_STATUS_0_ADDR, 1);
    _regChargerFlag = std::make_unique<I2C_Register>(_i2cDevice, REG_CHARGER_FLAG_0_ADDR, 2);
    _regFaultFlag0 = std::make_unique<I2C_Register>(_i2cDevice, REG_FAULT_FLAG_0_ADDR, 1);
    _regChargerMask = std::make_unique<I2C_Register>(_i2cDevice, REG_CHARGER_MASK_0_ADDR, 2);
    _regFaultMask0 = std::make_unique<I2C_Register>(_i2cDevice, REG_FAULT_MASK_0_ADDR, 1);
    _regADCControl = std::make_unique<I2C_Register>(_i2cDevice, REG_ADC_CONTROL_ADDR, 1);
    _regADCFunctionDisable0 = std::make_unique<I2C_Register>(_i2cDevice, REG_ADC_FUNCTION_DISABLE_0_ADDR, 1);
    _regIBUS_ADC = std::make_unique<I2C_Register>(_i2cDevice, REG_IBUS_ADC_ADDR, 2);
    _regIBAT_ADC = std::make_unique<I2C_Register>(_i2cDevice, REG_IBAT_ADC_ADDR, 2);
    _regVBUS_ADC = std::make_unique<I2C_Register>(_i2cDevice, REG_VBUS_ADC_ADDR, 2);
    _regVPMI_DADC = std::make_unique<I2C_Register>(_i2cDevice, REG_VPMID_ADC_ADDR, 2);
    _regVBAT_ADC = std::make_unique<I2C_Register>(_i2cDevice, REG_VBAT_ADC_ADDR, 2);
    _regVSYS_ADC = std::make_unique<I2C_Register>(_i2cDevice, REG_VSYS_ADC_ADDR, 2);
    _regTS_ADC = std::make_unique<I2C_Register>(_i2cDevice, REG_TS_ADC_ADDR, 2);
    _regTDIE_ADC = std::make_unique<I2C_Register>(_i2cDevice, REG_TDIE_ADC_ADDR, 2);
    _regPartInformation = std::make_unique<I2C_Register>(_i2cDevice, REG_PART_INFORMATION_ADDR, 1);
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
        auto reg_bits = I2C_RegisterBits(_regChargeCurrentLimit, 6, 4);
        reg_bits.write(current);
    }
    else
        throw std::runtime_error("Invalid charge current");
}

void BQ2562x::setWatchDog(WatchDogTimerConf timer)
{
    auto reg_bits = I2C_RegisterBits(*_regChargerControl, 2, 24);
    reg_bits.write(static_cast<uint8_t>(timer));
}

BQ2562x::WatchDogTimerConf BQ2562x::getWatchDog()
{
    auto reg_bits = I2C_RegisterBits(_regChargerControl, 2, 24);
    return static_cast<WatchDogTimerConf>(reg_bits.read());
}

bool BQ2562x::getTS(bool &enabled)
{
    bool tsIgnore = false, tsDis = false;
    if (readReg(NTC_Control_0_TS_IGNORE, tsIgnore))
    {
        Register reg = ADC_Function_Disable_0;
        reg.start = reg.end = static_cast<uint8_t>(Adc::TS);
        if (readReg(reg, tsDis))
        {
            enabled = (!tsIgnore) && (!tsDis);
            return true;
        }
    }
    return false;
}

bool BQ2562x::enableHIZ(bool enable)
{
    return writeReg(Charger_Control_0_EN_HIZ, enable);
}

bool BQ2562x::enableTS(bool enable)
{
    return writeReg(NTC_Control_0_TS_IGNORE, !enable) && enableADC(Adc::TS, enable);
}

bool BQ2562x::enableADC(Adc adc, bool enable)
{
    Register reg = ADC_Function_Disable_0;
    reg.start = reg.end = static_cast<uint8_t>(adc);
    return writeReg(reg, !enable);
}

bool BQ2562x::getPartInformation(uint8_t &value)
{
    return readReg(Part_Information, value);
}

bool BQ2562x::enableInterrupts(bool enable)
{
    uint8_t data = enable ? 0x00 : 0xFF;
    if (writeReg(Charger_Mask_0, data))
    {
        return writeReg(Charger_Mask_1, data);
    }
    return false;
}

bool BQ2562x::enableInterrupt(Interrupt num, bool en)
{
    uint8_t reg0 = 0;
    uint8_t reg1 = 0;

    if (readReg(Charger_Mask_0, reg0))
    {
        uint8_t mask0 = 0, mask1 = 0;

        if (readReg(Charger_Mask_1, reg1))
        {
            switch (num)
            {
            case Interrupt::VBUS:
                mask1 = 0b1 << 0;
                break;
            default:
                break;
            }

            reg0 = en ? reg0 & ~(mask0) : reg0 | mask0;
            reg1 = en ? reg1 & ~(mask1) : reg1 | mask1;

            if (writeReg(Charger_Mask_0, reg0))
            {
                return writeReg(Charger_Mask_1, reg1);
            }
        }
    }

    return false;
}

bool BQ2562x::enableCharging(bool state)
{
    return writeReg(Charger_Control_0_EN_CHG, state);
}

bool BQ2562x::getVBUS(uint16_t &value)
{
    uint16_t data = 0;
    if (readReg(VBUS_ADC, data))
    {
        value = data * 3.97f;
        return true;
    }

    return false;
}

bool BQ2562x::getIBAT(int16_t &value)
{
    uint16_t data = 0;
    if (readReg(IBAT_ADC, data))
    {
        if (data != 0x2000)
        {
            if (data >= 0x38AD && data <= 0x3FFF)
            {
                data = ((0x3FFF - data + 1) * 4) * -1;
            }
            else
            {
                data *= 4;
            }

            value = data;
            return true;
        }
    }
    return false;
}

bool BQ2562x::getIBUS(int16_t &value)
{
    uint16_t data = 0;
    if (readReg(IBUS_ADC, data))
    {
        if (data >= 0x7830 && data <= 0x7FFF)
        {
            data = (0x7FFF - data + 1) * -2;
        }
        else
        {
            data *= 2;
        }

        value = data;
        return true;
    }
    return false;
}

bool BQ2562x::getVBAT(uint16_t &value)
{
    uint16_t res = 0;
    if (readReg(VBAT_ADC, res))
    {
        value = (res * 1.99f);
        return true;
    }
    return false;
}

bool BQ2562x::setVINDPM(uint32_t mV)
{
    return writeReg(Input_Current_Limit_VINDPM, static_cast<uint16_t>((mV) / 40));
}

bool BQ2562x::setIINDPM(uint32_t mA)
{
    return writeReg(Input_Current_Limit_IINDPM, static_cast<uint16_t>((mA) / 40));
}

bool BQ2562x::setupADC(bool enable, ADCRate rate, ADCSampling sampling, ADCAverage average, ADCAverageInit averageInit)
{
    uint8_t value = enable << 7;

    if (enable)
    {
        value |= (rate == ADCRate::Oneshot) << 6;

        switch (sampling)
        {
        case ADCSampling::Bits_12:
            value |= 0 << 4;
            break;

        case ADCSampling::Bits_11:
            value |= 1 << 4;
            break;

        case ADCSampling::Bits_10:
            value |= 2 << 4;
            break;

        case ADCSampling::Bits_9:
            value |= 3 << 4;
            break;

        default:
            break;
        }

        value |= (average == ADCAverage::Running) << 3;
        value |= (averageInit == ADCAverageInit::New) << 2;
    }

    return writeReg(ADC_Control, value);
}

bool BQ2562x::getADCDone(bool &done)
{
    uint8_t data;
    if (readReg(Charger_Status_0, data))
    {
        done = data & (1 << 6);
        return true;
    }
    return false;
}

bool BQ2562x::getBatteryVoltage(float &value)
{
    uint16_t data = 0;
    if (readReg(VBAT_ADC, data))
    {
        value = data / 1000.0f;
        return true;
    }
    return false;
}

bool BQ2562x::getVBUSStat(VBUSStat &stat)
{
    uint8_t data = 0;
    if (readReg(Charger_Status_1_VBUS_STAT, data))
    {
        if (data == 0b100)
        {
            stat = VBUSStat::Adapter;
        }
        else
        {
            stat = VBUSStat::None;
        }
        return true;
    }
    return false;
}

bool BQ2562x::getChargeStat(ChargeStat &stat)
{
    uint8_t value = 0;
    if (readReg(Charger_Status_1_CHG_STAT, value))
    {
        ChargeStat res = ChargeStat::Terminated;

        switch (value)
        {
        case 0x01:
            res = ChargeStat::Trickle;
            break;

        case 0x02:
            res = ChargeStat::Taper;
            break;

        case 0x03:
            res = ChargeStat::TopOff;
            break;

        case 0x00:
        default:

            break;
        }

        stat = res;
        return true;
    }

    return false;
}

bool BQ2562x::setBATFETControl(BATFETControl control)
{
    uint8_t value = 0x0;
    switch (control)
    {
    case BATFETControl::ShutdownMode:
        value = 0x01;
        break;
    case BATFETControl::ShipMode:
        value = 0x02;
        break;
    case BATFETControl::SystemPowerReset:
        value = 0x03;
        break;
    case BATFETControl::Normal:
    default:
        break;
    }

    return writeReg(Charger_Control_2_BATFET_CTRL, value);
}

bool BQ2562x::setBATFETDelay(BATFETDelay delay)
{
    bool value = false;
    switch (delay)
    {
    case BATFETDelay::Delay10s:
        value = true;
        break;
    case BATFETDelay::Delay20ms:
    default:
        break;
    }

    return writeReg(Charger_Control_2_BATFET_DLY, value);
}

bool BQ2562x::enableWVBUS(bool enable)
{
    return writeReg(Charger_Control_2_WVBUS, enable);
}

bool BQ2562x::getTS_ADC(float &value)
{
    uint16_t res = 0;
    if (readReg(TS_ADC, res))
    {
        value = (res * 0.0961f) / 100.0f;
        return true;
    }
    return false;
}
