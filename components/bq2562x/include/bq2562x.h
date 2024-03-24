#ifndef __BQ2562X_H__
#define __BQ2562X_H__

#include "driver/i2c_master.h"
#include "i2c_register.h"
#include "esp_log.h"
#include "bq2562x_defs.h"

class BQ2562x
{
public:

    BQ2562x(i2c_master_bus_handle_t bus_handle, uint8_t address);
    ~BQ2562x();

    /** REG0x02_Charge_Current_Limit **/
    void setChargeCurrent(uint16_t current);
    uint16_t getChargeCurrent();

    /** REG0x04_Charge_Voltage_Limit **/
    void setChargeVoltage(uint16_t voltage);
    uint16_t getChargeVoltage();

    /** REG0x06_Input_Current_Limit **/
    void setInputCurrent(uint16_t current);
    uint16_t getInputCurrent();

    /** REG0x08_Input_Voltage_Limit **/
    void setInputVoltage(uint16_t voltage);
    uint16_t getInputVoltage();

    /** REG0x0E_Minimal_System_Voltage **/
    void setMinimalSystemVoltage(uint16_t voltage);
    uint16_t getMinimalSystemVoltage();

    /** REG0x10_Pre-charge_Control **/
    void setPreChargeCurrent(uint8_t current);
    uint8_t getPreChargeCurrent();

    /** REG0x12_Termination_Control **/
    void setTerminationCurrent(uint8_t current);
    uint8_t getTerminationCurrent();

    /** REG0x14_Charge_Control **/
    void setChargeControl(BQ2562X_DEFS::CHARGE_CONTROL_REG reg);
    BQ2562X_DEFS::CHARGE_CONTROL_REG getChargeControl();
    uint8_t getChargeControlRaw();

    /** REG0x15_Charge_Timer_Control **/
    void setChargeTimerControl(BQ2562X_DEFS::CHARGE_TIMER_CONTROL_REG reg);
    BQ2562X_DEFS::CHARGE_TIMER_CONTROL_REG getChargeTimerControl();
    uint8_t getChargeTimerControlRaw();

    /** REG0x16_Charger_Control_0, REG0x17_Charger_Control_1, REG0x18_Charger_Control_2, REG0x19_Charger_Control_3 **/
    void setChargerControl(BQ2562X_DEFS::CHARGER_CONTROL_REG reg);
    BQ2562X_DEFS::CHARGER_CONTROL_REG getChargerControl();
    uint32_t getChargerControlRaw();

    /** REG0x1A_NTC_Control_0, REG0x1B_NTC_Control_1, REG0x1C_NTC_Control_2 **/
    void setNTCControl(BQ2562X_DEFS::NTC_CONTROL_REG reg);
    BQ2562X_DEFS::NTC_CONTROL_REG getNTCControl();
    uint32_t getNTCControlRaw();

    /** REG0x1D_Charger_Status_0, REG0x1E_Charger_Status_1 **/
    BQ2562X_DEFS::CHARGER_STATUS_REG getChargerStatus();
    uint32_t getChargerStatusRaw();

    /** REG0x1F_FAULT_Status_0 **/
    BQ2562X_DEFS::FAULT_STATUS_REG getFaultStatus();
    uint8_t getFaultStatusRaw();

    /** REG0x20_Charger_Flag_0, REG0x21_Charger_Flag_1 **/
    BQ2562X_DEFS::CHARGER_FLAG_REG getChargerFlag();
    uint8_t getChargerFlagRaw();

    /** REG0x22_FAULT_Flag_0 **/
    BQ2562X_DEFS::FAULT_FLAG_REG getFaultFlag();
    uint8_t getFaultFlagRaw();

    /** REG0x23_Charger_Mask_0, REG0x24_Charger_Mask_1 **/
    void setChargerMask(BQ2562X_DEFS::CHARGER_MASK_REG reg);
    BQ2562X_DEFS::CHARGER_MASK_REG getChargerMask();
    uint8_t getChargerMaskRaw();

    /** REG0x25_FAULT_Mask_0 **/
    void setFaultMask(BQ2562X_DEFS::FAULT_MASK_REG reg);
    BQ2562X_DEFS::FAULT_MASK_REG getFaultMask();
    uint8_t getFaultMaskRaw();

    /** REG0x26_ADC_Control **/
    void setADCControl(BQ2562X_DEFS::ADC_CONTROL_REG reg);
    BQ2562X_DEFS::ADC_CONTROL_REG getADCControl();
    uint8_t getADCControlRaw();

    /** REG0x27_ADC_Function_Disable_0 **/
    void setADCFunctionDisable(BQ2562X_DEFS::ADC_FUNCTION_DISABLE_REG reg);
    BQ2562X_DEFS::ADC_FUNCTION_DISABLE_REG getADCFunctionDisable();
    uint8_t getADCFunctionDisableRaw();

    void resetWatchDog();
    void resetRegister();

    uint8_t getPartNumber();
    uint8_t getDeviceRevision();

    // const char *partNumberToString(uint8_t part_number);

private:
    void _initRegisters();

    i2c_master_bus_handle_t _i2cBus;
    i2c_master_dev_handle_t _i2cDevice;

    I2C_Register _regChargeCurrentLimit;
    I2C_Register _regChargeVoltageLimit;
    I2C_Register _regInputCurrentLimit;
    I2C_Register _regInputVoltageLimit;
    I2C_Register _regMinimalSystemVoltage;
    I2C_Register _regPreChargeControl;
    I2C_Register _regTerminationControl;
    I2C_Register _regChargeControl;
    I2C_Register _regChargeTimerControl;
    I2C_Register _regChargerControl;
    I2C_Register _regNTCControl;
    I2C_Register _regChargerStatus;
    I2C_Register _regFaultStatus0;
    I2C_Register _regChargerFlag;
    I2C_Register _regFaultFlag0;
    I2C_Register _regChargerMask;
    I2C_Register _regFaultMask0;
    I2C_Register _regADCControl;
    I2C_Register _regADCFunctionDisable0;
    I2C_Register _regIBUS_ADC;
    I2C_Register _regIBAT_ADC;
    I2C_Register _regVBUS_ADC;
    I2C_Register _regVPMI_DADC;
    I2C_Register _regVBAT_ADC;
    I2C_Register _regVSYS_ADC;
    I2C_Register _regTS_ADC;
    I2C_Register _regTDIE_ADC;
    I2C_Register _regPartInformation;
};

#endif
