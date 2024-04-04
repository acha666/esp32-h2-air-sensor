#ifndef __BQ27426_H_
#define __BQ27426_H_

#include "bq27426_defs.h"
#include "i2c_register.h"
#include "driver/i2c_master.h"

class BQ27426
{
public:
    BQ27426(i2c_master_bus_handle_t bus_handle, uint8_t address = 0x55);
    ~BQ27426();

    // control commands
    BQ27426_DEFS::CONTROL_STATUS getControlStatus();
    uint16_t getDeviceType();
    uint16_t getFwVersion();
    uint16_t getChemId();
    void batteryInsert();
    void batteryRemove();
    void setCfgUpdate();
    void reset();
    void softReset();

    // standard commands
    uint16_t getTemperature();
    uint16_t getVoltage();
    BQ27426_DEFS::FLAGS getFlags();
    uint16_t getNominalCapacity();
    uint16_t getFullAvailableCapacity();
    uint16_t getRemainingCapacity();
    uint16_t getFullChargeCapacity();
    uint16_t getAverageCurrent();
    uint16_t getAveragePower();
    uint16_t getStateOfCharge();
    uint16_t getInternalTemperature();
    uint16_t getStateOfHealth();
    uint16_t getRemainingCapacityUnfiltered();
    uint16_t getRemainingCapacityFiltered();
    uint16_t getFullChargeCapacityUnfiltered();

private:
    void _initRegisters();

    int _timeout = 500;
    i2c_master_bus_handle_t _i2cBus;
    i2c_master_dev_handle_t _i2cDevice;

    I2C_Register _commandControl;
    I2C_Register _commandTemperture;
    I2C_Register _commandVoltage;
    I2C_Register _commandFlags;
    I2C_Register _commandNomCapacity;
    I2C_Register _commandAvailCapacity;
    I2C_Register _commandRemCapacity;
    I2C_Register _commandFullCapacity;
    I2C_Register _commandAvgCurrent;
    I2C_Register _commandAvgPower;
    I2C_Register _commandSOC;
    I2C_Register _commandIntTemp;
    I2C_Register _commandSOH;
    I2C_Register _commandRemCapUnfil;
    I2C_Register _commandRemCapFil;
    I2C_Register _commandFullCapUnfil;
    I2C_Register _commandFullCapFil;
};

#endif