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

    uint16_t getDeviceType();

private:
    void _initRegisters();

    int _timeout = 500;
    i2c_master_bus_handle_t _i2cBus;
    i2c_master_dev_handle_t _i2cDevice;

    I2C_Register _commandControl;
    
};

#endif