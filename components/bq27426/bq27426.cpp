/******************************************************************************
BQ27426.cpp
BQ27426Arduino Library Main Source File

Based on library written by
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27426_Arduino_Library

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/

#include "bq27426.h"
#include "bq27426_defs.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "string.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define TAG "BQ27426"

/*****************************************************************************
 ************************** Initialization Functions *************************
 *****************************************************************************/
// Initializes class variables
BQ27426::BQ27426()
{
}

// Initializes I2C and verifies communication with the BQ27426.
esp_err_t BQ27426::begin(i2c_master_bus_handle_t i2c_master_bus, uint8_t address)
{

    esp_err_t ret;
    uint16_t deviceID = 0;

    _i2cBus = i2c_master_bus;

    ret = i2c_master_probe(_i2cBus, address, 500); // check if the device is connected
    ESP_LOGI(TAG, "Probe result: %d", ret);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(_i2cBus, &dev_cfg, &_i2cDevice);
    if (ret != ESP_OK)
    {
        return ret;
    }

    deviceID = deviceType(); // Read deviceType from BQ27426
    ESP_LOGI(TAG, "Device ID: 0x%04X", deviceID);

    if (deviceID == BQ27426_DEVICE_ID)
    {
        return ESP_OK; // If device ID is valid, return true
    }

    return ESP_ERR_INVALID_RESPONSE; // Otherwise return false
}

// Configures the design capacity of the connected battery.
bool BQ27426::setCapacity(uint16_t capacity)
{
    // Write to STATE subclass (82) of BQ27426 extended memory.
    // Offset 6
    // Design capacity is a 2-byte piece of data - MSB first
    // Unit: mAh
    uint8_t capMSB = capacity >> 8;
    uint8_t capLSB = capacity & 0x00FF;
    uint8_t capacityData[2] = {capMSB, capLSB};
    return writeExtendedData(BQ27426_ID_STATE, 6, capacityData, 2);
}

// Configures the design energy of the connected battery.
bool BQ27426::setDesignEnergy(uint16_t energy)
{
    // Write to STATE subclass (82) of BQ27426 extended memory.
    // Offset 8
    // Design energy is a 2-byte piece of data - MSB first
    // Unit: mWh
    uint8_t enMSB = energy >> 8;
    uint8_t enLSB = energy & 0x00FF;
    uint8_t energyData[2] = {enMSB, enLSB};
    return writeExtendedData(BQ27426_ID_STATE, 8, energyData, 2);
}

// Configures the terminate voltage.
bool BQ27426::setTerminateVoltage(uint16_t voltage)
{
    // Write to STATE subclass (82) of BQ27426 extended memory.
    // Offset 10
    // Termiante voltage is a 2-byte piece of data - MSB first
    // Unit: mV
    // Min 2500, Max 3700
    if (voltage < 2500)
        voltage = 2500;
    if (voltage > 3700)
        voltage = 3700;

    uint8_t tvMSB = voltage >> 8;
    uint8_t tvLSB = voltage & 0x00FF;
    uint8_t tvData[2] = {tvMSB, tvLSB};
    return writeExtendedData(BQ27426_ID_STATE, 10, tvData, 2);
}

// Configures taper rate of connected battery.
bool BQ27426::setTaperRate(uint16_t rate)
{
    // Write to STATE subclass (82) of BQ27426 extended memory.
    // Offset 21
    // Termiante voltage is a 2-byte piece of data - MSB first
    // Unit: 0.1h
    // Max 2000
    if (rate > 2000)
        rate = 2000;
    uint8_t trMSB = rate >> 8;
    uint8_t trLSB = rate & 0x00FF;
    uint8_t trData[2] = {trMSB, trLSB};
    return writeExtendedData(BQ27426_ID_STATE, 21, trData, 2);
}

/*****************************************************************************
 ********************** Battery Characteristics Functions ********************
 *****************************************************************************/

// Reads and returns the battery voltage
uint16_t BQ27426::voltage(void)
{
    return readWord(BQ27426_COMMAND_VOLTAGE);
}

// Reads and returns the specified current measurement
int16_t BQ27426::current(current_measure type)
{
    int16_t current = 0;
    switch (type)
    {
    case AVG:
        current = (int16_t)readWord(BQ27426_COMMAND_AVG_CURRENT);
        break;
    case STBY:
        current = (int16_t)readWord(BQ27426_COMMAND_STDBY_CURRENT);
        break;
    case MAX:
        current = (int16_t)readWord(BQ27426_COMMAND_MAX_CURRENT);
        break;
    }

    return current;
}

// Reads and returns the specified capacity measurement
uint16_t BQ27426::capacity(capacity_measure type)
{
    uint16_t capacity = 0;
    switch (type)
    {
    case REMAIN:
        return readWord(BQ27426_COMMAND_REM_CAPACITY);
        break;
    case FULL:
        return readWord(BQ27426_COMMAND_FULL_CAPACITY);
        break;
    case AVAIL:
        capacity = readWord(BQ27426_COMMAND_NOM_CAPACITY);
        break;
    case AVAIL_FULL:
        capacity = readWord(BQ27426_COMMAND_AVAIL_CAPACITY);
        break;
    case REMAIN_F:
        capacity = readWord(BQ27426_COMMAND_REM_CAP_FIL);
        break;
    case REMAIN_UF:
        capacity = readWord(BQ27426_COMMAND_REM_CAP_UNFL);
        break;
    case FULL_F:
        capacity = readWord(BQ27426_COMMAND_FULL_CAP_FIL);
        break;
    case FULL_UF:
        capacity = readWord(BQ27426_COMMAND_FULL_CAP_UNFL);
        break;
    case DESIGN:
        return readWord(BQ27426_COMMAND_FULL_CAPACITY); // need to fix
        // capacity = readWord(BQ27426_EXTENDED_CAPACITY);
    }

    return capacity;
}

// Reads and returns measured average power
int16_t BQ27426::power(void)
{
    return (int16_t)readWord(BQ27426_COMMAND_AVG_POWER);
}

// Reads and returns specified state of charge measurement
uint16_t BQ27426::soc(soc_measure type)
{
    uint16_t socRet = 0;
    switch (type)
    {
    case FILTERED:
        socRet = readWord(BQ27426_COMMAND_SOC);
        break;
    case UNFILTERED:
        socRet = readWord(BQ27426_COMMAND_SOC_UNFL);
        break;
    }

    return socRet;
}

// Reads and returns specified state of health measurement
uint8_t BQ27426::soh(soh_measure type)
{
    uint16_t sohRaw = readWord(BQ27426_COMMAND_SOH);
    uint8_t sohStatus = sohRaw >> 8;
    uint8_t sohPercent = sohRaw & 0x00FF;

    if (type == PERCENT)
        return sohPercent;
    else
        return sohStatus;
}

// Reads and returns specified temperature measurement
uint16_t BQ27426::temperature(temp_measure type)
{
    uint16_t temp = 0;
    switch (type)
    {
    case BATTERY:
        temp = readWord(BQ27426_COMMAND_TEMP);
        break;
    case INTERNAL_TEMP:
        temp = readWord(BQ27426_COMMAND_INT_TEMP);
        break;
    }
    return temp;
}

/*****************************************************************************
 ************************** GPOUT Control Functions **************************
 *****************************************************************************/
// Get GPOUT polarity setting (active-high or active-low)
bool BQ27426::GPOUTPolarity(void)
{
    uint16_t opConfigRegister = opConfig();

    return (opConfigRegister & BQ27426_OPCONFIG_GPIOPOL);
}

// Set GPOUT polarity to active-high or active-low
bool BQ27426::setGPOUTPolarity(bool activeHigh)
{
    uint16_t oldOpConfig = opConfig();

    // Check to see if we need to update opConfig:
    if ((activeHigh && (oldOpConfig & BQ27426_OPCONFIG_GPIOPOL)) ||
        (!activeHigh && !(oldOpConfig & BQ27426_OPCONFIG_GPIOPOL)))
        return true;

    uint16_t newOpConfig = oldOpConfig;
    if (activeHigh)
        newOpConfig |= BQ27426_OPCONFIG_GPIOPOL;
    else
        newOpConfig &= ~(BQ27426_OPCONFIG_GPIOPOL);

    return writeOpConfig(newOpConfig);
}

// Get GPOUT function (BAT_LOW or SOC_INT)
bool BQ27426::GPOUTFunction(void)
{
    uint16_t opConfigRegister = opConfig();

    return (opConfigRegister & BQ27426_OPCONFIG_BATLOWEN);
}

// Set GPOUT function to BAT_LOW or SOC_INT
bool BQ27426::setGPOUTFunction(gpout_function function)
{
    uint16_t oldOpConfig = opConfig();

    // Check to see if we need to update opConfig:
    if ((function && (oldOpConfig & BQ27426_OPCONFIG_BATLOWEN)) ||
        (!function && !(oldOpConfig & BQ27426_OPCONFIG_BATLOWEN)))
        return true;

    // Modify BATLOWN_EN bit of opConfig:
    uint16_t newOpConfig = oldOpConfig;
    if (function)
        newOpConfig |= BQ27426_OPCONFIG_BATLOWEN;
    else
        newOpConfig &= ~(BQ27426_OPCONFIG_BATLOWEN);

    // Write new opConfig
    return writeOpConfig(newOpConfig);
}

// Get SOC1_Set Threshold - threshold to set the alert flag
uint8_t BQ27426::SOC1SetThreshold(void)
{
    return readExtendedData(BQ27426_ID_DISCHARGE, 0);
}

// Get SOC1_Clear Threshold - threshold to clear the alert flag
uint8_t BQ27426::SOC1ClearThreshold(void)
{
    return readExtendedData(BQ27426_ID_DISCHARGE, 1);
}

// Set the SOC1 set and clear thresholds to a percentage
bool BQ27426::setSOC1Thresholds(uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];
    thresholds[0] = constrain(set, 0, 100);
    thresholds[1] = constrain(clear, 0, 100);
    return writeExtendedData(BQ27426_ID_DISCHARGE, 0, thresholds, 2);
}

// Get SOCF_Set Threshold - threshold to set the alert flag
uint8_t BQ27426::SOCFSetThreshold(void)
{
    return readExtendedData(BQ27426_ID_DISCHARGE, 2);
}

// Get SOCF_Clear Threshold - threshold to clear the alert flag
uint8_t BQ27426::SOCFClearThreshold(void)
{
    return readExtendedData(BQ27426_ID_DISCHARGE, 3);
}

// Set the SOCF set and clear thresholds to a percentage
bool BQ27426::setSOCFThresholds(uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];
    thresholds[0] = constrain(set, 0, 100);
    thresholds[1] = constrain(clear, 0, 100);
    return writeExtendedData(BQ27426_ID_DISCHARGE, 2, thresholds, 2);
}

// Check if the SOC1 flag is set
bool BQ27426::socFlag(void)
{
    uint16_t flagState = flags();

    return flagState & BQ27426_FLAG_SOC1;
}

// Check if the SOCF flag is set
bool BQ27426::socfFlag(void)
{
    uint16_t flagState = flags();

    return flagState & BQ27426_FLAG_SOCF;
}

// Check if the ITPOR flag is set
bool BQ27426::itporFlag(void)
{
    uint16_t flagState = flags();

    return flagState & BQ27426_FLAG_ITPOR;
}

// Check if the FC flag is set
bool BQ27426::fcFlag(void)
{
    uint16_t flagState = flags();

    return flagState & BQ27426_FLAG_FC;
}

// Check if the CHG flag is set
bool BQ27426::chgFlag(void)
{
    uint16_t flagState = flags();

    return flagState & BQ27426_FLAG_CHG;
}

// Check if the DSG flag is set
bool BQ27426::dsgFlag(void)
{
    uint16_t flagState = flags();

    return flagState & BQ27426_FLAG_DSG;
}

// Get the SOC_INT interval delta
uint8_t BQ27426::sociDelta(void)
{
    return readExtendedData(BQ27426_ID_STATE, 26);
}

// Set the SOC_INT interval delta to a value between 1 and 100
bool BQ27426::setSOCIDelta(uint8_t delta)
{
    uint8_t soci = constrain(delta, 0, 100);
    return writeExtendedData(BQ27426_ID_STATE, 26, &soci, 1);
}

// Pulse the GPOUT pin - must be in SOC_INT mode
bool BQ27426::pulseGPOUT(void)
{
    return executeControlWord(BQ27426_CONTROL_PULSE_SOC_INT);
}

/*****************************************************************************
 *************************** Control Sub-Commands ****************************
 *****************************************************************************/

// Read the device type - should be 0x0421
uint16_t BQ27426::deviceType(void)
{
    return readControlWord(BQ27426_CONTROL_DEVICE_TYPE);
}

// Enter configuration mode - set userControl if calling from an Arduino sketch
// and you want control over when to exitConfig
bool BQ27426::enterConfig(bool userControl)
{
    if (userControl)
        _userConfigControl = true;

    if (sealed())
    {
        _sealFlag = true;
        unseal(); // Must be unsealed before making changes
    }

    if (executeControlWord(BQ27426_CONTROL_SET_CFGUPDATE))
    {
        int16_t timeout = BQ72441_I2C_TIMEOUT;
        while ((timeout--) && (!(flags() & BQ27426_FLAG_CFGUPMODE)))
            // delay(1);
            vTaskDelay(1);

        if (timeout > 0)
            return true;
    }

    return false;
}

// Exit configuration mode with the option to perform a resimulation
bool BQ27426::exitConfig(bool resim)
{
    // There are two methods for exiting config mode:
    //    1. Execute the EXIT_CFGUPDATE command
    //    2. Execute the SOFT_RESET command
    // EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
    // measurement, and without resimulating to update unfiltered-SoC and SoC.
    // If a new OCV measurement or resimulation is desired, SOFT_RESET or
    // EXIT_RESIM should be used to exit config mode.
    if (resim)
    {
        if (softReset())
        {
            int16_t timeout = BQ72441_I2C_TIMEOUT;
            while ((timeout--) && ((flags() & BQ27426_FLAG_CFGUPMODE)))
                // delay(1);
                vTaskDelay(1);
            if (timeout > 0)
            {
                if (_sealFlag)
                    seal(); // Seal back up if we IC was sealed coming in
                return true;
            }
        }
        return false;
    }
    else
    {
        return executeControlWord(BQ27426_CONTROL_SOFT_RESET);
    }
}

// Read the flags() command
uint16_t BQ27426::flags(void)
{
    return readWord(BQ27426_COMMAND_FLAGS);
}

// Read the CONTROL_STATUS subcommand of control()
uint16_t BQ27426::status(void)
{
    return readControlWord(BQ27426_CONTROL_STATUS);
}

/***************************** Private Functions *****************************/

// Check if the BQ27426-G1A is sealed or not.
bool BQ27426::sealed(void)
{
    uint16_t stat = status();
    return stat & BQ27426_STATUS_SS;
}

// Seal the BQ27426-G1A
bool BQ27426::seal(void)
{
    return readControlWord(BQ27426_CONTROL_SEALED);
}

// UNseal the BQ27426-G1A
bool BQ27426::unseal(void)
{
    // To unseal the BQ27426, write the key to the control
    // command. Then immediately write the same key to control again.
    if (readControlWord(BQ27426_UNSEAL_KEY))
    {
        return readControlWord(BQ27426_UNSEAL_KEY);
    }
    return false;
}

// // Read the 16-bit opConfig register from extended data
// uint16_t BQ27426::opConfig(void)
// {
//     return readWord(BQ27426_EXTENDED_OPCONFIG);
// }

// Write the 16-bit opConfig register in extended data
bool BQ27426::writeOpConfig(uint16_t value)
{
    uint8_t opConfigMSB = value >> 8;
    uint8_t opConfigLSB = value & 0x00FF;
    uint8_t opConfigData[2] = {opConfigMSB, opConfigLSB};

    // OpConfig register location: BQ27426_ID_REGISTERS id, offset 0
    return writeExtendedData(BQ27426_ID_REGISTERS, 0, opConfigData, 2);
}

// Issue a soft-reset to the BQ27426-G1A
bool BQ27426::softReset(void)
{
    return executeControlWord(BQ27426_CONTROL_SOFT_RESET);
}

// Read a 16-bit command word from the BQ27426-G1A
uint16_t BQ27426::readWord(uint16_t subAddress)
{
    uint8_t data[2];
    i2cReadBytes(subAddress, data, 2);
    return ((uint16_t)data[1] << 8) | data[0];
}

// Read a 16-bit subcommand() from the BQ27426-G1A's control()
uint16_t BQ27426::readControlWord(uint16_t function)
{
    uint8_t subCommandMSB = (function >> 8);
    uint8_t subCommandLSB = (function & 0x00FF);
    uint8_t command[2] = {subCommandLSB, subCommandMSB};
    uint8_t data[2] = {0, 0};

    i2cWriteBytes(BQ27426_COMMAND_CONTROL, command, 2);

    if (i2cReadBytes(BQ27426_COMMAND_CONTROL, data, 2))
    {
        return ((uint16_t)data[1] << 8) | data[0];
    }

    return false;
}

// Execute a subcommand() from the BQ27426-G1A's control()
bool BQ27426::executeControlWord(uint16_t function)
{
    uint8_t subCommandMSB = (function >> 8);
    uint8_t subCommandLSB = (function & 0x00FF);
    uint8_t command[2] = {subCommandLSB, subCommandMSB};
    uint8_t data[2] = {0, 0};

    if (i2cWriteBytes(BQ27426_COMMAND_CONTROL, command, 2))
        return true;

    return false;
}

/*****************************************************************************
 ************************** Extended Data Commands ***************************
 *****************************************************************************/

// Issue a BlockDataControl() command to enable BlockData access
bool BQ27426::blockDataControl(void)
{
    uint8_t enableByte = 0x00;
    return i2cWriteBytes(BQ27426_EXTENDED_CONTROL, &enableByte, 1);
}

// Issue a DataClass() command to set the data class to be accessed
bool BQ27426::blockDataClass(uint8_t id)
{
    return i2cWriteBytes(BQ27426_EXTENDED_DATACLASS, &id, 1);
}

// Issue a DataBlock() command to set the data block to be accessed
bool BQ27426::blockDataOffset(uint8_t offset)
{
    return i2cWriteBytes(BQ27426_EXTENDED_DATABLOCK, &offset, 1);
}

// Read the current checksum using BlockDataCheckSum()
uint8_t BQ27426::blockDataChecksum(void)
{
    uint8_t csum;
    i2cReadBytes(BQ27426_EXTENDED_CHECKSUM, &csum, 1);
    return csum;
}

// Use BlockData() to read a byte from the loaded extended data
uint8_t BQ27426::readBlockData(uint8_t offset)
{
    uint8_t ret;
    uint8_t address = offset + BQ27426_EXTENDED_BLOCKDATA;
    i2cReadBytes(address, &ret, 1);
    return ret;
}

// Use BlockData() to write a byte to an offset of the loaded data
bool BQ27426::writeBlockData(uint8_t offset, uint8_t data)
{
    uint8_t address = offset + BQ27426_EXTENDED_BLOCKDATA;
    return i2cWriteBytes(address, &data, 1);
}

// Read all 32 bytes of the loaded extended data and compute a
// checksum based on the values.
uint8_t BQ27426::computeBlockChecksum(void)
{
    uint8_t data[32];
    i2cReadBytes(BQ27426_EXTENDED_BLOCKDATA, data, 32);

    uint8_t csum = 0;
    for (int i = 0; i < 32; i++)
    {
        csum += data[i];
    }
    csum = 255 - csum;

    return csum;
}

// Use the BlockDataCheckSum() command to write a checksum value
bool BQ27426::writeBlockChecksum(uint8_t csum)
{
    return i2cWriteBytes(BQ27426_EXTENDED_CHECKSUM, &csum, 1);
}

// Read a byte from extended data specifying a class ID and position offset
uint8_t BQ27426::readExtendedData(uint8_t classID, uint8_t offset)
{
    uint8_t retData = 0;
    if (!_userConfigControl)
        enterConfig(false);

    if (!blockDataControl())      // // enable block data memory control
        return false;             // Return false if enable fails
    if (!blockDataClass(classID)) // Write class ID using DataBlockClass()
        return false;

    blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)

    computeBlockChecksum(); // Compute checksum going in
    uint8_t oldCsum = blockDataChecksum();
    /*for (int i=0; i<32; i++)
        Serial.print(String(readBlockData(i)) + " ");*/
    retData = readBlockData(offset % 32); // Read from offset (limit to 0-31)

    if (!_userConfigControl)
        exitConfig();

    return retData;
}

// Write a specified number of bytes to extended data specifying a
// class ID, position offset.
bool BQ27426::writeExtendedData(uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len)
{
    if (len > 32)
        return false;

    if (!_userConfigControl)
        enterConfig(false);

    if (!blockDataControl())      // // enable block data memory control
        return false;             // Return false if enable fails
    if (!blockDataClass(classID)) // Write class ID using DataBlockClass()
        return false;

    blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)
    computeBlockChecksum();       // Compute checksum going in
    uint8_t oldCsum = blockDataChecksum();

    // Write data bytes:
    for (int i = 0; i < len; i++)
    {
        // Write to offset, mod 32 if offset is greater than 32
        // The blockDataOffset above sets the 32-bit block
        writeBlockData((offset % 32) + i, data[i]);
    }

    // Write new checksum using BlockDataChecksum (0x60)
    uint8_t newCsum = computeBlockChecksum(); // Compute the new checksum
    writeBlockChecksum(newCsum);

    if (!_userConfigControl)
        exitConfig();

    return true;
}

/*****************************************************************************
 ************************ I2C Read and Write Routines ************************
 *****************************************************************************/

// Read a specified number of bytes over I2C at a given subAddress
int16_t BQ27426::i2cReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count)
{

    return i2c_master_transmit_receive(_i2cDevice, &subAddress, 1, dest, count, _timeout);
}

// Write a specified number of bytes over I2C to a given subAddress
uint16_t BQ27426::i2cWriteBytes(uint8_t subAddress, uint8_t *src, uint8_t count)
{
    uint8_t write_size = 1 + count;
    uint8_t write_buffer[write_size];
    memcpy(write_buffer, &subAddress, 1);
    memcpy(write_buffer + 1, src, count);

    return i2c_master_transmit(_i2cDevice, write_buffer, write_size, _timeout);
}