#ifndef __I2C_REGISTER_H__
#define __I2C_REGISTER_H__

#include "driver/i2c_master.h"

enum I2C_Register_ByteOrder
{
    LSBFIRST,
    MSBFIRST
};

/*!
 * @brief The class which defines a device register (a location to read/write
 * data from)
 */
class I2C_Register
{
public:
    I2C_Register(); // default constructor, DO NOT USE
    I2C_Register(i2c_master_dev_handle_t i2cdevice, uint8_t reg_addr,
                 uint8_t width = 1, I2C_Register_ByteOrder byteorder = LSBFIRST);
    I2C_Register(i2c_master_dev_handle_t i2cdevice, uint8_t *address, uint8_t addrwidth,
                 uint8_t width, I2C_Register_ByteOrder byteorder = LSBFIRST);

    esp_err_t read(uint8_t *buffer, uint8_t len);
    esp_err_t read(uint8_t *value);
    esp_err_t read(uint16_t *value);
    uint32_t read(void);
    uint32_t readCached(void);
    esp_err_t write(uint8_t *buffer, uint8_t len);
    esp_err_t write(uint32_t value, uint8_t numbytes = 0);

    uint8_t width(void);

    void setWidth(uint8_t width);
    void setAddress(uint8_t *address, uint8_t addrwidth);
    void setAddressWidth(uint16_t address_width);

private:
    i2c_master_dev_handle_t _i2cdevice;
    uint8_t _address[4];
    uint8_t _width, _addrwidth, _byteorder;
    uint8_t _buffer[4];

    uint32_t _cached = 0;
    uint16_t _timeout = 500; // 500ms i2c timeout
};

/*!
 * @brief The class which defines a slice of bits from within a device register
 * (a location to read/write data from)
 */
class I2C_RegisterBits
{
public:
    I2C_RegisterBits(I2C_Register *reg, uint8_t bits,
                     uint8_t shift);
    bool write(uint32_t value);
    uint32_t read(void);

private:
    I2C_Register *_register;
    uint8_t _bits, _shift;
};

#endif
