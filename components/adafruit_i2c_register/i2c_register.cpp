/**
 * @file i2c_register.cpp
 *
 * @todo Rewrite comments
 */

#include <i2c_register.h>
#include <string.h>

/*!
 *    @brief  Create a register we access over an I2C Device (which defines the
 * bus and address)
 *    @param  i2cdevice The I2CDevice to use for underlying I2C access
 *    @param  reg_addr The address pointer value for the I2C/SMBus register, can
 * be 8 or 16 bits
 *    @param  width    The width of the register data itself, defaults to 1 byte
 *    @param  byteorder The byte order of the register (used when width is > 1),
 * defaults to LSBFIRST
 *    @param  address_width The width of the register address itself, defaults
 * to 1 byte
 */
I2C_Register::I2C_Register(i2c_master_dev_handle_t i2cdevice,
                           uint16_t reg_addr,
                           uint8_t width,
                           I2C_Register_ByteOrder byteorder)
{
    _i2cdevice = i2cdevice;
    _addrwidth = 2;
    _address = reg_addr;
    _byteorder = byteorder;
    _width = width;
}

I2C_Register::I2C_Register(i2c_master_dev_handle_t i2cdevice,
                           uint8_t reg_addr,
                           uint8_t width,
                           I2C_Register_ByteOrder byteorder)
{
    _i2cdevice = i2cdevice;
    _addrwidth = 1;
    _address = reg_addr;
    _byteorder = byteorder;
    _width = width;
}

/*!
 *    @brief  Write a buffer of data to the register location
 *    @param  buffer Pointer to data to write
 *    @param  len Number of bytes to write
 *    @return True on successful write (only really useful for I2C as SPI is
 * uncheckable)
 */
esp_err_t I2C_Register::write(uint8_t *buffer, uint8_t len)
{

    uint8_t addrbuffer[2] = {(uint8_t)(_address & 0xFF),
                             (uint8_t)(_address >> 8)};

    uint8_t write_size = len + _addrwidth;
    uint8_t write_buffer[write_size];
    memcpy(write_buffer, addrbuffer, _addrwidth);
    memcpy(write_buffer + _addrwidth, buffer, len);

    return i2c_master_transmit(_i2cdevice, write_buffer, write_size, _timeout);

    return false;
}

/*!
 *    @brief  Write up to 4 bytes of data to the register location
 *    @param  value Data to write
 *    @param  numbytes How many bytes from 'value' to write
 *    @return True on successful write (only really useful for I2C as SPI is
 * uncheckable)
 */
esp_err_t I2C_Register::write(uint32_t value, uint8_t numbytes)
{
    if (numbytes == 0)
    {
        numbytes = _width;
    }
    if (numbytes > 4)
    {
        return false;
    }

    // store a copy
    _cached = value;

    for (int i = 0; i < numbytes; i++)
    {
        if (_byteorder == LSBFIRST)
        {
            _buffer[i] = value & 0xFF;
        }
        else
        {
            _buffer[numbytes - i - 1] = value & 0xFF;
        }
        value >>= 8;
    }
    return write(_buffer, numbytes);
}

/*!
 *    @brief  Read data from the register location. This does not do any error
 * checking!
 *    @return Returns 0xFFFFFFFF on failure, value otherwise
 */
uint32_t I2C_Register::read(void)
{
    if (read(_buffer, _width) != ESP_OK)
    {
        return -1;
    }

    uint32_t value = 0;

    for (int i = 0; i < _width; i++)
    {
        value <<= 8;
        if (_byteorder == LSBFIRST)
        {
            value |= _buffer[_width - i - 1];
        }
        else
        {
            value |= _buffer[i];
        }
    }

    return value;
}

/*!
 *    @brief  Read cached data from last time we wrote to this register
 *    @return Returns 0xFFFFFFFF on failure, value otherwise
 */
uint32_t I2C_Register::readCached(void) { return _cached; }

/*!
 *    @brief  Read a buffer of data from the register location
 *    @param  buffer Pointer to data to read into
 *    @param  len Number of bytes to read
 *    @return True on successful write (only really useful for I2C as SPI is
 * uncheckable)
 */
esp_err_t I2C_Register::read(uint8_t *buffer, uint8_t len)
{
    uint8_t addrbuffer[2] = {(uint8_t)(_address & 0xFF),
                             (uint8_t)(_address >> 8)};

    return i2c_master_transmit_receive(_i2cdevice, addrbuffer, _addrwidth, buffer, len, _timeout);
}

/*!
 *    @brief  Read 2 bytes of data from the register location
 *    @param  value Pointer to uint16_t variable to read into
 *    @return True on successful write (only really useful for I2C as SPI is
 * uncheckable)
 */
esp_err_t I2C_Register::read(uint16_t *value)
{
    esp_err_t ret = read(_buffer, 2);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (_byteorder == LSBFIRST)
    {
        *value = _buffer[1];
        *value <<= 8;
        *value |= _buffer[0];
    }
    else
    {
        *value = _buffer[0];
        *value <<= 8;
        *value |= _buffer[1];
    }
    return ret;
}

/*!
 *    @brief  Read 1 byte of data from the register location
 *    @param  value Pointer to uint8_t variable to read into
 *    @return True on successful write (only really useful for I2C as SPI is
 * uncheckable)
 */
esp_err_t I2C_Register::read(uint8_t *value)
{
    esp_err_t ret = read(_buffer, 1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *value = _buffer[0];
    return ret;
}

/*!
 *    @brief  Create a slice of the register that we can address without
 * touching other bits
 *    @param  reg The I2C_Register which defines the bus/register
 *    @param  bits The number of bits wide we are slicing
 *    @param  shift The number of bits that our bit-slice is shifted from LSB
 */
I2C_RegisterBits::I2C_RegisterBits(
    I2C_Register *reg, uint8_t bits, uint8_t shift)
{
    _register = reg;
    _bits = bits;
    _shift = shift;
}

/*!
 *    @brief  Read 4 bytes of data from the register
 *    @return  data The 4 bytes to read
 */
uint32_t I2C_RegisterBits::read(void)
{
    uint32_t val = _register->read();
    val >>= _shift;
    return val & ((1 << (_bits)) - 1);
}

/*!
 *    @brief  Write 4 bytes of data to the register
 *    @param  data The 4 bytes to write
 *    @return True on successful write (only really useful for I2C as SPI is
 * uncheckable)
 */
bool I2C_RegisterBits::write(uint32_t data)
{
    uint32_t val = _register->read();

    // mask off the data before writing
    uint32_t mask = (1 << (_bits)) - 1;
    data &= mask;

    mask <<= _shift;
    val &= ~mask;          // remove the current data at that spot
    val |= data << _shift; // and add in the new data

    return _register->write(val, _register->width());
}

/*!
 *    @brief  The width of the register data, helpful for doing calculations
 *    @returns The data width used when initializing the register
 */
uint8_t I2C_Register::width(void) { return _width; }

/*!
 *    @brief  Set the default width of data
 *    @param width the default width of data read from register
 */
void I2C_Register::setWidth(uint8_t width) { _width = width; }

/*!
 *    @brief  Set register address
 *    @param address the address from register
 */
void I2C_Register::setAddress(uint16_t address)
{
    _address = address;
}

/*!
 *    @brief  Set the width of register address
 *    @param address_width the width for register address
 */
void I2C_Register::setAddressWidth(uint16_t address_width)
{
    _addrwidth = address_width;
}
