/*!
 *  @file Adafruit_SHT4x.cpp
 *
 *  @mainpage Adafruit SHT4x Digital Humidity & Temp Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the SHT4x Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the SHT4x Digital sensor from Adafruit
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/4885
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "sht4x.h"

static uint8_t crc8(const uint8_t *data, int len);

/*!
 * @brief  SHT4x constructor
 */
Adafruit_SHT4x::Adafruit_SHT4x(void) {}

/*!
 * @brief  SHT4x destructor
 */
Adafruit_SHT4x::~Adafruit_SHT4x(void)
{
    i2c_master_bus_rm_device(i2c_dev);
}

/**
 * Initialises the I2C bus, and assigns the I2C address to us.
 *
 * @param bus_handle  The handle of the I2C bus to use for communication.
 * @param sht4x_addr  The 7-bit I2C address of the sensor.
 *
 * @return ESP_OK if initialisation was successful, otherwise the error code.
 */
esp_err_t Adafruit_SHT4x::begin(i2c_master_bus_handle_t bus_handle, uint8_t sht4x_addr)
{
    esp_err_t ret;

    i2c_bus = bus_handle;

    ret = i2c_master_probe(i2c_bus, sht4x_addr, default_timeout / portTICK_PERIOD_MS); // check if the device is connected
    if (ret != ESP_OK)
    {
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = sht4x_addr,
        .scl_speed_hz = 100000,
    };

    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}

/**
 * Gets the ID register contents.
 *
 * @return The 32-bit ID register.
 */
uint32_t Adafruit_SHT4x::readSerial(void)
{
    uint8_t cmd = SHT4x_READSERIAL;
    uint8_t reply[6];
    esp_err_t ret;

    ret = i2c_master_transmit(i2c_dev, &cmd, 1, default_timeout / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return false;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    ret = i2c_master_receive(i2c_dev, reply, 6, default_timeout / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return false;
    }

    if ((crc8(reply, 2) != reply[2]) || (crc8(reply + 3, 2) != reply[5]))
    {
        return false;
    }

    uint32_t serial = 0;
    serial = reply[0];
    serial <<= 8;
    serial |= reply[1];
    serial <<= 8;
    serial |= reply[3];
    serial <<= 8;
    serial |= reply[4];

    return serial;
}

/**
 * Performs a soft reset of the sensor to put it into a known state.
 * @returns ESP_OK on success
 */
esp_err_t Adafruit_SHT4x::reset(void)
{
    uint8_t cmd = SHT4x_SOFTRESET;
    esp_err_t ret;
    ret = i2c_master_transmit(i2c_dev, &cmd, 1, default_timeout / portTICK_PERIOD_MS);
    vTaskDelay(1); // delay 1 tick to ensure the reset is complete
    return ret;
}

/**************************************************************************/
/*!
    @brief  Sets the precision rating - more precise takes longer!
    @param  prec The desired precision setting, will be used during reads
*/
/**************************************************************************/
void Adafruit_SHT4x::setPrecision(sht4x_precision_t prec) { _precision = prec; }

/**************************************************************************/
/*!
    @brief  Gets the precision rating - more precise takes longer!
    @returns  The current precision setting, will be used during reads
*/
/**************************************************************************/
sht4x_precision_t Adafruit_SHT4x::getPrecision(void) { return _precision; }

/**************************************************************************/
/*!
    @brief  Sets the heating setting - more heating uses more power and takes
   longer
    @param  heat The desired heater setting, will be used during reads
*/
/**************************************************************************/
void Adafruit_SHT4x::setHeater(sht4x_heater_t heat) { _heater = heat; }

/**************************************************************************/
/*!
    @brief  Gets the heating setting - more heating uses more power and takes
   longer
    @returns  The current heater setting, will be used during reads
*/
/**************************************************************************/
sht4x_heater_t Adafruit_SHT4x::getHeater(void) { return _heater; }

/**************************************************************************/
/*!
    @brief  Gets the humidity and temperature from the sensor
    @param  humidity Pointer to the location where the humidity will be written
    @param  temp Pointer to the location where the temperature will be written
    @returns ESP_OK if the event data was read successfully
*/
/**************************************************************************/
esp_err_t Adafruit_SHT4x::read(float *humidity,
                               float *temp)
{

    uint8_t readbuffer[6];
    uint8_t cmd = SHT4x_NOHEAT_HIGHPRECISION;
    uint16_t duration = 10;
    esp_err_t ret;

    if (_heater == SHT4X_NO_HEATER)
    {
        if (_precision == SHT4X_HIGH_PRECISION)
        {
            cmd = SHT4x_NOHEAT_HIGHPRECISION;
            duration = 1; // 10ms -> 1tick
        }
        if (_precision == SHT4X_MED_PRECISION)
        {
            cmd = SHT4x_NOHEAT_MEDPRECISION;
            duration = 1; // 5ms -> 1tick
        }
        if (_precision == SHT4X_LOW_PRECISION)
        {
            cmd = SHT4x_NOHEAT_LOWPRECISION;
            duration = 1; // 2ms -> 1tick
        }
    }

    if (_heater == SHT4X_HIGH_HEATER_1S)
    {
        cmd = SHT4x_HIGHHEAT_1S;
        duration = 11; // 1100ms -> 11ticks
    }
    if (_heater == SHT4X_HIGH_HEATER_100MS)
    {
        cmd = SHT4x_HIGHHEAT_100MS;
        duration = 2; // 110ms -> 2ticks
    }

    if (_heater == SHT4X_MED_HEATER_1S)
    {
        cmd = SHT4x_MEDHEAT_1S;
        duration = 11; // 1100ms -> 11ticks
    }
    if (_heater == SHT4X_MED_HEATER_100MS)
    {
        cmd = SHT4x_MEDHEAT_100MS;
        duration = 2; // 110ms -> 2ticks
    }

    if (_heater == SHT4X_LOW_HEATER_1S)
    {
        cmd = SHT4x_LOWHEAT_1S;
        duration = 11; // 1100ms -> 11ticks
    }
    if (_heater == SHT4X_LOW_HEATER_100MS)
    {
        cmd = SHT4x_LOWHEAT_100MS;
        duration = 2; // 110ms -> 2ticks
    }

    ret = i2c_master_transmit(i2c_dev, &cmd, 1, default_timeout / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    vTaskDelay(duration);

    ret = i2c_master_receive(i2c_dev, readbuffer, 6, default_timeout / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (readbuffer[2] != crc8(readbuffer, 2) ||
        readbuffer[5] != crc8(readbuffer + 3, 2))
        return ESP_ERR_INVALID_CRC;

    float t_ticks = (uint16_t)readbuffer[0] * 256 + (uint16_t)readbuffer[1];
    float rh_ticks = (uint16_t)readbuffer[3] * 256 + (uint16_t)readbuffer[4];
    _temperature = -45 + 175 * t_ticks / 65535;
    _humidity = -6 + 125 * rh_ticks / 65535;

    if (_humidity > 100.0)
    {
        _humidity = 100.0;
    }
    if (_humidity < 0.0)
    {
        _humidity = 0.0;
    }

    *temp = _temperature;
    *humidity = _humidity;
    return ESP_OK;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int len)
{
    /*
     *
     * CRC-8 formula from page 14 of SHT spec pdf
     *
     * Test data 0xBE, 0xEF should yield 0x92
     *
     * Initialization data 0xFF
     * Polynomial 0x31 (x8 + x5 +x4 +1)
     * Final XOR 0x00
     */

    const uint8_t POLYNOMIAL(0x31);
    uint8_t crc(0xFF);

    for (int j = len; j; --j)
    {
        crc ^= *data++;

        for (int i = 8; i; --i)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}