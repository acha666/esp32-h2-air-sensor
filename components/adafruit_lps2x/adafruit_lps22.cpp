#include <Adafruit_LPS2X.h>
#include <freertos/FreeRTOS.h>
#include "esp_log.h"

/**
 * @brief Destroy the Adafruit_LPS22::Adafruit_LPS22 object
 *
 */
Adafruit_LPS22::~Adafruit_LPS22(void)
{
  if (temp_sensor)
    delete temp_sensor;
  if (pressure_sensor)
    delete pressure_sensor;

  if (ctrl1_reg)
    delete ctrl1_reg;
  if (ctrl2_reg)
    delete ctrl2_reg;
  if (ctrl3_reg)
    delete ctrl3_reg;
  if (threshp_reg)
    delete threshp_reg;
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
esp_err_t Adafruit_LPS22::_init(int32_t sensor_id)
{

  I2C_Register chip_id = I2C_Register(
      i2c_dev, (uint8_t)LPS2X_WHOAMI, 1);

  // make sure we're talking to the right chip
  uint8_t id;
  ESP_ERROR_CHECK(chip_id.read(&id));

  ESP_LOGI("LPS22", "LPS22 ID: 0x%02x", id);

  if (id != LPS22HB_CHIP_ID && id != LPS22HH_CHIP_ID)
  {
    return ESP_ERR_INVALID_RESPONSE;
  }
  _sensorid_pressure = sensor_id;
  _sensorid_temp = sensor_id + 1;

  temp_scaling = 100;
  temp_offset = 0;

  ctrl1_reg = new I2C_Register(
      i2c_dev, (uint8_t)LPS22_CTRL_REG1, 1);
  ctrl2_reg = new I2C_Register(
      i2c_dev, (uint8_t)LPS22_CTRL_REG2, 1);
  ctrl3_reg = new I2C_Register(
      i2c_dev, (uint8_t)LPS22_CTRL_REG3, 1);
  threshp_reg = new I2C_Register(
      i2c_dev, (uint8_t)LPS22_THS_P_L_REG, 1);

  // ESP_LOGD("LPS22", "Resetting LPS22");
  // reset();
  // ESP_LOGD("LPS22", "Reset complete");
  vTaskDelay(portTICK_PERIOD_MS > 10 ? 1 : pdMS_TO_TICKS(10));
  // do any software reset or other initial setup
  setDataRate(LPS22_RATE_25_HZ);
  // ESP_LOGD("LPS22", "Data rate set");
  // interrupt on data ready
  configureInterrupt(true, false, true);
  // ESP_LOGD("LPS22", "Interrupt configured");

  pressure_sensor = new Adafruit_LPS2X_Pressure(this);
  temp_sensor = new Adafruit_LPS2X_Temp(this);

  vTaskDelay(1); // delay for first reading
  return ESP_OK;
}

/**
 * @brief Sets the rate at which pressure and temperature measurements
 *
 * @param new_data_rate The data rate to set. Must be a `lps22_rate_t`
 */
void Adafruit_LPS22::setDataRate(lps22_rate_t new_data_rate)
{
  I2C_RegisterBits data_rate =
      I2C_RegisterBits(ctrl1_reg, 3, 4);

  data_rate.write((uint8_t)new_data_rate);

  isOneShot = (new_data_rate == LPS22_RATE_ONE_SHOT) ? true : false;
}

/**
 * @brief Gets the current rate at which pressure and temperature measurements
 * are taken
 *
 * @return lps22_rate_t The current data rate
 */
lps22_rate_t Adafruit_LPS22::getDataRate(void)
{
  I2C_RegisterBits data_rate =
      I2C_RegisterBits(ctrl1_reg, 3, 4);

  return (lps22_rate_t)data_rate.read();
}

/**
 * @brief Configures the INT pin
 * @param activelow Pass true to make the INT pin drop low on interrupt
 * @param opendrain Pass true to make the INT pin an open drain output
 * @param data_ready If true, interrupt fires on new data ready
 * @param pres_high If true, interrupt fires on high threshold pass
 * @param pres_low If true, interrupt fires on low threshold pass
 * @param fifo_full If true, interrupt fires on fifo full
 * @param fifo_watermark If true, interrupt fires on fifo watermark pass
 * @param fifo_overflow If true, interrupt fires on fifo overflow
 */
void Adafruit_LPS22::configureInterrupt(bool activelow, bool opendrain,
                                        bool data_ready, bool pres_high,
                                        bool pres_low, bool fifo_full,
                                        bool fifo_watermark,
                                        bool fifo_overflow)
{
  uint8_t reg = (activelow << 7) | (opendrain << 6) | (fifo_full << 5) |
                (fifo_watermark << 4) | (fifo_overflow << 3) |
                (data_ready << 2) | (pres_low << 1) | (pres_high);
  ctrl3_reg->write(reg);
}

uint8_t Adafruit_LPS22::getChipID(void)
{
  I2C_Register chip_id = I2C_Register(
      i2c_dev, (uint8_t)LPS2X_WHOAMI, 1);

  // make sure we're talking to the right chip
  uint8_t id;
  ESP_ERROR_CHECK(chip_id.read(&id));
  return id;
}