#ifndef __LPS22_H__
#define __LPS22_H__

#define LPS2X_I2CADDR_DEFAULT 0x5D ///< LPS2X default i2c address
#define LPS2X_WHOAMI 0x0F          ///< Chip ID register

#define LPS22HB_CHIP_ID 0xB1   ///< LPS22 default device id from WHOAMI
#define LPS22_THS_P_L_REG 0x0C ///< Pressure threshold value for int
#define LPS22_CTRL_REG1 0x10   ///< First control register. Includes BD & ODR
#define LPS22_CTRL_REG2 0x11   ///< Second control register. Includes SW Reset
#define LPS22_CTRL_REG3 \
  0x12 ///< Third control register. Includes interrupt polarity

#define LPS25HB_CHIP_ID 0xBD ///< LPS25HB default device id from WHOAMI
#define LPS25_CTRL_REG1 0x20 ///< First control register. Includes BD & ODR
#define LPS25_CTRL_REG2 0x21 ///< Second control register. Includes SW Reset
#define LPS25_CTRL_REG3 \
  0x22 ///< Third control register. Includes interrupt polarity
#define LPS25_CTRL_REG4 \
  0x23                           ///< Fourth control register. Includes DRDY INT control
#define LPS25_INTERRUPT_CFG 0x24 ///< Interrupt control register
#define LPS25_THS_P_L_REG 0xB0   ///< Pressure threshold value for int

#define LPS2X_PRESS_OUT_XL \
  (0x28 | 0x80)                        ///< | 0x80 to set auto increment on multi-byte read
#define LPS2X_TEMP_OUT_L (0x2B | 0x80) ///< | 0x80 to set auto increment on

#include "driver/i2c_master.h"
#include "esp_err.h"

class LPS22
{
public:
  LPS22(void);
  ~LPS22(void);

  esp_err_t begin(i2c_master_bus_handle_t bus_handle, uint8_t lps22_addr);
  esp_err_t reset(void);
  esp_err_t read(float *pressure, float *temperature);
  esp_err_t setODR(uint8_t odr);
  esp_err_t setBD(uint8_t bd);
  esp_err_t setInterruptPolarity(uint8_t polarity);
  esp_err_t setPressureThreshold(uint8_t threshold);
  esp_err_t enableInterrupt(void);
  esp_err_t disableInterrupt(void);

protected:
  float _pressure,  ///< Last reading's pressure (hPa)
      _temperature; ///< Last reading's temperature (C)

  i2c_master_bus_handle_t i2c_bus; ///< Pointer to I2C bus interface
  i2c_master_dev_handle_t i2c_dev; ///< Pointer to I2C device

private:
  uint16_t default_timeout = 500; // I2C communicate timeout in ms
  uint8_t _lps22_addr;
};

#endif