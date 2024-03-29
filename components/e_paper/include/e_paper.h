#ifndef __EPD1IN54_H__
#define __EPD1IN54_H__

#include "esp_types.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

class SPI_Epaper
{
public:
    SPI_Epaper(gpio_num_t dc_pin, gpio_num_t rst_pin, gpio_num_t busy_pin, spi_device_handle_t spi_device);
    ~SPI_Epaper();

    virtual void init() = 0;
    virtual void clearDisplay() = 0;
    virtual void sleep() = 0;

protected:
    const uint16_t _busyTimeout = 5000; // in ms

    spi_device_handle_t _spiDev;

    gpio_num_t _csPin;
    gpio_num_t _dcPin;
    gpio_num_t _rstPin;
    gpio_num_t _busyPin;

    void _hardReset();
    void _sendCommand(uint8_t Reg);
    void _sendData(uint8_t Data);
    void _sendData(uint8_t *Data, uint16_t Length);

    void _waitIfBusy(void);
};

#endif
