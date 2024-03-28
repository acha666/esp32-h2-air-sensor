#ifndef __EPD1IN54_H__
#define __EPD1IN54_H__

#include "esp_types.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

class EPD1IN54
{
public:
    EPD1IN54(gpio_num_t dc_pin, gpio_num_t rst_pin, gpio_num_t busy_pin, spi_device_handle_t spi_device);
    ~EPD1IN54();

    void init(void);
    void initPartial(void);
    void clear(void);
    void display(uint8_t *Image);
    void displayPartBaseImage(uint8_t *Image);
    void displayPart(uint8_t *Image);
    void sleep(void);

private:
    uint8_t _displayHeight = 200;
    uint8_t _displayWidth = 200;

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
    void _turnOnDisplay(void);
    void _turnOnDisplayPart(void);
    // void lut(uint8_t *lut);
    // void setLut(uint8_t *lut);
    void _setWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend);
    void _setCursor(uint16_t Xstart, uint16_t Ystart);
};

#endif
