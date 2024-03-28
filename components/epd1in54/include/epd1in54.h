#ifndef __EPD1IN54_H__
#define __EPD1IN54_H__

#include "esp_types.h"
#include "driver/gpio.h"

class EPD1IN54
{
public:
    EPD1IN54();
    ~EPD1IN54();

    void init(void);
    void initPartial(void);
    void clear(void);
    void display(uint8_t *Image);
    void displayPartBaseImage(uint8_t *Image);
    void displayPart(uint8_t *Image);
    void sleep(void);
private:
    uint8_t _displayHeight=200;
    uint8_t _displayWidth=200;

    gpio_num_t _csPin;
    gpio_num_t _dcPin;
    gpio_num_t _rstPin;
    gpio_num_t _busyPin;

    void reset();
    void sendCommand(uint8_t Reg);
    void sendData(uint8_t Data);
    void readBusy(void);
    void turnOnDisplay(void);
    void turnOnDisplayPart(void);
    void lut(uint8_t *lut);
    void setLut(uint8_t *lut);
    void setWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend);
    void setCursor(uint16_t Xstart, uint16_t Ystart);
};

#endif
