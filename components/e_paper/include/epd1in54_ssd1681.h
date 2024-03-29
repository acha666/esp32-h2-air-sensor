#ifndef __EPD1IN54_SSD1681_H__
#define __EPD1IN54_SSD1681_H__

#include "e_paper.h"

class EPD1IN54_SSD1681 : public SPI_Epaper
{

public:
    EPD1IN54_SSD1681(gpio_num_t dc_pin, gpio_num_t rst_pin, gpio_num_t busy_pin, spi_device_handle_t spi_device) : SPI_Epaper(dc_pin, rst_pin, busy_pin, spi_device) {}
    ~EPD1IN54_SSD1681();

    void init() override;
    void clearDisplay() override;
    void writeDisplayBuffer(uint8_t *Image);
    void writeDisplayBuffer(uint8_t *image_buffer, int x, int y, int image_width, int image_height);
    void sleep() override;

private:
    uint8_t _displayHeight = 200;
    uint8_t _displayWidth = 200;

    enum class _register : uint8_t
    {
        DRIVER_OUTPUT_CONTROL = 0x01,
        BOOSTER_SOFT_START_CONTROL = 0x0C,
        GATE_SCAN_START_POSITION = 0x0F,
        DEEP_SLEEP_MODE = 0x10,
        DATA_ENTRY_MODE_SETTING = 0x11,
        SW_RESET = 0x12,
        TEMPERATURE_SENSOR_CONTROL = 0x1A,
        MASTER_ACTIVATION = 0x20,
        DISPLAY_UPDATE_CONTROL_1 = 0x21,
        DISPLAY_UPDATE_CONTROL_2 = 0x22,
        WRITE_RAM = 0x24,
        WRITE_VCOM_REGISTER = 0x2C,
        WRITE_LUT_REGISTER = 0x32,
        SET_DUMMY_LINE_PERIOD = 0x3A,
        SET_GATE_TIME = 0x3B,
        BORDER_WAVEFORM_CONTROL = 0x3C,
        SET_RAM_X_ADDRESS_START_END_POSITION = 0x44,
        SET_RAM_Y_ADDRESS_START_END_POSITION = 0x45,
        SET_RAM_X_ADDRESS_COUNTER = 0x4E,
        SET_RAM_Y_ADDRESS_COUNTER = 0x4F,
        TERMINATE_FRAME_READ_WRITE = 0xFF,
    };

    void _sendCommandReg(_register Reg);

    void _turnOnDisplay();
    void _setWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend);
    void _setCursor(uint16_t Xstart, uint16_t Ystart);
};

#endif