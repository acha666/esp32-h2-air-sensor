#include "epd1in54_ssd1681.h"
#include "e_paper.h"

#include <stdexcept>

void EPD1IN54_SSD1681::init()
{
    _hardReset();

    _waitIfBusy();
    _sendCommandReg(_register::SW_RESET);
    _waitIfBusy();

    _sendCommandReg(_register::DRIVER_OUTPUT_CONTROL);
    _sendData((_displayHeight - 1) % 256);
    _sendData((_displayHeight - 1) / 256);
    _sendData(0x01);

    _sendCommandReg(_register::DATA_ENTRY_MODE_SETTING);
    _sendData(0x01);

    _setWindow(0, _displayHeight - 1, _displayWidth - 1, 0);

    _sendCommandReg(_register::BORDER_WAVEFORM_CONTROL);
    _sendData(0x05);

    _sendCommand(0x18); // Read built-in temperature sensor
    _sendData(0x80);

    // _sendCommand(0x22); // Load Temperature and waveform setting.
    // _sendData(0XB1);
    // _sendCommand(0x20);

    _setCursor(0, _displayHeight - 1);

    _waitIfBusy();
}

void EPD1IN54_SSD1681::clearDisplay()
{
    uint16_t Width, Height;
    Width = (_displayWidth % 8 == 0) ? (_displayWidth / 8) : (_displayWidth / 8 + 1);
    Height = _displayHeight;

    _sendCommandReg(_register::WRITE_RAM);
    for (uint16_t j = 0; j < Width; j++)
    {
        for (uint16_t i = 0; i < Height; i++)
        {
            _sendData(0XFF);
        }
    }
    _turnOnDisplay();
}

void EPD1IN54_SSD1681::writeDisplayBuffer(uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (_displayWidth % 8 == 0) ? (_displayWidth / 8) : (_displayWidth / 8 + 1);
    Height = _displayHeight;
    uint32_t Addr = 0;

    _sendCommandReg(_register::WRITE_RAM);
    // for (uint16_t j = 0; j < Height; j++)
    // {
    //     for (uint16_t i = 0; i < Width; i++)
    //     {
    //         _sendData(Image[Addr]);
    //         Addr++;
    //     }
    // }
    _sendData(Image, Width * Height);
    _turnOnDisplay();
}


void EPD1IN54_SSD1681::sleep(void)
{
    _sendCommandReg(_register::DEEP_SLEEP_MODE);
    _sendData(0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Private functions

void EPD1IN54_SSD1681::_sendCommandReg(_register Reg)
{
    _sendCommand(static_cast<uint8_t>(Reg));
}

void EPD1IN54_SSD1681::_turnOnDisplay()
{
    _sendCommandReg(_register::DISPLAY_UPDATE_CONTROL_2);
    _sendData(0xf7);
    _sendCommandReg(_register::MASTER_ACTIVATION);
    _waitIfBusy();
}

void EPD1IN54_SSD1681::_setWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    _sendCommandReg(_register::SET_RAM_X_ADDRESS_START_END_POSITION);
    _sendData((Xstart >> 3) & 0xFF);
    _sendData((Xend >> 3) & 0xFF);

    _sendCommandReg(_register::SET_RAM_Y_ADDRESS_START_END_POSITION);
    _sendData(Ystart & 0xFF);
    _sendData((Ystart >> 8) & 0xFF);
    _sendData(Yend & 0xFF);
    _sendData((Yend >> 8) & 0xFF);
}

void EPD1IN54_SSD1681::_setCursor(uint16_t Xstart, uint16_t Ystart)
{
    _sendCommandReg(_register::SET_RAM_X_ADDRESS_COUNTER);
    _sendData(Xstart & 0xFF);

    _sendCommandReg(_register::SET_RAM_Y_ADDRESS_COUNTER);
    _sendData(Ystart & 0xFF);
    _sendData((Ystart >> 8) & 0xFF);
}
