/*****************************************************************************
* | File      	:   EPD_1in54_V2.c
* | Author      :   Waveshare team
* | Function    :   1.54inch e-paper V2
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2019-06-11
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "epd1in54.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

// waveform full refresh
unsigned char WF_Full_1IN54[159] =
    {
        0x80, 0x48, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x40, 0x48, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x80, 0x48, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x40, 0x48, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0xA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x8, 0x1, 0x0, 0x8, 0x1, 0x0, 0x2,
        0xA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,
        0x22, 0x17, 0x41, 0x0, 0x32, 0x20};

// waveform partial refresh(fast)
unsigned char WF_PARTIAL_1IN54_0[159] =
    {
        0x0, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x40, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,
        0x02, 0x17, 0x41, 0xB0, 0x32, 0x28};

void EPD1IN54::init(void)
{
    EPD1IN54::reset();

    EPD1IN54::readBusy();
    EPD1IN54::sendCommand(0x12); // SWRESET
    EPD1IN54::readBusy();

    EPD1IN54::sendCommand(0x01); // Driver output control
    EPD1IN54::sendData(0xC7);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x01);

    EPD1IN54::sendCommand(0x11); // data entry mode
    EPD1IN54::sendData(0x01);

    EPD1IN54::setWindows(0, EPD1IN54::_displayHeight - 1, EPD1IN54::_displayWidth - 1, 0);

    EPD1IN54::sendCommand(0x3C); // BorderWavefrom
    EPD1IN54::sendData(0x01);

    EPD1IN54::sendCommand(0x18);
    EPD1IN54::sendData(0x80);

    EPD1IN54::sendCommand(0x22); // //Load Temperature and waveform setting.
    EPD1IN54::sendData(0XB1);
    EPD1IN54::sendCommand(0x20);

    EPD1IN54::setCursor(0, EPD1IN54::_displayHeight - 1);
    EPD1IN54::readBusy();

    EPD1IN54::setLut(WF_Full_1IN54);
}

void EPD1IN54::initPartial(void)
{
    EPD1IN54::reset();
    EPD1IN54::readBusy();

    EPD1IN54::setLut(WF_PARTIAL_1IN54_0);
    EPD1IN54::sendCommand(0x37);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x40);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x00);
    EPD1IN54::sendData(0x00);

    EPD1IN54::sendCommand(0x3C); // BorderWavefrom
    EPD1IN54::sendData(0x80);

    EPD1IN54::sendCommand(0x22);
    EPD1IN54::sendData(0xc0);
    EPD1IN54::sendCommand(0x20);
    EPD1IN54::readBusy();
}

void EPD1IN54::clear(void)
{
    uint16_t Width, Height;
    Width = (EPD1IN54::_displayWidth % 8 == 0) ? (EPD1IN54::_displayWidth / 8) : (EPD1IN54::_displayWidth / 8 + 1);
    Height = EPD1IN54::_displayHeight;

    EPD1IN54::sendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            EPD1IN54::sendData(0XFF);
        }
    }
    EPD1IN54::sendCommand(0x26);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            EPD1IN54::sendData(0XFF);
        }
    }
    EPD1IN54::turnOnDisplay();
}

void EPD1IN54::display(uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (EPD1IN54::_displayWidth % 8 == 0) ? (EPD1IN54::_displayWidth / 8) : (EPD1IN54::_displayWidth / 8 + 1);
    Height = EPD1IN54::_displayHeight;

    double Addr = 0;
    EPD1IN54::sendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            Addr = i + j * Width;
            EPD1IN54::sendData(Image[Addr]);
        }
    }
    EPD1IN54::turnOnDisplay();
}

void EPD1IN54::displayPartBaseImage(uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (EPD1IN54::_displayWidth % 8 == 0) ? (EPD1IN54::_displayWidth / 8) : (EPD1IN54::_displayWidth / 8 + 1);
    Height = EPD1IN54::_displayHeight;

    double Addr = 0;
    EPD1IN54::sendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            Addr = i + j * Width;
            EPD1IN54::sendData(Image[Addr]);
        }
    }
    EPD1IN54::sendCommand(0x26);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            Addr = i + j * Width;
            EPD1IN54::sendData(Image[Addr]);
        }
    }
    EPD1IN54::turnOnDisplayPart();
}

void EPD1IN54::displayPart(uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (EPD1IN54::_displayWidth % 8 == 0) ? (EPD1IN54::_displayWidth / 8) : (EPD1IN54::_displayWidth / 8 + 1);
    Height = EPD1IN54::_displayHeight;

    double Addr = 0;
    EPD1IN54::sendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            Addr = i + j * Width;
            EPD1IN54::sendData(Image[Addr]);
        }
    }
    EPD1IN54::turnOnDisplayPart();
}

void EPD1IN54::sleep(void)
{
    EPD1IN54::sendCommand(0x10); // enter deep sleep
    EPD1IN54::sendData(0x01);
    DEV_Delay_ms(100);
}

// Private functions

void EPD1IN54::reset()
{
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(2);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
}

void EPD1IN54::sendCommand(uint8_t Reg)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

void EPD1IN54::sendData(uint8_t Data)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

void EPD1IN54::readBusy(void)
{
    Debug("e-Paper busy\r\n");
    while (DEV_Digital_Read(EPD_BUSY_PIN) == 1)
    { // LOW: idle, HIGH: busy
        DEV_Delay_ms(10);
    }
    Debug("e-Paper busy release\r\n");
}

void EPD1IN54::turnOnDisplay(void)
{
    sendCommand(0x22);
    sendData(0xc7);
    sendCommand(0x20);
    readBusy();
}

void EPD1IN54::turnOnDisplayPart(void)
{
    sendCommand(0x22);
    sendData(0xcF);
    sendCommand(0x20);
    readBusy();
}

void EPD1IN54::lut(uint8_t *lut)
{
    sendCommand(0x32);
    for (uint8_t i = 0; i < 153; i++)
        sendData(lut[i]);
    readBusy();
}

void EPD1IN54::setLut(uint8_t *lut_value)
{
    lut(lut_value);

    sendCommand(0x3f);
    sendData(lut_value[153]);

    sendCommand(0x03);
    sendData(lut_value[154]);

    sendCommand(0x04);
    sendData(lut_value[155]);
    sendData(lut_value[156]);
    sendData(lut_value[157]);

    sendCommand(0x2c);
    sendData(lut_value[158]);
}

void EPD1IN54::setWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    sendCommand(0x44); // SET_RAM_X_ADDRESS_START_END_POSITION
    sendData((Xstart >> 3) & 0xFF);
    sendData((Xend >> 3) & 0xFF);

    sendCommand(0x45); // SET_RAM_Y_ADDRESS_START_END_POSITION
    sendData(Ystart & 0xFF);
    sendData((Ystart >> 8) & 0xFF);
    sendData(Yend & 0xFF);
    sendData((Yend >> 8) & 0xFF);
}

void EPD1IN54::setCursor(uint16_t Xstart, uint16_t Ystart)
{
    sendCommand(0x4E); // SET_RAM_X_ADDRESS_COUNTER
    sendData(Xstart & 0xFF);

    sendCommand(0x4F); // SET_RAM_Y_ADDRESS_COUNTER
    sendData(Ystart & 0xFF);
    sendData((Ystart >> 8) & 0xFF);
}
