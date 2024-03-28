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
#include "e_paper.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "string.h"
#include <stdexcept>

using std::runtime_error;

EPD1IN54::EPD1IN54(gpio_num_t dc_pin, gpio_num_t rst_pin, gpio_num_t busy_pin, spi_device_handle_t spi_device)
{
    esp_err_t ret;
    ret = gpio_set_direction(dc_pin, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
        throw runtime_error("Failed to set direction for DC pin");

    ret = gpio_set_direction(rst_pin, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
        throw runtime_error("Failed to set direction for RST pin");

    ret = gpio_set_direction(busy_pin, GPIO_MODE_INPUT);
    if (ret != ESP_OK)
        throw runtime_error("Failed to set direction for BUSY pin");

    _dcPin = dc_pin;
    _rstPin = rst_pin;
    _busyPin = busy_pin;

    _spiDev = spi_device;

    gpio_set_level(_dcPin, 0);
    gpio_set_level(_rstPin, 1);
}

EPD1IN54::~EPD1IN54()
{
    spi_bus_remove_device(_spiDev);
}

void EPD1IN54::init(void)
{
    _hardReset();

    _waitIfBusy();
    _sendCommand(0x12); // SWRESET
    _waitIfBusy();

    _sendCommand(0x01); // Driver output control
    _sendData((_displayHeight - 1) % 256);
    _sendData((_displayHeight - 1) / 256);
    _sendData(0x01);

    _sendCommand(0x11); // data entry mode
    _sendData(0x01);

    _setWindow(0, _displayHeight - 1, _displayWidth - 1, 0);

    _sendCommand(0x3C); // BorderWavefrom
    _sendData(0x05);

    _sendCommand(0x18); // Read built-in temperature sensor
    _sendData(0x80);

    // _sendCommand(0x22); // Load Temperature and waveform setting.
    // _sendData(0XB1);
    // _sendCommand(0x20);

    _setCursor(0, _displayHeight - 1);

    _waitIfBusy();

    // setLut(WF_Full_1IN54);
}

void EPD1IN54::initPartial(void)
{
    _hardReset();
    _waitIfBusy();

    // setLut(WF_PARTIAL_1IN54_0);
    _sendCommand(0x37);
    _sendData(0x00);
    _sendData(0x00);
    _sendData(0x00);
    _sendData(0x00);
    _sendData(0x00);
    _sendData(0x40);
    _sendData(0x00);
    _sendData(0x00);
    _sendData(0x00);
    _sendData(0x00);

    _sendCommand(0x3C); // BorderWavefrom
    _sendData(0x80);

    _sendCommand(0x22);
    _sendData(0xc0);
    _sendCommand(0x20);
    _waitIfBusy();
}

void EPD1IN54::clear(void)
{
    uint16_t Width, Height;
    Width = (_displayWidth % 8 == 0) ? (_displayWidth / 8) : (_displayWidth / 8 + 1);
    Height = _displayHeight;

    _sendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            _sendData(0XFF);
        }
    }
    _sendCommand(0x26);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            _sendData(0XFF);
        }
    }
    _turnOnDisplay();
}

void EPD1IN54::display(uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (_displayWidth % 8 == 0) ? (_displayWidth / 8) : (_displayWidth / 8 + 1);
    Height = _displayHeight;

    uint64_t Addr = 0;
    _sendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            Addr = i + j * Width;
            _sendData(Image[Addr]);
        }
    }
    _turnOnDisplay();
}

void EPD1IN54::displayPartBaseImage(uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (_displayWidth % 8 == 0) ? (_displayWidth / 8) : (_displayWidth / 8 + 1);
    Height = _displayHeight;

    uint64_t Addr = 0;
    _sendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            Addr = i + j * Width;
            _sendData(Image[(int)Addr]);
        }
    }
    _sendCommand(0x26);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            Addr = i + j * Width;
            _sendData(Image[(int)Addr]);
        }
    }
    _turnOnDisplayPart();
}

void EPD1IN54::displayPart(uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (_displayWidth % 8 == 0) ? (_displayWidth / 8) : (_displayWidth / 8 + 1);
    Height = _displayHeight;

    uint64_t Addr = 0;
    _sendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++)
    {
        for (uint16_t i = 0; i < Width; i++)
        {
            Addr = i + j * Width;
            _sendData(Image[(int)Addr]);
        }
    }
    _turnOnDisplayPart();
}

void EPD1IN54::sleep(void)
{
    _sendCommand(0x10); // enter deep sleep
    _sendData(0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Private functions

void EPD1IN54::_hardReset()
{
    gpio_set_level(_rstPin, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(_rstPin, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void EPD1IN54::
_sendCommand(uint8_t Reg)
{
    gpio_set_level(_dcPin, 0);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_data[0] = Reg;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

    spi_device_polling_transmit(_spiDev, &t);
}

void EPD1IN54::_sendData(uint8_t Data)
{
    gpio_set_level(_dcPin, 1);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_data[0] = Data;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

    spi_device_polling_transmit(_spiDev, &t);
}

void EPD1IN54::_sendData(uint8_t *Data, uint16_t Length)
{
    gpio_set_level(_dcPin, 1);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = Length * 8;
    t.tx_buffer = Data;
    t.rx_buffer = NULL;

    spi_device_polling_transmit(_spiDev, &t);
}

void EPD1IN54::_waitIfBusy(void)
{
    while (gpio_get_level(_busyPin) == 1)
    { // LOW: idle, HIGH: busy
        vTaskDelay(1);
    }
}

void EPD1IN54::_turnOnDisplay(void)
{
    _sendCommand(0x22);
    _sendData(0xc7);
    _sendCommand(0x20);
    _waitIfBusy();
}

void EPD1IN54::_turnOnDisplayPart(void)
{
    _sendCommand(0x22);
    _sendData(0xcF);
    _sendCommand(0x20);
    _waitIfBusy();
}

// void EPD1IN54::lut(uint8_t *lut)
// {
//     sendCommand(0x32);
//     for (uint8_t i = 0; i < 153; i++)
//         sendData(lut[i]);
//     _waitIfBusy();
// }

// void EPD1IN54::setLut(uint8_t *lut_value)
// {
//     lut(lut_value);

//     sendCommand(0x3f);
//     sendData(lut_value[153]);

//     sendCommand(0x03);
//     sendData(lut_value[154]);

//     sendCommand(0x04);
//     sendData(lut_value[155]);
//     sendData(lut_value[156]);
//     sendData(lut_value[157]);

//     sendCommand(0x2c);
//     sendData(lut_value[158]);
// }

void EPD1IN54::_setWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    _sendCommand(0x44); // SET_RAM_X_ADDRESS_START_END_POSITION
    _sendData((Xstart >> 3) & 0xFF);
    _sendData((Xend >> 3) & 0xFF);

    _sendCommand(0x45); // SET_RAM_Y_ADDRESS_START_END_POSITION
    _sendData(Ystart & 0xFF);
    _sendData((Ystart >> 8) & 0xFF);
    _sendData(Yend & 0xFF);
    _sendData((Yend >> 8) & 0xFF);
}

void EPD1IN54::_setCursor(uint16_t Xstart, uint16_t Ystart)
{
    _sendCommand(0x4E); // SET_RAM_X_ADDRESS_COUNTER
    _sendData(Xstart & 0xFF);

    _sendCommand(0x4F); // SET_RAM_Y_ADDRESS_COUNTER
    _sendData(Ystart & 0xFF);
    _sendData((Ystart >> 8) & 0xFF);
}
