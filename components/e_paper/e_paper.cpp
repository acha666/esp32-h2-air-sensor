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
#include "esp_log.h"

#include "string.h"
#include <stdexcept>

using std::runtime_error;

SPI_Epaper::SPI_Epaper(gpio_num_t dc_pin, gpio_num_t rst_pin, gpio_num_t busy_pin, spi_device_handle_t spi_device)
{
    esp_err_t ret;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << rst_pin) | (1ULL << dc_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
        throw runtime_error("Failed to configure RST and DC pin");

    io_conf.pin_bit_mask = (1ULL << busy_pin);
    io_conf.mode = GPIO_MODE_INPUT;

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
        throw runtime_error("Failed to configure BUSY pin");

    _dcPin = dc_pin;
    _rstPin = rst_pin;
    _busyPin = busy_pin;

    _spiDev = spi_device;

    gpio_set_level(_rstPin, 1);
}

SPI_Epaper::~SPI_Epaper()
{
    spi_bus_remove_device(_spiDev);
}

// Private functions

void SPI_Epaper::_hardReset()
{
    gpio_set_level(_rstPin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(_rstPin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void SPI_Epaper::_sendCommand(uint8_t Reg)
{
    gpio_set_level(_dcPin, 0);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_data[0] = Reg;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

    spi_device_polling_transmit(_spiDev, &t);
}

void SPI_Epaper::_sendData(uint8_t Data)
{
    gpio_set_level(_dcPin, 1);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_data[0] = Data;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

    spi_device_polling_transmit(_spiDev, &t);
}

void SPI_Epaper::_sendData(uint8_t *Data, uint16_t Length)
{
    gpio_set_level(_dcPin, 1);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = Length * 8;
    t.tx_buffer = Data;
    t.rx_buffer = NULL;

    spi_device_polling_transmit(_spiDev, &t);
}

void SPI_Epaper::_waitIfBusy(void)
{

    for (int i = 0; i < _busyTimeout / 100; i++)
    {
        vTaskDelay(_busyTimeout / 100 / portTICK_PERIOD_MS);
        if (gpio_get_level(_busyPin) == 0)
        {
            break;
        }

        if (i == _busyTimeout / 100 - 1)
        {
            ESP_LOGE("E_Paper_Base", "Busy Timeout");
        }
    }
}
