#include "main.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

void vTaskDiplay(void *pvParameters)
{
    // init SPI bus
    spi_bus_config_t bus_config={
        .mosi_io_num=PIN_NUM_MOSI,
        .miso_io_num=PIN_NUM_MISO,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };
    spi_device_interface_config_t dev_config={
        .address_bits = 0,
        .command_bits = 0,
        .mode=0,
        .clock_speed_hz = 10*1000*1000,
        .spics_io_num=,

    };
    
}