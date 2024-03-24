#include "esp_log.h"
#include "driver/i2c_master.h"
#include "main.h"
#include "bq27426.h"
#include "bq2562x.h"
#include "bq2562x_defs.h"
#include <iostream>

static void power_i2c_init(void);

static const char *TAG = "Power";
static i2c_master_bus_handle_t power_i2c_master_bus_handle;

BQ27426 *BatteryGauge;
BQ2562x *Charger;

extern "C" void PowerTask(void *pvParameters)
{
    power_i2c_init();
    assert(xSemaphoreTake(xI2CSemaphore, 5000 / portTICK_PERIOD_MS) == pdTRUE);
    Charger = new BQ2562x(power_i2c_master_bus_handle, 0x6a);
    BatteryGauge = new BQ27426(power_i2c_master_bus_handle);

    uint32_t val = Charger->getChargeCurrent();
    ESP_LOGI(TAG, "Charge Current: %ld mA", val);

    Charger->resetRegister();

    xSemaphoreGive(xI2CSemaphore);
    vTaskDelete(NULL);
}

static void power_i2c_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = GPIO_NUM_2,
        .scl_io_num = GPIO_NUM_3,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    // i2c_mst_config.flags.enable_internal_pullup = true; // to be deleted

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &power_i2c_master_bus_handle));

    return;
}

static void print_charger_control(BQ2562X_DEFS::CHARGER_CONTROL_REG chargerControl)
{
    printf("EN_AUTO_IBATDIS = 0x%02x\n", chargerControl.EN_AUTO_IBATDIS.value());
    printf("FORCE_IBATDIS = 0x%02x\n", chargerControl.FORCE_IBATDIS.value());
    printf("EN_CHG = 0x%02x\n", chargerControl.EN_CHG.value());
    printf("EN_HIZ = 0x%02x\n", chargerControl.EN_HIZ.value());
    printf("FORCE_PMID_DIS = 0x%02x\n", chargerControl.FORCE_PMID_DIS.value());
    printf("WD_RST = 0x%02x\n", chargerControl.WD_RST.value());
    printf("WATCHDOG = 0x%02x\n", chargerControl.WATCHDOG.value());
    printf("REG_RST = 0x%02x\n", chargerControl.REG_RST.value());
    printf("TREG = 0x%02x\n", chargerControl.TREG.value());
    printf("SET_CONV_FREQ = 0x%02x\n", chargerControl.SET_CONV_FREQ.value());
    printf("SET_CONV_STRN = 0x%02x\n", chargerControl.SET_CONV_STRN.value());
    printf("VBUS_OVP = 0x%02x\n", chargerControl.VBUS_OVP.value());
    printf("PFM_FWD_DIS = 0x%02x\n", chargerControl.PFM_FWD_DIS.value());
    printf("BATFET_CTRL_WVBUS = 0x%02x\n", chargerControl.BATFET_CTRL_WVBUS.value());
    printf("BATFET_DLY = 0x%02x\n", chargerControl.BATFET_DLY.value());
    printf("BATFET_CTRL = 0x%02x\n", chargerControl.BATFET_CTRL.value());
    printf("IBAT_PK = 0x%02x\n", chargerControl.IBAT_PK.value());
    printf("VBAT_UVLO = 0x%02x\n", chargerControl.VBAT_UVLO.value());
    printf("EN_EXTILIM = 0x%02x\n", chargerControl.EN_EXTILIM.value());
    printf("CHG_RATE = 0x%02x\n", chargerControl.CHG_RATE.value());
}

extern "C" void BQScanTask()
{
    i2c_master_bus_handle_t tool_bus_handle;
    i2c_port_t i2c_port = I2C_NUM_0;

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = GPIO_NUM_2,
        .scl_io_num = GPIO_NUM_3,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &tool_bus_handle) != ESP_OK);

    esp_err_t ret = i2c_master_probe(tool_bus_handle, 0x55, 50);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "BQ27441 detected");
    }
    else
    {
        ESP_LOGE(TAG, "BQ27441 not detected");
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(tool_bus_handle));
}