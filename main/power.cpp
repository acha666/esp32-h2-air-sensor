#include "esp_log.h"
#include "driver/i2c_master.h"
#include <stdexcept>

#include "bq27426.h"
#include "bq27426_defs.h"
#include "bq2562x.h"
#include "bq2562x_defs.h"

#include "main.h"

static void power_i2c_init(void);

static constexpr char TAG[] = "Power";
static i2c_master_bus_handle_t power_i2c_master_bus_handle;
static bool CHARGER_FAULT_FLAG = false;
static bool GAUGE_FAULT_FLAG = false;

BQ27426 *BatteryGauge;
BQ2562x *Charger;

using std::runtime_error;

extern "C" void PowerTask(void *pvParameters)
{
    power_i2c_init();

    try
    {
        Charger = new BQ2562x(power_i2c_master_bus_handle, 0x6a);
    }
    catch (const runtime_error &e)
    {
        CHARGER_FAULT_FLAG = true;
        ESP_LOGE(TAG, "Charger init failed: %s", e.what());
    }

    try
    {
        BatteryGauge = new BQ27426(power_i2c_master_bus_handle);
    }
    catch (const runtime_error &e)
    {
        GAUGE_FAULT_FLAG = true;
        ESP_LOGE(TAG, "Battery Gauge init failed: %s", e.what());
    }

    if (!CHARGER_FAULT_FLAG && !GAUGE_FAULT_FLAG) // for testing
    {
        uint32_t val = Charger->getChargeCurrent();
        ESP_LOGI(TAG, "Charge Current: %ld mA", val);

        ESP_LOGI(TAG, "Bat Temp: %d", BatteryGauge->getTemperature());
        ESP_LOGI(TAG, "Bat Voltage: %d", BatteryGauge->getVoltage());
    }

    if (!CHARGER_FAULT_FLAG && !GAUGE_FAULT_FLAG)
    {
        // init charger and gauge(set some values)
    }

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

static void print_charger_charger_control(BQ2562X_DEFS::CHARGER_CONTROL_REG chargerControl)
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

static void print_gauge_control_status(BQ27426_DEFS::CONTROL_STATUS controlStatus)
{
    printf("SHUTDOWNEN = 0x%02x\n", controlStatus.SHUTDOWNEN);
    printf("WDRESET = 0x%02x\n", controlStatus.WDRESET);
    printf("SS = 0x%02x\n", controlStatus.SS);
    printf("CALMODE = 0x%02x\n", controlStatus.CALMODE);
    printf("CCA = 0x%02x\n", controlStatus.CCA);
    printf("BCA = 0x%02x\n", controlStatus.BCA);
    printf("QMAX_UP = 0x%02x\n", controlStatus.QMAX_UP);
    printf("RES_UP = 0x%02x\n", controlStatus.RES_UP);
    printf("INITCOMP = 0x%02x\n", controlStatus.INITCOMP);
    printf("SLEEP = 0x%02x\n", controlStatus.SLEEP);
    printf("LDMD = 0x%02x\n", controlStatus.LDMD);
    printf("RUP_DIS = 0x%02x\n", controlStatus.RUP_DIS);
    printf("VOK = 0x%02x\n", controlStatus.VOK);
    printf("CHEM_CHANGE = 0x%02x\n", controlStatus.CHEM_CHANGE);
}

static void print_gauge_flags(BQ27426_DEFS::FLAGS flags)
{
    printf("OT = 0x%02x\n", flags.OT);
    printf("UT = 0x%02x\n", flags.UT);
    printf("FC = 0x%02x\n", flags.FC);
    printf("CHG = 0x%02x\n", flags.CHG);
    printf("OCVTAKEN = 0x%02x\n", flags.OCVTAKEN);
    printf("DOD_CORRECT = 0x%02x\n", flags.DOD_CORRECT);
    printf("ITPOR = 0x%02x\n", flags.ITPOR);
    printf("CFGUPMODE = 0x%02x\n", flags.CFGUPMODE);
    printf("BAT_DET = 0x%02x\n", flags.BAT_DET);
    printf("SOC1 = 0x%02x\n", flags.SOC1);
    printf("SOCF = 0x%02x\n", flags.SOCF);
    printf("DSG = 0x%02x\n", flags.DSG);
}