#include "esp_log.h"
#include "driver/i2c_master.h"
#include <stdexcept>

#include "bq27426.h"
#include "bq27426_defs.h"
#include "bq2562x.h"
#include "bq2562x_defs.h"

#include "main.h"
#include "power.h"

static void power_i2c_init(void);
static void charger_watchdog_feed_task(void *pvParameters);

static constexpr char TAG[] = "Power";
static i2c_master_bus_handle_t power_i2c_master_bus_handle;
static bool CHARGER_FAULT_FLAG = false;
static bool GAUGE_FAULT_FLAG = false;

BQ27426 *BatteryGauge;
BQ2562x *Charger;

using std::runtime_error;

extern "C" void power_task(void *pvParameters)
{
    // Initialize I2C Bus
    power_i2c_init();

    try // Initialize Charger
    {
        Charger = new BQ2562x(power_i2c_master_bus_handle, 0x6a);
    }
    catch (const runtime_error &e)
    {
        CHARGER_FAULT_FLAG = true;
        ESP_LOGE(TAG, "Charger init failed: %s", e.what());
    }

    try // Initialize Battery Gauge
    {
        BatteryGauge = new BQ27426(power_i2c_master_bus_handle);
    }
    catch (const runtime_error &e)
    {
        GAUGE_FAULT_FLAG = true;
        ESP_LOGE(TAG, "Battery Gauge init failed: %s", e.what());
    }

    if (!CHARGER_FAULT_FLAG && !GAUGE_FAULT_FLAG)
    {
        BQ2562X_DEFS::CHARGER_CONTROL_REG charger_control;
        charger_control.REG_RST = 1; // reset charger
        Charger->setChargerControl(charger_control);

        charger_control= BQ2562X_DEFS::CHARGER_CONTROL_REG();
        charger_control.WATCHDOG = BQ2562X_DEFS::CHARGER_CONTROL_REG::WATCH_DOG_100S; // set watchdog to 100s
        Charger->setChargerControl(charger_control);

        Charger->setChargeCurrent(2000); // set charge current to 2000mA

        BQ2562X_DEFS::NTC_CONTROL_REG ntc_control;
        ntc_control.TS_IGNORE = 1; // ignore NTC
        Charger->setNTCControl(ntc_control);

        xTaskCreate(charger_watchdog_feed_task, "charger_watchdog_feed_task", 4096, NULL, 1, NULL);
    }

    while (1) // Main loop
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        power_data_t data;
        if (!CHARGER_FAULT_FLAG)
        {
            data.charger_fault = 0;

            BQ2562X_DEFS::CHARGER_STATUS_REG chargerStatus = Charger->getChargerStatus();
            if (chargerStatus.CHG_STAT.value() == BQ2562X_DEFS::CHARGER_STATUS_REG::CHG_STAT_NOT_CHARGING)
                data.charger_charging = false;
            else
                data.charger_charging = true;
        }
        else
            data.charger_fault = 1;

        if (!GAUGE_FAULT_FLAG)
        {
            data.gauge_fault = 0;
            data.gauge_battery_soc = BatteryGauge->getStateOfCharge();
            data.gauge_battery_soh = BatteryGauge->getStateOfHealth();
            data.gauge_battery_voltage = BatteryGauge->getVoltage();
            data.gauge_temperature = BatteryGauge->getTemperature();
            data.gauge_average_current = BatteryGauge->getAverageCurrent();
            data.gauge_average_power = BatteryGauge->getAveragePower();
            data.gauge_remaining_capacity = BatteryGauge->getRemainingCapacity();
            data.gauge_full_charge_capacity = BatteryGauge->getFullChargeCapacity();

            BQ27426_DEFS::FLAGS flags = BatteryGauge->getFlags();
            data.gauge_chg = flags.CHG;
            data.gauge_fc = flags.FC;
            if (flags.OT)
                data.gauge_temperature_state = power_data_t::OVER_TEMP;
            else if (flags.UT)
                data.gauge_temperature_state = power_data_t::UNDER_TEMP;
            else
                data.gauge_temperature_state = power_data_t::NORMAL_TEMP;
        }
        else
            data.gauge_fault = 1;

        xQueueSend(powerDataQueue, &data, 0);
    }

    vTaskDelete(NULL);
}

static void charger_watchdog_feed_task(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(50 * 1000));
        Charger->resetWatchDog();
    }
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

static void print_charger_charger_control(BQ2562X_DEFS::CHARGER_CONTROL_REG charger_control)
{
    printf("EN_AUTO_IBATDIS = 0x%02x\n", charger_control.EN_AUTO_IBATDIS.value());
    printf("FORCE_IBATDIS = 0x%02x\n", charger_control.FORCE_IBATDIS.value());
    printf("EN_CHG = 0x%02x\n", charger_control.EN_CHG.value());
    printf("EN_HIZ = 0x%02x\n", charger_control.EN_HIZ.value());
    printf("FORCE_PMID_DIS = 0x%02x\n", charger_control.FORCE_PMID_DIS.value());
    printf("WD_RST = 0x%02x\n", charger_control.WD_RST.value());
    printf("WATCHDOG = 0x%02x\n", charger_control.WATCHDOG.value());
    printf("REG_RST = 0x%02x\n", charger_control.REG_RST.value());
    printf("TREG = 0x%02x\n", charger_control.TREG.value());
    printf("SET_CONV_FREQ = 0x%02x\n", charger_control.SET_CONV_FREQ.value());
    printf("SET_CONV_STRN = 0x%02x\n", charger_control.SET_CONV_STRN.value());
    printf("VBUS_OVP = 0x%02x\n", charger_control.VBUS_OVP.value());
    printf("PFM_FWD_DIS = 0x%02x\n", charger_control.PFM_FWD_DIS.value());
    printf("BATFET_CTRL_WVBUS = 0x%02x\n", charger_control.BATFET_CTRL_WVBUS.value());
    printf("BATFET_DLY = 0x%02x\n", charger_control.BATFET_DLY.value());
    printf("BATFET_CTRL = 0x%02x\n", charger_control.BATFET_CTRL.value());
    printf("IBAT_PK = 0x%02x\n", charger_control.IBAT_PK.value());
    printf("VBAT_UVLO = 0x%02x\n", charger_control.VBAT_UVLO.value());
    printf("EN_EXTILIM = 0x%02x\n", charger_control.EN_EXTILIM.value());
    printf("CHG_RATE = 0x%02x\n", charger_control.CHG_RATE.value());
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