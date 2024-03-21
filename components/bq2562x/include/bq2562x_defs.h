#ifndef __BQ2562X__DEFS_H__
#define __BQ2562X__DEFS_H__

#include <inttypes.h>

namespace BQ2562X_DEFS
{
    constexpr uint8_t REG_CHARGE_CURRENT_LIMIT_ADDR = 0x02;
    constexpr uint8_t REG_CHARGE_VOLTAGE_LIMIT_ADDR = 0x04;
    constexpr uint8_t REG_INPUT_CURRENT_LIMIT_ADDR = 0x06;
    constexpr uint8_t REG_INPUT_VOLTAGE_LIMIT_ADDR = 0x08;
    constexpr uint8_t REG_MINIMAL_SYSTEM_VOLTAGE_ADDR = 0x0E;
    constexpr uint8_t REG_PRE_CHARGE_CONTROL_ADDR = 0x10;
    constexpr uint8_t REG_TERMINATION_CONTROL_ADDR = 0x12;
    constexpr uint8_t REG_CHARGE_CONTROL_ADDR = 0x14;
    constexpr uint8_t REG_CHARGE_TIMER_CONTROL_ADDR = 0x15;
    constexpr uint8_t REG_CHARGER_CONTROL_0_ADDR = 0x16;
    constexpr uint8_t REG_CHARGER_CONTROL_1_ADDR = 0x17;
    constexpr uint8_t REG_CHARGER_CONTROL_2_ADDR = 0x18;
    constexpr uint8_t REG_CHARGER_CONTROL_3_ADDR = 0x19;
    constexpr uint8_t REG_NTC_CONTROL_0_ADDR = 0x1A;
    constexpr uint8_t REG_NTC_CONTROL_1_ADDR = 0x1B;
    constexpr uint8_t REG_NTC_CONTROL_2_ADDR = 0x1C;
    constexpr uint8_t REG_CHARGER_STATUS_0_ADDR = 0x1D;
    constexpr uint8_t REG_CHARGER_STATUS_1_ADDR = 0x1E;
    constexpr uint8_t REG_FAULT_STATUS_0_ADDR = 0x1F;
    constexpr uint8_t REG_CHARGER_FLAG_0_ADDR = 0x20;
    constexpr uint8_t REG_CHARGER_FLAG_1_ADDR = 0x21;
    constexpr uint8_t REG_FAULT_FLAG_0_ADDR = 0x22;
    constexpr uint8_t REG_CHARGER_MASK_0_ADDR = 0x23;
    constexpr uint8_t REG_CHARGER_MASK_1_ADDR = 0x24;
    constexpr uint8_t REG_FAULT_MASK_0_ADDR = 0x25;
    constexpr uint8_t REG_ADC_CONTROL_ADDR = 0x26;
    constexpr uint8_t REG_ADC_FUNCTION_DISABLE_0_ADDR = 0x27;
    constexpr uint8_t REG_IBUS_ADC_ADDR = 0x28;
    constexpr uint8_t REG_IBAT_ADC_ADDR = 0x2A;
    constexpr uint8_t REG_VBUS_ADC_ADDR = 0x2C;
    constexpr uint8_t REG_VPMID_ADC_ADDR = 0x2E;
    constexpr uint8_t REG_VBAT_ADC_ADDR = 0x30;
    constexpr uint8_t REG_VSYS_ADC_ADDR = 0x32;
    constexpr uint8_t REG_TS_ADC_ADDR = 0x34;
    constexpr uint8_t REG_TDIE_ADC_ADDR = 0x36;
    constexpr uint8_t REG_PART_INFORMATION_ADDR = 0x38;
}
#endif