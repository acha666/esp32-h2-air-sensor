#ifndef __BQ2562X__DEFS_H__
#define __BQ2562X__DEFS_H__

#include <inttypes.h>
#include <optional>
#include <bitset>

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

    typedef struct CHARGE_CONTROL_REG
    {

        enum TOPOFF_TMR_TYPES : uint8_t
        {
            TIMER_DISABLE = 0x00,
            TIMER_17_MINS = 0x01,
            TIMER_35_MINS = 0x02,
            TIMER_52_MINS = 0x03,
        };

        std::optional<bool> Q1_FULLON;              // Forces RBFET (Q1) into low resistance state (26 mΩ), regardless of IINDPM setting.
        std::optional<bool> Q4_FULLON;              // Forces BATFET (Q4) into low resistance state (15 mΩ), regardless of ICHG setting (Only applies when VBAT > VSYSMIN).
        std::optional<bool> ITRICKLE;               // Trickle charging current setting: 0b = 10mA (default); 1b = 40mA
        std::optional<TOPOFF_TMR_TYPES> TOPOFF_TMR; // Top-off timer control
        std::optional<bool> EN_TERM;                // Enable termination: 0h = Disable; 1h = Enable (default)
        std::optional<bool> VINDPM_BAT_TRACK;       // Sets VINDPM to track BAT voltage. Actual VINDPM is higher of the VINDPM register value and VBAT + VINDPM_BAT_TRACK.
        std::optional<bool> VRECHG;                 // Battery Recharge Threshold Offset (Below VREG), 0h = 100mV (default); 1h = 200mV

        uint8_t to_byte(uint8_t original_value = 0x00) const
        {
            if (Q1_FULLON.has_value())
                original_value = (original_value & ~(0b1 << 7)) | (Q1_FULLON.value() << 7);
            if (Q4_FULLON.has_value())
                original_value = (original_value & ~(0b1 << 6)) | (Q4_FULLON.value() << 6);
            if (ITRICKLE.has_value())
                original_value = (original_value & ~(0b1 << 5)) | (ITRICKLE.value() << 5);
            if (TOPOFF_TMR.has_value())
                original_value = (original_value & ~(0b11 << 3)) | (TOPOFF_TMR.value() << 3);
            if (EN_TERM.has_value())
                original_value = (original_value & ~(0b1 << 2)) | (EN_TERM.value() << 2);
            if (VINDPM_BAT_TRACK.has_value())
                original_value = (original_value & ~(0b1 << 1)) | (VINDPM_BAT_TRACK.value() << 1);
            if (VRECHG.has_value())
                original_value = (original_value & ~(0b1 << 0)) | (VRECHG.value() << 0);
            return original_value;
        }

        static CHARGE_CONTROL_REG from_byte(uint8_t value)
        {
            CHARGE_CONTROL_REG reg;
            reg.Q1_FULLON = (value >> 7) & 0b1;
            reg.Q4_FULLON = (value >> 6) & 0b1;
            reg.ITRICKLE = (value >> 5) & 0b1;
            reg.TOPOFF_TMR = static_cast<TOPOFF_TMR_TYPES>((value >> 3) & 0b11);
            reg.EN_TERM = (value >> 2) & 0b1;
            reg.VINDPM_BAT_TRACK = (value >> 1) & 0b1;
            reg.VRECHG = (value >> 0) & 0b1;
            return reg;
        }
    };

    typedef struct CHARGE_TIMER_CONTROL_REG
    {

        std::optional<bool> DIS_STAT;       // Disable the STAT pin output, 0h = Enable (default); 1h = Disable
        std::optional<bool> TMR2X_EN;       // 2X charging timer control
        std::optional<bool> EN_SAFETY_TMRS; // Enable fast charge, pre-charge and trickle charge timers, 0h = Disable; 1h = Enable (default)
        std::optional<bool> PRECHG_TMR;     // Pre-charge safety timer setting, 0h = 2.5 hrs (default); 1h = 0.62 hrs
        std::optional<bool> CHG_TMR;        // Fast charge safety timer setting, 0h = 14.5 hrs (default); 1h = 28 hrs

        uint8_t to_byte(uint8_t original_value = 0x00) const
        {
            if (DIS_STAT.has_value())
                original_value = (original_value & ~(0b1 << 7)) | (DIS_STAT.value() << 7);
            if (TMR2X_EN.has_value())
                original_value = (original_value & ~(0b1 << 3)) | (TMR2X_EN.value() << 3);
            if (EN_SAFETY_TMRS.has_value())
                original_value = (original_value & ~(0b1 << 2)) | (EN_SAFETY_TMRS.value() << 2);
            if (PRECHG_TMR.has_value())
                original_value = (original_value & ~(0b1 << 1)) | (PRECHG_TMR.value() << 1);
            if (CHG_TMR.has_value())
                original_value = (original_value & ~(0b1 << 0)) | (CHG_TMR.value() << 0);
            return original_value;
        }

        static CHARGE_TIMER_CONTROL_REG from_byte(uint8_t value)
        {
            CHARGE_TIMER_CONTROL_REG reg;
            reg.DIS_STAT = (value >> 7) & 0b1;
            reg.TMR2X_EN = (value >> 3) & 0b1;
            reg.EN_SAFETY_TMRS = (value >> 2) & 0b1;
            reg.PRECHG_TMR = (value >> 1) & 0b1;
            reg.CHG_TMR = (value >> 0) & 0b1;
            return reg;
        }
    };

    


}
#endif