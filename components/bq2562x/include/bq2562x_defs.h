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

    struct CHARGE_CONTROL_REG
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

    struct CHARGE_TIMER_CONTROL_REG
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

    struct CHARGER_CONTROL_REG
    {
        enum WATCH_DOG_TIMER_TYPES : uint8_t
        {
            WATCH_DOG_DISABLE = 0x0,
            WATCH_DOG_50S = 0x1, // default
            WATCH_DOG_100S = 0x2,
            WATCH_DOG_200S = 0x3
        };

        enum ADC_CONV_FREQ_TYPES : uint8_t
        {
            CONV_FREQ_1500KHZ = 0x0, // default
            CONV_FREQ_1350KHZ = 0x1,
            CONV_FREQ_1650KHZ = 0x2
        };

        enum CONV_STRN_TYPES : uint8_t
        {
            CONV_STRN_WEAK = 0x0,
            CONV_STRN_NORMAL = 0x1,
            CONV_STRN_STRONG = 0x3
        };

        enum BATFET_CTRL_TYPES : uint8_t
        {
            BATFET_CTRL_NORMAL = 0x0,
            BATFET_CTRL_SHUTDOWN = 0x1,
            BATFET_CTRL_SHIP = 0x2,
            BATFET_CTRL_SYS_POWER_RST = 0x3
        };

        enum IBAT_PK_CTRL_TYPES : uint8_t
        {
            IBAT_PK_1500MA = 0x0,
            IBAT_PK_3A = 0x1,
            IBAT_PK_6A = 0x2,
            IBAT_PK_12A = 0x3 // default
        };

        enum CHG_RATE_TYPES : uint8_t
        {
            CHG_RATE_1C = 0x0, // default
            CHG_RATE_2C = 0x1,
            CHG_RATE_4C = 0x2,
            CHG_RATE_6C = 0x3
        };

        std::optional<bool> EN_AUTO_IBATDIS;           // Enable the auto battery discharging during the battery OVP fault
        std::optional<bool> FORCE_IBATDIS;             // Force a battery discharging current (~30mA)
        std::optional<bool> EN_CHG;                    // Charger enable configuration, 0h = Charge Disable; 1h = Charge Enable (default)
        std::optional<bool> EN_HIZ;                    // Enable HIZ mode. 0h = Disable (default); 1h = Enable
        std::optional<bool> FORCE_PMID_DIS;            // Force a PMID discharge current (~30mA.) 0h = Disable (default); 1h = Enable
        std::optional<bool> WD_RST;                    // I2C watch dog timer reset. 0h = Normal (default); 1h = Reset (this bit goes back to 0 after timer reset)
        std::optional<WATCH_DOG_TIMER_TYPES> WATCHDOG; // Watchdog timer setting

        std::optional<bool> REG_RST;                      // Reset registers to default values and reset timer. Value resets to 0 after reset completes.
        std::optional<bool> TREG;                         // Thermal regulation thresholds. 0h = 60C; 1h = 120C (default)
        std::optional<ADC_CONV_FREQ_TYPES> SET_CONV_FREQ; // Adjust switching frequency of the converter
        std::optional<CONV_STRN_TYPES> SET_CONV_STRN;     // Adjust the high side and low side drive strength of the converter to adjust efficiency versus EMI.
        // bit 1 reserved
        std::optional<bool> VBUS_OVP; // Sets VBUS overvoltage protection threshold. 0h = 6.3 V; 1h = 18.5 V (default)

        // bits 7:5 reserved
        std::optional<bool> PFM_FWD_DIS;              // Disable PFM in forward buck mode. 0h = Enable (Default); 1h = Disable
        std::optional<bool> BATFET_CTRL_WVBUS;        // Optionally allows BATFET off or system power reset with adapter present
        std::optional<bool> BATFET_DLY;               // Delay time added to the taking action in bits [1:0] of the BATFET_CTRL. 0h = Add 20 ms delay time; 1h = 10s (default)
        std::optional<BATFET_CTRL_TYPES> BATFET_CTRL; // The control logic of the BATFET to force the device enter different modes

        std::optional<IBAT_PK_CTRL_TYPES> IBAT_PK; // Battery discharging peak current protection threshold setting
        std::optional<bool> VBAT_UVLO;             // Select the VBAT_UVLO falling threshold and VBAT_SHORT threshold
        // bits 4:3 reserved
        std::optional<bool> EN_EXTILIM;         // only for BQ25628
        std::optional<CHG_RATE_TYPES> CHG_RATE; // The charge rate definition for the fast charge stage.

        uint32_t to_value(uint32_t original_value = 0) const
        {
            if (EN_AUTO_IBATDIS.has_value())
                original_value = (original_value & ~(0b1 << (7 + 8 * 3))) | (EN_AUTO_IBATDIS.value() << (7 + 8 * 3));
            if (FORCE_IBATDIS.has_value())
                original_value = (original_value & ~(0b1 << (6 + 8 * 3))) | (FORCE_IBATDIS.value() << (6 + 8 * 3));
            if (EN_CHG.has_value())
                original_value = (original_value & ~(0b1 << (5 + 8 * 3))) | (EN_CHG.value() << (5 + 8 * 3));
            if (EN_HIZ.has_value())
                original_value = (original_value & ~(0b1 << (4 + 8 * 3))) | (EN_HIZ.value() << (4 + 8 * 3));
            if (FORCE_PMID_DIS.has_value())
                original_value = (original_value & ~(0b1 << (3 + 8 * 3))) | (FORCE_PMID_DIS.value() << (3 + 8 * 3));
            if (WD_RST.has_value())
                original_value = (original_value & ~(0b1 << (2 + 8 * 3))) | (WD_RST.value() << (2 + 8 * 3));
            if (WATCHDOG.has_value())
                original_value = (original_value & ~(0b11 << (0 + 8 * 3))) | (WATCHDOG.value() << (0 + 8 * 3));

            if (REG_RST.has_value())
                original_value = (original_value & ~(0b1 << (7 + 8 * 2))) | (REG_RST.value() << (7 + 8 * 2));
            if (TREG.has_value())
                original_value = (original_value & ~(0b1 << (6 + 8 * 2))) | (TREG.value() << (6 + 8 * 2));
            if (SET_CONV_FREQ.has_value())
                original_value = (original_value & ~(0b11 << (4 + 8 * 2))) | (SET_CONV_FREQ.value() << (4 + 8 * 2));
            if (SET_CONV_STRN.has_value())
                original_value = (original_value & ~(0b11 << (2 + 8 * 2))) | (SET_CONV_STRN.value() << (2 + 8 * 2));
            if (VBUS_OVP.has_value())
                original_value = (original_value & ~(0b1 << (0 + 8 * 2))) | (VBUS_OVP.value() << (0 + 8 * 2));

            if (PFM_FWD_DIS.has_value())
                original_value = (original_value & ~(0b1 << (4 + 8 * 1))) | (PFM_FWD_DIS.value() << (4 + 8 * 1));
            if (BATFET_CTRL_WVBUS.has_value())
                original_value = (original_value & ~(0b1 << (3 + 8 * 1))) | (BATFET_CTRL_WVBUS.value() << (3 + 8 * 1));
            if (BATFET_DLY.has_value())
                original_value = (original_value & ~(0b1 << (2 + 8 * 1))) | (BATFET_DLY.value() << (2 + 8 * 1));
            if (BATFET_CTRL.has_value())
                original_value = (original_value & ~(0b11 << (0 + 8 * 1))) | (BATFET_CTRL.value() << (0 + 8 * 1));

            if (IBAT_PK.has_value())
                original_value = (original_value & ~(0b11 << (6 + 8 * 0))) | (IBAT_PK.value() << (6 + 8 * 0));
            if (VBAT_UVLO.has_value())
                original_value = (original_value & ~(0b1 << (5 + 8 * 0))) | (VBAT_UVLO.value() << (5 + 8 * 0));
            if (EN_EXTILIM.has_value())
                original_value = (original_value & ~(0b1 << (2 + 8 * 0))) | (EN_EXTILIM.value() << (2 + 8 * 0));
            if (CHG_RATE.has_value())
                original_value = (original_value & ~(0b11 << (0 + 8 * 0))) | (CHG_RATE.value() << (0 + 8 * 0));

            return original_value;
        }

        static CHARGER_CONTROL_REG from_value(uint32_t value)
        {
            CHARGER_CONTROL_REG reg;
            reg.EN_AUTO_IBATDIS = (value >> (7 + 8 * 3)) & 0b1;
            reg.FORCE_IBATDIS = (value >> (6 + 8 * 3)) & 0b1;
            reg.EN_CHG = (value >> (5 + 8 * 3)) & 0b1;
            reg.EN_HIZ = (value >> (4 + 8 * 3)) & 0b1;
            reg.FORCE_PMID_DIS = (value >> (3 + 8 * 3)) & 0b1;
            reg.WD_RST = (value >> (2 + 8 * 3)) & 0b1;
            reg.WATCHDOG = static_cast<WATCH_DOG_TIMER_TYPES>((value >> (0 + 8 * 3)) & 0b11);

            reg.REG_RST = (value >> (7 + 8 * 2)) & 0b1;
            reg.TREG = (value >> (6 + 8 * 2)) & 0b1;
            reg.SET_CONV_FREQ = static_cast<ADC_CONV_FREQ_TYPES>((value >> (4 + 8 * 2)) & 0b11);
            reg.SET_CONV_STRN = static_cast<CONV_STRN_TYPES>((value >> (2 + 8 * 2)) & 0b11);
            reg.VBUS_OVP = (value >> (0 + 8 * 2)) & 0b1;

            reg.PFM_FWD_DIS = (value >> (4 + 8 * 1)) & 0b1;
            reg.BATFET_CTRL_WVBUS = (value >> (3 + 8 * 1)) & 0b1;
            reg.BATFET_DLY = (value >> (2 + 8 * 1)) & 0b1;
            reg.BATFET_CTRL = static_cast<BATFET_CTRL_TYPES>((value >> (0 + 8 * 1)) & 0b11);

            reg.IBAT_PK = static_cast<IBAT_PK_CTRL_TYPES>((value >> (6 + 8 * 0)) & 0b11);
            reg.VBAT_UVLO = (value >> (5 + 8 * 0)) & 0b1;
            reg.EN_EXTILIM = (value >> (2 + 8 * 0)) & 0b1;
            reg.CHG_RATE = static_cast<CHG_RATE_TYPES>((value >> (0 + 8 * 0)) & 0b11);
            return reg;
        };
    };

    struct NTC_CONTROL_REG
    {
        enum TS_ISET_TYPE : uint8_t
        {
            ISET_CHARGE_SUSPEND = 0x0,
            ISET_20PERCENT = 0x1,
            ISET_40PERCENT = 0x2,
            ISET_UNCHANGED = 0x3

        };

        enum TS_VSET_TYPE : uint8_t
        {
            VSET_300MV_LOWER = 0x0,
            VSET_200MV_LOWER = 0x1,
            VSET_100MV_LOWER = 0x2,
            VSET_UNCHANGED = 0x3
        };

        std::optional<bool> TS_IGNORE;
        // bits 6:4 reserved
        std::optional<TS_ISET_TYPE> TS_ISET_WARM;
        std::optional<TS_ISET_TYPE> TS_ISET_COOL;

        std::optional<uint8_t> TS_TH1_TH2_TH3;
        std::optional<uint8_t> TS_TH4_TH5_TH6;
        std::optional<TS_VSET_TYPE> TS_VSET_WARM;

        // bit 7 reserved
        std::optional<bool> TS_VSET_SYM;
        std::optional<TS_VSET_TYPE> TS_VSET_PREWARM;
        std::optional<TS_ISET_TYPE> TS_ISET_PREWARM;
        std::optional<TS_ISET_TYPE> TS_ISET_PRECOOL;

        uint32_t to_value(uint32_t original_value = 0) const
        {
            if (TS_IGNORE.has_value())
                original_value = (original_value & ~(0b1 << (7 + 8 * 2))) | (TS_IGNORE.value() << (7 + 8 * 2));
            if (TS_ISET_WARM.has_value())
                original_value = (original_value & ~(0b11 << (2 + 8 * 2))) | (TS_ISET_WARM.value() << (2 + 8 * 2));
            if (TS_ISET_COOL.has_value())
                original_value = (original_value & ~(0b11 << (0 + 8 * 2))) | (TS_ISET_COOL.value() << (0 + 8 * 2));

            if (TS_TH1_TH2_TH3.has_value())
                original_value = (original_value & ~(0b111 << (5 + 8 * 1))) | (TS_TH1_TH2_TH3.value() << (5 + 8 * 1));
            if (TS_TH4_TH5_TH6.has_value())
                original_value = (original_value & ~(0b111 << (2 + 8 * 1))) | (TS_TH4_TH5_TH6.value() << (2 + 8 * 1));
            if (TS_VSET_WARM.has_value())
                original_value = (original_value & ~(0b11 << (0 + 8 * 1))) | (TS_VSET_WARM.value() << (0 + 8 * 1));

            if (TS_VSET_SYM.has_value())
                original_value = (original_value & ~(0b1 << (6 + 8 * 0))) | (TS_VSET_SYM.value() << (6 + 8 * 0));
            if (TS_VSET_PREWARM.has_value())
                original_value = (original_value & ~(0b11 << (4 + 8 * 0))) | (TS_VSET_PREWARM.value() << (4 + 8 * 0));
            if (TS_ISET_PREWARM.has_value())
                original_value = (original_value & ~(0b11 << (2 + 8 * 0))) | (TS_ISET_PREWARM.value() << (2 + 8 * 0));
            if (TS_ISET_PRECOOL.has_value())
                original_value = (original_value & ~(0b11 << (0 + 8 * 0))) | (TS_ISET_PRECOOL.value() << (0 + 8 * 0));

            return original_value;
        }

        static NTC_CONTROL_REG from_value(uint32_t value)
        {
            NTC_CONTROL_REG reg;
            reg.TS_IGNORE = (value >> (7 + 8 * 2)) & 0b1;
            reg.TS_ISET_WARM = static_cast<TS_ISET_TYPE>((value >> (2 + 8 * 2)) & 0b11);
            reg.TS_ISET_COOL = static_cast<TS_ISET_TYPE>((value >> (0 + 8 * 2)) & 0b11);

            reg.TS_TH1_TH2_TH3 = (value >> (5 + 8 * 1)) & 0b111;
            reg.TS_TH4_TH5_TH6 = (value >> (2 + 8 * 1)) & 0b111;
            reg.TS_VSET_WARM = static_cast<TS_VSET_TYPE>((value >> (0 + 8 * 1)) & 0b11);

            reg.TS_VSET_SYM = (value >> (6 + 8 * 0)) & 0b1;
            reg.TS_VSET_PREWARM = static_cast<TS_VSET_TYPE>((value >> (4 + 8 * 0)) & 0b11);
            reg.TS_ISET_PREWARM = static_cast<TS_ISET_TYPE>((value >> (2 + 8 * 0)) & 0b11);
            reg.TS_ISET_PRECOOL = static_cast<TS_ISET_TYPE>((value >> (0 + 8 * 0)) & 0b11);

            return reg;
        };
    };

    struct CHARGER_STATUS_REG
    {
        enum CHG_STAT_TYPE : uint8_t
        {
            CHG_STAT_NOT_CHARGING = 0x0,
            CHG_STAT_CC_MODE = 0x1,
            CHG_STAT_CV_MODE = 0x2,
            CHG_STAT_TOP_OFF_TIMER_ACTIVE = 0x3,
        };

        enum VBUS_STAT_TYPE : uint8_t
        {
            VBUS_STAT_UNKNOWN_ADAPTER = 0b100
        };

        // bit 7 reserved
        std::optional<bool> ADC_DONE_STAT;
        std::optional<bool> TREG_STAT;
        std::optional<bool> VSYS_STAT;
        std::optional<bool> IINDPM_STAT;
        std::optional<bool> VINDPM_STAT;
        std::optional<bool> SAFETY_TMR_STAT;
        std::optional<bool> WD_STAT;

        // bits 7:5 reserved
        std::optional<CHG_STAT_TYPE> CHG_STAT;
        std::optional<VBUS_STAT_TYPE> VBUS_STAT;

        // register not writable

        static CHARGER_STATUS_REG from_value(uint32_t value)
        {
            CHARGER_STATUS_REG reg;
            reg.ADC_DONE_STAT = (value >> (6 + 8 * 1)) & 0b1;
            reg.TREG_STAT = (value >> (5 + 8 * 1)) & 0b1;
            reg.VSYS_STAT = (value >> (4 + 8 * 1)) & 0b1;
            reg.IINDPM_STAT = (value >> (3 + 8 * 1)) & 0b1;
            reg.VINDPM_STAT = (value >> (2 + 8 * 1)) & 0b1;
            reg.SAFETY_TMR_STAT = (value >> (1 + 8 * 1)) & 0b1;
            reg.WD_STAT = (value >> (0 + 8 * 1)) & 0b1;

            reg.CHG_STAT = static_cast<CHG_STAT_TYPE>((value >> (3 + 8 * 0)) & 0b11);
            reg.VBUS_STAT = static_cast<VBUS_STAT_TYPE>((value >> (0 + 8 * 0)) & 0b111);
            return reg;
        };
    };

    struct FAULT_STATUS_REG
    {
        enum TS_STAT_TYPE : uint8_t
        {
            TS_STAT_NORMAL = 0x0,
            TS_STAT_COLD_OR_NOT_AVAILABLE = 0x1,
            TS_STAT_HOT = 0x2,
            TS_STAT_COOL = 0x3,
            TS_STAT_WARM = 0x4,
            TS_STAT_PRECOOL = 0x5,
            TS_STAT_PREWARM = 0x6,
            TS_STAT_REF_FAULT = 0x7
        };

        std::optional<bool> VBUS_FAULT_STAT;
        std::optional<bool> BAT_FAULT_STAT;
        std::optional<bool> SYS_FAULT_STAT;
        // bit 4 reserved
        std::optional<bool> TSHUT_STAT;
        std::optional<TS_STAT_TYPE> TS_STAT;

        // register not writable

        static FAULT_STATUS_REG from_byte(uint8_t value)
        {
            FAULT_STATUS_REG reg;
            reg.VBUS_FAULT_STAT = (value >> 7) & 0b1;
            reg.BAT_FAULT_STAT = (value >> 6) & 0b1;
            reg.SYS_FAULT_STAT = (value >> 5) & 0b1;
            reg.TSHUT_STAT = (value >> 3) & 0b1;
            reg.TS_STAT = static_cast<TS_STAT_TYPE>((value >> 0) & 0b111);
            return reg;
        };
    };

    struct CHARGER_FLAG_REG
    {
        // bit 7 reserved
        std::optional<bool> ADC_DONE_FLAG;
        std::optional<bool> TREG_FLAG;
        std::optional<bool> VSYS_FLAG;
        std::optional<bool> IINDPM_FLAG;
        std::optional<bool> VINDPM_FLAG;
        std::optional<bool> SAFETY_TMR_FLAG;
        std::optional<bool> WD_FLAG;

        // bits 7:4 reserved
        std::optional<bool> CHG_FLAG;
        // bits 2:1 reserved
        std::optional<bool> VBUS_FLAG;

        // register not writable

        static CHARGER_FLAG_REG from_value(uint32_t value)
        {
            CHARGER_FLAG_REG reg;
            reg.ADC_DONE_FLAG = (value >> (6 + 8 * 1)) & 0b1;
            reg.TREG_FLAG = (value >> (5 + 8 * 1)) & 0b1;
            reg.VSYS_FLAG = (value >> (4 + 8 * 1)) & 0b1;
            reg.IINDPM_FLAG = (value >> (3 + 8 * 1)) & 0b1;
            reg.VINDPM_FLAG = (value >> (2 + 8 * 1)) & 0b1;
            reg.SAFETY_TMR_FLAG = (value >> (1 + 8 * 1)) & 0b1;
            reg.WD_FLAG = (value >> (0 + 8 * 1)) & 0b1;

            reg.CHG_FLAG = (value >> (3 + 8 * 0)) & 0b1;
            reg.VBUS_FLAG = (value >> (0 + 8 * 0)) & 0b1;
            return reg;
        };
    };

    struct FAULT_FLAG_REG
    {
        std::optional<bool> VBUS_FAULT_FLAG;
        std::optional<bool> BAT_FAULT_FLAG;
        std::optional<bool> SYS_FAULT_FLAG;
        // bit 4 reserved
        std::optional<bool> TSHUT_FLAG;
        // bits 2:1 reserved
        std::optional<bool> TS_FLAG;

        // register not writable

        static FAULT_FLAG_REG from_byte(uint8_t value)
        {
            FAULT_FLAG_REG reg;
            reg.VBUS_FAULT_FLAG = (value >> 7) & 0b1;
            reg.BAT_FAULT_FLAG = (value >> 6) & 0b1;
            reg.SYS_FAULT_FLAG = (value >> 5) & 0b1;
            reg.TSHUT_FLAG = (value >> 3) & 0b1;
            reg.TS_FLAG = (value >> 0) & 0b1;
            return reg;
        };
    };

    struct CHARGER_MASK_REG
    {
        // bit 7 reserved
        std::optional<bool> ADC_DONE_MASK;
        std::optional<bool> TREG_MASK;
        std::optional<bool> VSYS_MASK;
        std::optional<bool> IINDPM_MASK;
        std::optional<bool> VINDPM_MASK;
        std::optional<bool> SAFETY_TMR_MASK;
        std::optional<bool> WD_MASK;

        // bits 7:4 reserved
        std::optional<bool> CHG_MASK;
        // bits 2:1 reserved
        std::optional<bool> VBUS_MASK;

        uint32_t to_value(uint32_t original_value = 0) const
        {
            if (ADC_DONE_MASK.has_value())
                original_value = (original_value & ~(0b1 << (6 + 8 * 1))) | (ADC_DONE_MASK.value() << (6 + 8 * 1));
            if (TREG_MASK.has_value())
                original_value = (original_value & ~(0b1 << (5 + 8 * 1))) | (TREG_MASK.value() << (5 + 8 * 1));
            if (VSYS_MASK.has_value())
                original_value = (original_value & ~(0b1 << (4 + 8 * 1))) | (VSYS_MASK.value() << (4 + 8 * 1));
            if (IINDPM_MASK.has_value())
                original_value = (original_value & ~(0b1 << (3 + 8 * 1))) | (IINDPM_MASK.value() << (3 + 8 * 1));
            if (VINDPM_MASK.has_value())
                original_value = (original_value & ~(0b1 << (2 + 8 * 1))) | (VINDPM_MASK.value() << (2 + 8 * 1));
            if (SAFETY_TMR_MASK.has_value())
                original_value = (original_value & ~(0b1 << (1 + 8 * 1))) | (SAFETY_TMR_MASK.value() << (1 + 8 * 1));
            if (WD_MASK.has_value())
                original_value = (original_value & ~(0b1 << (0 + 8 * 1))) | (WD_MASK.value() << (0 + 8 * 1));

            if (CHG_MASK.has_value())
                original_value = (original_value & ~(0b1 << (3 + 8 * 0))) | (CHG_MASK.value() << (3 + 8 * 0));
            if (VBUS_MASK.has_value())
                original_value = (original_value & ~(0b1 << (0 + 8 * 0))) | (VBUS_MASK.value() << (0 + 8 * 0));
            return original_value;
        }

        static CHARGER_MASK_REG from_value(uint32_t value)
        {
            CHARGER_MASK_REG reg;
            reg.ADC_DONE_MASK = (value >> (6 + 8 * 1)) & 0b1;
            reg.TREG_MASK = (value >> (5 + 8 * 1)) & 0b1;
            reg.VSYS_MASK = (value >> (4 + 8 * 1)) & 0b1;
            reg.IINDPM_MASK = (value >> (3 + 8 * 1)) & 0b1;
            reg.VINDPM_MASK = (value >> (2 + 8 * 1)) & 0b1;
            reg.SAFETY_TMR_MASK = (value >> (1 + 8 * 1)) & 0b1;
            reg.WD_MASK = (value >> (0 + 8 * 1)) & 0b1;

            reg.CHG_MASK = (value >> (3 + 8 * 0)) & 0b1;
            reg.VBUS_MASK = (value >> (0 + 8 * 0)) & 0b1;
            return reg;
        };
    };

    struct FAULT_MASK_REG
    {
        std::optional<bool> VBUS_FAULT_MASK;
        std::optional<bool> BAT_FAULT_MASK;
        std::optional<bool> SYS_FAULT_MASK;
        // bit 4 reserved
        std::optional<bool> TSHUT_MASK;
        // bits 2:1 reserved
        std::optional<bool> TS_MASK;

        uint8_t to_byte(uint8_t original_value = 0) const
        {
            if (VBUS_FAULT_MASK.has_value())
                original_value = (original_value & ~(0b1 << 7)) | (VBUS_FAULT_MASK.value() << 7);
            if (BAT_FAULT_MASK.has_value())
                original_value = (original_value & ~(0b1 << 6)) | (BAT_FAULT_MASK.value() << 6);
            if (SYS_FAULT_MASK.has_value())
                original_value = (original_value & ~(0b1 << 5)) | (SYS_FAULT_MASK.value() << 5);
            if (TSHUT_MASK.has_value())
                original_value = (original_value & ~(0b1 << 3)) | (TSHUT_MASK.value() << 3);
            if (TS_MASK.has_value())
                original_value = (original_value & ~(0b1 << 0)) | (TS_MASK.value() << 0);
            return original_value;
        }

        static FAULT_MASK_REG from_byte(uint8_t value)
        {
            FAULT_MASK_REG reg;
            reg.VBUS_FAULT_MASK = (value >> 7) & 0b1;
            reg.BAT_FAULT_MASK = (value >> 6) & 0b1;
            reg.SYS_FAULT_MASK = (value >> 5) & 0b1;
            reg.TSHUT_MASK = (value >> 3) & 0b1;
            reg.TS_MASK = (value >> 0) & 0b1;
            return reg;
        };
    };

    struct ADC_CONTROL_REG
    {
        enum ADC_RATE_TYPE : uint8_t
        {
            ADC_RATE_CONTINUOUS = 0x0, // default
            ADC_RATE_ONESHOT = 0x1
        };

        enum ADC_SAMPLE_TYPE : uint8_t
        {
            ADC_SAMPLE_12BITS = 0x0,
            ADC_SAMPLE_11BITS = 0x1,
            ADC_SAMPLE_10BITS = 0x2,
            ADC_SAMPLE_9BITS = 0x3 // default
        };

        enum ADC_AVG_TYPE : uint8_t
        {
            ADC_AVG_SINGLE = 0x0, // default
            ADC_AVG_RUNNING = 0x1
        };

        enum ADC_AVG_INIT_TYPE : uint8_t
        {
            ADC_AVG_INIT_EXISTING = 0x0, // default
            ADC_AVG_INIT_NEW = 0x1
        };

        std::optional<bool> ADC_EN;
        std::optional<ADC_RATE_TYPE> ADC_RATE;
        std::optional<ADC_SAMPLE_TYPE> ADC_SAMPLE;
        std::optional<ADC_AVG_TYPE> ADC_AVG;
        std::optional<ADC_AVG_INIT_TYPE> ADC_AVG_INIT;
        // bits 1:0 reserved

        uint8_t to_byte(uint8_t original_value = 0x00) const
        {
            if (ADC_EN.has_value())
                original_value = (original_value & ~(0b1 << 7)) | (ADC_EN.value() << 7);
            if (ADC_RATE.has_value())
                original_value = (original_value & ~(0b1 << 6)) | (ADC_RATE.value() << 6);
            if (ADC_SAMPLE.has_value())
                original_value = (original_value & ~(0b11 << 4)) | (ADC_SAMPLE.value() << 4);
            if (ADC_AVG.has_value())
                original_value = (original_value & ~(0b1 << 3)) | (ADC_AVG.value() << 3);
            if (ADC_AVG_INIT.has_value())
                original_value = (original_value & ~(0b1 << 2)) | (ADC_AVG_INIT.value() << 2);
            return original_value;
        }

        static ADC_CONTROL_REG from_byte(uint8_t value)
        {
            ADC_CONTROL_REG reg;
            reg.ADC_EN = (value >> 7) & 0b1;
            reg.ADC_RATE = static_cast<ADC_RATE_TYPE>((value >> 6) & 0b1);
            reg.ADC_SAMPLE = static_cast<ADC_SAMPLE_TYPE>((value >> 4) & 0b11);
            reg.ADC_AVG = static_cast<ADC_AVG_TYPE>((value >> 3) & 0b1);
            reg.ADC_AVG_INIT = static_cast<ADC_AVG_INIT_TYPE>((value >> 2) & 0b1);
            return reg;
        }
    };

    struct ADC_FUNCTION_DISABLE_REG
    {
        std::optional<bool> IBUS_ADC_DIS;
        std::optional<bool> IBAT_ADC_DIS;
        std::optional<bool> VBUS_ADC_DIS;
        std::optional<bool> VBAT_ADC_DIS;
        std::optional<bool> VSYS_ADC_DIS;
        std::optional<bool> TS_ADC_DIS;
        std::optional<bool> TDIE_ADC_DIS;
        std::optional<bool> VPMID_ADC_DIS;

        uint8_t to_byte(uint8_t original_value = 0x00) const
        {
            if (IBUS_ADC_DIS.has_value())
                original_value = (original_value & ~(0b1 << 7)) | (IBUS_ADC_DIS.value() << 7);
            if (IBAT_ADC_DIS.has_value())
                original_value = (original_value & ~(0b1 << 6)) | (IBAT_ADC_DIS.value() << 6);
            if (VBUS_ADC_DIS.has_value())
                original_value = (original_value & ~(0b1 << 5)) | (VBUS_ADC_DIS.value() << 5);
            if (VBAT_ADC_DIS.has_value())
                original_value = (original_value & ~(0b1 << 4)) | (VBAT_ADC_DIS.value() << 4);
            if (VSYS_ADC_DIS.has_value())
                original_value = (original_value & ~(0b1 << 3)) | (VSYS_ADC_DIS.value() << 3);
            if (TS_ADC_DIS.has_value())
                original_value = (original_value & ~(0b1 << 2)) | (TS_ADC_DIS.value() << 2);
            if (TDIE_ADC_DIS.has_value())
                original_value = (original_value & ~(0b1 << 1)) | (TDIE_ADC_DIS.value() << 1);
            if (VPMID_ADC_DIS.has_value())
                original_value = (original_value & ~(0b1 << 0)) | (VPMID_ADC_DIS.value() << 0);
            return original_value;
        }

        static ADC_FUNCTION_DISABLE_REG from_byte(uint8_t value)
        {
            ADC_FUNCTION_DISABLE_REG reg;
            reg.IBUS_ADC_DIS = (value >> 7) & 0b1;
            reg.IBAT_ADC_DIS = (value >> 6) & 0b1;
            reg.VBUS_ADC_DIS = (value >> 5) & 0b1;
            reg.VBAT_ADC_DIS = (value >> 4) & 0b1;
            reg.VSYS_ADC_DIS = (value >> 3) & 0b1;
            reg.TS_ADC_DIS = (value >> 2) & 0b1;
            reg.TDIE_ADC_DIS = (value >> 1) & 0b1;
            reg.VPMID_ADC_DIS = (value >> 0) & 0b1;
            return reg;
        }
    };

}
#endif