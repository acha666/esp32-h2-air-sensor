#ifndef __BQ27426_DEFS_H__
#define __BQ27426_DEFS_H__

#include "esp_types.h"
#include "optional"

using std::optional;

namespace BQ27426_DEFS
{

    constexpr uint16_t BQ27426_UNSEAL_KEY = 0x8000; // Secret code to unseal the BQ27426-G1A
    constexpr uint16_t BQ27426_DEVICE_ID = 0x0426;  // Default device ID

    typedef enum : uint8_t
    {
        BQ27426_COMMAND_CONTROL = 0x00,        // Control()
        BQ27426_COMMAND_TEMP = 0x02,           // Temperature()
        BQ27426_COMMAND_VOLTAGE = 0x04,        // Voltage()
        BQ27426_COMMAND_FLAGS = 0x06,          // Flags()
        BQ27426_COMMAND_NOM_CAPACITY = 0x08,   // NominalAvailableCapacity()
        BQ27426_COMMAND_AVAIL_CAPACITY = 0x0A, // FullAvailableCapacity()
        BQ27426_COMMAND_REM_CAPACITY = 0x0C,   // RemainingCapacity()
        BQ27426_COMMAND_FULL_CAPACITY = 0x0E,  // FullChargeCapacity()
        BQ27426_COMMAND_AVG_CURRENT = 0x10,    // AverageCurrent()
        BQ27426_COMMAND_STDBY_CURRENT = 0x12,  // StandbyCurrent()
        BQ27426_COMMAND_MAX_CURRENT = 0x14,    // MaxLoadCurrent()
        BQ27426_COMMAND_AVG_POWER = 0x18,      // AveragePower()
        BQ27426_COMMAND_SOC = 0x1C,            // StateOfCharge()
        BQ27426_COMMAND_INT_TEMP = 0x1E,       // InternalTemperature()
        BQ27426_COMMAND_SOH = 0x20,            // StateOfHealth()
        BQ27426_COMMAND_REM_CAP_UNFIL = 0x28,  // RemainingCapacityUnfiltered()
        BQ27426_COMMAND_REM_CAP_FIL = 0x2A,    // RemainingCapacityFiltered()
        BQ27426_COMMAND_FULL_CAP_UNFIL = 0x2C, // FullChargeCapacityUnfiltered()
        BQ27426_COMMAND_FULL_CAP_FIL = 0x2E,   // FullChargeCapacityFiltered()
        BQ27426_COMMAND_SOC_UNFL = 0x30,       // StateOfChargeUnfiltered()
    } STANDARD_COMMANDS;

    typedef enum : uint16_t
    {
        STATUS = 0x0000,
        DEVICE_TYPE = 0x0001,
        FW_VERSION = 0x0002,
        DM_CODE = 0x0004,
        PREV_MACWRITE = 0x0007,
        CHEM_ID = 0x0008,
        BAT_INSERT = 0x000C,
        BAT_REMOVE = 0x000D,
        SET_CFGUPDATE = 0x0013,
        SMOOTH_SYNC = 0x0019,
        SHUTDOWN_ENABLE = 0x001B,
        SHUTDOWN = 0x001C,
        SEALED = 0x0020,
        PULSE_SOC_INT = 0x0023,
        CHEM_A = 0x0030,
        CHEM_B = 0x0031,
        CHEM_C = 0x0032,
        RESET = 0x0041,
        SOFT_RESET = 0x0042,
    } CONTROL_SUBCOMMANDS;

    struct CONTROL_STATUS
    {
        bool SHUTDOWNEN;
        bool WDRESET;
        bool SS;
        bool CALMODE;
        bool CCA;
        bool BCA;
        bool QMAX_UP;
        bool RES_UP;
        bool INITCOMP;
        bool SLEEP;
        bool LDMD;
        bool RUP_DIS;
        bool VOK;
        bool CHEM_CHANGE;
        uint16_t raw;

        // read only, no setters

        static CONTROL_STATUS from_value(uint16_t value)
        {
            CONTROL_STATUS reg;
            reg.raw = value;
            reg.SHUTDOWNEN = (value >> 15) & 1;
            reg.WDRESET = (value >> 14) & 1;
            reg.SS = (value >> 13) & 1;
            reg.CALMODE = (value >> 12) & 1;
            reg.CCA = (value >> 11) & 1;
            reg.BCA = (value >> 10) & 1;
            reg.QMAX_UP = (value >> 9) & 1;
            reg.RES_UP = (value >> 8) & 1;
            reg.INITCOMP = (value >> 7) & 1;
            reg.SLEEP = (value >> 4) & 1;
            reg.LDMD = (value >> 3) & 1;
            reg.RUP_DIS = (value >> 2) & 1;
            reg.VOK = (value >> 1) & 1;
            reg.CHEM_CHANGE = value & 1;
            return reg;
        }
    };

    struct FLAGS
    {
        bool OT;
        bool UT;
        bool FC;
        bool CHG;
        bool OCVTAKEN;
        bool DOD_CORRECT;
        bool ITPOR;
        bool CFGUPMODE;
        bool BAT_DET;
        bool SOC1;
        bool SOCF;
        bool DSG;
        uint16_t raw;

        // read only, no setters

        static FLAGS from_value(uint16_t value)
        {
            FLAGS reg;
            reg.raw = value;
            reg.OT = (value >> 15) & 1;
            reg.UT = (value >> 14) & 1;
            reg.FC = (value >> 9) & 1;
            reg.CHG = (value >> 8) & 1;
            reg.OCVTAKEN = (value >> 7) & 1;
            reg.DOD_CORRECT = (value >> 6) & 1;
            reg.ITPOR = (value >> 5) & 1;
            reg.CFGUPMODE = (value >> 4) & 1;
            reg.BAT_DET = (value >> 3) & 1;
            reg.SOC1 = (value >> 2) & 1;
            reg.SOCF = (value >> 1) & 1;
            reg.DSG = value & 1;
            return reg;
        }
    };

////////////////////////////
// Extended Data Commands //
////////////////////////////
// Extended data commands offer additional functionality beyond the standard
// set of commands. They are used in the same manner; however, unlike standard
// commands, extended commands are not limited to 2-byte words.
#define BQ27426_EXTENDED_DATACLASS 0x3E // DataClass()
#define BQ27426_EXTENDED_DATABLOCK 0x3F // DataBlock()
#define BQ27426_EXTENDED_BLOCKDATA 0x40 // BlockData()
#define BQ27426_EXTENDED_CHECKSUM 0x60  // BlockDataCheckSum()
#define BQ27426_EXTENDED_CONTROL 0x61   // BlockDataControl()

////////////////////////////////////////
// Configuration Class, Subclass ID's //
////////////////////////////////////////
// To access a subclass of the extended data, set the DataClass() function
// with one of these values.
// Configuration Classes
#define BQ27426_ID_SAFETY 2           // Safety
#define BQ27426_ID_CHG_TERMINATION 36 // Charge Termination
#define BQ27426_ID_DISCHARGE 49       // Discharge
#define BQ27426_ID_REGISTERS 64       // Registers
// Gas Gauging Classes
#define BQ27426_ID_IT_CFG 80         // IT Cfg
#define BQ27426_ID_CURRENT_THRESH 81 // Current Thresholds
#define BQ27426_ID_STATE 82          // State
// Ra Tables Classes
#define BQ27426_ID_R_A_RAM 89 // R_a RAM
// Chemistry Info Classes
#define BQ27426_ID_CHEM_DATA 109 // Chem Data
// Calibration Classes
#define BQ27426_ID_CALIB_DATA 104 // Data
#define BQ27426_ID_CC_CAL 105     // CC Cal
#define BQ27426_ID_CURRENT 107    // Current
// Security Classes
#define BQ27426_ID_CODES 112 // Codes

/////////////////////////////////////////
// OpConfig Register - Bit Definitions //
/////////////////////////////////////////
// Bit positions of the OpConfig Register
#define BQ27426_OPCONFIG_BIE (1 << 13)
#define BQ27426_OPCONFIG_GPIOPOL (1 << 11)
#define BQ27426_OPCONFIG_RES_FACT_STEP (1 << 6)
#define BQ27426_OPCONFIG_SLEEP (1 << 5)
#define BQ27426_OPCONFIG_RMFCC (1 << 4)
#define BQ27426_OPCONFIG_FASTCONV_EN (1 << 3)
#define BQ27426_OPCONFIG_BATLOWEN (1 << 2)
#define BQ27426_OPCONFIG_TEMP_SOURCE (1 << 1) // TempSource[1:0]

}
#endif