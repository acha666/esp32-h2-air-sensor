#ifndef __BQ2562X_H__
#define __BQ2562X_H__

#include "driver/i2c_master.h"
#include "i2c_register.h"
#include "esp_log.h"
#include <memory>

class BQ2562x
{
public:
    struct Register
    {
        uint8_t address;
        uint8_t size;
        uint8_t start;
        uint8_t end;
    };

    enum class VBUSStat
    {
        None,
        Adapter
    };

    enum class ChargeStat
    {
        Terminated,
        Trickle,
        Taper,
        TopOff
    };

    enum class ADCRate
    {
        Oneshot,
        Continuous,
    };

    enum class ADCSampling
    {
        Bits_12,
        Bits_11,
        Bits_10,
        Bits_9,
    };

    enum class ADCAverage
    {
        Single,
        Running,
    };

    enum class ADCAverageInit
    {
        Existing,
        New,
    };

    enum class BATFETControl
    {
        Normal,
        ShutdownMode,
        ShipMode,
        SystemPowerReset
    };

    enum class BATFETDelay
    {
        Delay20ms,
        Delay10s,
    };

    enum class Interrupt : uint8_t
    {
        VBUS
    };

    enum class Adc : uint8_t
    {
        IBUS = 7,
        IBAT = 6,
        VBUS = 5,
        VBAT = 4,
        VSYS = 3,
        TS = 2,
        TDIE = 1,
        VPMID = 0
    };

    enum WatchDogTimerConf : uint8_t
    {
        WATCH_DOG_DISABLE = 0x0,
        WATCH_DOG_50S = 0x1,
        WATCH_DOG_100S = 0x2,
        WATCH_DOG_200S = 0x3
    };

    BQ2562x(i2c_master_bus_handle_t bus_handle, uint8_t address);
    ~BQ2562x();

    void setChargeCurrent(uint16_t current);
    void setWatchDog(WatchDogTimerConf timer);
    WatchDogTimerConf getWatchDog();

    bool getTS_ADC(float &value);
    bool enableCharging(bool state);
    bool enableSTAT(bool enable);
    bool enableTS(bool enable);
    bool enableWD(bool enable);
    bool enableHIZ(bool enable);
    bool setBATFETControl(BATFETControl control);
    bool setBATFETDelay(BATFETDelay delay);
    bool enableInterrupts(bool enable);
    bool enableInterrupt(Interrupt mask, bool en);
    bool enableWVBUS(bool enable);
    bool getVBAT(uint16_t &value);
    bool enableADC(Adc adc, bool enable);
    bool setupADC(bool enable, ADCRate rate = ADCRate::Continuous, ADCSampling sampling = ADCSampling::Bits_9,
                  ADCAverage average = ADCAverage::Single, ADCAverageInit averageInit = ADCAverageInit::Existing);
    bool getADCDone(bool &done);

    bool getVBUSStat(VBUSStat &stat);
    bool getPartInformation(uint8_t &value);
    bool getBatteryVoltage(float &voltage);
    bool getChargeStat(ChargeStat &stat);
    bool getIBUS(int16_t &value);
    bool getWD(bool &enabled);
    bool getTS(bool &enabled);

    bool getVBUS(uint16_t &value);
    bool getIBAT(int16_t &value);
    bool setVINDPM(uint32_t mV);
    bool setIINDPM(uint32_t mA);

private:
    void _initRegisters();

    i2c_master_bus_handle_t _i2cBus;
    i2c_master_dev_handle_t _i2cDevice;

    I2C_Register _regChargeCurrentLimit;
    I2C_Register _regChargeVoltageLimit;
    I2C_Register _regInputCurrentLimit;
    I2C_Register _regInputVoltageLimit;
    I2C_Register _regMinimalSystemVoltage;
    I2C_Register _regPreChargeControl;
    I2C_Register _regTerminationControl;
    I2C_Register _regChargeControl;
    I2C_Register _regChargeTimerControl;
    I2C_Register _regChargerControl;
    I2C_Register _regNTCControl;
    I2C_Register _regChargerStatus;
    I2C_Register _regFaultStatus0;
    I2C_Register _regChargerFlag;
    I2C_Register _regFaultFlag0;
    I2C_Register _regChargerMask;
    I2C_Register _regFaultMask0;
    I2C_Register _regADCControl;
    I2C_Register _regADCFunctionDisable0;
    I2C_Register _regIBUS_ADC;
    I2C_Register _regIBAT_ADC;
    I2C_Register _regVBUS_ADC;
    I2C_Register _regVPMI_DADC;
    I2C_Register _regVBAT_ADC;
    I2C_Register _regVSYS_ADC;
    I2C_Register _regTS_ADC;
    I2C_Register _regTDIE_ADC;
    I2C_Register _regPartInformation;
};

#endif
