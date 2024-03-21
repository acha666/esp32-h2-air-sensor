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

    std::unique_ptr<I2C_Register> _regChargeCurrentLimit = nullptr;
    std::unique_ptr<I2C_Register> _regChargeVoltageLimit = nullptr;
    std::unique_ptr<I2C_Register> _regInputCurrentLimit = nullptr;
    std::unique_ptr<I2C_Register> _regInputVoltageLimit = nullptr;
    std::unique_ptr<I2C_Register> _regMinimalSystemVoltage = nullptr;
    std::unique_ptr<I2C_Register> _regPreChargeControl = nullptr;
    std::unique_ptr<I2C_Register> _regTerminationControl = nullptr;
    std::unique_ptr<I2C_Register> _regChargeControl = nullptr;
    std::unique_ptr<I2C_Register> _regChargeTimerControl = nullptr;
    std::unique_ptr<I2C_Register> _regChargerControl = nullptr;
    std::unique_ptr<I2C_Register> _regNTCControl = nullptr;
    std::unique_ptr<I2C_Register> _regChargerStatus = nullptr;
    std::unique_ptr<I2C_Register> _regFaultStatus0 = nullptr;
    std::unique_ptr<I2C_Register> _regChargerFlag = nullptr;
    std::unique_ptr<I2C_Register> _regFaultFlag0 = nullptr;
    std::unique_ptr<I2C_Register> _regChargerMask = nullptr;
    std::unique_ptr<I2C_Register> _regFaultMask0 = nullptr;
    std::unique_ptr<I2C_Register> _regADCControl = nullptr;
    std::unique_ptr<I2C_Register> _regADCFunctionDisable0 = nullptr;
    std::unique_ptr<I2C_Register> _regIBUS_ADC = nullptr;
    std::unique_ptr<I2C_Register> _regIBAT_ADC = nullptr;
    std::unique_ptr<I2C_Register> _regVBUS_ADC = nullptr;
    std::unique_ptr<I2C_Register> _regVPMI_DADC = nullptr;
    std::unique_ptr<I2C_Register> _regVBAT_ADC = nullptr;
    std::unique_ptr<I2C_Register> _regVSYS_ADC = nullptr;
    std::unique_ptr<I2C_Register> _regTS_ADC = nullptr;
    std::unique_ptr<I2C_Register> _regTDIE_ADC = nullptr;
    std::unique_ptr<I2C_Register> _regPartInformation = nullptr;
};

#endif
