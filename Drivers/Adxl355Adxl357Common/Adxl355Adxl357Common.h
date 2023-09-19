///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2023 Mihai Ursu                                                 //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

/*
Adxl355Adxl357Common.h

This file contains the definitions for common functionality of the ADXL355 and
ADXL357 drivers.
*/

#ifndef Adxl355Adxl357Common_h
#define Adxl355Adxl357Common_h

#include <cstddef>
#include <cstdint>


//************************************************************************
// Class for handling the ADXL355 and ADXL357 drivers
//************************************************************************
class Adxl355Adxl357Common
{
    //************************************************************************
    // constants and types
    //************************************************************************
    protected:
        typedef enum : uint8_t
        {
            ADXL355_357_REG_DEVID_AD        = 0x00,
            ADXL355_357_REG_DEVID_MST       = 0x01,
            ADXL355_357_REG_PART_ID         = 0x02,
            ADXL355_357_REG_REV_ID          = 0x03,
            ADXL355_357_REG_STATUS          = 0x04,
            ADXL355_357_REG_FIFO_ENTRIES    = 0x05,
            ADXL355_357_REG_TEMP2           = 0x06,
            ADXL355_357_REG_TEMP1           = 0x07,
            ADXL355_357_REG_XDATA3          = 0x08,
            ADXL355_357_REG_XDATA2          = 0x09,
            ADXL355_357_REG_XDATA1          = 0x0a,
            ADXL355_357_REG_YDATA3          = 0x0b,
            ADXL355_357_REG_YDATA2          = 0x0c,
            ADXL355_357_REG_YDATA1          = 0x0d,
            ADXL355_357_REG_ZDATA3          = 0x0e,
            ADXL355_357_REG_ZDATA2          = 0x0f,
            ADXL355_357_REG_ZDATA1          = 0x10,
            ADXL355_357_REG_FIFO_DATA       = 0x11,
            ADXL355_357_REG_OFFSET_X_H      = 0x1e,
            ADXL355_357_REG_OFFSET_X_L      = 0x1f,
            ADXL355_357_REG_OFFSET_Y_H      = 0x20,
            ADXL355_357_REG_OFFSET_Y_L      = 0x21,
            ADXL355_357_REG_OFFSET_Z_H      = 0x22,
            ADXL355_357_REG_OFFSET_Z_L      = 0x23,
            ADXL355_357_REG_ACT_EN          = 0x24,
            ADXL355_357_REG_ACT_THRESH_H    = 0x25,
            ADXL355_357_REG_ACT_THRESH_L    = 0x26,
            ADXL355_357_REG_ACT_COUNT       = 0x27,
            ADXL355_357_REG_FILTER          = 0x28,
            ADXL355_357_REG_FIFO_SAMPLES    = 0x29,
            ADXL355_357_REG_INT_MAP         = 0x2a,
            ADXL355_357_REG_SYNC            = 0x2b,
            ADXL355_357_REG_RANGE           = 0x2c,
            ADXL355_357_REG_POWER_CTL       = 0x2d,
            ADXL355_357_REG_SELF_TEST       = 0x2e,
            ADXL355_357_REG_RESET           = 0x2f,
            ADXL355_357_REG_SHADOW_FIRST    = 0x50,
            ADXL355_357_REG_SHADOW_LAST     = 0x54
        }Adxl355Adxl357Reg;

        typedef enum : uint8_t
        {
            STATUS_NVM_BUSY = 0x10,         //!< NVM controller is busy
            STATUS_ACTIVITY = 0x08,         //!< Activity is detected, as defined by ACT_COUNT and ACT_THRESH_x
            STATUS_FIFO_OVR = 0x04,         //!< FIFO overrun
            STATUS_FIFO_FUL = 0x02,         //!< FIFO full
            STATUS_DATA_RDY = 0x01          //!< A complete measurement on all 3 axes was made, and results can be read
        }Status;

        static const uint8_t FIFO_ENTRIES = 0x7f;           //!< Number of data samples stored in the FIFO
        static const uint8_t FIFO_SAMPLES = 0x7f;           //!< Number of data samples in the FIFO which trigger a FIFO_FULL condition
        static const uint8_t FIFO_MAX_COUNT = 96;           //!< Maximum number of FIFO samples

        typedef enum : uint8_t
        {
            ACT_EN_Z = 0x04,                //!< Z-axis is part of the activity detection algorithm
            ACT_EN_Y = 0x02,                //!< Y-axis is part of the activity detection algorithm
            ACT_EN_X = 0x01                 //!< X-axis is part of the activity detection algorithm
        }ActEn;

        typedef enum : uint8_t
        {
            FILTER_HPF_CORNER   = 0x70,     //!< Mask for the HPF -3dB corner, as a function of current ODR
            FILTER_ODR_LPF      = 0x0f,     //!< Mask for both ODR and LPF -3dB corner

            FILTER_HPF_DC       = 0x00,     //!< DC, no HPF enabled
            FILTER_HPF_24_7     = 0x10,     //!< HPF -3dB corner = ODR * 24.7e-4
            FILTER_HPF_6_20     = 0x20,     //!< HPF -3dB corner = ODR * 6.2084e-4
            FILTER_HPF_1_55     = 0x30,     //!< HPF -3dB corner = ODR * 1.5545e-4
            FILTER_HPF_0_38     = 0x40,     //!< HPF -3dB corner = ODR * 0.3862e-4
            FILTER_HPF_0_09     = 0x50,     //!< HPF -3dB corner = ODR * 0.0954e-4
            FILTER_HPF_0_02     = 0x60,     //!< HPF -3dB corner = ODR * 0.0238e-4

            FILTER_ODR_4000     = 0x00,     //!< ODR = 4000 Hz       , LPF -3dB corner = 1000 Hz
            FILTER_ODR_2000     = 0x01,     //!< ODR = 2000 Hz       , LPF -3dB corner =  500 Hz
            FILTER_ODR_1000     = 0x02,     //!< ODR = 1000 Hz       , LPF -3dB corner =  250 Hz
            FILTER_ODR_500      = 0x03,     //!< ODR =  500 Hz       , LPF -3dB corner =  125 Hz
            FILTER_ODR_250      = 0x04,     //!< ODR =  250 Hz       , LPF -3dB corner =   62.5 Hz
            FILTER_ODR_125      = 0x05,     //!< ODR =  125 Hz       , LPF -3dB corner =   31.25 Hz
            FILTER_ODR_62_5     = 0x06,     //!< ODR =   62.5 Hz     , LPF -3dB corner =   15.625 Hz
            FILTER_ODR_31_25    = 0x07,     //!< ODR =   31.25 Hz    , LPF -3dB corner =    7.8125 Hz
            FILTER_ODR_15_625   = 0x08,     //!< ODR =   15.625 Hz   , LPF -3dB corner =    3.90625 Hz
            FILTER_ODR_7_8125   = 0x09,     //!< ODR =    7.8125 Hz  , LPF -3dB corner =    1.953125 Hz
            FILTER_ODR_3_90625  = 0x0a      //!< ODR =    3.90625 Hz , LPF -3dB corner =    0.9765625 Hz
        }Filter;

        static const uint8_t ODR_SETTINGS_COUNT = 11;   //!< number of ODR settings

        typedef enum : uint8_t
        {
            INT_MAP_ACT_EN2  = 0x80,        //!< Activity  interrupt enable on INT2
            INT_MAP_OVR_EN2  = 0x40,        //!< FIFO_OVR  interrupt enable on INT2
            INT_MAP_FULL_EN2 = 0x20,        //!< FIFO_FULL interrupt enable on INT2
            INT_MAP_RDY_EN2  = 0x10,        //!< DATA_RDY  interrupt enable on INT2

            INT_MAP_ACT_EN1  = 0x08,        //!< Activity  interrupt enable on INT1
            INT_MAP_OVR_EN1  = 0x04,        //!< FIFO_OVR  interrupt enable on INT1
            INT_MAP_FULL_EN1 = 0x02,        //!< FIFO_FULL interrupt enable on INT1
            INT_MAP_RDY_EN1  = 0x01         //!< DATA_RDY  interrupt enable on INT1
        }IntMap;

        typedef enum : uint8_t
        {
            SYNC_EXT_CLK            = 0x04, //!< Enable external clock
            SYNC_CTRL_EN            = 0x03, //!< Synchronization control
            SYNC_CTRL_EXT_INTERP    = 0x02, //!< External synchronization, interpolation filter
            SYNC_CTRL_EXT_NO_INTERP = 0x01, //!< External synchronization, no interpolation filter
            SYNC_CTRL_INTERNAL      = 0x00  //!< Internal synchronization
        }Sync;

        typedef enum : uint8_t
        {
            RANGE_I2C_HS           = 0x80,  //!< I2C high speed mode
            RANGE_INT_POL_ACTIVE_H = 0x40,  //!< Interrupt pins active high

            RANGE_HIGH = 0x03,              //!< Range is +/-8 g (ADXL355) or +/-10 g (ADXL357)
            RANGE_MID  = 0x02,              //!< Range is +/-4 g (ADXL355) or +/-20 g (ADXL357)
            RANGE_LOW  = 0x01               //!< Range is +/-2 g (ADXL355) or +/-40 g (ADXL357)
        }Range;

        typedef enum : uint8_t
        {
            POWER_CTL_DRDY_OFF     = 0x04,  //!< Data ready not signaled
            POWER_CTL_TEMP_OFF     = 0x02,  //!< Temperature processing off
            POWER_CTL_STANDBY_MODE = 0x01   //!< Standby mode active
        }PowerCtl;

        typedef enum : uint8_t
        {
            SELF_TEST_FORCE_EN = 0x02,     //!< Enable self test force
            SELF_TEST_MODE_EN  = 0x01      //!< Enable self test mode
        }SelfTest;

        typedef enum : uint32_t
        {
            // ADXL355 only
            ACCELERATION_SENSITIVITY_RANGE_2G = 256000,     //!< LSB count in 1g when using +/-2 g range
            ACCELERATION_SENSITIVITY_RANGE_4G = 128000,     //!< LSB count in 1g when using +/-4 g range
            ACCELERATION_SENSITIVITY_RANGE_8G =  64000,     //!< LSB count in 1g when using +/-8 g range
            // ADXL357 only
            ACCELERATION_SENSITIVITY_RANGE_10G = 51200,     //!< LSB count in 1g when using +/-10 g range
            ACCELERATION_SENSITIVITY_RANGE_20G = 25600,     //!< LSB count in 1g when using +/-20 g range
            ACCELERATION_SENSITIVITY_RANGE_40G = 12800      //!< LSB count in 1g when using +/-40 g range
        }AccelerationSensitivity;

        static constexpr double CROSS_AXIS_SENSITIVITY = 0.01;

        typedef struct
        {
            double x;                       //!< LSB-TO-g conversion factor for X-axis
            double y;                       //!< LSB-TO-g conversion factor for Y-axis
            double z;                       //!< LSB-TO-g conversion factor for Z-axis
        }LsbToG;

        typedef struct
        {
            double x;                       //!< X-axis offset [g]
            double y;                       //!< Y-axis offset [g]
            double z;                       //!< Z-axis offset [g]
        }OffsetG;

        typedef struct
        {
            double  freq;                   //!< ODR frequency [Hz]
            double  lpfCorner;              //!< LPF -3dB corner [Hz]
            double  combinedDelay;          //!< interpolator + decimator delay [s]
            Filter  filterReg;              //!< associated FILTER register mask
        }Odr;

        static constexpr double TEMP_25_C  =  25.0;
        static constexpr double TEMP_MIN_C = -40.0;
        static constexpr double TEMP_MAX_C = 125.0;

    public:
        typedef enum : uint8_t
        {
            ADXL355_357_I2C_ADDRESS_PRIMARY     = 0x1d,     //!< ADXL355 and ADXL357 default I2C 7-bit address
            ADXL355_357_I2C_ADDRESS_SECONDARY   = 0x53      //!< ADXL355 and ADXL357 alternative I2C 7-bit address
        }Adxl355I2cAddress;

        static const uint8_t REV_ID = 0x01; //!< chip revision ID

        typedef enum : uint8_t
        {
            ACCELERATION_RANGE_DEFAULT,
            // ADXL355 only
            ACCELERATION_RANGE_2G,          //!< range is +/-2 g
            ACCELERATION_RANGE_4G,          //!< range is +/-4 g
            ACCELERATION_RANGE_8G,          //!< range is +/-8 g
            // ADXL357 only
            ACCELERATION_RANGE_10G,         //!< range is +/-10 g
            ACCELERATION_RANGE_20G,         //!< range is +/-20 g
            ACCELERATION_RANGE_40G          //!< range is +/-40 g
        }AccelerationRange;

        typedef enum : uint8_t
        {
            ODR_SETTING_4000    = FILTER_ODR_4000,          //!< 4000 Hz
            ODR_SETTING_2000    = FILTER_ODR_2000,          //!< 2000 Hz
            ODR_SETTING_1000    = FILTER_ODR_1000,          //!< 1000 Hz
            ODR_SETTING_500     = FILTER_ODR_500,           //!<  500 Hz
            ODR_SETTING_250     = FILTER_ODR_250,           //!<  250 Hz
            ODR_SETTING_125     = FILTER_ODR_125,           //!<  125 Hz
            ODR_SETTING_62_5    = FILTER_ODR_62_5,          //!<   62.5 Hz
            ODR_SETTING_31_25   = FILTER_ODR_31_25,         //!<   31.25 Hz
            ODR_SETTING_15_625  = FILTER_ODR_15_625,        //!<   15.625 Hz
            ODR_SETTING_7_8125  = FILTER_ODR_7_8125,        //!<    7.8125 Hz
            ODR_SETTING_3_90625 = FILTER_ODR_3_90625        //!<    3.90625 Hz
        }OdrSetting;

        typedef enum : uint8_t
        {
            HPF_SETTING_DC      = FILTER_HPF_DC,            //!< DC
            HPF_SETTING_24_7    = FILTER_HPF_24_7,          //!< ODR * 24.7e-4
            HPF_SETTING_6_20    = FILTER_HPF_6_20,          //!< ODR * 6.2084e-4
            HPF_SETTING_1_55    = FILTER_HPF_1_55,          //!< ODR * 1.5545e-4
            HPF_SETTING_0_38    = FILTER_HPF_0_38,          //!< ODR * 0.3862e-4
            HPF_SETTING_0_09    = FILTER_HPF_0_09,          //!< ODR * 0.0954e-4
            HPF_SETTING_0_02    = FILTER_HPF_0_02           //!< ODR * 0.0238e-4
        }HpfSetting;

        typedef enum : uint8_t
        {
            AXIS_X,                         //!< x-axis
            AXIS_Y,                         //!< y-axis
            AXIS_Z                          //!< z-axis
        }Axis;

        typedef enum : uint8_t
        {
            AXIS_VERTICAL_UP,               //!< direction is opposite to Earth gravity
            AXIS_VERTICAL_DOWN              //!< direction is identical to Earth gravity
        }AxisDirection;

        typedef enum : uint8_t
        {
            INTERRUPT_PIN_1,                //!< INT1 (physical pin 12)
            INTERRUPT_PIN_2                 //!< INT2 (physical pin 13)
        }InterruptPin;

        typedef enum : uint8_t
        {
            INTERRUPT_ACTIVE_LOW,           //!< interrupt pin is active low
            INTERRUPT_ACTIVE_HIGH           //!< interrupt pin is active high
        }InterruptActive;

        typedef enum : uint8_t
        {
            INTERRUPT_SOURCE_ACTIVITY,      //!< interrupt is triggered by activity detection
            INTERRUPT_SOURCE_FIFO_OVR,      //!< interrupt is triggered by FIFO overrun
            INTERRUPT_SOURCE_FIFO_FULL,     //!< interrupt is triggered by FIFO full
            INTERRUPT_SOURCE_DATA_RDY       //!< interrupt is triggered by data ready
        }InterruptSource;

        typedef enum : uint8_t
        {
            I2C_MODE_HIGH_SPEED,            //!< I2C high speed mode (3400 kHz)
            I2C_MODE_FAST                   //!< I2C fast mode (400 kHz)
        }I2cMode;

        typedef enum : uint8_t
        {
            SYNC_TYPE_INTERNAL,                     //!< internal synchronization
            SYNC_TYPE_EXTERNAL_NO_INTERPOLATION,    //!< external synchronization, no interpolation filter
            SYNC_TYPE_EXTERNAL_WITH_INTERPOLATION   //!< external synchronization, with interpolation filter
        }SyncType;


    //************************************************************************
    // functions
    //************************************************************************
    public:
        Adxl355Adxl357Common
            (
            AccelerationRange aRange = ACCELERATION_RANGE_DEFAULT   //!< acceleration range
            );

        ~Adxl355Adxl357Common();

        bool disableDrdyPin
            (
            const bool aDrdyPinDisable      //!< true for disabling DRDY pin output
            );

        bool enableActivityAxis
            (
            const Axis aAxis,               //!< axis for changing the activity detection status
            const bool aEnable              //!< true for enabling the activity detection
            );

        bool enableExternalClk
            (
            const bool aEnable              //!< true for enabling the external CLK
            );

        bool enableInterruptSource
            (
            const InterruptPin    aPin,     //!< interrupt pin whose functionality to change
            const InterruptSource aSource,  //!< source of the interrupt
            const bool            aEnable   //!< true for enabling
            );

        bool enableStandbyMode
            (
            const bool aEnterStandby        //!< true for entering standby
            );

        bool getAccelerationOnAxis
            (
            const Axis aAxis,               //!< axis whose acceleration to get
            double*    aAcceleration        //!< acceleration value [g]
            );

        virtual bool getAccelerationRange
            (
            AccelerationRange* aRange       //!< acceleration range
            ) = 0;

        bool getAccelerationsOnAllAxes
            (
            double* aXAcceleration,         //!< acceleration value on X axis [g]
            double* aYAcceleration,         //!< acceleration value on Y axis [g]
            double* aZAcceleration          //!< acceleration value on Z axis [g]
            );

        double getCrossAxisSensitivity() const;

        bool getFifoData
            (
            uint8_t* aData                  //!< FIFO data
            );

        bool getFifoSamplesSize
            (
            uint8_t* aSize                  //!< number of samples stored in the FIFO
            );

        bool getFifoValidSamplesCount
            (
            uint8_t* aNumber                //!< number of valid data samples in the FIFO
            );

        bool getInclinationStepAngle
            (
            const Axis   aAxis,                 //!< axis
            const double aInclinationAngle,     //!< inclination angle [rad]
            double*      aInclinationStepAngle  //!< step angle [rad]
            ) const;

        double getOdrDelay() const;

        double getOdrFrequency() const;

        double getOdrHpfCorner() const;

        double getOdrLpfCorner() const;

        uint8_t getRevId() const;

        bool getStandbyMode
            (
            bool* aEnabled                  //!< true if standby mode is enabled
            );

        bool getSyncType
            (
            SyncType* aSyncType             //!< data synchronization type
            );

        bool getTemperatureValue
            (
            double* aTemperature            //!< temperature [C]
            );

        virtual bool getVibrationRectificationOffset
            (
            const Axis          aAxis,              //!< axis whose RMS acceleration to use
            const AxisDirection aDirection,         //!< axis direction (up or down)
            const double        aAccelerationRms,   //!< RMS of a series of accelerations [g]
            double*             aVreOffset          //!< calculated VRE offset [g]
            ) = 0;

        bool init
            (
            const uint8_t aI2cChannel,      //!< I2C channel
            const uint8_t aDeviceAddress    //!< 7-bit I2C device address
            );

        bool isActivityDetected
            (
            bool* aDetection                //!< true if activity is detected
            ) const;

        bool isDataReady
            (
            bool* aDataReady                //!< true if data is ready
            ) const;

        bool isExternalClkEnabled
            (
            bool* aEnabled                  //!< true if external clock is enabled
            ) const;

        bool isFifoOvr
            (
            bool* aFifoOvr                  //!< true if FIFO overrange
            ) const;

        bool isFifoFull
            (
            bool* aFifoFull                 //!< true if FIFO is full
            ) const;

        bool isNvmBusy
            (
            bool* aNvmBusy                  //!< true if NVM is busy
            ) const;

        bool reset();

        virtual bool runSelfTest
            (
            bool*   aResult,                //!< true if self test passed
            double* aTypCoef                //!< how close to typical ones test values were found (0 is ideal)
            ) = 0;

        virtual bool setAccelerationRange
            (
            const AccelerationRange aRange  //!< acceleration range
            ) = 0;

        bool setActivityCount
            (
            const uint8_t aNumber           //!< number of consecutive events
            );

        bool setActivityThreshold
            (
            const double aThreshold         //!< activity detection threshold [g]
            );

        bool setAxisOffset
            (
            const Axis   aAxis,             //!< axis whose offset to change
            const double aValue             //!< offset value [g]
            );

        bool setFifoSamplesSize
            (
            const uint8_t aSize             //!< number of samples to store in the FIFO
            );

        bool setHpf
            (
            const HpfSetting aHpfSetting    //!< HPF setting
            );

        bool setHpfFrequency
            (
            const double aHpfFrequency      //!< HPF frequency [Hz]
            );

        bool setI2cMode
            (
            const I2cMode aI2cMode          //!< I2C speed mode
            );

        bool setInterruptPinsPolarity
            (
            const InterruptActive aActiveMode   //!< the active low/high mode to set
            );

        bool setOdr
            (
            const OdrSetting aOdrSetting    //!< ODR setting
            );

        bool setSyncType
            (
            const SyncType aSyncType        //!< data synchronization type
            );

        bool setTemperatureOff
            (
            const bool aTemperatureOff      //!< true for disabling temperature processing
            );

    protected:
        bool calculateLogisticResponse
            (
            const double aX,                //!< argument
            const double aA1,               //!< initial value
            const double aA2,               //!< final value
            const double aX0,               //!< center
            const double aP,                //!< power
            double*      aY                 //!< calculated value
            );

        bool calculatePoly
            (
            const double aX,                //!< argument
            const size_t aN,                //!< degree
            double*      aC,                //!< coefficients array
            double*      aY                 //!< calculated value
            );

        bool checkOffsetRealRange
            (
            const Axis aAxis                //!< axis to verify offset range for
            ) const;

        virtual bool compensateOffsetTemperature
            (
            const Axis aAxis,           //!< axis whose offset to compensate
            double*    aAcceleration    //!< acceleration value [g]
            ) = 0;

        bool convertOffsetRealToInt
            (
            const Axis aAxis,               //!< axis
            int16_t*   aIntOffset           //!< converted axis offset
            );

        bool enableSelfTestMode
            (
            const bool aEnterSelfTest       //!< true for entering self test mode
            );

        bool getTemperatureOff
            (
            bool* aEnabled                  //!< true if temperature processing is off
            );

        void initOdrList();

        bool readByteArrayReg
            (
            const uint8_t aAddress,         //!< start register address
            const uint8_t aLength,          //!< array length
            uint8_t* 	  aByteArray        //!< byte array read
            ) const;

        bool readByteReg
            (
            const uint8_t aAddress,         //!< register address
            uint8_t*      aValue            //!< byte read
            ) const;

        bool readWordReg
            (
            const uint8_t aAddress,         //!< register address
            uint16_t* 	  aValue            //!< word read
            ) const;

        bool storeActivityThreshold();

        bool storeAxisOffset
            (
            const Axis aAxis                //!< axis whose offset to store
            );

        void updateHpfBySelection();

        virtual void updateLsbToG() = 0;

        void updateOdrBySelection();

        bool writeByteReg
            (
            const uint8_t aAddress,         //!< register address
            const uint8_t aValue            //!< byte to write
            ) const;

        bool writeWordReg
            (
            const uint8_t  aAddress,        //!< register address
            const uint16_t aValue           //!< word to write
            ) const;


    //************************************************************************
    // variables
    //************************************************************************
    protected:
        uint8_t             mI2cChannel;        //!< I2C channel
        uint8_t             mRevId;             //!< chip revision ID
        bool                mStandbyMode;       //!< if standby mode is active
        bool                mSelfTestMode;      //!< if self test mode is enabled
        bool                mTemperatureOff;    //!< if temperature processing is disabled
        double              mTemperatureC;      //!< temperature [C]
        AccelerationRange   mRange;             //!< acceleration range
        LsbToG              mLsbToG;            //!< LSB-TO-g conversions for x/y/z axes
        double              mActivityThreshold; //!< activity detection threshold [g]
        OffsetG             mOffsetG;           //!< offset [g] for x/y/z axes
        OdrSetting          mOdrSetting;        //!< Output Data Rate setting
        Odr                 mOdr;               //!< ODR structure
        Odr                 mOdrList[ODR_SETTINGS_COUNT];   //!< ODR list
        HpfSetting          mHpfSetting;        //!< HPF setting
        double              mHpfFrequency;      //!< HPF -3dB cutoff frequency
        uint8_t             mFifoSamplesCount;  //!< number of samples to store in the FIFO
};

#endif // Adxl355Adxl357Common_h

