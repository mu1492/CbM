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
Adxl355.cpp

This file contains the sources for common functionality of the ADXL355 and ADXL357
drivers.
*/

#include "Adxl355Adxl357Common.h"

#include <cmath>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>


//!************************************************************************
//! Constructor
//!************************************************************************
Adxl355Adxl357Common::Adxl355Adxl357Common
    (
    AccelerationRange aRange        //!< acceleration range
    )
    : mI2cChannel( 0 )
    , mRevId( REV_ID )
    , mStandbyMode( true )
    , mSelfTestMode( false )
    , mTemperatureOff( false )
    , mTemperatureC( TEMP_25_C )
    , mRange( aRange )
    , mActivityThreshold( 0 )
    , mOdrSetting( ODR_SETTING_4000 )
    , mHpfSetting( HPF_SETTING_DC )
    , mFifoSamplesCount( FIFO_MAX_COUNT )
{   
    memset( &mLsbToG, 0, sizeof( mLsbToG ) );
    memset( &mOffsetG, 0, sizeof( mOffsetG ) );
    initOdrList();
}


//!************************************************************************
//! Destructor
//!************************************************************************
Adxl355Adxl357Common::~Adxl355Adxl357Common()
{
}


//!************************************************************************
//! Calculate the logistic response function
//! y = A2 + ( A1 - A2 ) / ( 1 + ( x / x0 )^p )
//!
//! @returns: true if the value can be calculated
//!************************************************************************
bool Adxl355Adxl357Common::calculateLogisticResponse
    (
    const double aX,        //!< argument
    const double aA1,       //!< initial value
    const double aA2,       //!< final value
    const double aX0,       //!< center
    const double aP,        //!< power
    double*      aY         //!< calculated value
    )
{
    bool status = ( aY != nullptr ) && aX0 && ( aP > 0 );

    if( status )
    {
        *aY = aA2 + ( aA1 - aA2 ) / ( 1 + pow( aX / aX0, aP ) );
    }

    return status;
}


//!************************************************************************
//! Calculate the value of a polynomial using Horner's scheme
//! y = c[0] + c[1]*x + c[2]*x^2 +..+ c[N]*x^N
//!
//! @returns: true if the value can be calculated
//!************************************************************************
bool Adxl355Adxl357Common::calculatePoly
    (
    const double aX,     //!< argument
    const size_t aN,     //!< degree
    double*      aC,     //!< coefficients array
    double*      aY      //!< calculated value
    )
{
    bool status = ( aC != nullptr ) && ( aY != nullptr );

    if( status )
    {
        *aY = aC[aN];

        for( size_t i = 1; i <= aN; i++ )
        {
            *aY = *aY * aX + aC[aN - i];
        }
    }

    return status;
}


//!************************************************************************
//! Check if the acceleration offset [g] for the specified axis is within
//! an expected range.
//!
//! To pass, current mOffsetG values for any range must be less than the
//! maximum offset allowed for that range. Next values are valid for a
//! temperature of 25 C.
//!
//! +/- 2g  | A_max | = 0x7ffff / 256000 =  2.04799609 [g]
//!         | O_max | = 0x7fff0 / 256000 =  2.04793750 [g]
//!         | O_min | = 0x0000f / 256000 =  0.00005859 [g] (non-zero minimum)
//!
//! +/- 4g  | A_max | = 0x7ffff / 128000 =  4.09599219 [g]
//!         | O_max | = 0x7fff0 / 128000 =  4.09587500 [g]
//!         | O_min | = 0x0000f / 128000 =  0.00011719 [g] (non-zero minimum)
//!
//! +/- 8g  | A_max | = 0x7ffff /  64000 =  8.19198438 [g]
//!         | O_max | = 0x7fff0 /  64000 =  8.19175000 [g]
//!         | O_min | = 0x0000f /  64000 =  0.00023438 [g] (non-zero minimum)
//!
//! +/-10g  | A_max | = 0x7ffff /  51200 = 10.23998047 [g]
//!         | O_max | = 0x7fff0 /  51200 = 10.23968750 [g]
//!         | O_min | = 0x0000f /  51200 =  0.00029297 [g] (non-zero minimum)
//!
//! +/-20g  | A_max | = 0x7ffff /  25600 = 20.47996094 [g]
//!         | O_max | = 0x7fff0 /  25600 = 20.47937500 [g]
//!         | O_min | = 0x0000f /  25600 =  0.00058594 [g] (non-zero minimum)
//!
//! +/-40g  | A_max | = 0x7ffff /  12800 = 40.95992188 [g]
//!         | O_max | = 0x7fff0 /  12800 = 40.95875000 [g]
//!         | O_min | = 0x0000f /  12800 =  0.00117188 [g] (non-zero minimum)
//!
//! This function *fails* for attempts to use a large offset value, from a
//! higher acceleration range, in a lower range.  For example, it will fail
//! when trying to use an offset of 2.1 [g], belonging to a range of 4g or
//! 8g, when switching to a 2g range.
//!
//! @returns true if the checked value is within the expected range
//!************************************************************************
bool Adxl355Adxl357Common::checkOffsetRealRange
    (
    const Axis aAxis    //!< axis to verify offset range for
    ) const
{
    bool status = ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z );

    if( status )
    {
        const double OFFSET_MAX_X_G = 0x7fff0 * mLsbToG.x;
        const double OFFSET_MAX_Y_G = 0x7fff0 * mLsbToG.y;
        const double OFFSET_MAX_Z_G = 0x7fff0 * mLsbToG.z;

        switch( aAxis )
        {
            case AXIS_X:
                status = fabs( mOffsetG.x ) < OFFSET_MAX_X_G;
                break;

            case AXIS_Y:
                status = fabs( mOffsetG.y ) < OFFSET_MAX_Y_G;
                break;

            case AXIS_Z:
                status = fabs( mOffsetG.z ) < OFFSET_MAX_Z_G;
                break;

            default:
                break;
        }
    }

    return status;
}


//!************************************************************************
//! Convert the acceleration offset [g] for the specified axis to a 16-bit
//! signed integer
//!
//! The returned integer takes into account the value of 1 LSB, which
//! depends on the current acceleration range (+/- 2/4/8 [g]), and is
//! intended for immediate use in the OFFSET_axis_H and OFFSET_axis_L
//! registers.
//!
//! If offset overrange is detected, the value for the axis is reset.
//!
//! @returns true if the offset can be converted, or reset due to overrange
//!************************************************************************
bool Adxl355Adxl357Common::convertOffsetRealToInt
    (
    const Axis aAxis,       //!< axis
    int16_t*   aIntOffset   //!< converted axis offset
    )
{
    bool status = ( aIntOffset != nullptr ) && ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z );
    bool isOvr = false;

    if( status )
    {
        status = checkOffsetRealRange( aAxis );
        isOvr = !status;
    }

    if( status || isOvr )
    {
        *aIntOffset = 0;
        int32_t conv32 = 0;

        switch( aAxis )
        {
            case AXIS_X:
                if( isOvr )
                {
                    mOffsetG.x = 0;
                }
                conv32 = mOffsetG.x / mLsbToG.x;
                break;

            case AXIS_Y:
                if( isOvr )
                {
                    mOffsetG.y = 0;
                }
                conv32 = mOffsetG.y / mLsbToG.y;
                break;

            case AXIS_Z:
                if( isOvr )
                {
                    mOffsetG.z = 0;
                }
                conv32 = mOffsetG.z / mLsbToG.z;
                break;

            default:
                break;
        }

        int16_t conv16 = ( conv32 >> 4 ) & 0xffff;
        *aIntOffset = conv16;
    }

    return ( status || isOvr );
}


//!************************************************************************
//! Change the DRDY pin output functionality
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the DRDY pin functionality can be changed
//!************************************************************************
bool Adxl355Adxl357Common::disableDrdyPin
    (
    const bool aDrdyPinDisable      //!< true for disabling DRDY pin output
    )
{
    bool status = true;
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    status = getStandbyMode( &standbyMode );

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t powerCtlReg = 0;
        status = readByteReg( ADXL355_357_REG_POWER_CTL, &powerCtlReg );

        if( status )
        {
            if( aDrdyPinDisable )
            {
                powerCtlReg |= POWER_CTL_DRDY_OFF;
            }
            else
            {
                powerCtlReg &= ~POWER_CTL_DRDY_OFF;
            }

            status = writeByteReg( ADXL355_357_REG_POWER_CTL, powerCtlReg );
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Change the activity detection status for an axis
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the activity detection status can be set
//!************************************************************************
bool Adxl355Adxl357Common::enableActivityAxis
    (
    const Axis aAxis,       //!< axis for changing the activity detection status
    const bool aEnable      //!< true for enabling the activity detection
    )
{
    bool status = ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t activityEnReg = 0;
        status = readByteReg( ADXL355_357_REG_ACT_EN, &activityEnReg );

        if( status )
        {
            switch( aAxis )
            {
                case AXIS_X:
                    if( aEnable )
                    {
                        activityEnReg |= ACT_EN_X;
                    }
                    else
                    {
                        activityEnReg &= ~ACT_EN_X;
                    }
                    break;

                case AXIS_Y:
                    if( aEnable )
                    {
                        activityEnReg |= ACT_EN_Y;
                    }
                    else
                    {
                        activityEnReg &= ~ACT_EN_Y;
                    }
                    break;

                case AXIS_Z:
                    if( aEnable )
                    {
                        activityEnReg |= ACT_EN_Z;
                    }
                    else
                    {
                        activityEnReg &= ~ACT_EN_Z;
                    }
                    break;

                default:
                    break;
            }

            status = writeByteReg( ADXL355_357_REG_ACT_EN, activityEnReg );
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Enable or disable the external clock
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the external clock can be enabled or disabled
//!************************************************************************
bool Adxl355Adxl357Common::enableExternalClk
    (
    const bool aEnable      //!< true for enabling the external CLK
    )
{
    bool status = true;
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t syncReg = 0;
        status = readByteReg( ADXL355_357_REG_SYNC, &syncReg );

        if( status )
        {
            if( aEnable )
            {
                syncReg |= SYNC_EXT_CLK;
            }
            else
            {
                syncReg &= ~SYNC_EXT_CLK;
            }

            status = writeByteReg( ADXL355_357_REG_SYNC, syncReg );
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Enable or disable an interrupt source for the specified pin
//!
//! This change does not require standby mode to be active.
//!
//! @returns true if the interrupt source can be changed
//!************************************************************************
bool Adxl355Adxl357Common::enableInterruptSource
    (
    const InterruptPin    aPin,         //!< interrupt pin whose functionality to change
    const InterruptSource aSource,      //!< source of the interrupt
    const bool            aEnable       //!< true for enabling
    )
{
    bool status = ( aPin >= INTERRUPT_PIN_1 ) && ( aPin <= INTERRUPT_PIN_2 )
               && ( aSource >= INTERRUPT_SOURCE_ACTIVITY ) && ( aSource <= INTERRUPT_SOURCE_DATA_RDY );

    if( status )
    {
        uint8_t interruptMapReg = 0;
        status = readByteReg( ADXL355_357_REG_INT_MAP, &interruptMapReg );

        if( status )
        {
            uint8_t changeMask = 0;

            switch( aSource )
            {
                case INTERRUPT_SOURCE_ACTIVITY:
                    changeMask = INT_MAP_ACT_EN2 | INT_MAP_ACT_EN1;
                    break;

                case INTERRUPT_SOURCE_FIFO_OVR:
                    changeMask = INT_MAP_OVR_EN2 | INT_MAP_OVR_EN1;
                    break;

                case INTERRUPT_SOURCE_FIFO_FULL:
                    changeMask = INT_MAP_FULL_EN2 | INT_MAP_FULL_EN1;
                    break;

                case INTERRUPT_SOURCE_DATA_RDY:
                    changeMask = INT_MAP_RDY_EN2 | INT_MAP_RDY_EN1;
                    break;

                default:
                    break;
            }

            switch( aPin )
            {
                case INTERRUPT_PIN_2:
                    changeMask &= ( INT_MAP_ACT_EN2 | INT_MAP_OVR_EN2 | INT_MAP_FULL_EN2 | INT_MAP_RDY_EN2 );
                    break;

                case INTERRUPT_PIN_1:
                    changeMask &= ( INT_MAP_ACT_EN1 | INT_MAP_OVR_EN1 | INT_MAP_FULL_EN1 | INT_MAP_RDY_EN1 );
                    break;

                default:
                    break;
            }

            if( aEnable )
            {
                interruptMapReg |= changeMask;
            }
            else
            {
                interruptMapReg &= ~changeMask;
            }

            status = writeByteReg( ADXL355_357_REG_INT_MAP, interruptMapReg );
        }
    }

    return status;
}


//!************************************************************************
//! Enable the self test mode
//!
//! This change does not require standby mode to be active.
//!
//! @returns true if self test mode can be set
//!************************************************************************
bool Adxl355Adxl357Common::enableSelfTestMode
    (
    const bool aEnterSelfTest       //!< true for entering self test mode
    )
{
    uint8_t selftestReg = 0;
    bool status = readByteReg( ADXL355_357_REG_SELF_TEST, &selftestReg );

    if( status )
    {
        mSelfTestMode = selftestReg & ( SELF_TEST_MODE_EN | SELF_TEST_FORCE_EN );

        if( aEnterSelfTest != mSelfTestMode )
        {
            if( aEnterSelfTest )
            {
                selftestReg |= ( SELF_TEST_MODE_EN | SELF_TEST_FORCE_EN );
            }
            else
            {
                selftestReg &= ~( SELF_TEST_MODE_EN | SELF_TEST_FORCE_EN );
            }

            status = writeByteReg( ADXL355_357_REG_SELF_TEST, selftestReg );

            if( status )
            {
                mSelfTestMode = aEnterSelfTest;
            }
        }
    }

    return status;
}


//!************************************************************************
//! Enable the standby mode
//!
//! @returns true if standby mode can be set
//!************************************************************************
bool Adxl355Adxl357Common::enableStandbyMode
    (
    const bool aEnterStandby    //!< true for entering standby
    )
{
    uint8_t powerCtlReg = 0;
    bool status = readByteReg( ADXL355_357_REG_POWER_CTL, &powerCtlReg );

    if( status )
    {
        mStandbyMode = powerCtlReg & POWER_CTL_STANDBY_MODE;

        if( aEnterStandby != mStandbyMode )
        {
            if( aEnterStandby )
            {
                powerCtlReg |= POWER_CTL_STANDBY_MODE;
            }
            else
            {
                powerCtlReg &= ~POWER_CTL_STANDBY_MODE;
            }

            status = writeByteReg( ADXL355_357_REG_POWER_CTL, powerCtlReg );

            if( status )
            {
                mStandbyMode = aEnterStandby;
                usleep( 1000 );
            }
        }
    }

    return status;
}


//!************************************************************************
//! Get the acceleration value [g] for the specified axis
//!
//! @returns true if the acceleration can be read
//!************************************************************************
bool Adxl355Adxl357Common::getAccelerationOnAxis
    (
    const Axis aAxis,           //!< axis whose acceleration to get
    double*    aAcceleration    //!< acceleration value [g]
    )
{
    bool status = ( aAcceleration != nullptr ) && ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z );

    if( status )
    {
        *aAcceleration = 0;
        const uint8_t LENGTH = 3;
        uint8_t bArray[LENGTH] = { 0 };

        switch( aAxis )
        {
            case AXIS_X:
                status = readByteArrayReg( ADXL355_357_REG_XDATA3, LENGTH, bArray );
                break;

            case AXIS_Y:
                status = readByteArrayReg( ADXL355_357_REG_YDATA3, LENGTH, bArray );
                break;

            case AXIS_Z:
                status = readByteArrayReg( ADXL355_357_REG_ZDATA3, LENGTH, bArray );
                break;

            default:
                break;
        }

        if( status )
        {
            bool isNegative = bArray[0] & 0x80;
            int32_t rawAcceleration = ( bArray[0] << 16 ) | ( bArray[1] << 8 ) | bArray[2];
            rawAcceleration >>= 4;

            if( isNegative )
            {
                rawAcceleration |= 0xfff00000;
            }

            switch( aAxis )
            {
                case AXIS_X:
                    *aAcceleration = rawAcceleration * mLsbToG.x;
                    break;

                case AXIS_Y:
                    *aAcceleration = rawAcceleration * mLsbToG.y;
                    break;

                case AXIS_Z:
                    *aAcceleration = rawAcceleration * mLsbToG.z;
                    break;

                default:
                    break;
            }

            compensateOffsetTemperature( aAxis, aAcceleration );
        }
    }

    return status;
}


//!************************************************************************
//! Get the acceleration values [g] for all three axes in one call
//!
//! @returns true if the accelerations can be read
//!************************************************************************
bool Adxl355Adxl357Common::getAccelerationsOnAllAxes
    (
    double* aXAcceleration,     //!< acceleration value on X axis [g]
    double* aYAcceleration,     //!< acceleration value on Y axis [g]
    double* aZAcceleration      //!< acceleration value on Z axis [g]
    )
{
    bool status = ( aXAcceleration != nullptr ) && ( aYAcceleration != nullptr ) && ( aZAcceleration != nullptr );

    if( status )
    {
        *aXAcceleration = 0;
        *aYAcceleration = 0;
        *aZAcceleration = 0;
        const uint8_t LENGTH = 9;
        uint8_t bArray[LENGTH] = { 0 };
        status = readByteArrayReg( ADXL355_357_REG_XDATA3, LENGTH, bArray );

        if( status )
        {
            const int32_t NEG_MASK = 0xfff00000;

            //////////////////////
            /// X axis
            //////////////////////
            bool isNegativeX = bArray[0] & 0x80;
            int32_t rawAccelerationX = ( bArray[0] << 16 ) | ( bArray[1] << 8 ) | bArray[2];
            rawAccelerationX >>= 4;

            if( isNegativeX )
            {
                rawAccelerationX |= NEG_MASK;
            }

            *aXAcceleration = rawAccelerationX * mLsbToG.x;
            compensateOffsetTemperature( AXIS_X, aXAcceleration );

            //////////////////////
            /// Y axis
            //////////////////////
            bool isNegativeY = bArray[3] & 0x80;
            int32_t rawAccelerationY = ( bArray[3] << 16 ) | ( bArray[4] << 8 ) | bArray[5];
            rawAccelerationY >>= 4;

            if( isNegativeY )
            {
                rawAccelerationY |= NEG_MASK;
            }

            *aYAcceleration = rawAccelerationY * mLsbToG.y;
            compensateOffsetTemperature( AXIS_Y, aYAcceleration );

            //////////////////////
            /// Z axis
            //////////////////////
            bool isNegativeZ = bArray[6] & 0x80;
            int32_t rawAccelerationZ = ( bArray[6] << 16 ) | ( bArray[7] << 8 ) | bArray[8];
            rawAccelerationZ >>= 4;

            if( isNegativeZ )
            {
                rawAccelerationZ |= NEG_MASK;
            }

            *aZAcceleration = rawAccelerationZ * mLsbToG.z;
            compensateOffsetTemperature( AXIS_Z, aZAcceleration );
        }
    }

    return status;
}


//!************************************************************************
//! Get the cross-axis sensitivity (typ 1%)
//!
//! @returns the value of cross-axis sensitivity
//!************************************************************************
double Adxl355Adxl357Common::getCrossAxisSensitivity() const
{
    return CROSS_AXIS_SENSITIVITY;
}


//!************************************************************************
//! Get the FIFO data
//!
//! @returns true if FIFO data can be read
//!************************************************************************
bool Adxl355Adxl357Common::getFifoData
    (
    uint8_t* aData      //!< FIFO data
    )
{
    bool status = ( aData != nullptr );

    if( status )
    {
        *aData = 0;
        uint8_t fifoDataReg = 0;
        status = readByteReg( ADXL355_357_REG_FIFO_DATA, &fifoDataReg );

        if( status )
        {
            *aData = fifoDataReg;
        }
    }

    return status;
}


//!************************************************************************
//! Get the number of samples stored in the FIFO
//!
//! This is the number of samples which triggers the FIFO_FULL condition.
//!
//! @returns true if the number of samples stored in the FIFO can be read
//!************************************************************************
bool Adxl355Adxl357Common::getFifoSamplesSize
    (
    uint8_t* aSize      //!< number of samples stored in the FIFO
    )
{
    bool status = ( aSize != nullptr );

    if( status )
    {
        *aSize = 0;
        uint8_t fifoSamplesReg = 0;
        status = readByteReg( ADXL355_357_REG_FIFO_SAMPLES, &fifoSamplesReg );

        if( status )
        {
            uint8_t samplesSize = fifoSamplesReg & FIFO_SAMPLES;
            status = ( samplesSize >= 1 ) && ( samplesSize <= FIFO_MAX_COUNT );

            if( status )
            {
                mFifoSamplesCount = samplesSize;
                *aSize = mFifoSamplesCount;
            }
        }
    }

    return status;
}


//!************************************************************************
//! Get the number of valid data samples in FIFO
//!
//! @returns true if the number of valid samples from FIFO can be read
//!************************************************************************
bool Adxl355Adxl357Common::getFifoValidSamplesCount
    (
    uint8_t* aNumber    //!< number of valid data samples in the FIFO
    )
{
    bool status = ( aNumber != nullptr );

    if( status )
    {
        *aNumber = 0;
        uint8_t fifoEntriesReg = 0;
        status = readByteReg( ADXL355_357_REG_FIFO_ENTRIES, &fifoEntriesReg );

        if( status )
        {
            uint8_t validSamplesCount = fifoEntriesReg & FIFO_ENTRIES;
            status = validSamplesCount <= FIFO_MAX_COUNT;

            if( status )
            {
                *aNumber = validSamplesCount;
            }
        }
    }

    return status;
}


//!************************************************************************
//! Get the step angle [rad] corresponding to a given inclination [rad]
//!
//! @returns true if the inclination step angle can be calculated
//!************************************************************************
bool Adxl355Adxl357Common::getInclinationStepAngle
    (
    const Axis   aAxis,                     //!< axis
    const double aInclinationAngle,         //!< inclination angle [rad]
    double*      aInclinationStepAngle      //!< step angle [rad]
    ) const
{
    const double PI_2 = 2.0 * atan( 1.0 );
    double a = fabs( aInclinationAngle );
    bool status = ( aInclinationStepAngle != nullptr ) && ( a <= PI_2 );

    if( status )
    {
        double t0 = sin( a );

        switch( aAxis )
        {
            case AXIS_X:
                t0 += mLsbToG.x;
                break;

            case AXIS_Y:
                t0 += mLsbToG.y;
                break;

            case AXIS_Z:
                t0 += mLsbToG.z;
                break;

            default:
                break;
        }

        double t1 = t0 * t0;
        double t2 = pow( cos( a ), 2.0 ) + pow( t0 * sin( a ), 2.0 );
        double gamma = t0 * sin( a );

        if( t2 > t1 )
        {
            gamma += sqrt( t2 - t1 );
        }

        *aInclinationStepAngle = ( gamma < 1 ) ? acos( gamma ) : 0;
    }

    return status;
}


//!************************************************************************
//! Get the combined interpolator + decimator ODR delay [s]
//!
//! @returns The combined ODR delay
//!************************************************************************
double Adxl355Adxl357Common::getOdrDelay() const
{
    return mOdr.combinedDelay;
}


//!************************************************************************
//! Get the ODR frequency [Hz]
//!
//! @returns The ODR frequency
//!************************************************************************
double Adxl355Adxl357Common::getOdrFrequency() const
{
    return mOdr.freq;
}


//!************************************************************************
//! Get the ODR HPF -3dB corner frequency [Hz]
//!
//! @returns The HPF -3dB frequency
//!************************************************************************
double Adxl355Adxl357Common::getOdrHpfCorner() const
{
    return mHpfFrequency;
}


//!************************************************************************
//! Get the ODR LPF -3dB corner frequency [Hz]
//!
//! @returns The LPF -3dB frequency
//!************************************************************************
double Adxl355Adxl357Common::getOdrLpfCorner() const
{
    return mOdr.lpfCorner;
}


//!************************************************************************
//! Get the chip revision ID
//!
//! @returns The chip revision ID
//!************************************************************************
uint8_t Adxl355Adxl357Common::getRevId() const
{
    return mRevId;
}


//!************************************************************************
//! Get the standby mode
//!
//! @returns true if the status can be read
//!************************************************************************
bool Adxl355Adxl357Common::getStandbyMode
    (
    bool* aEnabled      //!< true if standby mode is enabled
    )
{
    bool status = ( aEnabled != nullptr );

    if( status )
    {
        *aEnabled = false;
        uint8_t powerCtlReg = 0;
        status = readByteReg( ADXL355_357_REG_POWER_CTL, &powerCtlReg );

        if( status )
        {
            mStandbyMode = powerCtlReg & POWER_CTL_STANDBY_MODE;
            *aEnabled = mStandbyMode;
        }
    }

    return status;
}


//!************************************************************************
//! Get the data synchronization mode
//!
//! @returns true if the data synchronization mode can be read
//!************************************************************************
bool Adxl355Adxl357Common::getSyncType
    (
    SyncType* aSyncType     //!< data synchronization type
    )
{
    bool status = ( aSyncType != nullptr );

    if( status )
    {
        *aSyncType = SYNC_TYPE_INTERNAL;
        uint8_t syncReg = 0;
        status = readByteReg( ADXL355_357_REG_SYNC, &syncReg );

        if( status )
        {
            switch( syncReg & SYNC_CTRL_EN )
            {
                case SYNC_CTRL_INTERNAL:
                    *aSyncType = SYNC_TYPE_INTERNAL;
                    break;

                case SYNC_CTRL_EXT_NO_INTERP:
                    *aSyncType = SYNC_TYPE_EXTERNAL_NO_INTERPOLATION;
                    break;

                case SYNC_CTRL_EXT_INTERP:
                    *aSyncType = SYNC_TYPE_EXTERNAL_WITH_INTERPOLATION;
                    break;

                default:
                    break;
            }
        }
    }

    return status;
}


//!************************************************************************
//! Get the temperature off status
//!
//! @returns true if the temperature status can be read
//!************************************************************************
bool Adxl355Adxl357Common::getTemperatureOff
    (
    bool* aEnabled      //!< true if temperature processing is off
    )
{
    bool status = ( aEnabled != nullptr );

    if( status )
    {
        *aEnabled = false;
        uint8_t powerCtlReg = 0;
        status = readByteReg( ADXL355_357_REG_POWER_CTL, &powerCtlReg );

        if( status )
        {
            mTemperatureOff = powerCtlReg & POWER_CTL_TEMP_OFF;
            *aEnabled = mTemperatureOff;
        }
    }

    return status;
}


//!************************************************************************
//! Get the temperature value from embedded sensor
//!
//! The function enables all masks required to read valid data, then restores
//! these masks.
//!
//! @returns true if temperature value can be read
//!************************************************************************
bool Adxl355Adxl357Common::getTemperatureValue
    (
    double* aTemperature    //!< temperature [C]
    )
{
    bool status = ( aTemperature != nullptr );

    if( status )
    {
        const double TEMP_INVALID_C = -273.15;
        *aTemperature = TEMP_INVALID_C;

        ////////////////////////////
        /// enable modes if needed
        ////////////////////////////
        bool standbyMode = true;
        status = getStandbyMode( &standbyMode );

        if( status )
        {
            if( standbyMode )
            {
                status = enableStandbyMode( false );
            }
        }

        bool temperatureOff = false;

        if( status )
        {
            status = getTemperatureOff( &temperatureOff );
        }

        if( status )
        {
            if( temperatureOff )
            {
                setTemperatureOff( false );
            }
        }

        ////////////////////////////
        /// get data
        ////////////////////////////
        if( status )
        {
            uint16_t tempRaw = 0;
            readWordReg( ADXL355_357_REG_TEMP2, &tempRaw );
            tempRaw &= 0x0fff;

            const uint16_t T0_INTERCEPT = 1885;     // [LSB]
            const double   T0_SLOPE = -9.05;        // [LSB/C]
            *aTemperature = TEMP_25_C + ( tempRaw - T0_INTERCEPT ) / T0_SLOPE;

            mTemperatureC = *aTemperature;

            updateLsbToG();
        }

        ////////////////////////////
        /// restore modes if needed
        ////////////////////////////
        if( status )
        {
            if( temperatureOff != mTemperatureOff )
            {
                status = setTemperatureOff( temperatureOff );
            }
        }

        if( status )
        {
            if( standbyMode != mStandbyMode )
            {
                status = enableStandbyMode( standbyMode );
            }
        }
    }

    return status;
}


//!************************************************************************
//! Initialize the accelerometer
//!
//! @returns true if initialization can be done
//!************************************************************************
bool Adxl355Adxl357Common::init
    (
    const uint8_t aI2cChannel,      //!< I2C channel
    const uint8_t aDeviceAddress    //!< 7-bit I2C device address
    )
{
    bool status = false;

    if( ioctl( aI2cChannel, I2C_SLAVE, aDeviceAddress ) >= 0 )
    {
        mI2cChannel = aI2cChannel;
        status = true;
    }

    uint8_t readByte = 0;

    if( status )
    {
        status = readByteReg( ADXL355_357_REG_DEVID_AD, &readByte );

        if( status )
        {
            const uint8_t DEVID_AD = 0xad;     //!< Analog Devices ID
            status = ( DEVID_AD == readByte );
        }
    }

    if( status )
    {
        status = readByteReg( ADXL355_357_REG_DEVID_MST, &readByte );

        if( status )
        {
            const uint8_t DEVID_MST = 0x1d;     //!< Analog Devices MEMS ID
            status = ( DEVID_MST == readByte );
        }
    }

    if( status )
    {
        status = readByteReg( ADXL355_357_REG_PART_ID, &readByte );

        if( status )
        {
            const uint8_t PART_ID = 0xed;     //!< Analog Devices part ID (common for ADXL355 and ADXL357)
            status = ( PART_ID == readByte );
        }
    }

    if( status )
    {
        status = readByteReg( ADXL355_357_REG_REV_ID, &readByte );

        if( status )
        {
            mRevId = readByte;
        }
    }

    if( status )
    {
        status = readByteReg( ADXL355_357_REG_POWER_CTL, &readByte );

        if( status )
        {
            mStandbyMode = readByte & POWER_CTL_STANDBY_MODE;
            mTemperatureOff = readByte & POWER_CTL_TEMP_OFF;
        }
    }

    if( status )
    {
        double tempC = 0;
        getTemperatureValue( &tempC );

        updateLsbToG();
    }

    return status;
}


//!************************************************************************
//! Initialize the ODR parameters list
//!
//! @returns nothing
//!************************************************************************
void Adxl355Adxl357Common::initOdrList()
{
    memset( &mOdrList, 0, sizeof( mOdrList ) );

    for( uint8_t i = 0; i < ODR_SETTINGS_COUNT; i++ )
    {
        //////////////////////////////////////////////////
        /// ODR frequency [Hz]
        /// //////////////////////////////////////////////////
        mOdrList[i].freq = 4000.0 / ( 1 << i );

        //////////////////////////////////////////////////
        /// LPF -3dB corner [Hz]
        //////////////////////////////////////////////////
        mOdrList[i].lpfCorner = 0.25 * mOdrList[i].freq;

        //////////////////////////////////////////////////
        /// combined (interpolator/decimator) delay [s]
        //////////////////////////////////////////////////
        double a = 0;

        if( mOdrList[i].lpfCorner >= 30 )
        {
            ////////////////////////////////////////////////////
            /// seismic / industrial / CbM
            /// LPF = { 1000,  500,  250, 125,  62.5, 31.25 } [Hz]
            ////////////////////////////////////////////////////
            a = 500.0 / mOdrList[i].lpfCorner;
            mOdrList[i].combinedDelay = 0.23234 + 1.2593 * a + 0.03378 * pow( 0.3657, a );
        }
        else
        {
            ////////////////////////////////////////////////////
            /// mainly infrasound
            /// LPF = { 15.625, 7.8125, 3.90625, 1.953125, 0.9765625 } [Hz]
            ////////////////////////////////////////////////////
            a = 250.0 / mOdrList[i].lpfCorner;
            mOdrList[i].combinedDelay = 0.30297 + 2.51614 * a - 2.7142794709e6 * pow( 0.06848, a );
        }

        // [ms] -> [s]
        mOdrList[i].combinedDelay *= 1.e-3;

        //////////////////////////////////////////////////
        /// FILTER register mask
        //////////////////////////////////////////////////
        mOdrList[i].filterReg = static_cast<Filter>( i );
    }
}


//!************************************************************************
//! Check if the activity detected flag is set
//!
//! Existence of activity is defined by ACT_COUNT, ACT_THRESH_H, ACT_THRESH_L,
//! and ACT_EN_(X/Y/Z).
//!
//! @returns true if the activity detection status can be get
//!************************************************************************
bool Adxl355Adxl357Common::isActivityDetected
    (
    bool* aDetection    //!< true if activity is detected
    ) const
{
    bool status = ( aDetection != nullptr );

    if( status )
    {
        *aDetection = false;
        uint8_t statusReg = 0;
        status = readByteReg( ADXL355_357_REG_STATUS, &statusReg );

        if( status )
        {
            *aDetection = statusReg & STATUS_ACTIVITY;
        }
    }

    return status;
}


//!************************************************************************
//! Check if a complete x-, y-, and z-axis measurement was made and results
//! can be read (data ready flag is set)
//!
//! @returns true if the data ready status can be get
//!************************************************************************
bool Adxl355Adxl357Common::isDataReady
    (
    bool* aDataReady    //!< true if data is ready
    ) const
{
    bool status = ( aDataReady != nullptr );

    if( status )
    {
        *aDataReady = false;
        uint8_t statusReg = 0;
        status = readByteReg( ADXL355_357_REG_STATUS, &statusReg );

        if( status )
        {
            *aDataReady = statusReg & STATUS_DATA_RDY;
        }
    }

    return status;
}


//!************************************************************************
//! Check if the external clock is enabled
//!
//! @returns true if the external clock status can be get
//!************************************************************************
bool Adxl355Adxl357Common::isExternalClkEnabled
    (
    bool* aEnabled      //!< true if external clock is enabled
    ) const
{
    bool status = ( aEnabled != nullptr );

    if( status )
    {
        *aEnabled = false;
        uint8_t syncReg = 0;
        status = readByteReg( ADXL355_357_REG_SYNC, &syncReg );

        if( status )
        {
            *aEnabled = syncReg & SYNC_EXT_CLK;
        }
    }

    return status;
}


//!************************************************************************
//! Check if FIFO overrange flag is set
//!
//! @returns true if FIFO overrange status can be get
//!************************************************************************
bool Adxl355Adxl357Common::isFifoOvr
    (
    bool* aFifoOvr      //!< true if FIFO overrange
    ) const
{
    bool status = ( aFifoOvr != nullptr );

    if( status )
    {
        *aFifoOvr = false;
        uint8_t statusReg = 0;
        status = readByteReg( ADXL355_357_REG_STATUS, &statusReg );

        if( status )
        {
            *aFifoOvr = statusReg & STATUS_FIFO_OVR;
        }
    }

    return status;
}


//!************************************************************************
//! Check if the FIFO full flag is set
//!
//! @returns true if FIFO full status can be get
//!************************************************************************
bool Adxl355Adxl357Common::isFifoFull
    (
    bool* aFifoFull     //!< true if FIFO is full
    ) const
{
    bool status = ( aFifoFull != nullptr );

    if( status )
    {
        *aFifoFull = false;
        uint8_t statusReg = 0;
        status = readByteReg( ADXL355_357_REG_STATUS, &statusReg );

        if( status )
        {
            *aFifoFull = statusReg & STATUS_FIFO_FUL;
        }
}

    return status;
}


//!************************************************************************
//! Check if the NVM is busy
//!
//! @returns true if the NVM busy status can be get
//!************************************************************************
bool Adxl355Adxl357Common::isNvmBusy
    (
    bool* aNvmBusy      //!< true if NVM is busy
    ) const
{
    bool status = ( aNvmBusy != nullptr );

    if( status )
    {
        *aNvmBusy = false;
        uint8_t statusReg = 0;
        status = readByteReg( ADXL355_357_REG_STATUS, &statusReg );

        if( status )
        {
            *aNvmBusy = statusReg & STATUS_NVM_BUSY;
        }
    }

    return status;
}


//!************************************************************************
//! Read a byte array from a start register address
//!
//! @returns true if the byte array can be read
//!************************************************************************
bool Adxl355Adxl357Common::readByteArrayReg
    (
    const uint8_t aAddress,     //!< start register address
    const uint8_t aLength,      //!< array length
    uint8_t* 	  aByteArray    //!< byte array read
    ) const
{
    bool status = ( aByteArray != nullptr );

    if( status )
    {
        uint8_t txBuffer[1] = { aAddress };
        status = ( sizeof( txBuffer ) == write( mI2cChannel, txBuffer, sizeof( txBuffer ) ) );
    }

    if( status )
    {
        uint8_t rxBuffer[aLength] = { 0 };

        if( sizeof( rxBuffer ) == read( mI2cChannel, rxBuffer, sizeof( rxBuffer ) ) )
        {
            memcpy( aByteArray, rxBuffer, aLength );
            status = true;
        }
    }

    return status;
}


//!************************************************************************
//! Read one byte from a register
//!
//! @returns true if the byte value can be read
//!************************************************************************
bool Adxl355Adxl357Common::readByteReg
    (
    const uint8_t aAddress,     //!< register address
    uint8_t* 	  aValue        //!< byte read
    ) const
{
    bool status = ( aValue != nullptr );
    uint8_t buffer[1] = { aAddress };

    if( status )
    {
        status = ( sizeof( buffer ) == write( mI2cChannel, buffer, sizeof( buffer ) ) );
    }

    if( status )
    {
        if( sizeof( buffer ) == read( mI2cChannel, buffer, sizeof( buffer ) ) )
        {
            *aValue = buffer[0];
            status = true ;
        }
    }

    return status;
}


//!************************************************************************
//! Read a word from two consecutive registers
//!
//! @returns true if the word value can be read
//!************************************************************************
bool Adxl355Adxl357Common::readWordReg
    (
    const uint8_t aAddress,     //!< register address
    uint16_t* 	  aValue        //!< word read
    ) const
{
    bool status = ( aValue != nullptr );

    if( status )
    {
        uint8_t txBuffer[1] = { aAddress };
        status = ( sizeof( txBuffer ) == write( mI2cChannel, txBuffer, sizeof( txBuffer ) ) );
    }

    if( status )
    {
        uint8_t rxBuffer[2] = { 0 };

        if( sizeof( rxBuffer ) == read( mI2cChannel, rxBuffer, sizeof( rxBuffer ) ) )
        {
            *aValue = ( rxBuffer[0] << 8 ) | rxBuffer[1];
            status = true;
        }
    }

    return status;
}


//!************************************************************************
//! Reset the device, similar to a power-on reset
//!
//! Soft reset does not require standby mode to be active.
//!
//! Five shadow registers are compared before and after soft reset if the
//! chip Rev ID is up to 1.
//!
//! @returns true if the reset command can be given
//!************************************************************************
bool Adxl355Adxl357Common::reset()
{
    bool status = true;
    const uint8_t RESET_CODE = 0x52;

    if( mRevId > 1 )
    {
        status = writeByteReg( ADXL355_357_REG_RESET, RESET_CODE );
    }
    else
    {
        const uint8_t REG_SHADOW_COUNT = 1 + ADXL355_357_REG_SHADOW_LAST - ADXL355_357_REG_SHADOW_FIRST;
        uint8_t beforeShadowRegArray[REG_SHADOW_COUNT] = { 0 };
        const uint8_t COMPARE_COUNT_MAX = 5;
        uint8_t i = 0;

        for( i = 0; i < COMPARE_COUNT_MAX; i++ )
        {
            status = readByteArrayReg( ADXL355_357_REG_SHADOW_FIRST, REG_SHADOW_COUNT, beforeShadowRegArray );

            if( status )
            {
                status = writeByteReg( ADXL355_357_REG_RESET, RESET_CODE );

                if( status )
                {
                    usleep( 1000 );

                    uint8_t afterShadowRegArray[REG_SHADOW_COUNT] = { 0 };
                    status = readByteArrayReg( ADXL355_357_REG_SHADOW_FIRST, REG_SHADOW_COUNT, afterShadowRegArray );

                    if( status )
                    {
                        if( 0 == memcmp( beforeShadowRegArray, afterShadowRegArray, REG_SHADOW_COUNT ) )
                        {
                            break;
                        }
                    }
                }
                else
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }

        if( COMPARE_COUNT_MAX == i )
        {
            status = false;
        }
    }

    usleep( 1000 );

    // reinitialize power-on members
    mStandbyMode = true;
    mSelfTestMode = false;
    mTemperatureOff = false;
    mTemperatureC = TEMP_25_C;
    mRange = ACCELERATION_RANGE_DEFAULT;
    mActivityThreshold = 0;
    mOdrSetting = ODR_SETTING_4000;
    mHpfSetting = HPF_SETTING_DC;
    mFifoSamplesCount = FIFO_MAX_COUNT;

    enableStandbyMode( mStandbyMode );
    enableSelfTestMode( mSelfTestMode );
    setTemperatureOff( mTemperatureOff );
    setAccelerationRange( mRange );
    updateLsbToG();
    setActivityThreshold( mActivityThreshold );
    updateOdrBySelection();
    updateHpfBySelection();
    setFifoSamplesSize( mFifoSamplesCount );

    return status;
}


//!************************************************************************
//! Set the activity count
//!
//! The parameter represents the number of consecutive events above the
//! predefined threshold (common for all three axes) required to signal
//! activity presence.
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the activity count can be set
//!************************************************************************
bool Adxl355Adxl357Common::setActivityCount
    (
    const uint8_t aNumber   //!< number of consecutive events
    )
{
    bool status = ( aNumber >= 1 );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        status = writeByteReg( ADXL355_357_REG_ACT_COUNT, aNumber );
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Set the activity detection threshold [g]
//!
//! This threshold is common to all three axes. The parameter can be given
//! with any sign, but the function will internally use its absolute value.
//! Next values are valid for a temperature of 25 C.
//!
//! +/- 2g  |  A_max  | = 0x7ffff / 256000 =  2.04799609 [g]
//!         | thd_max | = 0x7fff8 / 256000 =  2.04796875 [g]
//!
//! +/- 4g  |  A_max  | = 0x7ffff / 128000 =  4.09599219 [g]
//!         | thd_max | = 0x7fff8 / 128000 =  4.09593750 [g]
//!
//! +/- 8g  |  A_max  | = 0x7ffff /  64000 =  8.19198438 [g]
//!         | thd_max | = 0x7fff8 /  64000 =  8.19187500 [g]
//!
//! +/-10g  |  A_max  | = 0x7ffff /  51200 = 10.23998047 [g]
//!         | thd_max | = 0x7fff8 /  51200 = 10.23984375 [g]
//!
//! +/-20g  |  A_max  | = 0x7ffff /  25600 = 20.47996094 [g]
//!         | thd_max | = 0x7fff8 /  25600 = 20.47968750 [g]
//!
//! +/-40g  |  A_max  | = 0x7ffff /  12800 = 40.95992188 [g]
//!         | thd_max | = 0x7fff8 /  12800 = 40.95937500 [g]
//!
//! @returns true if the activity detection threshold can be set
//!************************************************************************
bool Adxl355Adxl357Common::setActivityThreshold
    (
    const double aThreshold     //!< activity detection threshold [g]
    )
{
    double lsbToGAvg = ( mLsbToG.x + mLsbToG.y + mLsbToG.z ) / 3.0;
    const double ACTIVITY_THD_MAX_G = 0x7fff8 * lsbToGAvg;
    bool status = fabs( aThreshold ) < ACTIVITY_THD_MAX_G;

    if( status )
    {
        mActivityThreshold = fabs( aThreshold );
        status = storeActivityThreshold();
    }

    return status;
}


//!************************************************************************
//! Set the acceleration offset value [g] for a selected axis
//!
//! This function should be called when the accelerometer is sensing only
//! the gravity of Earth, with the axis of interest parallel to the
//! vertical direction.
//!
//! It is *highly recommended* to call this function only at room
//! temperature, ideally close to 25 C. Temperature dependency of the
//! offsets is recalculated automatically, once periodic readings are done
//! during runtime. To trigger a new reading, the calling module should use
//! Adxl355::getTemperatureValue(double*).
//!
//! @returns true if the offset value can be set
//!************************************************************************
bool Adxl355Adxl357Common::setAxisOffset
    (
    const Axis   aAxis,     //!< axis whose offset to change
    const double aValue     //!< offset value [g]
    )
{
    bool status = ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z );

    if( status )
    {
        // see header of checkOffsetRealRange() for detailed values
        double offsetMaxG = 0x7fff0;

        switch( aAxis )
        {
            case AXIS_X:
                offsetMaxG *= mLsbToG.x;
                break;

            case AXIS_Y:
                offsetMaxG *= mLsbToG.y;
                break;

            case AXIS_Z:
                offsetMaxG *= mLsbToG.z;
                break;

            default:
                break;
        }

        status = fabs( aValue ) < offsetMaxG;
    }

    if( status )
    {
        switch( aAxis )
        {
            case AXIS_X:
                mOffsetG.x = aValue;
                break;

            case AXIS_Y:
                mOffsetG.y = aValue;
                break;

            case AXIS_Z:
                mOffsetG.z = aValue;
                break;

            default:
                break;
        }
    }

    if( status )
    {
        status = storeAxisOffset( aAxis );
    }

    return status;
}


//!************************************************************************
//! Set the number of samples to store in the FIFO
//!
//! This is the number of samples which triggers the FIFO_FULL condition.
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the number of samples to store in the FIFO can be set
//!************************************************************************
bool Adxl355Adxl357Common::setFifoSamplesSize
    (
    const uint8_t aSize     //!< number of samples to store in the FIFO
    )
{
    bool status = ( aSize >= 1 ) && ( aSize <= FIFO_MAX_COUNT );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t fifoSamplesReg = 0;
        status = readByteReg( ADXL355_357_REG_FIFO_SAMPLES, &fifoSamplesReg );

        if( status )
        {
            fifoSamplesReg &= ~FIFO_SAMPLES;
            fifoSamplesReg |= aSize;
            status = writeByteReg( ADXL355_357_REG_FIFO_SAMPLES, fifoSamplesReg );

            if( status )
            {
                mFifoSamplesCount = aSize;
            }
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Set the High Pass Filter (HPF) setting
//!
//! This change does not require standby mode to be active.
//!
//! @returns true if the HPF setting can be set
//!************************************************************************
bool Adxl355Adxl357Common::setHpf
    (
    const HpfSetting aHpfSetting    //!< HPF setting
    )
{
    bool status = false;

    switch( aHpfSetting )
    {
        case HPF_SETTING_DC:
        case HPF_SETTING_24_7:
        case HPF_SETTING_6_20:
        case HPF_SETTING_1_55:
        case HPF_SETTING_0_38:
        case HPF_SETTING_0_09:
        case HPF_SETTING_0_02:
            status = true;
            break;

        default:
            break;
    }

    if( status )
    {
        status = aHpfSetting != mHpfSetting;
    }

    if( status )
    {
        uint8_t filterReg = 0;
        status = readByteReg( ADXL355_357_REG_FILTER, &filterReg );

        if( status )
        {
            filterReg &= ~FILTER_HPF_CORNER;
            filterReg |= static_cast<uint8_t>( aHpfSetting );
            status = writeByteReg( ADXL355_357_REG_FILTER, filterReg );

            if( status )
            {
                mHpfSetting = aHpfSetting;
                updateHpfBySelection();
            }
        }
    }

    return status;
}


//!************************************************************************
//! Set the High Pass Filter (HPF) frequency [Hz]
//!
//! For a given frequency parameter (e.g. aHpfFrequency = 3.0) the highest
//! supported  high pass frequency is selected (e.g. 2.48336 Hz), so that
//! the desired  value is still contained in the pass band.
//! The "k" coefficient which is firstly obtained is real (1.86 for the
//! above example) and will need to be ceiled to 2 so that the corresponding
//! 2.48336 Hz is below 3.0 Hz.
//!
//! For another example (aHpfFrequency = 2.0), the calculated "k" needs to
//! be ceiled from 2.15 to 3 so that the corresponding 0.6218 Hz is the
//! highest frequency which determines the band containing 2.0 Hz.
//!
//! The given parameter shall be contained in the resulting passband due
//! to LPF and BPF.
//!
//! This change does not require standby mode to be active.
//!
//! @returns true if the HPF frequency can be set
//!************************************************************************
bool Adxl355Adxl357Common::setHpfFrequency
    (
    const double aHpfFrequency      //!< HPF frequency [Hz]
    )
{
    bool status = aHpfFrequency >= 0;

    if( status )
    {
        uint8_t filterReg = 0;
        status = readByteReg( ADXL355_357_REG_FILTER, &filterReg );

        if( status )
        {
            uint8_t hpfSetting = HPF_SETTING_DC;

            if( aHpfFrequency )
            {
                double realK = -log( ( aHpfFrequency / mOdr.freq + 3.9724e-7 ) / 9.83e-3 ) / 1.38072;
                int intK = 0;

                if( realK > 0 )
                {
                    intK = static_cast<int>( ceil( realK ) );

                    if( intK > 6 )
                    {
                        intK = 0;
                    }
                }

                hpfSetting = static_cast<uint8_t>( intK ) << 4;
            }

            filterReg &= ~FILTER_HPF_CORNER;
            filterReg |= static_cast<uint8_t>( hpfSetting );
            status = writeByteReg( ADXL355_357_REG_FILTER, filterReg );

            if( status )
            {
                mHpfSetting = static_cast<HpfSetting>( hpfSetting );
                updateHpfBySelection();
            }
        }
    }

    return status;
}


//!************************************************************************
//! Change the I2C speed mode
//!
//! Supported parameters are HIGH_SPEED (3400 kHz) and FAST (400 kHz).
//! They are defining the upper limits supported by the chip, although the
//! host processor may set lower clock values. Next is the dependency
//! between I2C speed modes and maximum ODRs of ADXL355.
//!
//!    Mode     CLK_max [kHz]   ODR_max [Hz]
//!  --------   -------------  -------------
//!  standard       100            200
//!  fast           400            800      <- max supported for FAST
//!  fast plus     1000           2000
//!  high speed    3400           4000      <- max supported for HIGH_SPEED
//!
//! Switching from HIGH_SPEED to FAST mode will determine a maximum ODR of
//! 800 Hz, meaning that {4000, 2000, 1000} values are not allowed.
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the I2C speed mode can be changed
//!************************************************************************
bool Adxl355Adxl357Common::setI2cMode
    (
    const I2cMode aI2cMode      //!< I2C speed mode
    )
{
    bool status = ( I2C_MODE_HIGH_SPEED == aI2cMode ) || ( I2C_MODE_FAST == aI2cMode );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t rangeReg = 0;
        status = readByteReg( ADXL355_357_REG_RANGE, &rangeReg );

        if( status )
        {
            if( I2C_MODE_HIGH_SPEED == aI2cMode )
            {
                rangeReg |= RANGE_I2C_HS;
            }
            else
            {
                rangeReg &= ~RANGE_I2C_HS;
            }

            status = writeByteReg( ADXL355_357_REG_RANGE, rangeReg );
            const double ODR_MAX_I2C_FAST_MODE_HZ = 800;

            if( status
             && I2C_MODE_HIGH_SPEED != aI2cMode
             && mOdr.freq > ODR_MAX_I2C_FAST_MODE_HZ
              )
            {
                // highest supported ODR below 800 Hz is 500 Hz
                mOdrSetting = ODR_SETTING_500;
                updateOdrBySelection();
                updateHpfBySelection();
            }
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Change the polarity of the interrupt pins (default is active low)
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the polarity for INT pins can be changed
//!************************************************************************
bool Adxl355Adxl357Common::setInterruptPinsPolarity
    (
    const InterruptActive aActiveMode   //!< the active low/high mode to set
    )
{
    bool status = ( INTERRUPT_ACTIVE_LOW == aActiveMode ) || ( INTERRUPT_ACTIVE_HIGH == aActiveMode );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t rangeReg = 0;
        status = readByteReg( ADXL355_357_REG_RANGE, &rangeReg );

        if( status )
        {
            if( INTERRUPT_ACTIVE_HIGH == aActiveMode )
            {
                rangeReg |= RANGE_INT_POL_ACTIVE_H;
            }
            else
            {
                rangeReg &= ~RANGE_INT_POL_ACTIVE_H;
            }

            status = writeByteReg( ADXL355_357_REG_RANGE, rangeReg );
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Set the Output Data Rate (ODR)
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the ODR can be set
//!************************************************************************
bool Adxl355Adxl357Common::setOdr
    (
    const OdrSetting aOdrSetting    //!< ODR setting
    )
{
    bool status = ( aOdrSetting >= ODR_SETTING_4000 ) && ( aOdrSetting <= ODR_SETTING_3_90625 );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t filterReg = 0;
        status = readByteReg( ADXL355_357_REG_FILTER, &filterReg );

        if( status )
        {
            filterReg &= ~FILTER_ODR_LPF;
            filterReg |= static_cast<uint8_t>( aOdrSetting );
            status = writeByteReg( ADXL355_357_REG_FILTER, filterReg );

            if( status )
            {
                mOdrSetting = aOdrSetting;
                updateOdrBySelection();
                updateHpfBySelection();
            }
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Set the data synchronization type
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the sync type can be set
//!************************************************************************
bool Adxl355Adxl357Common::setSyncType
    (
    const SyncType aSyncType    //!< data synchronization type
    )
{
    bool status = ( aSyncType >= SYNC_TYPE_INTERNAL ) && ( aSyncType <= SYNC_TYPE_EXTERNAL_WITH_INTERPOLATION );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t syncReg = 0;
        status = readByteReg( ADXL355_357_REG_SYNC, &syncReg );

        if( status )
        {
            syncReg &= ~SYNC_CTRL_EN;

            switch( aSyncType )
            {
                case SYNC_TYPE_INTERNAL:
                    syncReg |= SYNC_CTRL_INTERNAL;
                    break;

                case SYNC_TYPE_EXTERNAL_NO_INTERPOLATION:
                    syncReg |= SYNC_CTRL_EXT_NO_INTERP;
                    break;

                case SYNC_TYPE_EXTERNAL_WITH_INTERPOLATION:
                    syncReg |= SYNC_CTRL_EXT_INTERP;
                    break;

                default:
                    break;
            }

            status = writeByteReg( ADXL355_357_REG_SYNC, syncReg );
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Set the temperature off status
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the temperature status can be set
//!************************************************************************
bool Adxl355Adxl357Common::setTemperatureOff
    (
    const bool aTemperatureOff      //!< true for disabling temperature processing
    )
{
    bool status = true;
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    status = getStandbyMode( &standbyMode );

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// set new value
    ////////////////////////////
    if( status )
    {
        uint8_t powerCtlReg = 0;
        status = readByteReg( ADXL355_357_REG_POWER_CTL, &powerCtlReg );

        if( status )
        {
            mTemperatureOff = powerCtlReg & POWER_CTL_TEMP_OFF;

            if( aTemperatureOff != mTemperatureOff )
            {
                if( aTemperatureOff )
                {
                    powerCtlReg |= POWER_CTL_TEMP_OFF;
                }
                else
                {
                    powerCtlReg &= ~POWER_CTL_TEMP_OFF;
                }

                status = writeByteReg( ADXL355_357_REG_POWER_CTL, powerCtlReg );

                if( status )
                {
                    mTemperatureOff = aTemperatureOff;
                }
            }
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Write in registers the activity threshold
//!
//! A call to this function is required whenever mActivityThreshold needs
//! to be converted again in register ACT_THRESH format, e.g. due to an
//! acceleration range switch.
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the activity threshold can be stored
//!************************************************************************
bool Adxl355Adxl357Common::storeActivityThreshold()
{
    bool status = ( mActivityThreshold >= 0 );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// get the new value and write it to registers
    ////////////////////////////
    if( status )
    {
        double lsbToGAvg = ( mLsbToG.x + mLsbToG.y + mLsbToG.z ) / 3.0;
        uint32_t u32ActivityThd = mActivityThreshold / lsbToGAvg;
        uint16_t u16ActivityThd = ( u32ActivityThd >> 3 ) & 0xffff;

        if( status )
        {
            status = writeWordReg( ADXL355_357_REG_ACT_THRESH_H, u16ActivityThd );
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Write in registers the offset for a specified axis
//!
//! A call to this function is required whenever mOffsetG needs to be
//! converted again in register OFFSET format, e.g. due to an acceleration
//! range switch.
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the axis offset can be stored
//!************************************************************************
bool Adxl355Adxl357Common::storeAxisOffset
    (
    const Axis aAxis    //!< axis whose offset to store
    )
{
    bool status = ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z );
    bool standbyMode = false;

    ////////////////////////////
    /// get standby mode
    ////////////////////////////
    if( status )
    {
        status = getStandbyMode( &standbyMode );
    }

    if( status )
    {
        if( !standbyMode )
        {
            status = enableStandbyMode( true );
        }
    }

    ////////////////////////////
    /// get the new value and write it to registers
    ////////////////////////////
    if( status )
    {
        int16_t intOffset = 0;
        status = convertOffsetRealToInt( aAxis, &intOffset );

        if( status )
        {
            switch( aAxis )
            {
                case AXIS_X:
                    status = writeWordReg( ADXL355_357_REG_OFFSET_X_H, intOffset );
                    break;

                case AXIS_Y:
                    status = writeWordReg( ADXL355_357_REG_OFFSET_Y_H, intOffset );
                    break;

                case AXIS_Z:
                    status = writeWordReg( ADXL355_357_REG_OFFSET_Z_H, intOffset );
                    break;

                default:
                    break;
            }
        }
    }

    ////////////////////////////
    /// restore standby if needed
    ////////////////////////////
    if( status )
    {
        if( standbyMode != mStandbyMode )
        {
            status = enableStandbyMode( standbyMode );
        }
    }

    return status;
}


//!************************************************************************
//! Update the HPF as a function of selected entry
//!
//! @returns nothing
//!************************************************************************
void Adxl355Adxl357Common::updateHpfBySelection()
{
    uint8_t k = ( mHpfSetting >> 4 ) & 0x07;
    mHpfFrequency = 0;

    if( k >= 1 && k <= 6 )
    {
        mHpfFrequency = mOdr.freq * ( -3.9724e-7 + 9.83e-3 * exp( -1.38072 * k ) );
    }
}


//!************************************************************************
//! Update the ODR information as a function of selected entry
//!
//! @returns nothing
//!************************************************************************
void Adxl355Adxl357Common::updateOdrBySelection()
{
    for( uint8_t i = 0; i < ODR_SETTINGS_COUNT; i++ )
    {
        if( mOdrList[i].filterReg == static_cast<Filter>( mOdrSetting ) )
        {
            memcpy( &mOdr, &mOdrList[i], sizeof( mOdr ) );
            break;
        }
    }
}


//!************************************************************************
//! Write one byte to a register
//!
//! @returns true if the byte value can be written
//!************************************************************************
bool Adxl355Adxl357Common::writeByteReg
    (
    const uint8_t aAddress,     //!< register address
    const uint8_t aValue        //!< byte to write
    ) const
{
    uint8_t txBuffer[2] = { aAddress, aValue };
    bool status = ( sizeof( txBuffer ) == write( mI2cChannel, txBuffer, sizeof( txBuffer ) ) );

    return status;
}


//!************************************************************************
//! Write one word to two consecutive registers
//!
//! @returns true if the word value can be written
//!************************************************************************
bool Adxl355Adxl357Common::writeWordReg
    (
    const uint8_t  aAddress,    //!< register address
    const uint16_t aValue       //!< word to write
    ) const
{
    uint8_t txBuffer[3] = { aAddress,
                            static_cast<uint8_t>( aValue >> 8 ),
                            static_cast<uint8_t>( aValue & 0x00ff ) };
    bool status = ( sizeof( txBuffer ) == write( mI2cChannel, txBuffer, sizeof( txBuffer ) ) );

    return status;
}
