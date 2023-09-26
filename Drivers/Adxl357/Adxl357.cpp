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
Adxl357.cpp

This file contains the sources for the ADXL357 driver.
*/

#include "Adxl357.h"

#include <cmath>
#include <unistd.h>


Adxl357* Adxl357::sInstance = nullptr;

//!************************************************************************
//! Constructor
//!************************************************************************
Adxl357::Adxl357()
    : Adxl355Adxl357Common( ACCELERATION_RANGE_DEFAULT )
{   
}


//!************************************************************************
//! Destructor
//!************************************************************************
Adxl357::~Adxl357()
{
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
Adxl357* Adxl357::getInstance()
{
    if( !sInstance )
    {
        sInstance = new Adxl357;
    }

    return sInstance;
}


//!************************************************************************
//! Compensate the offset dependency on temperature
//!
//! If zero-g offsets are valid for 25 C, without this type of compensation
//! acceleration readings are accurate only at the same temperature. If
//! away from room temperature (e.g. -40 C or +125 C), acceleration
//! readings may have shifts of a few [mg] from real values.
//!
//! @returns true if the offset can be temperature compensated
//!************************************************************************
/* virtual */ bool Adxl357::compensateOffsetTemperature
    (
    const Axis aAxis,           //!< axis whose offset to compensate
    double*    aAcceleration    //!< acceleration value [g]
    )
{
    bool status = ( aAcceleration != nullptr ) && ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z );

    if( status )
    {
        double tempFactor = ( mTemperatureC - TEMP_25_C ) / ( TEMP_MAX_C - TEMP_MIN_C );

        const double OFFSET_TEMP_SLOPE_X = 9.2e-3;
        const double OFFSET_TEMP_SLOPE_Y = 2.5e-3;
        const double OFFSET_TEMP_SLOPE_Z = 11.7e-3;

        switch( aAxis )
        {
            case AXIS_X:
                *aAcceleration -= tempFactor * OFFSET_TEMP_SLOPE_X;
                break;

            case AXIS_Y:
                *aAcceleration -= tempFactor * OFFSET_TEMP_SLOPE_Y;
                break;

            case AXIS_Z:
                *aAcceleration -= tempFactor * OFFSET_TEMP_SLOPE_Z;
                break;

            default:
                break;
        }

    }

    return status;
}


//!************************************************************************
//! Get the acceleration range (+/-10g, +/-20g, +/-40g)
//!
//! @returns true if the range can be read
//!************************************************************************
/* virtual */ bool Adxl357::getAccelerationRange
    (
    AccelerationRange* aRange   //!< acceleration range
    )
{
    bool status = ( aRange != nullptr );

    if( status )
    {
        *aRange = ACCELERATION_RANGE_DEFAULT;
        uint8_t rangeReg = 0;
        status = readByteReg( ADXL355_357_REG_RANGE, &rangeReg );

        if( status )
        {
            const uint8_t RANGE_MASK = RANGE_HIGH;

            switch( rangeReg & RANGE_MASK )
            {
                case RANGE_HIGH:
                    mRange = ACCELERATION_RANGE_40G;
                    break;

                case RANGE_MID:
                    mRange = ACCELERATION_RANGE_20G;
                    break;

                case RANGE_LOW:
                default:
                    mRange = ACCELERATION_RANGE_10G;
                    break;
            }

            *aRange = mRange;
        }
    }

    return status;
}


//!************************************************************************
//! Calculate the offset of an axis due to VRE (vibration rectification
//! error)
//!
//! This function should be called from a module which has acquired a series
//! of accelerations on the axis of interest, and has calculated their RMS.
//!
//! VRE is the response of the accelerometer at AC vibrations which get
//! rectified to DC, manifesting as an anomalous shift in the offset of the
//! measured data. VRE is unpredictable in applications where frequency and
//! amplitude vary, therefore a general usage scenario is not provided.
//! Instead, the DAQ (calling) module needs to handle first the context of
//! measurement, knowing the spectrum and basic information about the signal
//! over the duration of interest and for the selected axis, then select the
//! right series of data for passing here the RMS.
//!
//! In general, measurements around +1g (AXIS_VERTICAL_UP) lead to negative
//! VRE offsets, while measurements around -1g (AXIS_VERTICAL_DOWN) lead to
//! positive VRE offsets.
//!
//! @returns true if the VRE offsets can be calculated
//!************************************************************************
/* virtual */ bool Adxl357::getVibrationRectificationOffset
    (
    const Axis          aAxis,              //!< axis whose RMS acceleration to use
    const AxisDirection aDirection,         //!< axis direction (up or down)
    const double        aAccelerationRms,   //!< RMS of a series of accelerations [g]
    double*             aVreOffset          //!< calculated VRE offset [g]
    )
{
    bool status = ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z )
               && ( aDirection >= AXIS_VERTICAL_UP ) && ( aDirection <= AXIS_VERTICAL_DOWN )
               && ( aVreOffset != nullptr );

    if( status )
    {
        *aVreOffset = 0;

        switch( mRange )
        {
            case ACCELERATION_RANGE_10G:
                switch( aAxis )
                {
                    case AXIS_X:
                        status = calculateLogisticResponse( aAccelerationRms, -0.001919727981, 0.217609249213, 9.731554477629, 6.116005440274, aVreOffset );
                        break;

                    case AXIS_Y:
                        status = calculateLogisticResponse( aAccelerationRms, -0.001550167510, 0.198414457317, 9.434585787665, 6.688368562989, aVreOffset );
                        break;

                    case AXIS_Z:
                        status = calculateLogisticResponse( aAccelerationRms, -0.000435171407, 0.225172255134, 9.270673016754, 5.847468282143, aVreOffset );
                        break;

                    default:
                        break;
                }
                break;

            case ACCELERATION_RANGE_20G:
                switch( aAxis )
                {
                    case AXIS_X:
                        status = calculateLogisticResponse( aAccelerationRms, -0.000681837591, 0.118229758682, 9.763380406589, 5.827221614997, aVreOffset );
                        break;

                    case AXIS_Y:
                        status = calculateLogisticResponse( aAccelerationRms, -0.000802545611, 0.108697330553, 9.587252285022, 6.258936784771, aVreOffset );
                        break;

                    case AXIS_Z:
                        status = calculateLogisticResponse( aAccelerationRms, -0.000936917290, 0.118963080719, 9.432676623109, 5.350328799490, aVreOffset );
                        break;

                    default:
                        break;
                }
                break;

            case ACCELERATION_RANGE_40G:
                switch( aAxis )
                {
                    case AXIS_X:
                        status = calculateLogisticResponse( aAccelerationRms, 0.000147932470, 0.022411997457, 11.596902595386, 2.763330139277, aVreOffset );
                        break;

                    case AXIS_Y:
                        status = calculateLogisticResponse( aAccelerationRms, 0.000201538679, 0.025314811789, 14.197584562587, 3.676910631922, aVreOffset );
                        break;

                    case AXIS_Z:
                        {
                            const size_t N = 5;
                            double coeff[N] = { 0, -0.001014811412, 0.000316393698, -0.000028638302, 0.000000882947 };
                            status = calculatePoly( aAccelerationRms, N - 1, coeff, aVreOffset );
                        }
                        break;

                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }

    if( AXIS_VERTICAL_UP == aDirection )
    {
        *aVreOffset *= -1;
    }

    return status;
}


//!************************************************************************
//! Data initalizer
//!
//! @returns nothing
//!************************************************************************
void Adxl357::initData()
{
    enableStandbyMode( mStandbyMode );
    enableSelfTestMode( mSelfTestMode );
    setTemperatureOff( mTemperatureOff );
    setAccelerationRange( mRange );
    updateLsbToG();
    setActivityThreshold( mActivityThreshold );
    updateOdrBySelection();
    updateHpfBySelection();
    setFifoSamplesSize( mFifoSamplesCount );
}


//!************************************************************************
//! Perform a self test
//!
//! If not already selected, set ODR to 4000 Hz, then restore the previous
//! value.
//!
//! @returns true if the self test can be performed
//!************************************************************************
/* virtual */ bool Adxl357::runSelfTest
    (
    bool*   aResult,        //!< true if self test passed
    double* aTypCoef        //!< how close to typical ones test values were found (0 is ideal)
    )
{
    bool status = ( aResult != nullptr ) && ( aTypCoef != nullptr );
    OdrSetting previousOdrSetting = mOdrSetting;

    if( status )
    {
        if( ODR_SETTING_4000 != previousOdrSetting )
        {
            status = setOdr( ODR_SETTING_4000 );
        }
    }

    if( status )
    {
        *aResult = false;
        *aTypCoef = 1;
        status = enableSelfTestMode( true );
    }

    if( status )
    {
        // wait a short time until the sensor electrostatically produces the test output
        usleep( 1000 );

        double testAccelZ = 0;
        status = getAccelerationOnAxis( AXIS_Z, &testAccelZ );

        if( status )
        {
            *aResult = ( 0.5 < testAccelZ ) && ( testAccelZ < 3.0 );

            const double TEST_ACCEL_Z_TYP_G = 1.25;
            // the smaller the better
            *aTypCoef = fabs( testAccelZ - TEST_ACCEL_Z_TYP_G );
        }
    }

    if( status )
    {
        status = enableSelfTestMode( false );
    }

    if( status )
    {
        if( ODR_SETTING_4000 != previousOdrSetting )
        {
            status = setOdr( previousOdrSetting );
        }
    }

    return status;
}


//!************************************************************************
//! Set the acceleration range (+/-10g, +/-20g, +/-40g)
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the acceleration range can be set
//!************************************************************************
/* virtual */ bool Adxl357::setAccelerationRange
    (
    const AccelerationRange aRange      //!< acceleration range
    )
{
    bool status = ( aRange >= ACCELERATION_RANGE_10G ) && ( aRange <= ACCELERATION_RANGE_40G );
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
            const uint8_t RANGE_MASK = RANGE_HIGH;
            rangeReg &= ~RANGE_MASK;

            switch( aRange )
            {
                case ACCELERATION_RANGE_40G:
                    rangeReg |= RANGE_HIGH;
                    break;

                case ACCELERATION_RANGE_20G:
                    rangeReg |= RANGE_MID;
                    break;

                case ACCELERATION_RANGE_10G:
                default:
                    rangeReg |= RANGE_LOW;
                    break;
            }

            status = writeByteReg( ADXL355_357_REG_RANGE, rangeReg );

            if( status )
            {
                mRange = aRange;

                // update the meaning of 1 LSB
                updateLsbToG();

                // update the values stored in all offset registers
                storeAxisOffset( AXIS_X );
                storeAxisOffset( AXIS_Y );
                storeAxisOffset( AXIS_Z );

                // update the values stored in activity threshold registers
                storeActivityThreshold();
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
//! Update the amount of acceleration [g] represented by 1 LSB
//!
//! Equivalent to a LSB-TO-g conversion factor.
//!
//! @returns nothing
//!************************************************************************
/* virtual */ void Adxl357::updateLsbToG()
{
    double tempFactor = ( mTemperatureC - TEMP_25_C ) / ( TEMP_MAX_C - TEMP_MIN_C );

    const double SENS_SLOPE_X = 0.57e-2;
    const double SENS_SLOPE_Y = 0.53e-2;
    const double SENS_SLOPE_Z = 0.21e-2;

    double sensFactorX = 1 + tempFactor * SENS_SLOPE_X;
    double sensFactorY = 1 + tempFactor * SENS_SLOPE_Y;
    double sensFactorZ = 1 + tempFactor * SENS_SLOPE_Z;

    switch( mRange )
    {
        case ACCELERATION_RANGE_40G:
            sensFactorX *= ACCELERATION_SENSITIVITY_RANGE_40G;
            sensFactorY *= ACCELERATION_SENSITIVITY_RANGE_40G;
            sensFactorZ *= ACCELERATION_SENSITIVITY_RANGE_40G;
            break;

        case ACCELERATION_RANGE_20G:
            sensFactorX *= ACCELERATION_SENSITIVITY_RANGE_20G;
            sensFactorY *= ACCELERATION_SENSITIVITY_RANGE_20G;
            sensFactorZ *= ACCELERATION_SENSITIVITY_RANGE_20G;
            break;

        case ACCELERATION_RANGE_10G:
        default:
            sensFactorX *= ACCELERATION_SENSITIVITY_RANGE_10G;
            sensFactorY *= ACCELERATION_SENSITIVITY_RANGE_10G;
            sensFactorZ *= ACCELERATION_SENSITIVITY_RANGE_10G;
            break;
    }

    mLsbToG.x = 1.0 / sensFactorX;
    mLsbToG.y = 1.0 / sensFactorY;
    mLsbToG.z = 1.0 / sensFactorZ;
}
