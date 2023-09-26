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

This file contains the sources for the ADXL355 driver.
*/

#include "Adxl355.h"

#include <cmath>
#include <unistd.h>


Adxl355* Adxl355::sInstance = nullptr;

//!************************************************************************
//! Constructor
//!************************************************************************
Adxl355::Adxl355()
    : Adxl355Adxl357Common( ACCELERATION_RANGE_DEFAULT )
{   
}


//!************************************************************************
//! Destructor
//!************************************************************************
Adxl355::~Adxl355()
{
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
Adxl355* Adxl355::getInstance()
{
    if( !sInstance )
    {
        sInstance = new Adxl355;
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
/* virtual */ bool Adxl355::compensateOffsetTemperature
    (
    const Axis aAxis,           //!< axis whose offset to compensate
    double*    aAcceleration    //!< acceleration value [g]
    )
{
    bool status = ( aAcceleration != nullptr ) && ( aAxis >= AXIS_X ) && ( aAxis <= AXIS_Z );

    if( status )
    {
        double tempFactor = ( mTemperatureC - TEMP_25_C ) / ( TEMP_MAX_C - TEMP_MIN_C );

        const double OFFSET_TEMP_SLOPE_X = 1.7e-3;
        const double OFFSET_TEMP_SLOPE_Y = 1.4e-3;
        const double OFFSET_TEMP_SLOPE_Z = 5.3e-3;

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
//! Get the acceleration range (+/-2g, +/-4g, +/-8g)
//!
//! @returns true if the range can be read
//!************************************************************************
/* virtual */ bool Adxl355::getAccelerationRange
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
                    mRange = ACCELERATION_RANGE_8G;
                    break;

                case RANGE_MID:
                    mRange = ACCELERATION_RANGE_4G;
                    break;

                case RANGE_LOW:
                default:
                    mRange = ACCELERATION_RANGE_2G;
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
/* virtual */ bool Adxl355::getVibrationRectificationOffset
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
            case ACCELERATION_RANGE_2G:
                switch( aAxis )
                {
                    case AXIS_X:
                        status = calculateLogisticResponse( aAccelerationRms, -0.003387892238, 0.762025827078, 2.327224480732, 2.608152634853, aVreOffset );
                        break;

                    case AXIS_Y:
                        status = calculateLogisticResponse( aAccelerationRms, -0.006530658233, 0.778025380974, 2.376010415638, 2.433043292342, aVreOffset );
                        break;

                    case AXIS_Z:
                        status = calculateLogisticResponse( aAccelerationRms, -0.003490868874, 0.738195887319, 2.237467044143, 2.601541031491, aVreOffset );
                        break;

                    default:
                        break;
                }

                if( *aVreOffset < 0 )
                {
                    *aVreOffset = 0;
                }

                break;

            case ACCELERATION_RANGE_4G:
                switch( aAxis )
                {
                    case AXIS_X:
                        status = calculateLogisticResponse( aAccelerationRms, -0.002441409250, 0.449933128610, 2.770492856421, 2.355049625902, aVreOffset );
                        break;

                    case AXIS_Y:
                        status = calculateLogisticResponse( aAccelerationRms, -0.004066466239, 0.474686690249, 2.919276442989, 2.178870821243, aVreOffset );
                        break;

                    case AXIS_Z:
                        status = calculateLogisticResponse( aAccelerationRms, -0.004083989181, 0.421584153623, 2.474323937293, 2.414902709699, aVreOffset );
                        break;

                    default:
                        break;
                }
                break;

            case ACCELERATION_RANGE_8G:
                switch( aAxis )
                {
                    case AXIS_X:
                        {
                            const size_t N = 5;
                            double coeff[N] = { 0, 0.011192614610, -0.023773450384, 0.007641457220, -0.000501873938 };
                            status = calculatePoly( aAccelerationRms, N - 1, coeff, aVreOffset );
                        }
                        break;

                    case AXIS_Y:
                        {
                            const size_t N = 5;
                            double coeff[N] = { 0, 0.010962006334, -0.022905976909, 0.007452124822, -0.000493187357 };
                            status = calculatePoly( aAccelerationRms, N - 1, coeff, aVreOffset );
                        }
                        break;

                    case AXIS_Z:
                        status = calculateLogisticResponse( aAccelerationRms, -0.002570580185, 0.631237734825, 7.100727783110, 4.549522536146, aVreOffset );
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
void Adxl355::initData()
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
/* virtual */ bool Adxl355::runSelfTest
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

        double testAccelX = 0;
        double testAccelY = 0;
        double testAccelZ = 0;
        status = getAccelerationsOnAllAxes( &testAccelX, &testAccelY, &testAccelZ );

        if( status )
        {
            *aResult = ( 0.1 < testAccelX ) && ( testAccelX < 0.6 )
                    && ( 0.1 < testAccelY ) && ( testAccelY < 0.6 )
                    && ( 0.5 < testAccelZ ) && ( testAccelZ < 3.0 );

            const double TEST_ACCEL_X_TYP_G = 0.3;
            const double TEST_ACCEL_Y_TYP_G = 0.3;
            const double TEST_ACCEL_Z_TYP_G = 1.5;
            // the smaller the better
            *aTypCoef = fabs( testAccelX - TEST_ACCEL_X_TYP_G )
                      * fabs( testAccelY - TEST_ACCEL_Y_TYP_G )
                      * fabs( testAccelZ - TEST_ACCEL_Z_TYP_G );
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
//! Set the acceleration range (+/-2g, +/-4g, +/-8g)
//!
//! If not already enabled, the function enters standby mode, then restores
//! it before return.
//!
//! @returns true if the acceleration range can be set
//!************************************************************************
/* virtual */ bool Adxl355::setAccelerationRange
    (
    const AccelerationRange aRange      //!< acceleration range
    )
{
    bool status = ( aRange >= ACCELERATION_RANGE_2G ) && ( aRange <= ACCELERATION_RANGE_8G );
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
                case ACCELERATION_RANGE_8G:
                    rangeReg |= RANGE_HIGH;
                    break;

                case ACCELERATION_RANGE_4G:
                    rangeReg |= RANGE_MID;
                    break;

                case ACCELERATION_RANGE_2G:
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
/* virtual */ void Adxl355::updateLsbToG()
{
    double tempFactor = ( mTemperatureC - TEMP_25_C ) / ( TEMP_MAX_C - TEMP_MIN_C );

    const double SENS_SLOPE_X = 0.72e-2;
    const double SENS_SLOPE_Y = 0.72e-2;
    const double SENS_SLOPE_Z = 0.3e-2;

    double sensFactorX = 1 + tempFactor * SENS_SLOPE_X;
    double sensFactorY = 1 + tempFactor * SENS_SLOPE_Y;
    double sensFactorZ = 1 + tempFactor * SENS_SLOPE_Z;

    switch( mRange )
    {
        case ACCELERATION_RANGE_8G:
            sensFactorX *= ACCELERATION_SENSITIVITY_RANGE_8G;
            sensFactorY *= ACCELERATION_SENSITIVITY_RANGE_8G;
            sensFactorZ *= ACCELERATION_SENSITIVITY_RANGE_8G;
            break;

        case ACCELERATION_RANGE_4G:
            sensFactorX *= ACCELERATION_SENSITIVITY_RANGE_4G;
            sensFactorY *= ACCELERATION_SENSITIVITY_RANGE_4G;
            sensFactorZ *= ACCELERATION_SENSITIVITY_RANGE_4G;
            break;

        case ACCELERATION_RANGE_2G:
        default:
            sensFactorX *= ACCELERATION_SENSITIVITY_RANGE_2G;
            sensFactorY *= ACCELERATION_SENSITIVITY_RANGE_2G;
            sensFactorZ *= ACCELERATION_SENSITIVITY_RANGE_2G;
            break;
    }

    mLsbToG.x = 1.0 / sensFactorX;
    mLsbToG.y = 1.0 / sensFactorY;
    mLsbToG.z = 1.0 / sensFactorZ;
}
