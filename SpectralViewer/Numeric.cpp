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
Numeric.cpp

This file contains the sources for numerical constants and functions.
*/

#include "Numeric.h"

Numeric* Numeric::sInstance = nullptr;


//!************************************************************************
//! Constructor
//!************************************************************************
Numeric::Numeric()
{
}


//!************************************************************************
//! Destructor
//!************************************************************************
Numeric::~Numeric()
{
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
Numeric* Numeric::getInstance()
{
    if( !sInstance )
    {
        sInstance = new Numeric;
    }

    return sInstance;
}


//!************************************************************************
//! Calculate the root mean square of a data array
//!
//! @returns the RMS value
//!************************************************************************
double Numeric::calculateRms
    (
    double      aData[],        //!< data array
    uint32_t    aStartIndex,    //!< first index used
    uint32_t    aStopIndex      //!< last index used
    ) const
{
    double retValue = 0;
    bool status = ( nullptr != aData ) && ( aStartIndex <= aStopIndex );

    if( status )
    {
        for( uint32_t i = aStartIndex; i <= aStopIndex; i++ )
        {
            retValue += aData[i] * aData[i];
        }

        retValue = sqrt( retValue / static_cast<double>( 1 + aStopIndex - aStartIndex ) );
    }

    return retValue;
}


//!************************************************************************
//! Calculate the root mean square of a data vector
//!
//! @returns the RMS value
//!************************************************************************
double Numeric::calculateRms
    (
    const std::vector<double> aData     //!< data vector
    ) const
{
    double retValue = 0;
    size_t len = aData.size();

    if( len )
    {
        for( size_t i = 0; i < len; i++ )
        {
            retValue += aData[i] * aData[i];
        }

        retValue = sqrt( retValue / static_cast<double>( len ) );
    }

    return retValue;
}


//!************************************************************************
//! Find a power of 10 for the maximum value on "log" vertical axes
//!
//! @returns The power of 10 for max value
//!************************************************************************
double Numeric::findLogMaxPwrOf10
    (
    double aValue           //!< value
    ) const
{
    double pwrOf10 = INVALID_MAX_LOG;

    if( ( aValue > 10 * INVALID_MAX_LOG ) && ( aValue < INVALID_MIN_LOG / 10 ) )
    {
        pwrOf10 = pow( 10.0, 1.0 + floor( log10( aValue ) ) );
    }

    return pwrOf10;
}


//!************************************************************************
//! Find a power of 10 for the minimum value on "log" vertical axes
//!
//! @returns The power of 10 for min value
//!************************************************************************
double Numeric::findLogMinPwrOf10
    (
    double aValue           //!< value
    ) const
{
    double pwrOf10 = INVALID_MIN_LOG;

    if( ( aValue > 10 * INVALID_MAX_LOG ) && ( aValue < INVALID_MIN_LOG / 10 ) )
    {
        pwrOf10 = pow( 10.0, floor( log10( aValue ) ) );
    }

    return pwrOf10;
}


//!************************************************************************
//! Find a multiple of 10 for the maximum value on "dB" vertical axes
//! *** magnitude ***
//!
//! @returns The multiple of 10 integer for max value
//!************************************************************************
int16_t Numeric::findMagnitudeMaxDbMultipleOf10
    (
    double aValue       //!< value
    ) const
{
    int32_t dec = INVALID_MAX_10_DB;

    if( aValue > 0 )
    {
        double exactDb = 20 * log10( aValue );
        int16_t truncAbsDiv = static_cast<int16_t>( fabs( exactDb ) / 10 );

        if( exactDb >= 0 )
        {
            dec = 10 * ( 1 + truncAbsDiv );
        }
        else
        {
            dec = -10 * truncAbsDiv;
        }
    }

    return dec;
}


//!************************************************************************
//! Find a multiple of 10 for the minimum value on "dB" vertical axes
//! *** magnitude ***
//!
//! @returns The multiple of 10 integer for min value
//!************************************************************************
int16_t Numeric::findMagnitudeMinDbMultipleOf10
    (
    double aValue       //!< value
    ) const
{
    int32_t dec = INVALID_MIN_10_DB;

    if( aValue > 0 )
    {
        double exactDb = 20 * log10( aValue );
        int16_t truncAbsDiv = static_cast<int16_t>( fabs( exactDb ) / 10 );

        if( exactDb >= 0 )
        {
            dec = 10 * truncAbsDiv;
        }
        else
        {
            dec = -10 * ( 1 + truncAbsDiv );
        }
    }

    return dec;
}


//!************************************************************************
//! Find a multiple of 10 for the maximum value on "dB" vertical axes
//! *** power ***
//!
//! @returns The multiple of 10 integer for max value
//!************************************************************************
int16_t Numeric::findPowerMaxDbMultipleOf10
    (
    double aValue       //!< value
    ) const
{
    int32_t dec = INVALID_MAX_10_DB;

    if( aValue > 0 )
    {
        double exactDb = 10 * log10( aValue );
        int16_t truncAbsDiv = static_cast<int16_t>( fabs( exactDb ) / 10 );

        if( exactDb >= 0 )
        {
            dec = 10 * ( 1 + truncAbsDiv );
        }
        else
        {
            dec = -10 * truncAbsDiv;
        }
    }

    return dec;
}


//!************************************************************************
//! Find a multiple of 10 for the minimum value on "dB" vertical axes
//! *** power ***
//!
//! @returns The multiple of 10 integer for min value
//!************************************************************************
int16_t Numeric::findPowerMinDbMultipleOf10
    (
    double aValue       //!< value
    ) const
{
    int32_t dec = INVALID_MIN_10_DB;

    if( aValue > 0 )
    {
        double exactDb = 10 * log10( aValue );
        int16_t truncAbsDiv = static_cast<int16_t>( fabs( exactDb ) / 10 );

        if( exactDb >= 0 )
        {
            dec = 10 * truncAbsDiv;
        }
        else
        {
            dec = -10 * ( 1 + truncAbsDiv );
        }
    }

    return dec;
}


//!************************************************************************
//! Check if a number is a power of two
//!
//! @returns true if the parameter is a power of two
//!************************************************************************
bool Numeric::isPowerOfTwo
    (
    const uint32_t aNumber  //!< number
    ) const
{
    bool retval = false;

    if( aNumber > 0 )
    {
        retval = ( 0 == ( aNumber & ( aNumber - 1 ) ) );
    }

    return retval;
}


//!************************************************************************
//! Swap the values of two numbers
//!
//! @returns nothing
//!************************************************************************
void Numeric::swap
    (
    double&     aVariable1,     //!< 1st variable
    double&     aVariable2      //!< 2nd variable
) const
{
    double temp = aVariable1;
    aVariable1 = aVariable2;
    aVariable2 = temp;
}
