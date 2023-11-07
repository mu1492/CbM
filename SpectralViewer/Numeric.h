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
Numeric.h

This file contains the definitions for numerical constants and functions.
*/

#ifndef Numeric_h
#define Numeric_h

#include <cmath>
#include <cstdint>
#include <vector>


//************************************************************************
// Class for handling vibrations monitoring
//************************************************************************
class Numeric
{
    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        static constexpr double PI       = 4.0 * atan( 1.0 );   //!< pi
        static constexpr double TWO_PI   = 2.0 * PI;            //!< 2*pi

        static const int32_t INVALID_MAX_10_DB = -300;          //!< value for invalid maximum, dB scale
        static const int32_t INVALID_MIN_10_DB = 300;           //!< value for invalid minimum, dB scale

        static constexpr double INVALID_MAX_LOG = 1.e-10;       //!< value for invalid maximum, log scale
        static constexpr double INVALID_MIN_LOG = 1.e10;        //!< value for invalid minimum, log scale


    //************************************************************************
    // functions
    //************************************************************************
    public:
        Numeric();

        ~Numeric();

        static Numeric* getInstance();

        double calculateRms
            (
            double      aData[],        //!< data array
            uint32_t    aStartIndex,    //!< first index used
            uint32_t    aStopIndex      //!< last index used
            ) const;

        double calculateRms
            (
            const std::vector<double> aData   //!< data vector
            ) const;

        double findLogMaxPwrOf10
            (
            double aValue           //!< value
            ) const;

        double findLogMinPwrOf10
            (
            double aValue           //!< value
            ) const;

        int16_t findMagnitudeMaxDbMultipleOf10
            (
            double aValue           //!< value
            ) const;

        int16_t findMagnitudeMinDbMultipleOf10
            (
            double aValue           //!< value
            ) const;

        int16_t findPowerMaxDbMultipleOf10
            (
            double aValue           //!< value
            ) const;

        int16_t findPowerMinDbMultipleOf10
            (
            double aValue           //!< value
            ) const;

        bool isPowerOfTwo
            (
            const uint32_t aNumber      //!< number
            ) const;

        void swap
            (
            double&     aVariable1,     //!< 1st variable
            double&     aVariable2      //!< 2nd variable
            ) const;

    //************************************************************************
    // variables
    //************************************************************************
    private:
        static Numeric*   sInstance;          //!< singleton
};

#endif // Numeric_h
