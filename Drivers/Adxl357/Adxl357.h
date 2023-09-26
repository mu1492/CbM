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
Adxl357.h

This file contains the definitions for the ADXL357 driver.
*/

#ifndef Adxl357_h
#define Adxl357_h

#include "Adxl355Adxl357Common.h"


//************************************************************************
// Class for handling the ADXL357 driver
//************************************************************************
class Adxl357 : public Adxl355Adxl357Common
{
    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        const AccelerationRange ACCELERATION_RANGE_DEFAULT = ACCELERATION_RANGE_10G;

    //************************************************************************
    // functions
    //************************************************************************
    public:
        Adxl357();

        ~Adxl357();

        static Adxl357* getInstance();

        bool getAccelerationRange
            (
            AccelerationRange* aRange       //!< acceleration range
            );

        bool getVibrationRectificationOffset
            (
            const Axis          aAxis,              //!< axis whose RMS acceleration to use
            const AxisDirection aDirection,         //!< axis direction (up or down)
            const double        aAccelerationRms,   //!< RMS of a series of accelerations [g]
            double*             aVreOffset          //!< calculated VRE offset [g]
            );

        void initData();

        bool runSelfTest
            (
            bool*   aResult,                //!< true if self test passed
            double* aTypCoef                //!< how close to typical ones test values were found (0 is ideal)
            );

        bool setAccelerationRange
            (
            const AccelerationRange aRange  //!< acceleration range
            );

    private:
        bool compensateOffsetTemperature
            (
            const Axis aAxis,           //!< axis whose offset to compensate
            double*    aAcceleration    //!< acceleration value [g]
            );

        void updateLsbToG();

    //************************************************************************
    // variables
    //************************************************************************
    private:
        static Adxl357*     sInstance;          //!< singleton
};

#endif // Adxl357_h

