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
Adxl355Demo.h

This file contains the definitions for the ADXL355 demo.
*/

#ifndef Adxl355Demo_h
#define Adxl355Demo_h

#include "Adxl355.h"
#include "GlWidget.h"

#include <cmath>
#include <QMainWindow>

QT_BEGIN_NAMESPACE
    namespace Ui
    {
        class Adxl355Demo;
    }
QT_END_NAMESPACE


//************************************************************************
// Class for handling the ADXL355 demo
//************************************************************************
class Adxl355Demo : public QMainWindow
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        typedef enum : uint8_t
        {
            I2C_BUS_MEMS_RASPBERRY_PI_4             = 1,
            I2C_BUS_MEMS_NVIDIA_JETSON_ORIN_NANO    = 7
        }I2c_Bus_Mems;

        // I2C bus number of the MEMS accelerometer
        static const uint8_t I2C_BUS_MEMS = I2C_BUS_MEMS_NVIDIA_JETSON_ORIN_NANO;

        const QString DEG         = QString::fromUtf8( "\u00B0" );          //!< degree sign
        const QString ALPHA_SMALL = QString::fromUtf8( "\u03B1" );          //!< small alpha
        const QString BETA_SMALL  = QString::fromUtf8( "\u03B2" );          //!< small beta
        const QString GAMMA_SMALL = QString::fromUtf8( "\u03B3" );          //!< small gamma
        const QString SQRT        = QString::fromUtf8( "\u221A" );          //!< square root

        static constexpr double PI = 4.0 * atan( 1.0 );

    //************************************************************************
    // functions
    //************************************************************************
    public:
        Adxl355Demo
            (
            QWidget* aParent = nullptr  //!< parent widget
            );

        ~Adxl355Demo();

    signals:
        void tiltAnglesChanged
            (
            double aAccelX,             //!< new value on X axis [g]
            double aAccelY,             //!< new value on Y axis [g]
            double aAccelZ              //!< new value on Z axis [g]
            );

    private:
        void updateContent();

    private slots:
        void readAccelerations();

        void readTemperature();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        Ui::Adxl355Demo*    mMainUi;            //!< main UI

        int                 mI2cBusChannel;     //!< I2C bus channel
        bool                mI2cIsOpen;         //!< if I2C bus could be open

        Adxl355*            mAdxl355Instance;   //!< ADXL355 instance

        double              mAccelX;            //!< acceleration on X axis [g]
        double              mAccelY;            //!< acceleration on Y axis [g]
        double              mAccelZ;            //!< acceleration on Z axis [g]
        double              mAccelNorm;         //!< square root of the sum of squares [g]

        double              mTiltAngleXdeg;     //!< tilt angle on X axis [deg]
        double              mTiltAngleYdeg;     //!< tilt angle on Y axis [deg]
        double              mTiltAngleZdeg;     //!< tilt angle on Z axis [deg]

        double              mTemperature;       //!< temperature from the on-chip sensor [C]

        GlWidget*           mGlWidget;          //!< OpenGL widget for drawing to
};

#endif // Adxl355Demo_h
