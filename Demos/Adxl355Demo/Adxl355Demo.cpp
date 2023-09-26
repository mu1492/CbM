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
Adxl355Demo.cpp

This file contains the sources for the ADXL355 demo.
*/

#include "Adxl355Demo.h"
#include "./ui_Adxl355Demo.h"


#include <QMessageBox>
#include <QTimer>

#include <iostream>
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
Adxl355Demo::Adxl355Demo
    (
    QWidget* aParent        //!< parent widget
    )
    : QMainWindow( aParent )
    , mMainUi( new Ui::Adxl355Demo )
    , mI2cBusChannel( 0 )
    , mI2cIsOpen( false )
    , mAccelX( 0 )
    , mAccelY( 0 )
    , mAccelZ( 0 )
    , mAccelNorm( 0 )
    , mTiltAngleXdeg( 0 )
    , mTiltAngleYdeg( 0 )
    , mTiltAngleZdeg( 0 )
    , mTemperature( 0 )
{
    mMainUi->setupUi( this );

    //****************************************
    // ADXL355 setup
    //****************************************
    bool adxl355InitOk = false;
    std::string i2cPath = "/dev/i2c-" + std::to_string( I2C_BUS_MEMS );
    mI2cBusChannel = ::open( i2cPath.c_str(), O_RDWR );

    if( mI2cBusChannel >= 0 )
    {
        mI2cIsOpen = true;
    }
    else
    {
        QMessageBox::critical( this, "ADXL355 Demo",
                               "Could not open I2C bus " + QString::fromStdString( i2cPath ),
                               QMessageBox::Ok );
        delete mMainUi;
        exit( 0 );
    }

    mAdxl355Instance = Adxl355::getInstance();

    if( !mAdxl355Instance )
    {
        QMessageBox::critical( this, "ADXL355 Demo",
                               "Could not create an instance of ADXL355",
                               QMessageBox::Ok );
        if( mI2cIsOpen )
        {
            ::close( mI2cBusChannel );
        }

        delete mMainUi;
        exit( 0 );
    }
    else
    {
        bool initStatus = mAdxl355Instance->init( mI2cBusChannel, Adxl355::ADXL355_357_I2C_ADDRESS_PRIMARY );

        if( !initStatus )
        {
            QMessageBox::critical( this, "ADXL355 Demo",
                                   "Could not initialize ADXL355",
                                   QMessageBox::Ok );
            if( mI2cIsOpen )
            {
                ::close( mI2cBusChannel );
            }

            delete mMainUi;
            exit( 0 );
        }
        else
        {
            adxl355InitOk = true;
            mAdxl355Instance->initData();
            uint8_t revId = mAdxl355Instance->getRevId();

            if( Adxl355::REV_ID != revId )
            {
                QMessageBox::information( this, "ADXL355 Demo",
                                       "The detected ADXL355 chip revision ID is " + QString::number( revId ),
                                       QMessageBox::Ok );
            }

            if( !mAdxl355Instance->reset() )
            {
                QMessageBox::critical( this, "ADXL355 Demo",
                                       "Could not reset ADXL355",
                                       QMessageBox::Ok );
            }

            // enter measurement mode
            mAdxl355Instance->enableStandbyMode( false );

            bool selfTestPassed = false;
            double typCoeff = 1;
            bool selfTestStatus = mAdxl355Instance->runSelfTest( &selfTestPassed, &typCoeff );

            if( !selfTestStatus )
            {
                QMessageBox::critical( this, "ADXL355 Demo",
                                       "Could not run ADXL355 self test",
                                       QMessageBox::Ok );
            }
            else
            {
                if( !selfTestPassed )
                {
                    QMessageBox::warning( this, "ADXL355 Demo",
                                           "ADXL355 self test failed",
                                           QMessageBox::Ok );
                }
            }

            // lower bandwidth for inclinometer
            mAdxl355Instance->setOdr( Adxl355::ODR_SETTING_7_8125 );
        }
    }

    //****************************************
    // UI setup
    //****************************************
    mMainUi->accelNormStatic->setText( SQRT + "(a<sub>X</sub><sup>2</sup>+a<sub>Y</sub><sup>2</sup>+a<sub>Z</sub><sup>2</sup>) =" );

    mMainUi->tiltAngleXStatic->setText( "X: " + ALPHA_SMALL + " =" );
    mMainUi->tiltAngleYStatic->setText( "Y: " + BETA_SMALL + " =" );
    mMainUi->tiltAngleZStatic->setText( "Z: " + GAMMA_SMALL + " =" );

    QObject::connect( mMainUi->exitButton, SIGNAL( clicked() ), qApp, SLOT( quit() ) );
    QObject::connect( mMainUi->exitButton, SIGNAL( pressed() ), qApp, SLOT( quit() ) );

    if( adxl355InitOk )
    {
        //****************************************
        // accelerations readings
        //****************************************
        QTimer* accelerationsReadTimer = new QTimer( this );
        connect( accelerationsReadTimer, SIGNAL( timeout() ), this, SLOT( readAccelerations() ) );
        const uint32_t ACCEL_READ_MS = 200;
        accelerationsReadTimer->start( ACCEL_READ_MS );
        readAccelerations();

        //****************************************
        // temperature readings
        //****************************************
        QTimer* temperatureReadTimer = new QTimer( this );
        connect( temperatureReadTimer, SIGNAL( timeout() ), this, SLOT( readTemperature() ) );
        const uint32_t TEMPERATURE_READ_MS = 1000;
        temperatureReadTimer->start( TEMPERATURE_READ_MS );
        readTemperature();

        //****************************************
        // OpenGl widget
        //****************************************
        mGlWidget = new GlWidget( this );
        mMainUi->glGraphTilt->addWidget( mGlWidget );
        connect( this, &Adxl355Demo::tiltAnglesChanged, mGlWidget, &GlWidget::setAngles );
    }
    else
    {
        updateContent();
    }
}


//!************************************************************************
//! Destructor
//!************************************************************************
Adxl355Demo::~Adxl355Demo()
{
    if( mI2cIsOpen )
    {
        ::close( mI2cBusChannel );
    }

    delete mMainUi;
}


//!************************************************************************
//! Read the accelerations
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void Adxl355Demo::readAccelerations()
{
    mAdxl355Instance = Adxl355::getInstance();

    if( mAdxl355Instance )
    {
        double ax = 0;
        double ay = 0;
        double az = 0;

        if( mAdxl355Instance->getAccelerationsOnAllAxes( &ax, &ay, &az ) )
        {
            mAccelX = ax;
            mAccelY = ay;
            mAccelZ = az;

            double ax2 = ax * ax;
            double ay2 = ay * ay;
            double az2 = az * az;

            mAccelNorm = sqrt( ax2 + ay2 + az2 );

            mTiltAngleXdeg = atan2( ax, sqrt( ay2 + az2 ) ) * 180.0 / PI;
            mTiltAngleYdeg = atan2( ay, sqrt( ax2 + az2 ) ) * 180.0 / PI;
            mTiltAngleZdeg = atan2( sqrt( ax2 + ay2 ), az ) * 180.0 / PI;
            emit tiltAnglesChanged( mTiltAngleXdeg, mTiltAngleYdeg, mTiltAngleZdeg );
            updateContent();
        }
    }
}


//!************************************************************************
//! Read the temperature from the on-chip sensor
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void Adxl355Demo::readTemperature()
{
    mAdxl355Instance = Adxl355::getInstance();

    if( mAdxl355Instance )
    {
        double tempCelsius = 0;

        if( mAdxl355Instance->getTemperatureValue( &tempCelsius ) )
        {
            mTemperature = tempCelsius;
            updateContent();
        }
    }
}


//!************************************************************************
//! Update the UI content
//!
//! @returns: nothing
//!************************************************************************
void Adxl355Demo::updateContent()
{
    // accelerations
    mMainUi->accelXLabel->setText( QString::number( mAccelX, 'f', 3 ) + " [g]" );
    mMainUi->accelYLabel->setText( QString::number( mAccelY, 'f', 3 ) + " [g]" );
    mMainUi->accelZLabel->setText( QString::number( mAccelZ, 'f', 3 ) + " [g]" );

    mMainUi->accelNormLabel->setText( QString::number( mAccelNorm, 'f', 3 ) + " [g]" );

    mMainUi->tiltAngleXLabel->setText( QString::number( mTiltAngleXdeg, 'f', 3 ) + " [" +  DEG + "]" );
    mMainUi->tiltAngleYLabel->setText( QString::number( mTiltAngleYdeg, 'f', 3 ) + " [" +  DEG + "]" );
    mMainUi->tiltAngleZLabel->setText( QString::number( mTiltAngleZdeg, 'f', 3 ) + " [" +  DEG + "]" );

    // temperature
    mMainUi->temperatureLabel->setText( QString::number( mTemperature, 'f', 1 ) + " [" +  DEG + "C]" );
}
