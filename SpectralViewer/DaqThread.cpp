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
DaqThread.cpp

This file contains the sources for the data acquisition thread.
*/

#include "DaqThread.h"

#include <unistd.h>
#include <QDateTime>


//!************************************************************************
//! Constructor
//!************************************************************************
DaqThread::DaqThread
    (
    QObject*    aParent,        //!< parent object
#if BUILD_ADXL355
    Adxl355*
#elif BUILD_ADXL357
    Adxl357*
#endif
                aAccelInstance  //!< accelerometer instance
    )
    : QThread( aParent )
    , mAccelInstance( aAccelInstance )
    , mIsPaused( false )
    , mDaqDelayUs( 500 )
{    
}


//!************************************************************************
//! Destructor
//!************************************************************************
DaqThread::~DaqThread()
{
    mMutex.unlock();
}


//!************************************************************************
//! DAQ thread execution function
//! Do not move portions of code to run().
//!
//! @returns zero
//!************************************************************************
int DaqThread::exec()
{
    if( mAccelInstance )
    {
        const uint8_t TEMP_READ_PERIOD = 5;  // [s]
        uint32_t tempCount = 0;
        double tempC = 0;

        int sampleCount = 0;
        bool haveData = false;
        double accelX = 0;
        double accelY = 0;
        double accelZ = 0;

        QDateTime chronoStart = QDateTime::currentDateTime();

        forever
        {
            if( !mIsPaused )
            {
                tempCount++;

                mMutex.lock();
                    mAccelInstance->isDataReady( &haveData );

                    if( haveData )
                    {
                        if( mAccelInstance->getAccelerationsOnAllAxes( &accelX, &accelY, &accelZ ) )
                        {
                            emit haveNewData( accelX, accelY, accelZ );

                            sampleCount++;
                            QDateTime chronoStop = QDateTime::currentDateTime();
                            qint64 millisecDiff = chronoStart.msecsTo( chronoStop );

                            if( millisecDiff >= 1000 )
                            {
                                emit haveNewSps( sampleCount );
                                sampleCount = 0;
                                chronoStart = chronoStop;
                            }
                        }
                    }

                    if( tempCount * mDaqDelayUs >= TEMP_READ_PERIOD * 1e6 )
                    {
                        tempCount = 0;

                        if( mAccelInstance->getTemperatureValue( &tempC ) )
                        {
                            emit haveNewTemperature( tempC );
                        }
                     }
                mMutex.unlock();
            }
            else
            {
                tempCount = 0;
            }

            usleep( mDaqDelayUs );
        }
    }

    return 0;
}


//!************************************************************************
//! Pause the thread
//!
//! @returns nothing
//!************************************************************************
void DaqThread::pause()
{
    QMutexLocker locker( &mMutex );
    mIsPaused = true;
}


//!************************************************************************
//! Resume the thread
//!
//! @returns nothing
//!************************************************************************
void DaqThread::resume()
{
    QMutexLocker locker( &mMutex );
    mIsPaused = false;
}


//!************************************************************************
//! DAQ thread main function
//! Do not move here portions from exec().
//!
//! @returns nothing
//!************************************************************************
/* virtual */ void DaqThread::run()
{
    exec();
}


//!************************************************************************
//! Update the delay
//!
//! @returns nothing
//!************************************************************************
void DaqThread::updateDelay
    (
    const uint16_t aDelayUs //!< delay [us]
    )
{
    QMutexLocker locker( &mMutex );
    mDaqDelayUs = aDelayUs;
}
