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
DaqThread.h

This file contains the definitions for the data acquisition thread.
*/

#ifndef DaqThread_h
#define DaqThread_h

#if BUILD_ADXL355
    #include "Adxl355.h"
#elif BUILD_ADXL357
    #include "Adxl357.h"
#endif

#include <QMutex>
#include <QThread>


//************************************************************************
// Class for handling the DAQ thread
//************************************************************************
class DaqThread : public QThread
{
    Q_OBJECT

    //************************************************************************
    // functions
    //************************************************************************
    public:
        DaqThread
            (
            QObject*    aParent = nullptr,          //!< parent object
        #if BUILD_ADXL355
            Adxl355*
        #elif BUILD_ADXL357
            Adxl357*
        #endif
                        aAccelInstance = nullptr    //!< accelerometer instance
            );

        ~DaqThread();

    void updateDelay
        (
        const uint16_t aDelayUs //!< delay [us]
        );

    void pause();

    void resume();

    protected:
        void run();

    private:
        int exec();

    signals:
        void haveNewData
            (
            double  aXaccel,    //!< acceleration on X axis
            double  aYaccel,    //!< acceleration on Y axis
            double  aZaccel     //!< acceleration on Z axis
            );

        void haveNewSps
            (
            int     aSps        //!< SPS
            );

        void haveNewTemperature
            (
            double  aTemperature    //!< temperature [C]
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
#if BUILD_ADXL355
        Adxl355*
#elif BUILD_ADXL357
        Adxl357*
#endif
                        mAccelInstance;         //!< accelerometer instance

        QMutex          mMutex;                 //!< DAQ mutex
        bool            mIsPaused;              //!< true if DAQ is paused
        uint16_t        mDaqDelayUs;            //!< delay [us]
};

#endif // DaqThread_h
