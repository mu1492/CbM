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
SrsThread.cpp

This file contains the sources for the SRS thread.
*/

#include "SrsThread.h"

#include "FrequencyAnalysis.h"

#include <algorithm>
#include <cmath>


//!************************************************************************
//! Constructor
//!************************************************************************
SrsThread::SrsThread
    (
    QObject* aParent    //!< parent object
    )
    : QThread( aParent )
    , mFftSize( 0 )
    , mIsRestarting( false )
    , mIsAborting( false )
{
    mFftSubthreadsVec = std::vector<FftThread>( NR_OF_POINTS );
    mFftSubthreadsReady = std::vector<bool>( NR_OF_POINTS, false );

    mNaturalFreqVec = std::vector<double>( NR_OF_POINTS, 0 );
    mSrsValuesVec = std::vector<double>( NR_OF_POINTS, 0 );

    for( int crtPoint = 0; crtPoint < NR_OF_POINTS; crtPoint++ )
    {
        connect( &mFftSubthreadsVec.at( crtPoint ), &FftThread::fftComputeDone, this, &SrsThread::receiveSubthreadFft );
    }
}


//!************************************************************************
//! Destructor
//!************************************************************************
SrsThread::~SrsThread()
{
    mMutex.lock();
        mIsAborting = true;
        mWaitCondition.wakeOne();
    mMutex.unlock();

    wait();
}


//!************************************************************************
//! Compute data loader
//! It provides input data and information for calculating the SRS.
//!
//! @returns nothing
//!************************************************************************
void SrsThread::compute
    (
    std::vector<double> aDataVec    //!< input data for SRS computing
    )
{
    QMutexLocker locker( &mMutex );
    mDataVec = aDataVec;
    mFftSize = ( mDataVec.size() >= 1 ? mDataVec.size() - 1 : 0 );

    mFftSubthreadsReady = std::vector<bool>( NR_OF_POINTS, false );

    mNaturalFreqVec = std::vector<double>( NR_OF_POINTS, 0 );
    mSrsValuesVec = std::vector<double>( NR_OF_POINTS, 0 );

    if( !isRunning() )
    {
        start();
    }
    else
    {
        mIsRestarting = true;
        mWaitCondition.wakeOne();
    }
}


//!************************************************************************
//! Receive computed FFT values from dedicated subthreads
//!
//! Once the entire vector with SRS maximax values has been filled, emit the
//! srsComputeDone signal.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SrsThread::receiveSubthreadFft
    (
    double*  aDataArray, //!< array with computed FFT values
    int      aLength,    //!< array length
    int      aIndex      //!< subthread index
    )
{    
    std::vector<double> fftVec;

    if( aDataArray )
    {
        fftVec.assign( aDataArray, aDataArray + aLength );
    }

    mMutex.lock();
        const std::vector<double> FFT_VEC = fftVec;
        const uint32_t FFT_SIZE = mFftSize;

        double yMaxCrtPoint = 0;

        if( fabs( FFT_VEC.at( 1 ) ) > yMaxCrtPoint )
        {
            yMaxCrtPoint = fabs( FFT_VEC.at( 1 ) );
        }

        if( fabs( FFT_VEC.at( 2 ) ) > yMaxCrtPoint )
        {
            yMaxCrtPoint = fabs( FFT_VEC.at( 2 ) );
        }

        for( uint32_t i = 3; i <= FFT_SIZE - 1; i += 2 )
        {
            double mag = sqrt( pow( FFT_VEC.at( i ), 2.0 ) + pow( FFT_VEC.at( i + 1 ), 2.0 ) );

            if( mag > yMaxCrtPoint )
            {
                yMaxCrtPoint = mag;
            }
        }

        mSrsValuesVec.at( aIndex ) = yMaxCrtPoint;
        mFftSubthreadsReady.at( aIndex ) = true;

        bool isIncomplete = ( std::find( mFftSubthreadsReady.begin(), mFftSubthreadsReady.end(), false ) != mFftSubthreadsReady.end() );

        if( !mIsRestarting )
        {
            if( !isIncomplete )
            {
                emit srsComputeDone( mNaturalFreqVec.data(), mSrsValuesVec.data(), NR_OF_POINTS );
            }
        }
    mMutex.unlock();
}


//!************************************************************************
//! SRS thread main function
//!
//! @returns nothing
//!************************************************************************
/* virtual */ void SrsThread::run()
{
    forever
    {
        mMutex.lock();
            const std::vector<double> ACCEL_DATA_VEC = mDataVec;
            const uint32_t FFT_SIZE = mFftSize;
            FrequencyAnalysis* faInstance = FrequencyAnalysis::getInstance();
            const double NATURAL_FREQ_MAX = faInstance->getFftFreqMax();
        mMutex.unlock();

        if( mIsAborting )
        {
            return;
        }

        double exactNrOfDecades = log10( NATURAL_FREQ_MAX / NATURAL_FREQ_MIN );
        uint16_t ceiledNrOfDecades = static_cast<uint16_t>( ceil( exactNrOfDecades ) );
        bool haveIntegerNrOfDecades = ( ceil( exactNrOfDecades ) == floor( exactNrOfDecades ) );

        for( int crtPoint = 0; crtPoint < NR_OF_POINTS; crtPoint++ )
        {
            int16_t nrOfPointsPerDecade = 0;

            if( haveIntegerNrOfDecades )
            {
                nrOfPointsPerDecade = NR_OF_POINTS / ceiledNrOfDecades;
            }
            else
            {
                nrOfPointsPerDecade = static_cast<uint16_t>( ceil( NR_OF_POINTS / exactNrOfDecades ) );
            }

            uint16_t crtDecade = crtPoint / nrOfPointsPerDecade;
            double crtNaturalFreq = NATURAL_FREQ_MIN * pow( 10.0, static_cast<double>( crtDecade ) );
            crtNaturalFreq *= pow( 10.0, static_cast<double>( crtPoint % nrOfPointsPerDecade ) / nrOfPointsPerDecade );

            mMutex.lock();
                mNaturalFreqVec.at( crtPoint ) = crtNaturalFreq;
            mMutex.unlock();

            faInstance->getSrsParameters().naturalFreq = crtNaturalFreq;
            faInstance->updateSrsCoefficients();

            FrequencyAnalysis::SrsCoefficients srsCoeffs = faInstance->getSrsCoefficients();
            std::vector<double> crtPointDataVec( FFT_SIZE + 1, 0 );

            double xm1 = 0;
            double xm2 = 0;
            double ym1 = 0;
            double ym2 = 0;

            for( uint32_t i = 0; i < FFT_SIZE; i++ )
            {
                double y = srsCoeffs.b0 * ACCEL_DATA_VEC.at( i + 1 ) +
                           srsCoeffs.b1 * xm1 +
                           srsCoeffs.b2 * xm2 +
                           ( 2 * ym1 - ym2 ) - ( srsCoeffs.a1p2 * ym1 + srsCoeffs.a2m1 * ym2 );

                 ym2 = ym1;
                 ym1 = y;
                 xm2 = xm1;
                 xm1 = ACCEL_DATA_VEC.at( i + 1 );
                 crtPointDataVec.at( i + 1 ) = y;
            }

            mFftSubthreadsVec.at( crtPoint ).compute( crtPointDataVec, FrequencyAnalysis::FFT_SENSE_DIRECT, crtPoint, true );
        }

        mMutex.lock();
            if( !mIsRestarting )
            {
                mWaitCondition.wait( &mMutex );
            }

            mIsRestarting = false;
        mMutex.unlock();
    }
}
