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
VibrationHandler.cpp

This file contains the sources for the mechanical vibrations handler.
*/

/*
References (selected list):
===========================
[1] Broch, J.T. - Mechanical Vibration and Shock Measurements, 2nd Ed, Bruel&Kjaer, 1984

[2] Harris, C.M., Piersol, A.G. - Harris' Shock and Vibration Handbook, 5th Ed, McGraw-Hill, 2002
*/

#include "VibrationHandler.h"

#include "Numeric.h"

#include <float.h>

#include <cstring>


VibrationHandler* VibrationHandler::sInstance = nullptr;

//!************************************************************************
//! Constructor
//!************************************************************************
VibrationHandler::VibrationHandler()
    : mFftSize( 0 )
    , mFftIndexCount( 0 )
    , mSpsExpected( 0 )
    , mSpsReceived( 0 )
{   
    connect( &mFftThreadX, &FftThread::fftComputeDone, this, &VibrationHandler::receiveNewFftX );
    connect( &mFftThreadY, &FftThread::fftComputeDone, this, &VibrationHandler::receiveNewFftY );
    connect( &mFftThreadZ, &FftThread::fftComputeDone, this, &VibrationHandler::receiveNewFftZ );

    connect( &mCepstrumInvFftThreadX, &FftThread::fftComputeDone, this, &VibrationHandler::receiveNewCepstrumX );
    connect( &mCepstrumInvFftThreadY, &FftThread::fftComputeDone, this, &VibrationHandler::receiveNewCepstrumY );
    connect( &mCepstrumInvFftThreadZ, &FftThread::fftComputeDone, this, &VibrationHandler::receiveNewCepstrumZ );

    connect( &mSrsThreadX, &SrsThread::srsComputeDone, this, &VibrationHandler::receiveNewSrsX );
    connect( &mSrsThreadY, &SrsThread::srsComputeDone, this, &VibrationHandler::receiveNewSrsY );
    connect( &mSrsThreadZ, &SrsThread::srsComputeDone, this, &VibrationHandler::receiveNewSrsZ );

    memset( &mVibRmsSpeeds, 0, sizeof( mVibRmsSpeeds ) );
}


//!************************************************************************
//! Destructor
//!************************************************************************
VibrationHandler::~VibrationHandler()
{
    mMutex.unlock();
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
/* static */ VibrationHandler* VibrationHandler::getInstance()
{
    if( !sInstance )
    {
        sInstance = new VibrationHandler();
    }

    return sInstance;
}


//!************************************************************************
//! Calculate the spectrum items on X axis
//!
//! @returns nothing
//!************************************************************************
void VibrationHandler::calculateSpectrumX()
{
    if( mSpsReceived )
    {
        mMutex.lock();
            if( mFFtValues.xFft.size() != mFftSize + 1 )
            {
                mFFtValues.xFft.clear();
                mFFtValues.xFft.resize( mFftSize + 1 );
            }

            uint32_t nrOfBins = mFftSize / 2;
            std::vector<double> absFftVec( nrOfBins + 2, 0 );
            double receivedBinWidth = static_cast<double>( mSpsReceived ) / mFftSize;
            FftBin binObj = { 0, 0 };
            FftPsd psdObj = { 0, 0 };
            uint32_t crtBin = 0;

            mFftBins.xBins.clear();
            mFftBins.xBins.resize( nrOfBins );

            mFftPsd.xPsdVec.clear();
            mFftPsd.xPsdVec.resize( nrOfBins );

            std::vector<double> logPwrVec( nrOfBins + 1, 0 );
            bool haveZeros = false;

            absFftVec.at( 1 ) = fabs( mFFtValues.xFft.at( 1 ) );
            absFftVec.at( nrOfBins + 1 ) = fabs( mFFtValues.xFft.at( 2 ) );

            for( crtBin = 2; crtBin <= nrOfBins; crtBin++ )
            {
                absFftVec.at( crtBin ) = sqrt( pow( mFFtValues.xFft.at( 2 * crtBin - 1 ), 2.0 )
                                             + pow( mFFtValues.xFft.at( 2 * crtBin ), 2.0 ) );
            }

            for( crtBin = 1; crtBin <= nrOfBins; crtBin++ )
            {
                binObj.frequency = receivedBinWidth * ( crtBin - 0.5 );
                binObj.value = absFftVec.at( crtBin + 1 );
                mFftBins.xBins.at( crtBin - 1 ) = binObj;

                psdObj.frequency = binObj.frequency;
                psdObj.value = ( binObj.value * binObj.value ) / ( mFftSize * mSpsReceived );
                mFftPsd.xPsdVec.at( crtBin - 1 ) = psdObj;

                if( !haveZeros )
                {
                    if( binObj.value )
                    {
                        logPwrVec.at( crtBin ) = log10( binObj.value * binObj.value );
                    }
                    else
                    {
                        haveZeros = true;
                    }
                }
            }

            if( !haveZeros )
            {
                mCepstrumInvFftThreadX.compute( logPwrVec, FrequencyAnalysis::FFT_SENSE_INVERSE, 0, false );
            }

            mVibRmsSpeeds.xRmsSpeed = 0;

            for( crtBin = 1; crtBin <= nrOfBins; crtBin++ )
            {
                mVibRmsSpeeds.xRmsSpeed += mFftPsd.xPsdVec.at( crtBin - 1 ).value / pow( mFftPsd.xPsdVec.at( crtBin - 1 ).frequency, 2.0 );
            }

            mVibRmsSpeeds.xRmsSpeed *= receivedBinWidth;
            mVibRmsSpeeds.xRmsSpeed = sqrt( mVibRmsSpeeds.xRmsSpeed ) / Numeric::TWO_PI;
        mMutex.unlock();

        emit haveNewFftBins( static_cast<int>( Adxl355Adxl357Common::AXIS_X ) );
        emit haveNewPeriodogram( static_cast<int>( Adxl355Adxl357Common::AXIS_X ) );
    }
}


//!************************************************************************
//! Calculate the spectrum items on Y axis
//!
//! @returns nothing
//!************************************************************************
void VibrationHandler::calculateSpectrumY()
{
    if( mSpsReceived )
    {
        mMutex.lock();
            if( mFFtValues.yFft.size() != mFftSize + 1 )
            {
                mFFtValues.yFft.clear();
                mFFtValues.yFft.resize( mFftSize + 1 );
            }

            uint32_t nrOfBins = mFftSize / 2;
            std::vector<double> absFftVec( nrOfBins + 2, 0 );
            double receivedBinWidth = static_cast<double>( mSpsReceived ) / mFftSize;
            FftBin binObj = { 0, 0 };
            FftPsd psdObj = { 0, 0 };
            uint32_t crtBin = 0;

            mFftBins.yBins.clear();
            mFftBins.yBins.resize( nrOfBins );

            mFftPsd.yPsdVec.clear();
            mFftPsd.yPsdVec.resize( nrOfBins );

            std::vector<double> logPwrVec( nrOfBins + 1, 0 );
            bool haveZeros = false;

            absFftVec.at( 1 ) = fabs( mFFtValues.yFft.at( 1 ) );
            absFftVec.at( nrOfBins + 1 ) = fabs( mFFtValues.yFft.at( 2 ) );

            for( crtBin = 2; crtBin <= nrOfBins; crtBin++ )
            {
                absFftVec.at( crtBin ) = sqrt( pow( mFFtValues.yFft.at( 2 * crtBin - 1 ), 2.0 )
                                             + pow( mFFtValues.yFft.at( 2 * crtBin ), 2.0 ) );
            }

            for( crtBin = 1; crtBin <= nrOfBins; crtBin++ )
            {
                binObj.frequency = receivedBinWidth * ( crtBin - 0.5 );
                binObj.value = absFftVec.at( crtBin + 1 );
                mFftBins.yBins.at( crtBin - 1 ) = binObj;

                psdObj.frequency = binObj.frequency;
                psdObj.value = ( binObj.value * binObj.value ) / ( mFftSize * mSpsReceived );
                mFftPsd.yPsdVec.at( crtBin - 1 ) = psdObj;

                if( !haveZeros )
                {
                    if( binObj.value )
                    {
                        logPwrVec.at( crtBin ) = log10( binObj.value * binObj.value );
                    }
                    else
                    {
                        haveZeros = true;
                    }
                }
            }

            if( !haveZeros )
            {
                mCepstrumInvFftThreadY.compute( logPwrVec, FrequencyAnalysis::FFT_SENSE_INVERSE, 0, false );
            }

            mVibRmsSpeeds.yRmsSpeed = 0;

            for( crtBin = 1; crtBin <= nrOfBins; crtBin++ )
            {
                mVibRmsSpeeds.yRmsSpeed += mFftPsd.yPsdVec.at( crtBin - 1 ).value / pow( mFftPsd.yPsdVec.at( crtBin - 1 ).frequency, 2.0 );
            }

            mVibRmsSpeeds.yRmsSpeed *= receivedBinWidth;
            mVibRmsSpeeds.yRmsSpeed = sqrt( mVibRmsSpeeds.yRmsSpeed ) / Numeric::TWO_PI;
        mMutex.unlock();

        emit haveNewFftBins( static_cast<int>( Adxl355Adxl357Common::AXIS_Y ) );
        emit haveNewPeriodogram( static_cast<int>( Adxl355Adxl357Common::AXIS_Y ) );
    }
}


//!************************************************************************
//! Calculate the spectrum items on Z axis
//!
//! @returns nothing
//!************************************************************************
void VibrationHandler::calculateSpectrumZ()
{
    if( mSpsReceived )
    {
        mMutex.lock();
            if( mFFtValues.zFft.size() != mFftSize + 1 )
            {
                mFFtValues.zFft.clear();
                mFFtValues.zFft.resize( mFftSize + 1 );
            }

            uint32_t nrOfBins = mFftSize / 2;
            std::vector<double> absFftVec( nrOfBins + 2, 0 );
            double receivedBinWidth = static_cast<double>( mSpsReceived ) / mFftSize;
            FftBin binObj = { 0, 0 };
            FftPsd psdObj = { 0, 0 };
            uint32_t crtBin = 0;

            mFftBins.zBins.clear();
            mFftBins.zBins.resize( nrOfBins );

            mFftPsd.zPsdVec.clear();
            mFftPsd.zPsdVec.resize( nrOfBins );

            std::vector<double> logPwrVec( nrOfBins + 1, 0 );
            bool haveZeros = false;

            absFftVec.at( 1 ) = fabs( mFFtValues.zFft.at( 1 ) );
            absFftVec.at( nrOfBins + 1 ) = fabs( mFFtValues.zFft.at( 2 ) );

            for( crtBin = 2; crtBin <= nrOfBins; crtBin++ )
            {
                absFftVec.at( crtBin ) = sqrt( pow( mFFtValues.zFft.at( 2 * crtBin - 1 ), 2.0 )
                                             + pow( mFFtValues.zFft.at( 2 * crtBin ), 2.0 ) );
            }

            for( crtBin = 1; crtBin <= nrOfBins; crtBin++ )
            {
                binObj.frequency = receivedBinWidth * ( crtBin - 0.5 );
                binObj.value = absFftVec.at( crtBin + 1 );
                mFftBins.zBins.at( crtBin - 1 ) = binObj;

                psdObj.frequency = binObj.frequency;
                psdObj.value = ( binObj.value * binObj.value ) / ( mFftSize * mSpsReceived );
                mFftPsd.zPsdVec.at( crtBin - 1 ) = psdObj;

                if( !haveZeros )
                {
                    if( binObj.value )
                    {
                        logPwrVec.at( crtBin ) = log10( binObj.value * binObj.value );
                    }
                    else
                    {
                        haveZeros = true;
                    }
                }
            }

            if( !haveZeros )
            {
                mCepstrumInvFftThreadZ.compute( logPwrVec, FrequencyAnalysis::FFT_SENSE_INVERSE, 0, false );
            }

            mVibRmsSpeeds.zRmsSpeed = 0;

            for( crtBin = 1; crtBin <= nrOfBins; crtBin++ )
            {
                mVibRmsSpeeds.zRmsSpeed += mFftPsd.zPsdVec.at( crtBin - 1 ).value / pow( mFftPsd.zPsdVec.at( crtBin - 1 ).frequency, 2.0 );
            }

            mVibRmsSpeeds.zRmsSpeed *= receivedBinWidth;
            mVibRmsSpeeds.zRmsSpeed = sqrt( mVibRmsSpeeds.zRmsSpeed ) / Numeric::TWO_PI;
        mMutex.unlock();

        emit haveNewFftBins( static_cast<int>( Adxl355Adxl357Common::AXIS_Z ) );
        emit haveNewPeriodogram( static_cast<int>( Adxl355Adxl357Common::AXIS_Z ) );
    }
}


//!************************************************************************
//! Convert an acceleration from [g] to [m/s^2]
//!
//! The result is based on the standard gravity of Earth, and does not take
//! into account the dependency on latitude or altitude.
//! https://en.wikipedia.org/wiki/Gravity_of_Earth
//!
//! @returns The acceleration value in [m/s^2]
//!************************************************************************
double VibrationHandler::convertAccelG2Ms2
    (
    const double aAccelerationG   //!< acceleration [g]
    ) const
{
    const double CONV_STD_G_TO_MS2 = 9.80665; // m/s^2
    return aAccelerationG * CONV_STD_G_TO_MS2;
}


//!************************************************************************
//! Get the FFT bins for a specified axis
//!
//! @returns the bins vector
//!************************************************************************
std::vector<VibrationHandler::FftBin> VibrationHandler::getFftBinsOnAxis
    (
    const Adxl355Adxl357Common::Axis aAxis      //!< axis
    ) const
{
    std::vector<FftBin> fftBins;

    switch( aAxis )
    {
        case Adxl355Adxl357Common::AXIS_X:
            fftBins = mFftBins.xBins;
            break;

        case Adxl355Adxl357Common::AXIS_Y:
            fftBins = mFftBins.yBins;
            break;

        case Adxl355Adxl357Common::AXIS_Z:
            fftBins = mFftBins.zBins;
            break;

        default:
            break;
    }

    return fftBins;
}


//!************************************************************************
//! Get the cepstrum for a specified axis
//!
//! @returns the cepstrum vector
//!************************************************************************
std::vector<VibrationHandler::FftCepstrum> VibrationHandler::getFftCepstrumOnAxis
    (
    const Adxl355Adxl357Common::Axis aAxis      //!< axis
    ) const
{
    std::vector<FftCepstrum> fftCepstrum;

    switch( aAxis )
    {
        case Adxl355Adxl357Common::AXIS_X:
            fftCepstrum = mFftCepstrum.xCepstrumVec;
            break;

        case Adxl355Adxl357Common::AXIS_Y:
            fftCepstrum = mFftCepstrum.yCepstrumVec;
            break;

        case Adxl355Adxl357Common::AXIS_Z:
            fftCepstrum = mFftCepstrum.zCepstrumVec;
            break;

        default:
            break;
    }

    return fftCepstrum;
}


//!************************************************************************
//! Get the PSD for a specified axis
//!
//! @returns the PSD vector
//!************************************************************************
std::vector<VibrationHandler::FftPsd> VibrationHandler::getFftPsdOnAxis
    (
    const Adxl355Adxl357Common::Axis aAxis      //!< axis
    ) const
{
    std::vector<FftPsd> fftPsd;

    switch( aAxis )
    {
        case Adxl355Adxl357Common::AXIS_X:
            fftPsd = mFftPsd.xPsdVec;
            break;

        case Adxl355Adxl357Common::AXIS_Y:
            fftPsd = mFftPsd.yPsdVec;
            break;

        case Adxl355Adxl357Common::AXIS_Z:
            fftPsd = mFftPsd.zPsdVec;
            break;

        default:
            break;
    }

    return fftPsd;
}


//!************************************************************************
//! Get the SRS for a specified axis
//!
//! @returns the SRS vector
//!************************************************************************
std::vector<VibrationHandler::Srs> VibrationHandler::getSrsOnAxis
    (
    const Adxl355Adxl357Common::Axis aAxis      //!< axis
    ) const
{
    std::vector<Srs> srs;

    switch( aAxis )
    {
        case Adxl355Adxl357Common::AXIS_X:
            srs = mSrs.xSrsVec;
            break;

        case Adxl355Adxl357Common::AXIS_Y:
            srs = mSrs.ySrsVec;
            break;

        case Adxl355Adxl357Common::AXIS_Z:
            srs = mSrs.zSrsVec;
            break;

        default:
            break;
    }

    return srs;
}


//!************************************************************************
//! Get the triaxial acceleration arrays [g]
//!
//! @returns the triaxial acceleration arrays
//!************************************************************************
VibrationHandler::AccelerometerDataTriaxial VibrationHandler::getTriaxialAccelArrays() const
{
    return mAccelerationData;
}


//!************************************************************************
//! Get the triaxial vibration RMS speeds [m/s]
//!
//! @returns the triaxial vibration RMS speeds
//!************************************************************************
VibrationHandler::VibrationRmsSpeedsTriaxial VibrationHandler::getTriaxialRmsSpeeds() const
{
    return mVibRmsSpeeds;
}


//!************************************************************************
//! Get the triaxial vibration monitoring settings
//!
//! @returns the triaxial vibration monitoring settings
//!************************************************************************
VibrationHandler::VibrationMonitoringSettingsTriaxial& VibrationHandler::getTriaxialSettings()
{
    return mVibMonSettings;
}


//!************************************************************************
//! Receive new accelerometer data sent by the DAQ thread
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewData
    (
    double  aXaccel,    //!< acceleration on X axis
    double  aYaccel,    //!< acceleration on Y axis
    double  aZaccel     //!< acceleration on Z axis
    )
{
    if( mSpsReceived )
    {
        mMutex.lock();
            shiftLeft( mAccelerationData.xArray );
            shiftLeft( mAccelerationData.yArray );
            shiftLeft( mAccelerationData.zArray );

            mAccelerationData.xArray.back() = aXaccel;
            mAccelerationData.yArray.back() = aYaccel;
            mAccelerationData.zArray.back() = aZaccel;
        mMutex.unlock();

        emit haveNewData();

        mMutex.lock();
            mFftIndexCount++;

            if( mFftSize == mFftIndexCount )
            {
                if( mFFtValues.xFft.size() )
                {
                    mFFtValues.xFft.at( 0 ) = 0;
                    std::copy( mAccelerationData.xArray.begin(), mAccelerationData.xArray.end(), mFFtValues.xFft.begin() + 1 );
                }

                if( mFFtValues.yFft.size() )
                {
                    mFFtValues.yFft.at( 0 ) = 0;
                    std::copy( mAccelerationData.yArray.begin(), mAccelerationData.yArray.end(), mFFtValues.yFft.begin() + 1 );
                }

                if( mFFtValues.zFft.size() )
                {
                    mFFtValues.zFft.at( 0 ) = 0;
                    std::copy( mAccelerationData.zArray.begin(), mAccelerationData.zArray.end(), mFFtValues.zFft.begin() + 1 );
                }

                mFftIndexCount = 0;
                double windowValue = 0;

                FrequencyAnalysis* faInstance = FrequencyAnalysis::getInstance();

                for( size_t i = 1; i < mFFtValues.xFft.size(); i++ )
                {
                    mFFtValues.xFft.at( i ) = convertAccelG2Ms2( mFFtValues.xFft.at( i ) );
                    mFFtValues.yFft.at( i ) = convertAccelG2Ms2( mFFtValues.yFft.at( i ) );
                    mFFtValues.zFft.at( i ) = convertAccelG2Ms2( mFFtValues.zFft.at( i ) );

                    mSrsFeed.xArray.at( i ) = mFFtValues.xFft.at( i );
                    mSrsFeed.yArray.at( i ) = mFFtValues.yFft.at( i );
                    mSrsFeed.zArray.at( i ) = mFFtValues.zFft.at( i );

                    if( faInstance->getWindowFunction().getWindowValue( i - 1, &windowValue ) )
                    {
                        mFFtValues.xFft.at( i ) *= windowValue;
                        mFFtValues.yFft.at( i ) *= windowValue;
                        mFFtValues.zFft.at( i ) *= windowValue;
                    }
                }

                mFftThreadX.compute( mFFtValues.xFft, FrequencyAnalysis::FFT_SENSE_DIRECT, 0, false );
                mFftThreadY.compute( mFFtValues.yFft, FrequencyAnalysis::FFT_SENSE_DIRECT, 0, false );
                mFftThreadZ.compute( mFFtValues.zFft, FrequencyAnalysis::FFT_SENSE_DIRECT, 0, false );

                if( faInstance->getSrsIsRunning() )
                {
                    mSrsThreadX.compute( mSrsFeed.xArray );
                    mSrsThreadY.compute( mSrsFeed.yArray );
                    mSrsThreadZ.compute( mSrsFeed.zArray );
                }
            }
        mMutex.unlock();
    }
}


//!************************************************************************
//! Receive computed cepstrum values from dedicated thread
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewCepstrumX
    (
    double*  aDataArray,    //!< array with calculated cepstrum values
    int      aLength,       //!< array length
    int      aIndex         //!< index
    )
{
    Q_UNUSED( aIndex );
    std::vector<double> cepstrumVec;

    if( aDataArray )
    {
        cepstrumVec.assign( aDataArray, aDataArray + aLength);
    }

    mMutex.lock();
        int32_t nrOfLines = cepstrumVec.size() - 1;
        FftCepstrum cepstrumObj = { 0, 0 };

        mFftCepstrum.xCepstrumVec.clear();
        mFftCepstrum.xCepstrumVec.resize( nrOfLines );

        for( uint32_t crtLine = 1; crtLine <= nrOfLines; crtLine++ )
        {
            cepstrumObj.quefrency = static_cast<double>( 2 * crtLine - 1 ) / mSpsReceived;
            cepstrumObj.value = cepstrumVec.at( crtLine ) * cepstrumVec.at( crtLine );
            mFftCepstrum.xCepstrumVec.at( crtLine - 1 ) = cepstrumObj;
        }
    mMutex.unlock();

    if( nrOfLines )
    {
        emit haveNewCepstrum( static_cast<int>( Adxl355Adxl357Common::AXIS_X ) );
    }
}


//!************************************************************************
//! Receive computed cepstrum values from dedicated thread
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewCepstrumY
    (
    double*  aDataArray,    //!< array with calculated cepstrum values
    int      aLength,       //!< array length
    int      aIndex         //!< index
    )
{
    Q_UNUSED( aIndex );
    std::vector<double> cepstrumVec;

    if( aDataArray )
    {
        cepstrumVec.assign( aDataArray, aDataArray + aLength);
    }

    mMutex.lock();
        int32_t nrOfLines = cepstrumVec.size() - 1;
        FftCepstrum cepstrumObj = { 0, 0 };

        mFftCepstrum.yCepstrumVec.clear();
        mFftCepstrum.yCepstrumVec.resize( nrOfLines );

        for( uint32_t crtLine = 1; crtLine <= nrOfLines; crtLine++ )
        {
            cepstrumObj.quefrency = static_cast<double>( 2 * crtLine - 1 ) / mSpsReceived;
            cepstrumObj.value = cepstrumVec.at( crtLine ) * cepstrumVec.at( crtLine );
            mFftCepstrum.yCepstrumVec.at( crtLine - 1 ) = cepstrumObj;
        }
    mMutex.unlock();

    if( nrOfLines )
    {
        emit haveNewCepstrum( static_cast<int>( Adxl355Adxl357Common::AXIS_Y ) );
    }
}


//!************************************************************************
//! Receive computed cepstrum values from dedicated thread
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewCepstrumZ
    (
    double*  aDataArray,    //!< array with calculated cepstrum values
    int      aLength,       //!< array length
    int      aIndex         //!< index
    )
{
    Q_UNUSED( aIndex );
    std::vector<double> cepstrumVec;

    if( aDataArray )
    {
        cepstrumVec.assign( aDataArray, aDataArray + aLength);
    }

    mMutex.lock();
        int32_t nrOfLines = cepstrumVec.size() - 1;
        FftCepstrum cepstrumObj = { 0, 0 };

        mFftCepstrum.zCepstrumVec.clear();
        mFftCepstrum.zCepstrumVec.resize( nrOfLines );

        for( uint32_t crtLine = 1; crtLine <= nrOfLines; crtLine++ )
        {
            cepstrumObj.quefrency = static_cast<double>( 2 * crtLine - 1 ) / mSpsReceived;
            cepstrumObj.value = cepstrumVec.at( crtLine ) * cepstrumVec.at( crtLine );
            mFftCepstrum.zCepstrumVec.at( crtLine - 1 ) = cepstrumObj;
        }
    mMutex.unlock();

    if( nrOfLines )
    {
        emit haveNewCepstrum( static_cast<int>( Adxl355Adxl357Common::AXIS_Z ) );
    }
}


//!************************************************************************
//! Receive computed FFT values from dedicated thread
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewFftX
    (
    double*  aDataArray,    //!< array with calculated FFT values
    int      aLength,       //!< array length
    int      aIndex         //!< index
    )
{
    Q_UNUSED( aIndex );
    std::vector<double> fftVec;

    if( aDataArray )
    {
        fftVec.assign( aDataArray, aDataArray + aLength );
    }

    mMutex.lock();
        mFFtValues.xFft = fftVec;
    mMutex.unlock();

    calculateSpectrumX();
}


//!************************************************************************
//! Receive computed FFT values from dedicated thread
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewFftY
    (
    double*  aDataArray,    //!< array with calculated FFT values
    int      aLength,       //!< array length
    int      aIndex         //!< index
    )
{
    Q_UNUSED( aIndex );
    std::vector<double> fftVec;

    if( aDataArray )
    {
        fftVec.assign( aDataArray, aDataArray + aLength );
    }

    mMutex.lock();
        mFFtValues.yFft = fftVec;
    mMutex.unlock();

    calculateSpectrumY();
}


//!************************************************************************
//! Receive computed FFT values from dedicated thread
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewFftZ
    (
    double*  aDataArray,    //!< array with calculated FFT values
    int      aLength,       //!< array length
    int      aIndex         //!< index
    )
{
    Q_UNUSED( aIndex );
    std::vector<double> fftVec;

    if( aDataArray )
    {
        fftVec.assign( aDataArray, aDataArray + aLength );
    }

    mMutex.lock();
        mFFtValues.zFft = fftVec;
    mMutex.unlock();

    calculateSpectrumZ();
}


//!************************************************************************
//! Receive computed SRS values from dedicated thread
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewSrsX
    (
    double*  aNaturalFrequenciesArray,  //!< natural frequencies
    double*  aValuesArray,              //!< computed SRS values
    int      aLength                    //!< length of both arrays
    )
{
    std::vector<double> natFreqVec;
    std::vector<double> srsVec;

    if( aNaturalFrequenciesArray )
    {
        natFreqVec.assign( aNaturalFrequenciesArray, aNaturalFrequenciesArray + aLength );
    }

    if( aValuesArray )
    {
        srsVec.assign( aValuesArray, aValuesArray + aLength );
    }

    mMutex.lock();
        size_t nrOfPoints = natFreqVec.size();

        mSrs.xSrsVec.clear();
        mSrs.xSrsVec.resize( nrOfPoints );

        for( size_t crtPoint = 0; crtPoint < nrOfPoints; crtPoint++ )
        {
            mSrs.xSrsVec.at( crtPoint ) = { natFreqVec.at( crtPoint ), srsVec.at( crtPoint ) };
        }
    mMutex.unlock();

    emit haveNewSrs( static_cast<int>( Adxl355Adxl357Common::AXIS_X ) );
}


//!************************************************************************
//! Receive computed SRS values from dedicated thread
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewSrsY
    (
    double*  aNaturalFrequenciesArray,  //!< natural frequencies
    double*  aValuesArray,              //!< computed SRS values
    int      aLength                    //!< length of both arrays
    )
{
    std::vector<double> natFreqVec;
    std::vector<double> srsVec;

    if( aNaturalFrequenciesArray )
    {
        natFreqVec.assign( aNaturalFrequenciesArray, aNaturalFrequenciesArray + aLength );
    }

    if( aValuesArray )
    {
        srsVec.assign( aValuesArray, aValuesArray + aLength );
    }

    mMutex.lock();
        size_t nrOfPoints = natFreqVec.size();

        mSrs.ySrsVec.clear();
        mSrs.ySrsVec.resize( nrOfPoints );

        for( size_t crtPoint = 0; crtPoint < nrOfPoints; crtPoint++ )
        {
            mSrs.ySrsVec.at( crtPoint ) = { natFreqVec.at( crtPoint ), srsVec.at( crtPoint ) };
        }
    mMutex.unlock();

    emit haveNewSrs( static_cast<int>( Adxl355Adxl357Common::AXIS_Y ) );
}


//!************************************************************************
//! Receive computed SRS values from dedicated thread
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewSrsZ
    (
    double*  aNaturalFrequenciesArray,  //!< natural frequencies
    double*  aValuesArray,              //!< computed SRS values
    int      aLength                    //!< length of both arrays
    )
{
    std::vector<double> natFreqVec;
    std::vector<double> srsVec;

    if( aNaturalFrequenciesArray )
    {
        natFreqVec.assign( aNaturalFrequenciesArray, aNaturalFrequenciesArray + aLength );
    }

    if( aValuesArray )
    {
        srsVec.assign( aValuesArray, aValuesArray + aLength );
    }

    mMutex.lock();
        size_t nrOfPoints = natFreqVec.size();

        mSrs.zSrsVec.clear();
        mSrs.zSrsVec.resize( nrOfPoints );

        for( size_t crtPoint = 0; crtPoint < nrOfPoints; crtPoint++ )
        {
            mSrs.zSrsVec.at( crtPoint ) = { natFreqVec.at( crtPoint ), srsVec.at( crtPoint ) };
        }
    mMutex.unlock();

    emit haveNewSrs( static_cast<int>( Adxl355Adxl357Common::AXIS_Z ) );
}


//!************************************************************************
//! Receive the SPS value sent by the DAQ thread
//!
//! *** The parameter is always integer ***
//! e.g.  4000, 2000, ..., 7, 3
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void VibrationHandler::receiveNewSps
    (
    int aSps    //!< SPS
    )
{
    mMutex.lock();
        mSpsReceived = aSps > 0 ? aSps : 0;
    mMutex.unlock();
}


//!************************************************************************
//! Set the FFT size
//! Also update the size of SRS data feed, which depends on the FFT size.
//!
//! @returns: nothing
//!************************************************************************
void VibrationHandler::setFftSize
    (
    const uint32_t aFFtSize     //!< FFT size
    )
{
    if( aFFtSize > 0 )
    {
        mMutex.lock();
            mFftSize = aFFtSize;

            mFftIndexCount = 0;

            mAccelerationData.xArray.clear();
            mAccelerationData.xArray.resize( mFftSize );

            mAccelerationData.yArray.clear();
            mAccelerationData.yArray.resize( mFftSize );

            mAccelerationData.zArray.clear();
            mAccelerationData.zArray.resize( mFftSize );


            mFFtValues.xFft.clear();
            mFFtValues.xFft.resize( mFftSize + 1 );

            mFFtValues.yFft.clear();
            mFFtValues.yFft.resize( mFftSize + 1 );

            mFFtValues.zFft.clear();
            mFFtValues.zFft.resize( mFftSize + 1 );


            mSrsFeed.xArray.clear();
            mSrsFeed.xArray.resize( mFftSize + 1 );

            mSrsFeed.yArray.clear();
            mSrsFeed.yArray.resize( mFftSize + 1 );

            mSrsFeed.zArray.clear();
            mSrsFeed.zArray.resize( mFftSize + 1 );
        mMutex.unlock();
    }
}


//!************************************************************************
//! Set the expected (theoretical) sampling rate [Hz]
//!
//! *** The parameter is not always integer ***
//! e.g.  4000, 2000, ..., 7.8125, 3.90625
//!
//! @returns: nothing
//!************************************************************************
void VibrationHandler::setSpsExpected
    (
    const double aSps       //!< sampling rate [Hz]
    )
{
    if( aSps > 0 )
    {
        mMutex.lock();
            mSpsExpected = aSps;
        mMutex.unlock();
    }
}


//!************************************************************************
//! Shift left the elements of a vector
//! Last element remains unchanged.
//!
//! @returns: nothing
//!************************************************************************
void VibrationHandler::shiftLeft
    (
    std::vector<double>&    aVector //!< vector
    ) const
{
    size_t len = aVector.size();

    if( len >= 2 )
    {
        for( size_t i = 0; i <= len - 2; i++ )
        {
            aVector.at( i ) = aVector.at( i + 1 );
        }
    }
}
