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
FrequencyAnalysis.cpp

This file contains the sources for the frequency analysis.
*/

/*
References (selected list):
===========================
[1] Agilent Technologies - The Fundamentals of Signal Analysis. Application Note 243, 2000

[2] Gaberson, H.A. - Shock Spectrum Calculation from Acceleration Time Histories, Civil Eng. Lab.
        Technical Note N-1590, 1980

[3] O'Hara, G.J. - A Numerical Procedure for Shock and Fourier Analysis, NRL Report 5772, 1962

[4] Press, W.H. et al - Numerical Recipes in C. The Art of Scientific Computing, 2nd Ed,
        Cambridge Univ. Press, 1992

[5] Randall, R.B. - Frequency Analysis, 3rd Ed, Bruel&Kjaer, 1987

[6] Randall, R.B., Hee, J. - Cepstrum Analysis, Bruel&Kjaer Technical Review 3, 1981, pp. 3-40

[7] Roberts, M.J. - Signals and Systems: Analysis Using Transform Methods and MATLAB, McGraw-Hill, 2003

[8] Smallwood, D.O. - An Improved Recursive Formula for Calculating Shock Response Spectra,
        The Shock and Vibration Bulletin 51 (2), 1981, pp. 211-217

[9] Smallwood, D.O. - The Shock Response Spectrum at Low Frequencies, Proc. of the 56th Shock and
        Vibration Symposium, 1985, pp. 279-288

[10] ISO 18431-3:2014. Mechanical vibration and shock - Signal processing - Part 3: Methods of
        time-frequency analysis

[11] ISO 18431-4:2007. Mechanical vibration and shock - Signal processing - Part 4: Shock-response
        spectrum analysis
*/

#include "FrequencyAnalysis.h"

#include "Numeric.h"
#include "VibrationHandler.h"


FrequencyAnalysis* FrequencyAnalysis::sInstance = nullptr;

const std::map<FrequencyAnalysis::FftSizeOption, uint32_t> FrequencyAnalysis::FFT_SIZE_VALUES =
{
    { FrequencyAnalysis::FFT_SIZE_32,           32 },
    { FrequencyAnalysis::FFT_SIZE_64,           64 },
    { FrequencyAnalysis::FFT_SIZE_128,         128 },
    { FrequencyAnalysis::FFT_SIZE_256,         256 },
    { FrequencyAnalysis::FFT_SIZE_512,         512 },
    { FrequencyAnalysis::FFT_SIZE_1024,       1024 },
    { FrequencyAnalysis::FFT_SIZE_2048,       2048 },
    { FrequencyAnalysis::FFT_SIZE_4096,       4096 } /*,
    { FrequencyAnalysis::FFT_SIZE_8192,       8192 },
    { FrequencyAnalysis::FFT_SIZE_16384,     16384 },
    { FrequencyAnalysis::FFT_SIZE_32768,     32768 },
    { FrequencyAnalysis::FFT_SIZE_65536,     65536 },
    { FrequencyAnalysis::FFT_SIZE_131072,   131072 },
    { FrequencyAnalysis::FFT_SIZE_262144,   262144 },
    { FrequencyAnalysis::FFT_SIZE_524288,   524288 },
    { FrequencyAnalysis::FFT_SIZE_1048576, 1048576 } */
};

const std::map<FrequencyAnalysis::SrSModel, std::string> FrequencyAnalysis::SRS_METHOD_NAMES =
{
    { FrequencyAnalysis::SRS_MODEL_ABSOLUTE_ACCELERATION, "Absolute acceleration" },
    { FrequencyAnalysis::SRS_MODEL_RELATIVE_DISPLACEMENT, "Relative displacement" }
};

//!************************************************************************
//! Constructor
//!************************************************************************
FrequencyAnalysis::FrequencyAnalysis()
    : mSps( 1 )
    , mFftFreqMax( mSps / 2 )
    , mFftSizeIndex( FFT_SIZE_2048 )
    , mFftSizeValue( FFT_SIZE_VALUES.at( mFftSizeIndex ) )
    , mFftBinWidth( 1 )
    , mFftTimeGate( 1 / mFftBinWidth )
    , mSrsIsRunning( false )
{    
    mWindowFunction.setFftSize( mFftSizeValue );

    mSrsParameters =
    {
        SRS_MODEL_RELATIVE_DISPLACEMENT,
        mSps,
        { 0, 0.05, 0.99, ZETA_SMALL },
        0
    };

    memset( &mSrsCoefficients, 0, sizeof( mSrsCoefficients ) );

    mNumericInstance = Numeric::getInstance();

    mVibrationHndlInstance = VibrationHandler::getInstance();
    mVibrationHndlInstance->setFftSize( mFftSizeValue );
    mVibrationHndlInstance->setSpsExpected( mSps );
}


//!************************************************************************
//! Destructor
//!************************************************************************
FrequencyAnalysis::~FrequencyAnalysis()
{
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
FrequencyAnalysis* FrequencyAnalysis::getInstance()
{
    if( !sInstance )
    {
        sInstance = new FrequencyAnalysis;
    }

    return sInstance;
}


//!************************************************************************
//! Calculate the Fast Fourier Transform for an array of real values
//!
//! @returns nothing
//!************************************************************************
void FrequencyAnalysis::calculateFft
    (
    double			aData[],		//!< real values, data [1]..[aFftSize]
    const uint32_t	aFftSize,		//!< FFT size, must be a power of two
    const FftSense  aTransformSense	//!< direct or inverse transform
    )
{
    if( aData )
    {
        if( mNumericInstance->isPowerOfTwo( aFftSize ) )
        {
            double theta = Numeric::PI / static_cast<double>( aFftSize >> 1 );
            double c2 = -0.5;

            if( FFT_SENSE_DIRECT == aTransformSense )
            {
                calculateFourierTransform( aData, aFftSize >> 1, FFT_SENSE_DIRECT );
            }
            else if( FFT_SENSE_INVERSE == aTransformSense )
            {
                c2 = -c2;
                theta = -theta;
            }

            double wtemp = sin( 0.5 * theta );
            double wpr = -2.0 * wtemp * wtemp;
            double wpi = sin( theta );
            double wr = 1.0 + wpr;
            double wi = wpi;
            double c1 = 0.5;
            double h1r = 0;
            double h1i = 0;
            double h2r = 0;
            double h2i = 0;
            uint32_t i1 = 0;
            uint32_t i2 = 0;
            uint32_t i3 = 0;
            uint32_t i4 = 0;
            uint32_t np3 = aFftSize + 3;

            for( uint32_t i = 2; i <= ( aFftSize >> 2 ); i++ )
            {               
                i1 = 2 * i - 1;
                i2 = 1 + i1;
                i3 = np3 - i2;
                i4 = 1 + i3;

                h1r = c1 * ( aData[i1] + aData[i3] );
                h1i = c1 * ( aData[i2] - aData[i4] );

                h2r = -c2 * ( aData[i2] + aData[i4] );
                h2i = c2 * ( aData[i1] - aData[i3] );

                aData[i1] = h1r + wr * h2r - wi * h2i;
                aData[i2] = h1i + wr * h2i + wi * h2r;
                aData[i3] = h1r - wr * h2r + wi * h2i;
                aData[i4] = -h1i + wr * h2i + wi * h2r;

                wtemp = wr;
                wr = wtemp * wpr - wi * wpi + wr;
                wi = wi * wpr + wtemp * wpi + wi;
            }

            h1r = aData[1];

            if( FFT_SENSE_DIRECT == aTransformSense )
            {
                aData[1] = h1r + aData[2];
                aData[2] = h1r - aData[2];
            }
            else if( FFT_SENSE_INVERSE == aTransformSense )
            {
                aData[1] = c1 * ( h1r + aData[2] );
                aData[2] = c1 * ( h1r - aData[2] );

                calculateFourierTransform( aData, aFftSize >> 1, FFT_SENSE_INVERSE );
            }
        }
    }
}


//!************************************************************************
//! Calculate the 1D Fourier transform for real values
//!
//! @returns nothing
//!************************************************************************
void FrequencyAnalysis::calculateFourierTransform
    (
    double          aData[],		//!< data, dimension is right shifted FFT size
    const uint32_t	aShrFftSize,	//!< right shifted FFT size, must be a power of two
    const FftSense	aTransformSense	//!< direct or inverse transform
    )
{
    if( aData )
    {
        if( mNumericInstance->isPowerOfTwo( aShrFftSize ) )
        {
            const uint32_t FFT_SIZE = aShrFftSize << 1;
            uint32_t i = 0;
            uint32_t j = 1;
            uint32_t m = 0;

            for( i = 1; i < FFT_SIZE; i += 2 )
            {
                if( j > i )
                {
                    mNumericInstance->swap( aData[j], aData[i] );
                    mNumericInstance->swap( aData[j + 1], aData[i + 1] );
                }

                m = FFT_SIZE >> 1;

                while( m >= 2 && j > m )
                {
                    j -= m;
                    m >>= 1;
                }

                j += m;
            }

            uint32_t mmax = 2;
            uint32_t istep = 0;

            double theta = 0;
            double tempr = 0;
            double tempi = 0;

            while( FFT_SIZE > mmax )
            {
                istep = mmax << 1;
                theta = aTransformSense * ( Numeric::TWO_PI / mmax );
                double wtemp = sin( 0.5 * theta );
                double wpr = -2.0 * wtemp * wtemp;
                double wpi = sin( theta );
                double wr = 1.0;
                double wi = 0.0;

                for( m = 1; m < mmax; m += 2 )
                {
                    for( i = m; i <= FFT_SIZE; i += istep )
                    {
                        j = i + mmax;

                        tempr = wr * aData[j] - wi * aData[j + 1];
                        tempi = wr * aData[j + 1] + wi * aData[j];

                        aData[j] = aData[i] - tempr;
                        aData[j + 1] = aData[i + 1] - tempi;

                        aData[i] += tempr;
                        aData[i + 1] += tempi;
                    }

                    wtemp = wr;
                    wr = wtemp * wpr - wi * wpi + wr;
                    wi = wi * wpr + wtemp * wpi + wi;
                }

                mmax = istep;
            }
        }
    }
}


//!************************************************************************
//! Get the FFT bin width (frequency resolution) [Hz/bin]
//!
//! @returns The FFT bin width
//!************************************************************************
double FrequencyAnalysis::getFftBinWidth() const
{
    return mFftBinWidth;
}


//!************************************************************************
//! Get the FFT maximum frequency [Hz]
//!
//! @returns The FFT maximum frequency
//!************************************************************************
double FrequencyAnalysis::getFftFreqMax() const
{
    return mFftFreqMax;
}


//!************************************************************************
//! Get the index of the FFT size option
//!
//! @returns The FFT size index
//!************************************************************************
FrequencyAnalysis::FftSizeOption FrequencyAnalysis::getFftSizeOptionIndex() const
{
    return mFftSizeIndex;
}


//!************************************************************************
//! Get the value of the FFT size
//!
//! @returns The FFT size
//!************************************************************************
uint32_t FrequencyAnalysis::getFftSizeValue() const
{
    return mFftSizeValue;
}


//!************************************************************************
//! Get the FFT sampling rate [Hz]
//!
//! @returns The FFT SPS
//!************************************************************************
double FrequencyAnalysis::getFftSps() const
{
    return mSps;
}


//!************************************************************************
//! Get the FFT time gate [s]
//! The time gate [s] is equal to the inverse of the bin width [Hz/bin].
//!
//! @returns The FFT time gate
//!************************************************************************
double FrequencyAnalysis::getFftTimeGate() const
{
    return mFftTimeGate;
}


//!************************************************************************
//! Get the SRS coefficients
//!
//! @returns The structure with SRS coefficients
//!************************************************************************
FrequencyAnalysis::SrsCoefficients FrequencyAnalysis::getSrsCoefficients() const
{
    return mSrsCoefficients;
}


//!************************************************************************
//! Get the address of SRS running status
//!
//! @returns The SRS running status address
//!************************************************************************
bool& FrequencyAnalysis::getSrsIsRunning()
{
    return mSrsIsRunning;
}


//!************************************************************************
//! Get the address of the SRS parameters
//!
//! @returns The SRS parameters address
//!************************************************************************
FrequencyAnalysis::SrsParameters& FrequencyAnalysis::getSrsParameters()
{
    return mSrsParameters;
}


//!************************************************************************
//! Get the address of FFT window function
//!
//! @returns The FFT window function address
//!************************************************************************
WindowFunction& FrequencyAnalysis::getWindowFunction()
{
    return mWindowFunction;
}


//!************************************************************************
//! Set a new index in the FFT size options
//!
//! @returns true if the value could be set
//!************************************************************************
bool FrequencyAnalysis::setFftSizeOptionIndex
    (
    const FftSizeOption aFftSizeIndex          //!< FFT size index
    )
{
    bool status = aFftSizeIndex < FFT_SIZE_MAX;

    if( status && ( aFftSizeIndex != mFftSizeIndex ) )
    {
        mFftSizeIndex = aFftSizeIndex;
        mFftSizeValue = FFT_SIZE_VALUES.at( mFftSizeIndex );

        mFftBinWidth = mSps / mFftSizeValue;
        mFftTimeGate = 1.0 /  mFftBinWidth;

        getWindowFunction().setFftSize( mFftSizeValue );

        mVibrationHndlInstance->setFftSize( mFftSizeValue );
    }

    return status;
}


//!************************************************************************
//! Set a new FFT sampling rate
//!
//! @returns true if the value could be set
//!************************************************************************
bool FrequencyAnalysis::setFftSps
    (
    const double aSps   //!< SPS [Hz]
    )
{
    bool status = aSps > 0;

    if( status && ( aSps != mSps ) )
    {
        mSps = aSps;

        mFftFreqMax = mSps / 2.0;
        mFftBinWidth = mSps / mFftSizeValue;
        mFftTimeGate = 1.0 /  mFftBinWidth;

        mSrsParameters.samplingRate = mSps;

        mVibrationHndlInstance->setSpsExpected( mSps );
    }

    return status;
}


//!************************************************************************
//! Update the SRS filter coefficients
//!
//! @returns nothing
//!************************************************************************
void FrequencyAnalysis::updateSrsCoefficients()
{
    memset( &mSrsCoefficients, 0, sizeof( mSrsCoefficients ) );

    double ot = Numeric::TWO_PI * mSrsParameters.naturalFreq / mSrsParameters.samplingRate;
    double zetaSqr = pow( mSrsParameters.zeta.crtValue, 2.0 );
    double zetaOt = mSrsParameters.zeta.crtValue * ot;

    if( ot < 1.e-3 ) // undersampling
    {
        double otSqr = ot * ot;
        mSrsCoefficients.a1p2 = 2.0 * zetaOt + ( 1.0 - 2.0 * zetaSqr ) * otSqr;
        mSrsCoefficients.a2m1 = -2.0 * zetaOt + 2.0 * zetaSqr * otSqr;
        const double ONE_SIXTH = 1.0 / 6.0;
        const double TWO_THIRDS = 2.0 / 3.0;

        switch( mSrsParameters.model )
        {
            case SRS_MODEL_ABSOLUTE_ACCELERATION:
                mSrsCoefficients.b0 = zetaOt + otSqr * ( ONE_SIXTH - zetaSqr * TWO_THIRDS );
                mSrsCoefficients.b1 = otSqr * ( 1.0 - zetaSqr ) * TWO_THIRDS;
                mSrsCoefficients.b2 = -zetaOt + otSqr * ( ONE_SIXTH + zetaSqr * 4.0 / 3.0 );
                break;

            case SRS_MODEL_RELATIVE_DISPLACEMENT:
                mSrsCoefficients.b0 = otSqr * ONE_SIXTH;
                mSrsCoefficients.b1 = otSqr * TWO_THIRDS;
                mSrsCoefficients.b2 = mSrsCoefficients.b0;
                break;

            default:
                break;
        }
    }
    else // Nyquist
    {
        double e = exp( -zetaOt );
        double eSqr = e * e;
        double k = ot * sqrt( 1.0 - zetaSqr );
        double c = e * cos( k );
        double s = e * sin( k );
        mSrsCoefficients.a1p2 = 2.0 * ( 1.0 - c );
        mSrsCoefficients.a2m1 = eSqr - 1.0;

        switch( mSrsParameters.model )
        {
            case SRS_MODEL_ABSOLUTE_ACCELERATION:
                {
                    double sp = s / k;
                    mSrsCoefficients.b0 = 1.0 - sp;
                    mSrsCoefficients.b1 = 2.0 * ( sp - c );
                    mSrsCoefficients.b2 = eSqr - sp;
                }
                break;

            case SRS_MODEL_RELATIVE_DISPLACEMENT:
                {
                    double twoZeta = 2.0 * mSrsParameters.zeta.crtValue;
                    double fact = s * ( 2.0 * zetaSqr - 1.0 ) / sqrt( 1.0 - zetaSqr );
                    mSrsCoefficients.b0 = ( twoZeta * ( c - 1.0 ) + fact + ot ) / ot;
                    mSrsCoefficients.b1 = ( -2.0 * c * ot + twoZeta * ( 1.0 - eSqr ) - 2.0 * fact ) / ot;
                    mSrsCoefficients.b2 = ( eSqr * ( ot + twoZeta ) - twoZeta * c + fact ) / ot;
                }
                break;

            default:
                break;
        }
    }
}
