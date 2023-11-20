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
FrequencyAnalysis.h

This file contains the definitions for the frequency analysis.
*/

#ifndef FrequencyAnalysis_h
#define FrequencyAnalysis_h

#include "WindowFunction.h"

#include <cstdint>
#include <map>

class Numeric;
class VibrationHandler;


//************************************************************************
// Class for handling frequency analysis
//************************************************************************
class FrequencyAnalysis
{
    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        //************************************************************************
        // Fast Fourier Transform
        //************************************************************************
        typedef enum : uint8_t
        {
            FFT_SIZE_32,
            FFT_SIZE_64,
            FFT_SIZE_128,
            FFT_SIZE_256,
            FFT_SIZE_512,
            FFT_SIZE_1024,
            FFT_SIZE_2048,
            FFT_SIZE_4096,
            /*
            FFT_SIZE_8192,
            FFT_SIZE_16384,
            FFT_SIZE_32768,
            FFT_SIZE_65536,
            FFT_SIZE_131072,
            FFT_SIZE_262144,
            FFT_SIZE_524288,
            FFT_SIZE_1048576,
            */

            // keep this last
            FFT_SIZE_MAX
        }FftSizeOption;

        static const std::map<FftSizeOption, uint32_t> FFT_SIZE_VALUES;

        typedef enum : int8_t
        {
            FFT_SENSE_DIRECT  =  1,
            FFT_SENSE_INVERSE = -1
        }FftSense;

        //************************************************************************
        // Shock Response Spectrum
        //************************************************************************
        typedef enum : uint8_t
        {
            SRS_MODEL_ABSOLUTE_ACCELERATION,
            SRS_MODEL_RELATIVE_DISPLACEMENT
        }SrSModel;

        static const std::map<SrSModel, std::string> SRS_METHOD_NAMES;

        ///////////////////////////
        /// SRS damping coefficient
        ///////////////////////////
        typedef struct
        {
            double minValue;        //!< minimum allowed value
            double crtValue;        //!< current value
            double maxValue;        //!< maximum allowed value
            QString paramSymbol;    //!< symbol for the parameter
        }SrsZeta;

        typedef struct
        {
            SrSModel    model;              //!< filter model
            double      samplingRate;       //!< SPS [Hz]
            SrsZeta     zeta;               //!< damping coefficient [-]
            double      naturalFreq;        //!< natural frequency [Hz]
        }SrsParameters;

        typedef struct
        {
            double b0;      //!< numerator coefficient for z^(0)
            double b1;      //!< numerator coefficient for z^(-1)
            double b2;      //!< numerator coefficient for z^(-2)
            double a1p2;    //!< recursive a1 + 2
            double a2m1;    //!< recursive a2 - 1
        }SrsCoefficients;

    private:
        const QString ZETA_SMALL = QString::fromUtf8( "\u03B6" );   //!< small zeta


    //************************************************************************
    // functions
    //************************************************************************
    public:
        FrequencyAnalysis();

        ~FrequencyAnalysis();

        static FrequencyAnalysis* getInstance();

        void calculateFft
            (
            double          aData[],            //!< real values, data [1]..[n]
            const uint32_t  aFftSize,           //!< FFT size, must be a power of two
            const FftSense  aTransformSense     //!< direct or inverse transform
            );

        double getFftBinWidth() const;

        double getFftFreqMax() const;

        FftSizeOption getFftSizeOptionIndex() const;

        uint32_t getFftSizeValue() const;

        double getFftSps() const;

        double getFftTimeGate() const;

        SrsCoefficients getSrsCoefficients() const;

        bool& getSrsIsRunning();

        SrsParameters& getSrsParameters();

        WindowFunction& getWindowFunction();

        bool setFftSizeOptionIndex
            (
            const FftSizeOption aFftSizeIndex           //!< FFT size index
            );

        bool setFftSps
            (
            const double        aSps                    //!< SPS [Hz]
            );

        void updateSrsCoefficients();

    private:
        void calculateFourierTransform
            (
            double          aData[],            //!< data, dimension is right shifted FFT size
            const uint32_t  aShrFftSize,        //!< right shifted FFT size, must be a power of two
            const FftSense  aTransformSense     //!< direct or inverse transform
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        static FrequencyAnalysis*   sInstance;              //!< singleton
        Numeric*                    mNumericInstance;       //!< Numeric instance
        VibrationHandler*           mVibrationHndlInstance; //!< VibrationHandler instance

        double                      mSps;                   //!< sampling rate [Hz]

        double                      mFftFreqMax;            //!< FFT max frequency [Hz]
        FftSizeOption               mFftSizeIndex;          //!< FFT size index
        uint32_t                    mFftSizeValue;          //!< FFT size value
        double                      mFftBinWidth;           //!< FFT bin width [Hz/bin]
        double                      mFftTimeGate;           //!< FFT time gate [s]

        WindowFunction              mWindowFunction;        //!< FFT window function

        SrsParameters               mSrsParameters;         //!< SRS parameters
        SrsCoefficients             mSrsCoefficients;       //!< SRS coefficients
        bool                        mSrsIsRunning;          //!< true if SRS is running
};

#endif // FrequencyAnalysis_h
