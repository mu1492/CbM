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
VibrationHandler.h

This file contains the definitions for the mechanical vibrations handler.
*/

#ifndef VibrationHandler_h
#define VibrationHandler_h

#include "Adxl355Adxl357Common.h"
#include "FftThread.h"
#include "FrequencyAnalysis.h"
#include "SrsThread.h"
#include "VibrationMonitoringSettings.h"

#include <map>
#include <vector>

#include <QMutex>
#include <QObject>


//************************************************************************
// Class for handling the mechanical vibrations status
//************************************************************************
class VibrationHandler : public QObject
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        typedef struct
        {
            VibrationMonitoringSettings     xAxis;  //!< vibration monitoring settings for X axis
            VibrationMonitoringSettings     yAxis;  //!< vibration monitoring settings for Y axis
            VibrationMonitoringSettings     zAxis;  //!< vibration monitoring settings for Z axis
        }VibrationMonitoringSettingsTriaxial;

        typedef struct
        {
            std::vector<double> xArray;     //!< accelerations on X axis [g]
            std::vector<double> yArray;     //!< accelerations on Y axis [g]
            std::vector<double> zArray;     //!< accelerations on Z axis [g]
        }AccelerometerDataTriaxial;

        typedef struct
        {
            std::vector<double> xFft;       //!< calculated FFT values on X axis
            std::vector<double> yFft;       //!< calculated FFT values on Y axis
            std::vector<double> zFft;       //!< calculated FFT values on Z axis
        }FftTriaxial;

        typedef struct
        {
            double frequency;               //!< bin frequency
            double value;                   //!< bin value
        }FftBin;

        typedef struct
        {
            std::vector<FftBin> xBins;      //!< FFT bins on X axis
            std::vector<FftBin> yBins;      //!< FFT bins on Y axis
            std::vector<FftBin> zBins;      //!< FFT bins on Z axis
        }FftBinsTriaxial;

        typedef struct
        {
            double frequency;               //!< PSD frequency
            double value;                   //!< PSD value
        }FftPsd;

        typedef struct
        {
            std::vector<FftPsd> xPsdVec;    //!< PSD vector on X axis
            std::vector<FftPsd> yPsdVec;    //!< PSD vector on Y axis
            std::vector<FftPsd> zPsdVec;    //!< PSD vector on Z axis
        }FftPsdTriaxial;

        typedef struct
        {
            double quefrency;
            double value;
        }FftCepstrum;

        typedef struct
        {
            std::vector<FftCepstrum> xCepstrumVec;  //!< cepstrum vector on X axis
            std::vector<FftCepstrum> yCepstrumVec;  //!< cepstrum vector on Y axis
            std::vector<FftCepstrum> zCepstrumVec;  //!< cepstrum vector on Z axis
        }FftCepstrumTriaxial;


        typedef struct
        {
            std::vector<double> xArray;         //!< SRS feed data on X axis [m/s2]
            std::vector<double> yArray;         //!< SRS feed data on Y axis [m/s2]
            std::vector<double> zArray;         //!< SRS feed data on Z axis [m/s2]
        }SrsFeedTriaxial;

        typedef struct
        {
            double naturalFrequency;            //!< natural frequency [Hz]
            double value;                       //!< SRS value
        }Srs;

        typedef struct
        {
            std::vector<Srs> xSrsVec;           //!< SRS vector on X axis
            std::vector<Srs> ySrsVec;           //!< SRS vector on Y axis
            std::vector<Srs> zSrsVec;           //!< SRS vector on Z axis
        }SrsTriaxial;


        typedef struct
        {
            double xRmsSpeed;                   //!< RMS speed on X axis [m/s]
            double yRmsSpeed;                   //!< RMS speed on Y axis [m/s]
            double zRmsSpeed;                   //!< RMS speed on Z axis [m/s]
        }VibrationRmsSpeedsTriaxial;


    //************************************************************************
    // functions
    //************************************************************************
    public:
        VibrationHandler();

        ~VibrationHandler();

        static VibrationHandler* getInstance();

        double convertAccelG2Ms2
            (
            const double aAccelerationG   //!< acceleration [g]
            ) const;

        std::vector<FftBin> getFftBinsOnAxis
            (
            const Adxl355Adxl357Common::Axis aAxis      //!< axis
            ) const;

        std::vector<FftCepstrum> getFftCepstrumOnAxis
            (
            const Adxl355Adxl357Common::Axis aAxis      //!< axis
            ) const;

        std::vector<FftPsd> getFftPsdOnAxis
            (
            const Adxl355Adxl357Common::Axis aAxis      //!< axis
            ) const;

        std::vector<Srs> getSrsOnAxis
            (
            const Adxl355Adxl357Common::Axis aAxis      //!< axis
            ) const;

        AccelerometerDataTriaxial getTriaxialAccelArrays() const;

        VibrationRmsSpeedsTriaxial getTriaxialRmsSpeeds() const;

        VibrationMonitoringSettingsTriaxial& getTriaxialSettings();

        void setFftSize
            (
            const uint32_t aFFtSize     //!< FFT size
            );

        void setSpsExpected
            (
            const double aSps   //!< sampling rate [Hz]
            );

    public slots:
        void receiveNewData
            (
            double aXaccel,     //!< acceleration on X axis
            double aYaccel,     //!< acceleration on Y axis
            double aZaccel      //!< acceleration on Z axis
            );


        void receiveNewCepstrumX
            (
            double*  aDataArray, //!< array with computed cepstrum values
            int      aLength,    //!< array length
            int      aIndex      //!< index
            );

        void receiveNewCepstrumY
            (
            double*  aDataArray, //!< array with computed cepstrum values
            int      aLength,    //!< array length
            int      aIndex      //!< index
            );

        void receiveNewCepstrumZ
            (
            double*  aDataArray, //!< array with computed cepstrum values
            int      aLength,    //!< array length
            int      aIndex      //!< index
            );


        void receiveNewFftX
            (
            double*  aDataArray, //!< array with computed FFT values
            int      aLength,    //!< array length
            int      aIndex      //!< index
            );

        void receiveNewFftY
            (
            double*  aDataArray, //!< array with computed FFT values
            int      aLength,    //!< array length
            int      aIndex      //!< index
            );

        void receiveNewFftZ
            (
            double*  aDataArray, //!< array with computed FFT values
            int      aLength,    //!< array length
            int      aIndex      //!< index
            );


        void receiveNewSrsX
            (
            double*  aNaturalFrequenciesArray,  //!< natural frequencies
            double*  aValuesArray,              //!< computed SRS values
            int      aLength                    //!< length of both arrays
            );

        void receiveNewSrsY
            (
            double*  aNaturalFrequenciesArray,  //!< natural frequencies
            double*  aValuesArray,              //!< computed SRS values
            int      aLength                    //!< length of both arrays
            );

        void receiveNewSrsZ
            (
            double*  aNaturalFrequenciesArray,  //!< natural frequencies
            double*  aValuesArray,              //!< computed SRS values
            int      aLength                    //!< length of both arrays
            );


        void receiveNewSps
            (
            int aSps            //!< sampling rate [Hz]
            );

        void shiftLeft
            (
            std::vector<double>& aVector    //!< vector
            ) const;

    private:
        void calculateSpectrumX();

        void calculateSpectrumY();

        void calculateSpectrumZ();

    signals:
        void haveNewCepstrum
            (
            int aAxis           //!< axis
            );

        void haveNewData();

        void haveNewFftBins
            (
            int aAxis           //!< axis
            );

        void haveNewPeriodogram
            (
            int aAxis           //!< axis
            );

        void haveNewSrs
            (
            int aAxis           //!< axis
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        static VibrationHandler*            sInstance;          //!< singleton object
        QMutex                              mMutex;             //!< mutex

        VibrationMonitoringSettingsTriaxial mVibMonSettings;    //!< vibration monitoring settings

        uint32_t                            mFftSize;           //!< FFT size
        uint32_t                            mFftIndexCount;     //!< FFT index count

        double                              mSpsExpected;       //!< expected (theoretical) sampling rate [Hz]
        int                                 mSpsReceived;       //!< DAQ (realtime) sampling rate [Hz]

        AccelerometerDataTriaxial           mAccelerationData;  //!< data from accelerometer

        FftTriaxial                         mFFtValues;         //!< calculated FFT values

        FftThread                           mFftThreadX;        //!< compute thread for direct FFT on X axis
        FftThread                           mFftThreadY;        //!< compute thread for direct FFT on Y axis
        FftThread                           mFftThreadZ;        //!< compute thread for direct FFT on Z axis

        FftBinsTriaxial                     mFftBins;           //!< FFT bins
        FftPsdTriaxial                      mFftPsd;            //!< PSD
        FftCepstrumTriaxial                 mFftCepstrum;       //!< cepstrum

        FftThread                           mCepstrumInvFftThreadX;//!< compute thread for inverse FFT on X axis
        FftThread                           mCepstrumInvFftThreadY;//!< compute thread for inverse FFT on Y axis
        FftThread                           mCepstrumInvFftThreadZ;//!< compute thread for inverse FFT on Z axis

        SrsFeedTriaxial                     mSrsFeed;           //!< SRS feed data
        SrsTriaxial                         mSrs;               //!< SRS

        SrsThread                           mSrsThreadX;        //!< compute thread for SRS on X axis
        SrsThread                           mSrsThreadY;        //!< compute thread for SRS on Y axis
        SrsThread                           mSrsThreadZ;        //!< compute thread for SRS on Z axis

        VibrationRmsSpeedsTriaxial          mVibRmsSpeeds;      //!< vibration RMS speeds
};

#endif // VibrationHandler_h
