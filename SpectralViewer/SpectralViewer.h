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
SpectralViewer.h

This file contains the definitions for the mechanical vibrations spectral viewer.
*/

#ifndef SpectralViewer_h
#define SpectralViewer_h

#include "./ui_Accelerometer.h"
#include "./ui_FrequencyAnalysis.h"
#include "./ui_VibrationMonitoringSettings.h"
#include "./ui_PlotOptions.h"
#include "./ui_About.h"

#if BUILD_ADXL355
    #include "Adxl355.h"
#elif BUILD_ADXL357
    #include "Adxl357.h"
#endif

#include "Plot.h"
#include "VibrationHandler.h"

#if BUILD_CUDA
    #include <cuda.h>
    #include "helper_cuda_drvapi.h"
#endif

#include <QDialog>
#include <QMainWindow>
#include <QString>

class CbmCanvas;
class DaqThread;
class FrequencyAnalysis;

QT_BEGIN_NAMESPACE
    namespace Ui
    {
        class SpectralViewer;
    }
QT_END_NAMESPACE


//************************************************************************
// Class for handling the spectral viewer
//************************************************************************
class SpectralViewer : public QMainWindow
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        static const QString APP_NAME;

    private:
#if BUILD_CUDA
        typedef struct
        {
            uint8_t major;                  //!< major part
            uint8_t minor;                  //!< minor part
        }Version;

        const Version CUDA_VER_MIN = { 6, 5 };  //!< CUDA >= 6.5
        const Version CUDA_CC_MIN  = { 5, 3 };  //!< Compute Capability >= 5.3

        typedef struct
        {
            bool        noDevice;           //!< true if no device found
            CUdevice    devId;              //!< CUDA device ID
            QString     name;               //!< CUDA device name
            Version     version;            //!< CUDA toolkit version
            Version     cc;                 //!< CUDA Compute Capability
        }CudaInfo;
#endif

        typedef enum : uint8_t
        {
            I2C_BUS_MEMS_RASPBERRY_PI_4             = 1,
            I2C_BUS_MEMS_NVIDIA_JETSON_ORIN_NANO    = 7
        }I2c_Bus_Mems;

        // I2C bus number of the MEMS accelerometer
        static const uint8_t I2C_BUS_MEMS = I2C_BUS_MEMS_NVIDIA_JETSON_ORIN_NANO;

        const QString DEG        = QString::fromUtf8( "\u00B0" );   //!< degree sign
        const QString PLUS_MINUS = QString::fromUtf8( "\u00B1" );   //!< +/- sign       

        const QString ACCEL_NAME =
#if BUILD_ADXL355
            "ADXL355";
#elif BUILD_ADXL357
            "ADXL357";
#endif
        const QString ACCEL_SETTINGS_STR = ACCEL_NAME + " Settings";

        const double DBL_SPIN_EPS = 1.e-6;  //!< small factor for controlling spin values

        typedef std::map<Plot::PlotType, bool> ActivePlots;

        typedef struct
        {
            ActivePlots     xAxis;      //!< active plots on X axis
            ActivePlots     yAxis;      //!< active plots on Y axis
            ActivePlots     zAxis;      //!< active plots on Z axis
        }PlotOptions;

    //************************************************************************
    // functions
    //************************************************************************
    public:
        SpectralViewer
            (
            QWidget* aParent = nullptr //!< parent widget
            );

        ~SpectralViewer();

    private:
#if BUILD_CUDA
        void checkCudaRequirements();
#endif

        QString convertSeconds2String
            (
            double aNumber      //!< seconds amount
            ) const;

        uint16_t getDaqDelayUs();

        PlotOptions& getPlotOptions();

        void initAccelDialogControls();

        void initFreqAnalysisDialogControls();

        void initPlotOptionsControls();

        void initVibMonDialogControls();

        void readTemperature();

        void updateAccelDialogControls();

        void updatePlotsFrequencyParams();

        void updatePlotsVerticalMaxTransient();

    private slots:
        void handleAbout();

        void handleChangedAccelRange
            (
            int aIndex      //!< index
            );

        void handleChangedAccelOdr
            (
            int aIndex      //!< index
            );


        void handleChangedFreqAnalysisFftSize
            (
            int aIndex      //!< index
            );

        void handleChangedFreqAnalysisSrsModel
            (
            int aIndex      //!< index
            );

        void handleChangedFreqAnalysisSrsZeta
            (
            double aValue   //!< value
            );

        void handleChangedFreqAnalysisWinFuncParam
            (
            double aValue   //!< value
            );

        void handleChangedFreqAnalysisWinFuncType
            (
            int aIndex      //!< index
            );


        void handleChangedPlotOptionXtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionXfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionXperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionXsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionXcepstrum
            (
            bool aEnabled   //!< enabled status
            );


        void handleChangedPlotOptionYtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionYfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionYperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionYsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionYcepstrum
            (
            bool aEnabled   //!< enabled status
            );


        void handleChangedPlotOptionZtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionZfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionZperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionZsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlotOptionZcepstrum
            (
            bool aEnabled   //!< enabled status
            );


        void handleChangedVibMonXaxisItem
            (
            int aIndex      //!< index
            );

        void handleChangedVibMonXvAB
            (
            double aValue   //!< value
            );
        void handleChangedVibMonXvBC
            (
            double aValue   //!< value
            );
        void handleChangedVibMonXvCD
            (
            double aValue   //!< value
            );


        void handleChangedVibMonYaxisItem
            (
            int aIndex      //!< index
            );

        void handleChangedVibMonYvAB
            (
            double aValue   //!< value
            );
        void handleChangedVibMonYvBC
            (
            double aValue   //!< value
            );
        void handleChangedVibMonYvCD
            (
            double aValue   //!< value
            );


        void handleChangedVibMonZaxisItem
            (
            int aIndex      //!< index
            );

        void handleChangedVibMonZvAB
            (
            double aValue   //!< value
            );
        void handleChangedVibMonZvBC
            (
            double aValue   //!< value
            );
        void handleChangedVibMonZvCD
            (
            double aValue   //!< value
            );


        void handlePlotOptionsXDeselectAll();

        void handlePlotOptionsXSelectAll();

        void handlePlotOptionsYDeselectAll();

        void handlePlotOptionsYSelectAll();

        void handlePlotOptionsZDeselectAll();

        void handlePlotOptionsZSelectAll();


        void handleExit();

        void handleMenuAccel();

        void handleMenuFreqAnalysis();

        void handleMenuSelectPlots();

        void handleMenuVibMon();

        void handleNullifyValuesAccel();

        void handlePlotOptionsClosed();

        void handleResetAccel();

        void handleRestoreDefaultsVibMonSettings();

        void handleSaveVibMonSettings();

        void receiveClosedPlot
            (
            int aType,          //!< type of closed window
            int aAxis           //!< axis of closed window
            );

        void receiveNewSps
            (
            int aSps            //!< SPS
            );

        void receiveNewTemperature
            (
            double aTemperature //!< temperature [C]
            );

        void updateDateTime();


        void updatePlotOptionsButtonsX();

        void updatePlotOptionsButtonsY();

        void updatePlotOptionsButtonsZ();


        void updatePlotOptionsCheckBoxesX();

        void updatePlotOptionsCheckBoxesY();

        void updatePlotOptionsCheckBoxesZ();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        Ui::SpectralViewer*             mMainUi;                //!< main UI

        Ui::AccelerometerDialog*        mAccelerometerUi;       //!< accelerometer UI
        QDialog                         mAccelerometerDlg;      //!< accelerometer dialog

        Ui::FrequencyAnalysisDialog*    mFrequencyAnalysisUi;   //!< frequency analysis UI
        QDialog                         mFrequencyAnalysisDlg;  //!< frequency analysis dialog

        Ui::VibrationMonitoringSettingsDialog*  mVibrationMonitoringSettingsUi;     //!< vibration monitoring settings UI
        QDialog                                 mVibrationMonitoringSettingsDlg;    //!< vibration monitoring settings dialog

        Ui::PlotOptionsDialog*          mPlotOptionsUi;         //!< plot options UI
        QDialog                         mPlotOptionsDlg;        //!< plot options dialog

        Ui::AboutDialog*                mAboutUi;               //!< about dialog

        int                             mI2cBusChannel;         //!< I2C bus channel
        bool                            mI2cIsOpen;             //!< if I2C bus could be open

#if BUILD_ADXL355
        Adxl355*
#elif BUILD_ADXL357
        Adxl357*
#endif       
                                        mAccelInstance;         //!< accelerometer instance

        DaqThread*                      mDaqThread;             //!< DAQ thread
        int                             mReceivedSps;           //!< DAQ sampling rate

        FrequencyAnalysis*              mFreqAnalysisInstance;  //!< frequency analysis instance

        double                          mTemperature;           //!< temperature from the on-chip sensor [C]

        VibrationHandler*               mVibrationHndlInstance; //!< vibration handler instance
        VibrationHandler::VibrationMonitoringSettingsTriaxial mVibMonSettingsTemporary;   //!< temporary/unsaved vibration monitoring settings

#if BUILD_CUDA
        CudaInfo                        mCudaInfo;              //!< relevant info about CUDA platform in use
        bool                            mHaveCudaRequirements;  //!< true if detected CUDA provides expected support
#endif

        QLabel                          mSpsStatusbarLabel;     //!< SPS label on status bar
        QLabel                          mDateTimeStatusbarLabel;//!< date&time label on status bar

        CbmCanvas*                      mCbmCanvas;             //!< canvas to draw on

        PlotOptions                     mPlotOptions;           //!< plot options

        Plot*                           mPlotXtransient;        //!< plot window for X axis transient
        Plot*                           mPlotXfft;              //!< plot window for X axis FFT
        Plot*                           mPlotXperiodogram;      //!< plot window for X axis periodogram
        Plot*                           mPlotXsrs;              //!< plot window for X axis SRS
        Plot*                           mPlotXcepstrum;         //!< plot window for X axis cepstrum

        Plot*                           mPlotYtransient;        //!< plot window for Y axis transient
        Plot*                           mPlotYfft;              //!< plot window for Y axis FFT
        Plot*                           mPlotYperiodogram;      //!< plot window for Y axis periodogram
        Plot*                           mPlotYsrs;              //!< plot window for Y axis SRS
        Plot*                           mPlotYcepstrum;         //!< plot window for Y axis cepstrum

        Plot*                           mPlotZtransient;        //!< plot window for Z axis transient
        Plot*                           mPlotZfft;              //!< plot window for Z axis FFT
        Plot*                           mPlotZperiodogram;      //!< plot window for Z axis periodogram
        Plot*                           mPlotZsrs;              //!< plot window for Z axis SRS
        Plot*                           mPlotZcepstrum;         //!< plot window for Z axis cepstrum
};

#endif // SpectralViewer_h
