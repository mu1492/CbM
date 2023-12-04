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
#include "./ui_Plot2dOptions.h"
#include "./ui_Plot3dOptions.h"
#include "./ui_About.h"

#if BUILD_ADXL355
    #include "Adxl355.h"
#elif BUILD_ADXL357
    #include "Adxl357.h"
#endif

#include "Plot2d.h"
#include "Plot3d.h"
#include "VibrationHandler.h"

#if BUILD_CUDA
    #include <cuda.h>
    #include "helper_cuda_drvapi.h"
#endif

#include <fstream>
#include <string>

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

        const Version CUDA_VER_MIN = { 11, 0 };  //!< CUDA >= 11.0
        const Version CUDA_CC_MIN  = { 6, 2 };  //!< Compute Capability >= 6.2

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
            I2C_BUS_MEMS_NVIDIA_JETSON_XAVIER_NX    = 8,
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

        typedef std::map<Plot2d::Plot2dType, bool> Active2dPlots;

        typedef struct
        {
            Active2dPlots   xAxis;      //!< active 2D plots on X axis
            Active2dPlots   yAxis;      //!< active 2D plots on Y axis
            Active2dPlots   zAxis;      //!< active 2D plots on Z axis
        }Plot2dOptions;

        typedef std::map<Plot3d::Plot3dType, bool> Active3dPlots;

        typedef struct
        {
            Active3dPlots   xAxis;      //!< active 3D plots on X axis
            Active3dPlots   yAxis;      //!< active 3D plots on Y axis
            Active3dPlots   zAxis;      //!< active 3D plots on Z axis
        }Plot3dOptions;

        typedef struct
        {
            QString                                 accelerometerType;  //!< ADXL355 or ADXL357
            Adxl355Adxl357Common::AccelerationRange range;              //!< acceleration range
            Adxl355Adxl357Common::OdrSetting        odrSetting;         //!< ODR setting
            bool                                    nullifyActive;      //!< true if nullify is applied
            uint32_t                                samplesCount;       //!< number of samples
        }Config;


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
        bool changeAccelOdr
            (
            const Adxl355Adxl357Common::OdrSetting aOdrSetting      //!< ODR setting
            );

        bool changeAccelRange
            (
            const Adxl355Adxl357Common::AccelerationRange aRange    //!< acceleration range
            );

#if BUILD_CUDA
        void checkCudaRequirements();
#endif

        static bool compareIchar
            (
            char aFirstChar,    //!< 1st character
            char aSecondChar    //!< 2nd character
            );

        bool compareIstr
            (
            const std::string& aFirstString,    //!< 1st string
            const std::string& aSecondString    //!< 2nd string
            ) const;

        QString convertSeconds2String
            (
            double aNumber      //!< seconds amount
            ) const;

        std::string createCsvDataFilename
            (
            const std::string aInputFilename    //!< input filename
            ) const;

        uint16_t getDaqDelayUs();

        void initAccelDialogControls();

        void initFreqAnalysisDialogControls();

        void initPlot2dOptionsControls();

        void initPlot3dOptionsControls();

        void initVibMonDialogControls();

        bool nullifyValuesAccel();

        void readTemperature();

        bool resetAccel();

        void runConfiguration
            (
            const std::string aInputFilename    //!< input filename
            );

        void stopConfiguration();

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

        void handleChangedFreqAnalysisSrsInUse
            (
            bool aEnabled   //!< enabled status
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


        void handleChangedPlot2dOptionXtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionXfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionXperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionXsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionXcepstrum
            (
            bool aEnabled   //!< enabled status
            );


        void handleChangedPlot2dOptionYtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionYfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionYperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionYsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionYcepstrum
            (
            bool aEnabled   //!< enabled status
            );


        void handleChangedPlot2dOptionZtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionZfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionZperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionZsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot2dOptionZcepstrum
            (
            bool aEnabled   //!< enabled status
            );


        void handleChangedPlot3dOptionXtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionXfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionXperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionXsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionXcepstrum
            (
            bool aEnabled   //!< enabled status
            );


        void handleChangedPlot3dOptionYtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionYfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionYperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionYsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionYcepstrum
            (
            bool aEnabled   //!< enabled status
            );


        void handleChangedPlot3dOptionZtransient
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionZfft
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionZperiodogram
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionZsrs
            (
            bool aEnabled   //!< enabled status
            );

        void handleChangedPlot3dOptionZcepstrum
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


        void handleOpenConfiguration();


        void handlePlot2dOptionsXDeselectAll();

        void handlePlot2dOptionsXSelectAll();

        void handlePlot2dOptionsYDeselectAll();

        void handlePlot2dOptionsYSelectAll();

        void handlePlot2dOptionsZDeselectAll();

        void handlePlot2dOptionsZSelectAll();


        void handlePlot3dOptionsXDeselectAll();

        void handlePlot3dOptionsXSelectAll();

        void handlePlot3dOptionsYDeselectAll();

        void handlePlot3dOptionsYSelectAll();

        void handlePlot3dOptionsZDeselectAll();

        void handlePlot3dOptionsZSelectAll();


        void handleExit();

        void handleMenuAccel();

        void handleMenuFreqAnalysis();

        void handleMenuSelectPlots2D();

        void handleMenuSelectPlots3D();

        void handleMenuVibMon();

        void handleNullifyValuesAccel();

        void handlePlot2dOptionsClosed();

        void handlePlot3dOptionsClosed();

        void handleResetAccel();

        void handleRestoreDefaultsVibMonSettings();

        void handleSaveVibMonSettings();

        void receiveClosedPlot2d
            (
            int aType,          //!< type of closed window
            int aAxis           //!< axis of closed window
            );

        void receiveClosedPlot3d
            (
            int aType,          //!< type of closed window
            int aAxis           //!< axis of closed window
            );

        void receiveNewData
            (
            double aXaccel,     //!< acceleration on X axis
            double aYaccel,     //!< acceleration on Y axis
            double aZaccel      //!< acceleration on Z axis
            );

        void receiveNewSps
            (
            int aSps            //!< SPS
            );

        void receiveNewTemperature
            (
            double aTemperature //!< temperature [C]
            );

        void triggerConfigSetupRun();

        void updateDateTime();


        void updatePlot2dOptionsButtonsX();

        void updatePlot2dOptionsButtonsY();

        void updatePlot2dOptionsButtonsZ();


        void updatePlot2dOptionsCheckBoxesX();

        void updatePlot2dOptionsCheckBoxesY();

        void updatePlot2dOptionsCheckBoxesZ();


        void updatePlot3dOptionsButtonsX();

        void updatePlot3dOptionsButtonsY();

        void updatePlot3dOptionsButtonsZ();


        void updatePlot3dOptionsCheckBoxesX();

        void updatePlot3dOptionsCheckBoxesY();

        void updatePlot3dOptionsCheckBoxesZ();


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

        Ui::Plot2dOptionsDialog*        mPlot2dOptionsUi;       //!< 2D plot options UI
        QDialog                         mPlot2dOptionsDlg;      //!< 2D plot options dialog

        Ui::Plot3dOptionsDialog*        mPlot3dOptionsUi;       //!< 3D plot options UI
        QDialog                         mPlot3dOptionsDlg;      //!< 3D plot options dialog

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

        Plot2dOptions                   mPlot2dOptions;         //!< 2D plot options
        Plot3dOptions                   mPlot3dOptions;         //!< 3D plot options

        // 2D plots
        Plot2d*                         mPlot2dXtransient;      //!< 2D plot window for X axis transient
        Plot2d*                         mPlot2dXfft;            //!< 2D plot window for X axis FFT
        Plot2d*                         mPlot2dXperiodogram;    //!< 2D plot window for X axis periodogram
        Plot2d*                         mPlot2dXsrs;            //!< 2D plot window for X axis SRS
        Plot2d*                         mPlot2dXcepstrum;       //!< 2D plot window for X axis cepstrum

        Plot2d*                         mPlot2dYtransient;      //!< 2D plot window for Y axis transient
        Plot2d*                         mPlot2dYfft;            //!< 2D plot window for Y axis FFT
        Plot2d*                         mPlot2dYperiodogram;    //!< 2D plot window for Y axis periodogram
        Plot2d*                         mPlot2dYsrs;            //!< 2D plot window for Y axis SRS
        Plot2d*                         mPlot2dYcepstrum;       //!< 2D plot window for Y axis cepstrum

        Plot2d*                         mPlot2dZtransient;      //!< 2D plot window for Z axis transient
        Plot2d*                         mPlot2dZfft;            //!< 2D plot window for Z axis FFT
        Plot2d*                         mPlot2dZperiodogram;    //!< 2D plot window for Z axis periodogram
        Plot2d*                         mPlot2dZsrs;            //!< 2D plot window for Z axis SRS
        Plot2d*                         mPlot2dZcepstrum;       //!< 2D plot window for Z axis cepstrum

        // 3D plots
        Plot3d*                         mPlot3dXtransient;      //!< 3D plot window for X axis transient
        Plot3d*                         mPlot3dXfft;            //!< 3D plot window for X axis FFT
        Plot3d*                         mPlot3dXperiodogram;    //!< 3D plot window for X axis periodogram
        Plot3d*                         mPlot3dXsrs;            //!< 3D plot window for X axis SRS
        Plot3d*                         mPlot3dXcepstrum;       //!< 3D plot window for X axis cepstrum

        Plot3d*                         mPlot3dYtransient;      //!< 3D plot window for Y axis transient
        Plot3d*                         mPlot3dYfft;            //!< 3D plot window for Y axis FFT
        Plot3d*                         mPlot3dYperiodogram;    //!< 3D plot window for Y axis periodogram
        Plot3d*                         mPlot3dYsrs;            //!< 3D plot window for Y axis SRS
        Plot3d*                         mPlot3dYcepstrum;       //!< 3D plot window for Y axis cepstrum

        Plot3d*                         mPlot3dZtransient;      //!< 3D plot window for Z axis transient
        Plot3d*                         mPlot3dZfft;            //!< 3D plot window for Z axis FFT
        Plot3d*                         mPlot3dZperiodogram;    //!< 3D plot window for Z axis periodogram
        Plot3d*                         mPlot3dZsrs;            //!< 3D plot window for Z axis SRS
        Plot3d*                         mPlot3dZcepstrum;       //!< 3D plot window for Z axis cepstrum

        // config file
        Config                          mConfig;                //!< configuration settings
        bool                            mIsConfigSetupRunning;  //!< true if a config setup is still running

        // data dump
        std::ofstream                   mDumpCsvFile;           //!< csv output file
        uint32_t                        mDumpRemainingSamples;  //!< remaining samples to dump into output file
};

#endif // SpectralViewer_h
