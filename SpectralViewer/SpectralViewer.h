///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2023-2024 Mihai Ursu                                            //
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
    #include "PlotInferCanvas.h"
    #include "TrtCbmOnnx.h"
    #include "./ui_TrtSettings.h"
#endif

#include <fstream>
#include <string>

#include <QCloseEvent>
#include <QDialog>
#include <QMainWindow>
#include <QMutex>
#include <QString>
#include <QTimer>

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

        typedef struct
        {
            std::vector<float> zData;       //!< Inference feed data - Z axis
        }InferFeedData;

        typedef struct
        {
            double       zAxis;             //!< TensorRT inference calculated loss - Z axis
        }InferLossCalculated;

        typedef struct
        {
            std::vector<double> zAxis;      //!< TensorRt inference history - Z axis
        }InferLossHistory;

        typedef struct
        {
            float       zAxis;              //!< TensorRT inference loss threshold - Z axis
        }InferLossThreshold;

        static const uint8_t INFER_PERIOD = 1;              //!< [s]
        static const uint16_t INFER_HISTORY_LENGTH = 18000; //!< [s] <=> 5 hrs
        static constexpr double INFER_LOSS_THD = 8.5;       //!< default inference loss threshold

        typedef struct
        {
            TrtCbmOnnx                          cbmOnnx;            //!< ONNX-based TensorRT object for CbM
            QThread*                            engineBuildThread;  //!< thread for building the engine
            bool                                engineTried;        //!< true if already tried building the DL engine
            bool                                engineBuilt;        //!< true if the DL engine can be built
            FrequencyAnalysis::FftSizeOption    fftSize;            //!< FFT size for data feed
            int32_t                             feedBufferSize;     //!< buffer size for feed data
            InferFeedData                       feedData;           //!< triaxial feed data
            bool                                isEnabled;          //!< true if the engine is enabled
            InferLossCalculated                 lossCalc;           //!< inference calculated loss
            InferLossHistory                    lossHistoryData;    //!< inference history
            InferLossThreshold                  lossThd;            //!< inference loss thresholds
        }TensorRt;
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

        static const uint16_t NULLIFY_SAMPLES_DEFAULT = 200;     //!< default number of samples for nullify

        typedef enum : uint8_t
        {
            CONFIG_FIELD_TYPE,
            CONFIG_FIELD_RANGE,
            CONFIG_FIELD_ODR,
            CONFIG_FIELD_NULLIFY,
            CONFIG_FIELD_NULLIFY_SAMPLES,
            CONFIG_FIELD_BATCHES_COUNT,
            CONFIG_FIELD_SAMPLES_PER_BATCH,
            CONFIG_FIELD_BATCHES_PERIOD
        }ConfigField;

        static const std::map<ConfigField, std::string> CONFIG_FIELD_NAMES;

        typedef struct
        {
            QString                                 accelerometerType;  //!< ADXL355 or ADXL357
            Adxl355Adxl357Common::AccelerationRange range;              //!< acceleration range
            Adxl355Adxl357Common::OdrSetting        odrSetting;         //!< ODR setting
            bool                                    nullifyActive;      //!< true if nullify is applied
            uint16_t                                nullifySamples;     //!< number of samples for nullify
            uint32_t                                batchesCount;       //!< number of batches
            uint32_t                                samplesCount;       //!< number of samples/batch
            double                                  batchesPeriod;      //!< period of batches [s]
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

    protected:
        void closeEvent
            (
            QCloseEvent* aEvent         //!< close event
            );

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

        uint16_t checkInferEventsAboveThd();
#endif

        static bool compareIchar
            (
            char aFirstChar,    //!< 1st character
            char aSecondChar    //!< 2nd character
            );

#if BUILD_CUDA
        static bool compareInferFnc
            (
            double aFirstInfer, //!< first inference
            double aSecondInfer //!< second inference
            );
#endif

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
            const std::string aInputFilename,   //!< input filename
            const uint32_t    aCounter,         //!< counter
            const uint8_t     aNumericStrLen    //!< numeric string length
            ) const;

        uint16_t getDaqDelayUs();

        void initAccelDialogControls();

        void initFreqAnalysisDialogControls();

        void initPlot2dOptionsControls();

        void initPlot3dOptionsControls();

#if BUILD_CUDA
        void initTrtSettingsDialogControls();
#endif

        void initVibMonDialogControls();

        bool nullifyValuesAccel();

        void readTemperature();

        bool resetAccel();

        void runConfiguration();

        void stopBatch();

        void stopConfiguration();

        void updateAccelDialogControls();

        void updatePlotsFrequencyParams();

        void updatePlotsVerticalMaxTransient();

#if BUILD_CUDA
        void updateTrtSettingsDialogControls();
#endif

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

#if BUILD_CUDA
        void handleChangedTrtEventsWin
            (
            int aValue      //!< value
            );

        void handleChangedTrtInferZloss
            (
            double aValue   //!< value
            );

        void handleChangedTrtUpperRange
            (
            double aValue   //!< value
            );
#endif

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

#if BUILD_CUDA
        void handleMenuTrtSettings();
#endif

        void handleMenuVibMon();

        void handleNullifyValuesAccel();

        void handlePlot2dOptionsClosed();

        void handlePlot3dOptionsClosed();

        void handleResetAccel();

        void handleRestoreDefaultsVibMonSettings();

        void handleSaveVibMonSettings();

#if BUILD_CUDA
        void handleTrtBuildDone();
#endif

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

#if BUILD_CUDA
        void receiveNewFft
            (
            int aAxis           //!< axis
            );
#endif

        void receiveNewSps
            (
            int aSps            //!< SPS
            );

        void receiveNewTemperature
            (
            double aTemperature //!< temperature [C]
            );

        void runBatches();

        void startBatch();

#if BUILD_CUDA
        void startTrtInference();
#endif

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

        TensorRt                        mTrt;                   //!< Deep Learning TensorRT object
        QTimer*                         mTrtTimer;              //!< TensorRT timer
        QMutex                          mTrtMutex;              //!< TensorRT mutex

        QMenu*                          mTrtMenuInfer;          //!< Inference menu entry
        QAction*                        mTrtActionSettings;     //!< Inference settings submenu

        Ui::TrtSettingsDialog*          mTrtSettingsUi;         //!< TensorRT settings UI
        QDialog                         mTrtSettingsDlg;        //!< TensorRT settings dialog

        PlotInferCanvas*                mTrtInfer01Canvas;      //!< inference plot canvas for 1 second
        uint16_t                        mTrtInfer01Size;        //!< number of elements for 1s spacing

        PlotInferCanvas*                mTrtInfer10Canvas;      //!< inference plot canvas for avg. 10 seconds
        uint16_t                        mTrtInfer10Size;        //!< number of elements for 10s spacing

        PlotInferCanvas*                mTrtInfer60Canvas;      //!< inference plot canvas for avg. 60 second
        uint16_t                        mTrtInfer60Size;        //!< number of elements for 10s spacing

        double                          mTrtUpperRange;         //!< Inference upper range for vertical axis
        uint16_t                        mTrtWindowLength;       //!< Nr of most recent seconds for inference events
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
        std::string                     mConfigFilename;        //!< configuration filename
        Config                          mConfig;                //!< configuration settings
        bool                            mIsConfigSetupRunning;  //!< true if a config setup is still running
        QTimer*                         mConfigBatchesTimer;    //!< timer for running the batches

        // data dump
        std::ofstream                   mDumpCsvFile;           //!< csv output file
        uint32_t                        mDumpRemainingBatches;  //!< remaining batches to dump
        uint32_t                        mDumpRemainingSamples;  //!< remaining samples/batch to dump into output file
};

#endif // SpectralViewer_h
