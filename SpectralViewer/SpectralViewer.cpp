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
SpectralViewer.cpp

This file contains the sources for the mechanical vibrations spectral viewer.
*/

#include "SpectralViewer.h"
#include "./ui_SpectralViewer.h"

#include "CbmCanvas.h"
#include "DaqThread.h"
#include "FrequencyAnalysis.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <list>
#include <map>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTime>
#include <QVBoxLayout>


const QString SpectralViewer::APP_NAME = "Spectral Viewer";

const std::map<SpectralViewer::ConfigField, std::string> SpectralViewer::CONFIG_FIELD_NAMES =
{
    { SpectralViewer::CONFIG_FIELD_TYPE,              "type"              },
    { SpectralViewer::CONFIG_FIELD_RANGE,             "range"             },
    { SpectralViewer::CONFIG_FIELD_ODR,               "odr"               },
    { SpectralViewer::CONFIG_FIELD_NULLIFY,           "nullify"           },
    { SpectralViewer::CONFIG_FIELD_NULLIFY_SAMPLES,   "nullify_samples"   },
    { SpectralViewer::CONFIG_FIELD_BATCHES_COUNT,     "batches_count"     },
    { SpectralViewer::CONFIG_FIELD_SAMPLES_PER_BATCH, "samples_per_batch" },
    { SpectralViewer::CONFIG_FIELD_BATCHES_PERIOD,    "batches_period"    }
};

//!************************************************************************
//! Constructor
//!************************************************************************
SpectralViewer::SpectralViewer
    (
    QWidget* aParent    //!< parent widget
    )
    : QMainWindow( aParent )
    // UIs
    , mMainUi( new Ui::SpectralViewer )
    , mAccelerometerUi( new Ui::AccelerometerDialog )
    , mFrequencyAnalysisUi( new Ui::FrequencyAnalysisDialog )
    , mVibrationMonitoringSettingsUi( new Ui::VibrationMonitoringSettingsDialog )
    , mPlot2dOptionsUi( new Ui::Plot2dOptionsDialog )
    , mPlot3dOptionsUi( new Ui::Plot3dOptionsDialog )
#if BUILD_CUDA
    , mTrtSettingsUi( new Ui::TrtSettingsDialog )
#endif
    , mAboutUi( new Ui::AboutDialog )
    // I2C resources
    , mI2cBusChannel( 0 )
    , mI2cIsOpen( false )
    // DAQ
    , mDaqThread( nullptr )
    , mReceivedSps( 0 )
    // on-chip temperature
    , mTemperature( 0 )
    // CUDA
#if BUILD_CUDA
    , mHaveCudaRequirements( true )
    , mTrtTimer( new QTimer( this ) )
    , mTrtInfer01Canvas( nullptr )
    , mTrtInfer01Size( 600 ) // 10m
    , mTrtInfer10Canvas( nullptr )
    , mTrtInfer10Size( 360 ) // 1h
    , mTrtInfer60Canvas( nullptr )
    , mTrtInfer60Size( 300 ) // 5h
#endif
    // paint&draw
    , mCbmCanvas( nullptr )
    // 2D - X axis plots
    , mPlot2dXtransient( nullptr )
    , mPlot2dXfft( nullptr )
    , mPlot2dXperiodogram( nullptr )
    , mPlot2dXsrs( nullptr )
    , mPlot2dXcepstrum( nullptr )
    // 2D - Y axis plots
    , mPlot2dYtransient( nullptr )
    , mPlot2dYfft( nullptr )
    , mPlot2dYperiodogram( nullptr )
    , mPlot2dYsrs( nullptr )
    , mPlot2dYcepstrum( nullptr )
    // 2D - Z axis plots
    , mPlot2dZtransient( nullptr )
    , mPlot2dZfft( nullptr )
    , mPlot2dZperiodogram( nullptr )
    , mPlot2dZsrs( nullptr )
    , mPlot2dZcepstrum( nullptr )
    // 3D - X axis plots
    , mPlot3dXtransient( nullptr )
    , mPlot3dXfft( nullptr )
    , mPlot3dXperiodogram( nullptr )
    , mPlot3dXsrs( nullptr )
    , mPlot3dXcepstrum( nullptr )
    // 3D - Y axis plots
    , mPlot3dYtransient( nullptr )
    , mPlot3dYfft( nullptr )
    , mPlot3dYperiodogram( nullptr )
    , mPlot3dYsrs( nullptr )
    , mPlot3dYcepstrum( nullptr )
    // 3D - Z axis plots
    , mPlot3dZtransient( nullptr )
    , mPlot3dZfft( nullptr )
    , mPlot3dZperiodogram( nullptr )
    , mPlot3dZsrs( nullptr )
    , mPlot3dZcepstrum( nullptr )
    // config
    , mIsConfigSetupRunning( false )
    , mConfigBatchesTimer( new QTimer( this ) )
    // dump
    , mDumpRemainingBatches( 0 )
    , mDumpRemainingSamples( 0 )
{
    mMainUi->setupUi( this );

    //****************************************
    // Dialog windows setup
    //****************************************
    mAccelerometerUi->setupUi( &mAccelerometerDlg );
    mFrequencyAnalysisUi->setupUi( &mFrequencyAnalysisDlg );
    mVibrationMonitoringSettingsUi->setupUi( &mVibrationMonitoringSettingsDlg );    
    mPlot2dOptionsUi->setupUi( &mPlot2dOptionsDlg );
    mPlot3dOptionsUi->setupUi( &mPlot3dOptionsDlg );
#if BUILD_CUDA
    mTrtSettingsUi->setupUi( &mTrtSettingsDlg );
#endif

    //****************************************
    // NVIDIA CUDA
    //****************************************
#if BUILD_CUDA
    checkCudaRequirements();
#endif

    //****************************************
    // Accelerometer setup
    //****************************************
    std::string i2cPath = "/dev/i2c-" + std::to_string( I2C_BUS_MEMS );
    mI2cBusChannel = ::open( i2cPath.c_str(), O_RDWR );

    if( mI2cBusChannel >= 0 )
    {
        mI2cIsOpen = true;
    }
    else
    {
        QMessageBox::critical( this, APP_NAME,
                               "Could not open I2C bus " + QString::fromStdString( i2cPath ),
                               QMessageBox::Ok );
        delete mMainUi;
        exit( 0 );
    }

    mAccelInstance =
#if BUILD_ADXL355
        Adxl355::getInstance();
#elif BUILD_ADXL357
        Adxl357::getInstance();
#endif

    if( !mAccelInstance )
    {
        QMessageBox::critical( this, APP_NAME,
                               "Could not create an instance of " + ACCEL_NAME,
                               QMessageBox::Ok );
        if( mI2cIsOpen )
        {
            ::close( mI2cBusChannel );
        }

        delete mMainUi;
        exit( 0 );
    }
    else
    {
        bool initStatus = mAccelInstance->init( mI2cBusChannel,
#if BUILD_ADXL355
                                                Adxl355::ADXL355_357_I2C_ADDRESS_PRIMARY
#elif BUILD_ADXL357
                                                Adxl357::ADXL355_357_I2C_ADDRESS_PRIMARY
#endif
                                                );

        if( !initStatus )
        {
            QMessageBox::critical( this, APP_NAME,
                                   "Could not initialize " + ACCEL_NAME,
                                   QMessageBox::Ok );
            if( mI2cIsOpen )
            {
                ::close( mI2cBusChannel );
            }

            delete mMainUi;
            exit( 0 );
        }
        else
        {
            mAccelInstance->initData();

            if( !mAccelInstance->reset() )
            {
                QMessageBox::critical( this, APP_NAME,
                                       "Could not reset " + ACCEL_NAME,
                                       QMessageBox::Ok );
            }

            //****************************************
            // one-shot temperature read - GUI thread
            //****************************************
            readTemperature();

            //****************************************
            // select ODR
            //****************************************           
#if BUILD_I2C_HIGH_SPEED
            mAccelInstance->setOdr( Adxl355Adxl357Common::ODR_SETTING_4000 );
#elif BUILD_I2C_FAST_PLUS
            mAccelInstance->setOdr( Adxl355Adxl357Common::ODR_SETTING_2000 );
#elif BUILD_I2C_FAST
            mAccelInstance->setOdr( Adxl355Adxl357Common::ODR_SETTING_500 );
#else
            mAccelInstance->setOdr( Adxl355Adxl357Common::ODR_SETTING_125 );
#endif

            //****************************************
            // accelerations DAQ
            //****************************************
            // enter measurement mode
            mAccelInstance->enableStandbyMode( false );

            mDaqThread = new DaqThread( this, mAccelInstance );
            connect( mDaqThread, SIGNAL( haveNewSps(int) ), this, SLOT( receiveNewSps(int) ) );
            connect( mDaqThread, SIGNAL( haveNewTemperature(double) ), this, SLOT( receiveNewTemperature(double) ) );            
            connect( mDaqThread, SIGNAL( haveNewData(double, double, double) ), this, SLOT( receiveNewData(double, double, double) ) );
            mDaqThread->updateDelay( getDaqDelayUs() );
            mDaqThread->start();
        }
    }

    mFreqAnalysisInstance = FrequencyAnalysis::getInstance();
    mFreqAnalysisInstance->setFftSps( mAccelInstance->getOdrFrequency() );

    mVibrationHndlInstance = VibrationHandler::getInstance();
    connect( mDaqThread, SIGNAL( haveNewSps(int) ), this->mVibrationHndlInstance, SLOT( receiveNewSps(int) ) );
    connect( mDaqThread, SIGNAL( haveNewData(double, double, double) ), this->mVibrationHndlInstance, SLOT( receiveNewData(double, double, double) ) );

    //****************************************
    // menus
    //****************************************
    connect( mMainUi->actionOpenConfiguration, &QAction::triggered, this, &SpectralViewer::handleOpenConfiguration );
    connect( mMainUi->actionExit, &QAction::triggered, this, &SpectralViewer::handleExit );

    mMainUi->actionAccelerometer->setText( ACCEL_SETTINGS_STR );

    connect( mMainUi->actionAccelerometer, &QAction::triggered, this, &SpectralViewer::handleMenuAccel );
    initAccelDialogControls();

    connect( mMainUi->actionFrequencyAnalysis, &QAction::triggered, this, &SpectralViewer::handleMenuFreqAnalysis );
    initFreqAnalysisDialogControls();

    connect( mMainUi->actionVibrationMonitoring, &QAction::triggered, this, &SpectralViewer::handleMenuVibMon );
    initVibMonDialogControls();

    connect( mMainUi->actionSelectPlots2D, &QAction::triggered, this, &SpectralViewer::handleMenuSelectPlots2D );
    connect( mMainUi->actionSelectPlots3D, &QAction::triggered, this, &SpectralViewer::handleMenuSelectPlots3D );
    initPlot2dOptionsControls();
    initPlot3dOptionsControls();

    connect( mMainUi->actionAbout, &QAction::triggered, this, &SpectralViewer::handleAbout );


    //****************************************
    // timers
    //****************************************
    QTimer* dateTimeTimer = new QTimer();
    connect( dateTimeTimer, SIGNAL( timeout() ), this, SLOT( updateDateTime() ) );
    dateTimeTimer->start( 500 );

    //****************************************
    // status bar
    //****************************************
    mSpsStatusbarLabel.setStyleSheet( "font-weight: bold; color: red;" );
    mMainUi->statusbar->addPermanentWidget( &mSpsStatusbarLabel, 1 );
    mSpsStatusbarLabel.setText( ACCEL_NAME + ": " + QString::number( mAccelInstance->getOdrFrequency() ) + " SPS" );

    mMainUi->statusbar->addPermanentWidget( &mDateTimeStatusbarLabel, 0 );
    updateDateTime();

    //****************************************
    // paint&draw
    //****************************************
    mCbmCanvas = new CbmCanvas;
    setCentralWidget( mCbmCanvas );
    this->adjustSize();
    mCbmCanvas->update();    

    //****************************************
    // config
    //****************************************
    mConfig.accelerometerType = "";
    mConfig.range = Adxl355Adxl357Common::ACCELERATION_RANGE_DEFAULT;
    mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_3_90625;
    mConfig.nullifyActive = false;
    mConfig.nullifySamples = NULLIFY_SAMPLES_DEFAULT;
    mConfig.samplesCount = 0;

#if BUILD_CUDA
    //****************************************
    // Deep Learning
    //****************************************
    if( mHaveCudaRequirements )
    {
        mTrt.engineBuildThread = new QThread();

        mTrt.engineTried = false;
        mTrt.engineBuilt = false;

        mTrt.fftSize = FrequencyAnalysis::FFT_SIZE_256;
        mTrt.feedBufferSize = TrtCbmOnnx::INPUT_BUFFER_SIZE;
        mTrt.isEnabled = false;

        memset( &mTrt.lossCalc, 0, sizeof( mTrt.lossCalc ) );

        mTrt.lossHistoryData.zAxis.resize( INFER_HISTORY_LENGTH, 0 );

        mTrt.lossThd.zAxis = INFER_LOSS_THD;

        mTrt.cbmOnnx.moveToThread( mTrt.engineBuildThread );
        connect( mTrt.engineBuildThread, SIGNAL( started() ), &mTrt.cbmOnnx, SLOT( startBuild() ) );
        connect( &mTrt.cbmOnnx, SIGNAL( buildFinished() ), this, SLOT( handleTrtBuildDone() ) );
        mTrt.engineBuildThread->start();

        mTrtMenuInfer = new QMenu( "&Inference" );
        mMainUi->menubar->insertMenu( mMainUi->menuHelp->menuAction(), mTrtMenuInfer );

        mTrtActionSettings = new QAction( "&Settings" );
        mTrtMenuInfer->addAction( mTrtActionSettings );

        connect( mTrtActionSettings, &QAction::triggered, this, &SpectralViewer::handleMenuTrtSettings );
        initTrtSettingsDialogControls();

        // disable Inference menu entry
        mTrtMenuInfer->setEnabled( false );
        // disable Exit submenu while DL TensorRT engine is being built
        mMainUi->actionExit->setEnabled( false );
        mMainUi->actionExit->setText( "*** building TensorRT engine ***" );
    }
#endif
}


//!************************************************************************
//! Destructor
//!************************************************************************
SpectralViewer::~SpectralViewer()
{
    if( mIsConfigSetupRunning )
    {
        stopConfiguration();
    }

    if( mDaqThread )
    {
        mDaqThread->quit();

        if( !mDaqThread->wait( 500 ) )
        {
            mDaqThread->terminate();
            mDaqThread->wait();
        }

        delete mDaqThread;
    }

#if BUILD_CUDA
    mTrtMutex.unlock();
    cudaDeviceReset();
#endif

    delete mMainUi;
}


//!************************************************************************
//! Change the acceleration ODR
//!
//! @returns true at success
//!************************************************************************
bool SpectralViewer::changeAccelOdr
    (
    const Adxl355Adxl357Common::OdrSetting aOdrSetting      //!< ODR setting
    )
{
    bool statusOk =
#if BUILD_I2C_HIGH_SPEED
        aOdrSetting >= Adxl355Adxl357Common::ODR_SETTING_4000
#elif BUILD_I2C_FAST_PLUS
        aOdrSetting >= Adxl355Adxl357Common::ODR_SETTING_2000
#elif BUILD_I2C_FAST
        aOdrSetting >= Adxl355Adxl357Common::ODR_SETTING_500
#else
        aOdrSetting >= Adxl355Adxl357Common::ODR_SETTING_125
#endif
        ;

    if( statusOk && mAccelInstance )
    {
        uint16_t delayUs = getDaqDelayUs();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->pause();
            mDaqThread->usleep( delayUs );
        }

        statusOk = mAccelInstance->setOdr( aOdrSetting );
        mFreqAnalysisInstance->setFftSps( mAccelInstance->getOdrFrequency() );

        updatePlotsFrequencyParams();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            delayUs = getDaqDelayUs();
            mDaqThread->updateDelay( delayUs );
            mDaqThread->resume();
        }

        if( mAccelerometerDlg.isVisible() )
        {
            mAccelerometerUi->LpfFrequencyValue->setText( QString::number( mAccelInstance->getOdrLpfCorner() ) );
        }
    }

    return statusOk;
}


//!************************************************************************
//! Change the acceleration range
//!
//! @returns true at success
//!************************************************************************
bool SpectralViewer::changeAccelRange
    (
    const Adxl355Adxl357Common::AccelerationRange aRange    //!< acceleration range
    )
{
    bool statusOk = false;

    if( mAccelInstance )
    {
        uint16_t delayUs = getDaqDelayUs();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->pause();
            mDaqThread->usleep( delayUs );
        }

        statusOk = mAccelInstance->setAccelerationRange( aRange );
        updatePlotsVerticalMaxTransient();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->resume();
        }
    }

    return statusOk;
}


#if BUILD_CUDA
    //!************************************************************************
    //! Check if the detected CUDA platform has minimum expected requirements
    //!
    //! @returns nothing
    //!************************************************************************
    void SpectralViewer::checkCudaRequirements()
    {
        mHaveCudaRequirements = true;

        mCudaInfo.noDevice = true;
        mCudaInfo.devId = 0;
        mCudaInfo.name.clear();
        memset( &mCudaInfo.version, 0, sizeof( mCudaInfo.version ) );
        memset( &mCudaInfo.cc, 0, sizeof( mCudaInfo.cc ) );

        checkCudaErrors( cuInit( 0 ) );

        int driverVersion = 0;
        checkCudaErrors( cuDriverGetVersion( &driverVersion ) );
        mCudaInfo.version.major = driverVersion / 1000;
        mCudaInfo.version.minor = ( driverVersion % 100 ) / 10;

        bool versionOk = ( CUDA_VER_MIN.major < mCudaInfo.version.major )
                      || ( CUDA_VER_MIN.major == mCudaInfo.version.major && CUDA_VER_MIN.minor <= mCudaInfo.version.minor );

        int deviceCount = 0;
        checkCudaErrors( cuDeviceGetCount( &deviceCount ) );
        mCudaInfo.noDevice = !deviceCount;
        bool ccOk = false;

        for( CUdevice dev = 0; dev < deviceCount; dev++ )
        {
            int ccMajor = 0;
            int ccMinor = 0;
            checkCudaErrors( cuDeviceGetAttribute( &ccMajor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, dev ) );
            checkCudaErrors( cuDeviceGetAttribute( &ccMinor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, dev ) );

            const int MAX_NAME_LEN = 256;
            char deviceName[MAX_NAME_LEN] = "";
            checkCudaErrors( cuDeviceGetName( deviceName, MAX_NAME_LEN, dev ) );

            mCudaInfo.devId = dev;
            mCudaInfo.cc.major = ccMajor;
            mCudaInfo.cc.minor = ccMinor;
            mCudaInfo.name = deviceName;

            if( ( CUDA_CC_MIN.major < ccMajor )
             || ( CUDA_CC_MIN.major == ccMajor && CUDA_CC_MIN.minor <= ccMinor ) )
            {
                ccOk = true;
                break;
            }
        }

        if( !deviceCount || !versionOk || !ccOk )
        {
            mHaveCudaRequirements = false;
        }

        if( !mHaveCudaRequirements )
        {
            QString msg;

            if( mCudaInfo.noDevice )
            {
                msg = "No CUDA devices were found on this system.";
                msg += "\n\nComputations will run in the CPU only.";
            }
            else
            {
                QString cudaVerRequiredStr = QString::number( CUDA_VER_MIN.major ) + "." + QString::number( CUDA_VER_MIN.minor );
                QString cudaVerFoundStr = QString::number( mCudaInfo.version.major) + "." + QString::number( mCudaInfo.version.minor );

                QString cudaCcRequiredStr = QString::number( CUDA_CC_MIN.major ) + "." + QString::number( CUDA_CC_MIN.minor );
                QString cudaCcFoundStr = QString::number( mCudaInfo.cc.major) + "." + QString::number( mCudaInfo.cc.minor );

                msg = "Minimum CUDA requirements were not found on this platform (" + mCudaInfo.name + "):";
                msg += "\n - minimum CUDA required is " + cudaVerRequiredStr + ", found is " + cudaVerFoundStr;
                msg += "\n - minimum CC required is " + cudaCcRequiredStr + ", found is " + cudaCcFoundStr;
                msg += "\n\nCUDA will not be used and computations will run in the CPU only.";
            }

            QMessageBox::warning( this, "SpectralViewer - CUDA", msg, QMessageBox::Ok );
        }
    }


    //!************************************************************************
    //! Check for the number of inference losses above the threshold.
    //! Focus is on the array of 1 second inferences.
    //!
    //! @returns The number of events above threshold
    //!************************************************************************
    uint16_t SpectralViewer::checkInferEventsAboveThd()
    {
        uint16_t eventsCount = 0;

        for( size_t i = 0; i < mTrtInfer01Canvas->getDataVector().size(); i++ )
        {
            if( mTrtInfer01Canvas->getDataVector().at( i ) >= mTrt.lossThd.zAxis )
            {
                eventsCount++;
            }
        }

        return eventsCount;
    }
#endif


//!************************************************************************
//! Close event handler
//!
//! @returns nothing
//!************************************************************************
void SpectralViewer::closeEvent
    (
    QCloseEvent*    aEvent      //!< close event
    )
{
#if BUILD_CUDA
    if( mHaveCudaRequirements && !mTrt.engineTried )
    {
        aEvent->ignore();
        QMessageBox::information( this, APP_NAME,
                                  "Building the Deep Learning TensorRT engine.\nCannot exit now.",
                                  QMessageBox::Ok );
    }
    else
    {
        if( mTrtSettingsDlg.isVisible() )
        {
            mTrtSettingsDlg.hide();
        }

        aEvent->accept();
    }
#else
    aEvent->accept();
#endif
}


//!************************************************************************
//! Insensitive character comparison
//!
//! @returns true if the characters are insensitive equal
//!************************************************************************
bool SpectralViewer::compareIchar
    (
    char aFirstChar,    //!< 1st character
    char aSecondChar    //!< 2nd character
    )
{
    return std::tolower( static_cast<unsigned char>( aFirstChar ) ) ==
           std::tolower( static_cast<unsigned char>( aSecondChar ) );
}


#if BUILD_CUDA
    //!************************************************************************
    //! Comparison function for inference values
    //!
    //! @returns true if first number is smaller than second
    //!************************************************************************
    bool SpectralViewer::compareInferFnc
        (
        double aFirstInfer, //!< first inference
        double aSecondInfer //!< second inference
        )
    {
        return aFirstInfer < aSecondInfer;
    }
#endif


//!************************************************************************
//! Insensitive string comparison
//!
//! @returns true if the strings are insensitive equal
//!************************************************************************
bool SpectralViewer::compareIstr
    (
    const std::string& aFirstString,    //!< 1st string
    const std::string& aSecondString    //!< 2nd string
    ) const
{
    return std::equal( aFirstString.begin(), aFirstString.end(),
                       aSecondString.begin(), aSecondString.end(),
                       compareIchar );
}


//!************************************************************************
//! Convert a number of seconds to a formatted string
//!
//! @returns The formatted string
//!************************************************************************
QString SpectralViewer::convertSeconds2String
    (
    double aNumber    //!< seconds amount
    ) const
{
    QString str;

    const uint32_t SEC_PER_DAY = 86400;
    uint8_t d = aNumber / SEC_PER_DAY;
    if( d )
    {
        aNumber -= ( d * SEC_PER_DAY );
        str += QString::number( d );
        str += "d";
    }

    const uint16_t SEC_PER_HR = 3600;
    uint8_t h = aNumber / SEC_PER_HR;
    if( h )
    {
        aNumber -= ( h * SEC_PER_HR );
        str += QString::number( h );
        str += "h";
    }

    const uint8_t SEC_PER_MIN = 60;
    uint8_t m = aNumber / SEC_PER_MIN;
    if( m )
    {
        aNumber -= ( m * SEC_PER_MIN );
        str += QString::number( m );
        str += "m";
    }

    str += QString::number( aNumber, 'f', 3 );
    str += "s";

    return str;
}


//!************************************************************************
//! Create a filename for CSV data, using an input filename and a counter
//!
//! @returns The string for the output file
//!************************************************************************
std::string SpectralViewer::createCsvDataFilename
    (
    const std::string aInputFilename,   //!< input filename
    const uint32_t    aCounter,         //!< counter
    const uint8_t     aNumericStrLen    //!< numeric string length
    ) const
{
    std::filesystem::path fullPath( aInputFilename );
    std::string csvFilename = fullPath.stem();
    std::string counterStr = std::to_string( aCounter );

    if( counterStr.size() < aNumericStrLen )
    {
        counterStr.insert( 0, aNumericStrLen - counterStr.size(), '0' );
    }

    csvFilename += "_";
    csvFilename += counterStr;
    QDateTime crtDateTime = QDateTime::currentDateTime();
    csvFilename += crtDateTime.toString( ".yyyyMMdd_hhmmss" ).toStdString();
    csvFilename += ".csv";

    return csvFilename;
}


//!************************************************************************
//! Get the calculated DAQ delay
//!
//! @returns The delay [us]
//!************************************************************************
uint16_t SpectralViewer::getDaqDelayUs()
{
    uint16_t delayUs = 50;

    if( mAccelInstance )
    {
        double expectedOdrHz = mAccelInstance->getOdrFrequency();

        if( expectedOdrHz >= 2000 )
        {
            delayUs = ( 1.e6 / expectedOdrHz ) / 10.0;
        }
        else
        {
            delayUs = ( 1.e6 / expectedOdrHz ) / 4.0;
        }
    }

    return delayUs;
}


//!************************************************************************
//! About dialog box
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleAbout()
{
    QDialog dialog;
    mAboutUi->setupUi( &dialog );

    QString builtString = "Built for ";

#if __ARM_ARCH_ISA_A64
    builtString += "ARM64";
#elif __ARM_ARCH
    builtString += "ARM32";
#else
    builtString += "non-ARM";
#endif

#if BUILD_CUDA
    builtString += " with CUDA enabled";
#endif

    mAboutUi->BuiltLabel->setText( builtString );

    connect( mAboutUi->OkButton, SIGNAL( clicked() ), &dialog, SLOT( close() ) );
    dialog.exec();
}


//!************************************************************************
//! Handle for changing the acceleration range
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedAccelRange
    (
    int aIndex      //!< index
    )
{
    Adxl355Adxl357Common::AccelerationRange range = static_cast<Adxl355Adxl357Common::AccelerationRange>( aIndex +
#if BUILD_ADXL355
        Adxl355Adxl357Common::ACCELERATION_RANGE_2G
#elif BUILD_ADXL357
        Adxl355Adxl357Common::ACCELERATION_RANGE_10G
#endif
        );

    changeAccelRange( range );
}


//!************************************************************************
//! Handle for changing the acceleration ODR
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedAccelOdr
    (
    int aIndex      //!< index
    )
{
    Adxl355Adxl357Common::OdrSetting odrSetting = static_cast<Adxl355Adxl357Common::OdrSetting>( aIndex );
    changeAccelOdr( odrSetting );
}


//!************************************************************************
//! Handle for changing the FFT size in frequency analysis settings
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedFreqAnalysisFftSize
    (
    int aIndex      //!< index
    )
{
    if( aIndex < FrequencyAnalysis::FFT_SIZE_MAX )
    {
        mFreqAnalysisInstance->setFftSizeOptionIndex( static_cast<FrequencyAnalysis::FftSizeOption>( aIndex ) );
        updatePlotsFrequencyParams();

        mFrequencyAnalysisUi->BinWidthValue->setText( QString::number( mFreqAnalysisInstance->getFftBinWidth() ) + " Hz/bin" );
        mFrequencyAnalysisUi->TimeGateValue->setText( convertSeconds2String( mFreqAnalysisInstance->getFftTimeGate() ) );

#if BUILD_CUDA
        if( mHaveCudaRequirements )
        {
            mTrt.isEnabled = ( mTrt.fftSize == aIndex );

            if( mTrt.isEnabled )
            {
                mTrtTimer->start( 1000 * INFER_PERIOD );
            }
            else
            {
                mTrtTimer->stop();
            }

            if( mTrtSettingsDlg.isVisible() )
            {
                mTrtSettingsUi->StatusValue->setText( mTrt.isEnabled ? "ENABLED" : "DISABLED" );
            }
        }
#endif
    }
}


//!************************************************************************
//! Handle for changing the SRS usage status
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedFreqAnalysisSrsInUse
    (
    bool aEnabled   //!< enabled status
    )
{
    mFreqAnalysisInstance->getSrsIsRunning() = aEnabled;

    mFrequencyAnalysisUi->SrsMethodComboBox->setEnabled( aEnabled );
    mFrequencyAnalysisUi->SrsZetaSpinBox->setEnabled( aEnabled );

    if( !aEnabled )
    {
        // 2D
        if( mPlot2dXsrs )
        {
            mPlot2dXsrs->close();
        }

        mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = false;

        if( mPlot2dYsrs )
        {
            mPlot2dYsrs->close();
        }

        mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = false;

        if( mPlot2dZsrs )
        {
            mPlot2dZsrs->close();
        }

        mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = false;

        // 3D
        if( mPlot3dXsrs )
        {
            mPlot3dXsrs->close();
        }

        mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = false;

        if( mPlot3dYsrs )
        {
            mPlot3dYsrs->close();
        }

        mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = false;

        if( mPlot3dZsrs )
        {
            mPlot3dZsrs->close();
        }

        mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = false;
    }

    // 2D
    updatePlot2dOptionsButtonsX();
    updatePlot2dOptionsButtonsY();
    updatePlot2dOptionsButtonsZ();

    // 3D
    updatePlot3dOptionsButtonsX();
    updatePlot3dOptionsButtonsY();
    updatePlot3dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing the SRS model in frequency analysis settings
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedFreqAnalysisSrsModel
    (
    int aIndex      //!< index
    )
{
    mFreqAnalysisInstance->getSrsParameters().model = static_cast<FrequencyAnalysis::SrSModel>( aIndex );
}


//!************************************************************************
//! Handle for changing the SRS zeta parameter in frequency analysis settings
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedFreqAnalysisSrsZeta
    (
    double aValue      //!< value
    )
{
    if( aValue >= 0 && aValue < 1 )
    {
        mFreqAnalysisInstance->getSrsParameters().zeta.crtValue = aValue;
    }
}


//!************************************************************************
//! Handle for changing the FFT window function parameter in frequency
//! analysis settings
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedFreqAnalysisWinFuncParam
    (
    double aValue      //!< value
    )
{
    WindowFunction::WindowFunctionType winFuncType = mFreqAnalysisInstance->getWindowFunction().getActiveType();
    std::map<WindowFunction::WindowFunctionType, WindowFunction::ParameterData> paramWinFunc = mFreqAnalysisInstance->getWindowFunction().getParametersMap();

    if( paramWinFunc.at( winFuncType ).needsParameter )
    {
        mFreqAnalysisInstance->getWindowFunction().setParameter( winFuncType, aValue );
    }
}


//!************************************************************************
//! Handle for changing the FFT window function type in frequency analysis
//! settings
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedFreqAnalysisWinFuncType
    (
    int aIndex      //!< index
    )
{
    WindowFunction::WindowFunctionType winFuncType = static_cast<WindowFunction::WindowFunctionType>( aIndex );
    mFreqAnalysisInstance->getWindowFunction().setActiveType( winFuncType );

    std::map<WindowFunction::WindowFunctionType, WindowFunction::ParameterData> paramWinFuncMap = mFreqAnalysisInstance->getWindowFunction().getParametersMap();

    bool haveParameter = paramWinFuncMap.at( winFuncType ).needsParameter;
    mFrequencyAnalysisUi->WinFuncParamLabel->setEnabled( haveParameter );
    mFrequencyAnalysisUi->WinFuncParamSpinBox->setEnabled( haveParameter);

    mFrequencyAnalysisUi->WinFuncParamLabel->setText( paramWinFuncMap.at( winFuncType ).paramSymbol + ( haveParameter ? " =" : "" ) );
    mFrequencyAnalysisUi->WinFuncParamSpinBox->setMinimum( paramWinFuncMap.at( winFuncType ).minValue );
    mFrequencyAnalysisUi->WinFuncParamSpinBox->setMaximum( paramWinFuncMap.at( winFuncType ).maxValue );
    mFrequencyAnalysisUi->WinFuncParamSpinBox->setValue( paramWinFuncMap.at( winFuncType ).crtValue );
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! X axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionXtransient
    (
    bool aEnabled
    )
{
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = aEnabled;
    updatePlot2dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! X axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionXfft
    (
    bool aEnabled
    )
{
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = aEnabled;
    updatePlot2dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! X axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionXperiodogram
    (
    bool aEnabled
    )
{
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlot2dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! X axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionXsrs
    (
    bool aEnabled
    )
{
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = aEnabled;
    updatePlot2dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! X axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionXcepstrum
    (
    bool aEnabled
    )
{
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = aEnabled;
    updatePlot2dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Y axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionYtransient
    (
    bool aEnabled
    )
{
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = aEnabled;
    updatePlot2dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Y axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionYfft
    (
    bool aEnabled
    )
{
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = aEnabled;
    updatePlot2dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Y axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionYperiodogram
    (
    bool aEnabled
    )
{
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlot2dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Y axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionYsrs
    (
    bool aEnabled
    )
{
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = aEnabled;
    updatePlot2dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Y axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionYcepstrum
    (
    bool aEnabled
    )
{
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = aEnabled;
    updatePlot2dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Z axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionZtransient
    (
    bool aEnabled
    )
{
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = aEnabled;
    updatePlot2dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Z axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionZfft
    (
    bool aEnabled
    )
{
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = aEnabled;
    updatePlot2dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Z axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionZperiodogram
    (
    bool aEnabled
    )
{
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlot2dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Z axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionZsrs
    (
    bool aEnabled
    )
{
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = aEnabled;
    updatePlot2dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 2D plot option
//! Z axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot2dOptionZcepstrum
    (
    bool aEnabled
    )
{
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = aEnabled;
    updatePlot2dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! X axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionXtransient
    (
    bool aEnabled
    )
{
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = aEnabled;
    updatePlot3dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! X axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionXfft
    (
    bool aEnabled
    )
{
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = aEnabled;
    updatePlot3dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! X axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionXperiodogram
    (
    bool aEnabled
    )
{
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlot3dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! X axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionXsrs
    (
    bool aEnabled
    )
{
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = aEnabled;
    updatePlot3dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! X axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionXcepstrum
    (
    bool aEnabled
    )
{
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = aEnabled;
    updatePlot3dOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Y axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionYtransient
    (
    bool aEnabled
    )
{
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = aEnabled;
    updatePlot3dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Y axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionYfft
    (
    bool aEnabled
    )
{
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = aEnabled;
    updatePlot3dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Y axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionYperiodogram
    (
    bool aEnabled
    )
{
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlot3dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Y axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionYsrs
    (
    bool aEnabled
    )
{
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = aEnabled;
    updatePlot3dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Y axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionYcepstrum
    (
    bool aEnabled
    )
{
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = aEnabled;
    updatePlot3dOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Z axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionZtransient
    (
    bool aEnabled
    )
{
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = aEnabled;
    updatePlot3dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Z axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionZfft
    (
    bool aEnabled
    )
{
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = aEnabled;
    updatePlot3dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Z axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionZperiodogram
    (
    bool aEnabled
    )
{
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlot3dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Z axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionZsrs
    (
    bool aEnabled
    )
{
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = aEnabled;
    updatePlot3dOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a 3D plot option
//! Z axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlot3dOptionZcepstrum
    (
    bool aEnabled
    )
{
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = aEnabled;
    updatePlot3dOptionsButtonsZ();
}


#if BUILD_CUDA
    //!************************************************************************
    //! Handle for changing the TensorRT inference loss threshold for Z-axis
    //!
    //! @returns nothing
    //!************************************************************************
    /* slot */ void SpectralViewer::handleChangedTrtInferZloss
        (
        double aValue   //!< value
        )
    {
        mTrt.lossThd.zAxis = aValue;

        if( mTrtInfer01Canvas )
        {
            mTrtInfer01Canvas->setThreshold( mTrt.lossThd.zAxis );
            mTrtInfer01Canvas->update();
        }

        if( mTrtInfer10Canvas )
        {
            mTrtInfer10Canvas->setThreshold( mTrt.lossThd.zAxis );
            mTrtInfer10Canvas->update();
        }

        if( mTrtInfer60Canvas )
        {
            mTrtInfer60Canvas->setThreshold( mTrt.lossThd.zAxis );
            mTrtInfer60Canvas->update();
        }
    }
#endif


//!************************************************************************
//! Handle for changing the vibration monitoring option item on X-axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonXaxisItem
    (
    int aIndex      //!< index
    )
{
    if( aIndex >= VibrationMonitoringSettings::OPTION_ITEM_ISO_10816_1_CLASS_1
     && aIndex <= VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_21_TOWER )
    {
        mVibMonSettingsTemporary.xAxis.setActiveOption( static_cast<VibrationMonitoringSettings::OptionItem>( aIndex ) );
        mVibrationMonitoringSettingsUi->XGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == aIndex );
    }
}


//!************************************************************************
//! Handle for changing the X-axis RMS speed for transitioning A<->B zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonXvAB
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.xAxis.setIso20816Part1NonRotatingTransitionAB( aValue );

    mVibrationMonitoringSettingsUi->XvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, aValue ) - DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for changing the X-axis RMS speed for transitioning B<->C zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonXvBC
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.xAxis.setIso20816Part1NonRotatingTransitionBC( aValue );

    mVibrationMonitoringSettingsUi->XvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, aValue ) + DBL_SPIN_EPS ) );
    mVibrationMonitoringSettingsUi->XvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, aValue ) - DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for changing the X-axis RMS speed for transitioning C<->D zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonXvCD
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.xAxis.setIso20816Part1NonRotatingTransitionCD( aValue );

    mVibrationMonitoringSettingsUi->XvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, aValue ) + DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for changing the vibration monitoring option item on Y-axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonYaxisItem
    (
    int aIndex      //!< index
    )
{
    if( aIndex >= VibrationMonitoringSettings::OPTION_ITEM_ISO_10816_1_CLASS_1
     && aIndex <= VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_21_TOWER )
    {
        mVibMonSettingsTemporary.yAxis.setActiveOption( static_cast<VibrationMonitoringSettings::OptionItem>( aIndex ) );
        mVibrationMonitoringSettingsUi->YGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == aIndex );
    }
}


//!************************************************************************
//! Handle for changing the Y-axis RMS speed for transitioning A<->B zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonYvAB
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.yAxis.setIso20816Part1NonRotatingTransitionAB( aValue );

    mVibrationMonitoringSettingsUi->YvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, aValue ) - DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for changing the Y-axis RMS speed for transitioning B<->C zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonYvBC
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.yAxis.setIso20816Part1NonRotatingTransitionBC( aValue );

    mVibrationMonitoringSettingsUi->YvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, aValue ) + DBL_SPIN_EPS ) );
    mVibrationMonitoringSettingsUi->YvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, aValue ) - DBL_SPIN_EPS ) );

}


//!************************************************************************
//! Handle for changing the Y-axis RMS speed for transitioning C<->D zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonYvCD
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.yAxis.setIso20816Part1NonRotatingTransitionCD( aValue );

    mVibrationMonitoringSettingsUi->YvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, aValue ) + DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for changing the vibration monitoring option item on Z-axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonZaxisItem
    (
    int aIndex      //!< index
    )
{
    if( aIndex >= VibrationMonitoringSettings::OPTION_ITEM_ISO_10816_1_CLASS_1
     && aIndex <= VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_21_TOWER )
    {
        mVibMonSettingsTemporary.zAxis.setActiveOption( static_cast<VibrationMonitoringSettings::OptionItem>( aIndex ) );
        mVibrationMonitoringSettingsUi->ZGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == aIndex );
    }
}


//!************************************************************************
//! Handle for changing the Z-axis RMS speed for transitioning A<->B zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonZvAB
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.zAxis.setIso20816Part1NonRotatingTransitionAB( aValue );

    mVibrationMonitoringSettingsUi->ZvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, aValue ) - DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for changing the Z-axis RMS speed for transitioning B<->C zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonZvBC
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.zAxis.setIso20816Part1NonRotatingTransitionBC( aValue );

    mVibrationMonitoringSettingsUi->ZvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, aValue ) + DBL_SPIN_EPS ) );
    mVibrationMonitoringSettingsUi->ZvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, aValue ) - DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for changing the Z-axis RMS speed for transitioning C<->D zones
//! when monitoring mechanical vibrations
//! It applies to ISO 20816-1, boundaries are for non-rotating parts only.
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedVibMonZvCD
    (
    double aValue   //!< value
    )
{
    aValue *= 1.e-3;
    mVibMonSettingsTemporary.zAxis.setIso20816Part1NonRotatingTransitionCD( aValue );

    mVibrationMonitoringSettingsUi->XvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, aValue ) + DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for exit event
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleExit()
{
    QApplication::quit();
}


//!************************************************************************
//! Handle for accelerometer settings
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleMenuAccel()
{
    updateAccelDialogControls();
    mAccelerometerDlg.exec();
}


//!************************************************************************
//! Handle for frequency analysis settings
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleMenuFreqAnalysis()
{
    //////////////
    /// FFT
    //////////////
    mFrequencyAnalysisUi->FftSizeComboBox->setCurrentIndex( mFreqAnalysisInstance->getFftSizeOptionIndex() );

    mFrequencyAnalysisUi->BinWidthValue->setText( QString::number( mFreqAnalysisInstance->getFftBinWidth() ) + " Hz/bin" );
    mFrequencyAnalysisUi->TimeGateValue->setText( convertSeconds2String( mFreqAnalysisInstance->getFftTimeGate() ) );

    WindowFunction::WindowFunctionType winFncType = mFreqAnalysisInstance->getWindowFunction().getActiveType();
    mFrequencyAnalysisUi->WinFuncComboBox->setCurrentIndex( winFncType );

    std::map<WindowFunction::WindowFunctionType, WindowFunction::ParameterData> paramWinFuncMap = mFreqAnalysisInstance->getWindowFunction().getParametersMap();
    bool haveParameter = paramWinFuncMap.at( winFncType ).needsParameter;
    mFrequencyAnalysisUi->WinFuncParamLabel->setEnabled( haveParameter );
    mFrequencyAnalysisUi->WinFuncParamSpinBox->setEnabled( haveParameter );

    mFrequencyAnalysisUi->WinFuncParamLabel->setText( paramWinFuncMap.at( winFncType ).paramSymbol + ( haveParameter ? " =" : "" ) );
    mFrequencyAnalysisUi->WinFuncParamSpinBox->setMinimum( paramWinFuncMap.at( winFncType ).minValue );
    mFrequencyAnalysisUi->WinFuncParamSpinBox->setMaximum( paramWinFuncMap.at( winFncType ).maxValue );
    mFrequencyAnalysisUi->WinFuncParamSpinBox->setValue( paramWinFuncMap.at( winFncType ).crtValue );

    //////////////
    /// SRS
    //////////////
    mFrequencyAnalysisUi->SrsMethodComboBox->setCurrentIndex( mFreqAnalysisInstance->getSrsParameters().model );

    FrequencyAnalysis::SrsZeta zeta = mFreqAnalysisInstance->getSrsParameters().zeta;
    mFrequencyAnalysisUi->SrsZetaLabel->setText( zeta.paramSymbol + " =" );
    mFrequencyAnalysisUi->SrsZetaSpinBox->setMinimum( zeta.minValue );
    mFrequencyAnalysisUi->SrsZetaSpinBox->setMaximum( zeta.maxValue );

    mFrequencyAnalysisDlg.exec();
}


//!************************************************************************
//! Handle for 2D plotting options
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleMenuSelectPlots2D()
{
    updatePlot2dOptionsCheckBoxesX();
    updatePlot2dOptionsCheckBoxesY();
    updatePlot2dOptionsCheckBoxesZ();

    bool isSrsRunning = mFreqAnalysisInstance->getSrsIsRunning();
    mPlot2dOptionsUi->XaxisSrsCheckBox->setEnabled( isSrsRunning );
    mPlot2dOptionsUi->YaxisSrsCheckBox->setEnabled( isSrsRunning );
    mPlot2dOptionsUi->ZaxisSrsCheckBox->setEnabled( isSrsRunning );

    mPlot2dOptionsDlg.exec();
}


//!************************************************************************
//! Handle for 3D plotting options
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleMenuSelectPlots3D()
{
    updatePlot3dOptionsCheckBoxesX();
    updatePlot3dOptionsCheckBoxesY();
    updatePlot3dOptionsCheckBoxesZ();

    bool isSrsRunning = mFreqAnalysisInstance->getSrsIsRunning();
    mPlot3dOptionsUi->XaxisSrsCheckBox->setEnabled( isSrsRunning );
    mPlot3dOptionsUi->YaxisSrsCheckBox->setEnabled( isSrsRunning );
    mPlot3dOptionsUi->ZaxisSrsCheckBox->setEnabled( isSrsRunning );

    mPlot3dOptionsDlg.exec();
}


#if BUILD_CUDA
//!************************************************************************
    //! Handle for TensorRT settings
    //!
    //! @returns: nothing
    //!************************************************************************
    /* slot */ void SpectralViewer::handleMenuTrtSettings()
    {
        updateTrtSettingsDialogControls();
        mTrtSettingsDlg.exec();
    }
#endif


//!************************************************************************
//! Handle for vibration monitoring settings
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleMenuVibMon()
{
    mVibMonSettingsTemporary.xAxis = mVibrationHndlInstance->getTriaxialSettings().xAxis;
    mVibMonSettingsTemporary.yAxis = mVibrationHndlInstance->getTriaxialSettings().yAxis;
    mVibMonSettingsTemporary.zAxis = mVibrationHndlInstance->getTriaxialSettings().zAxis;

    VibrationMonitoringSettings::OptionItem xAxisOption = mVibrationHndlInstance->getTriaxialSettings().xAxis.getActiveOption();
    VibrationMonitoringSettings::OptionItem yAxisOption = mVibrationHndlInstance->getTriaxialSettings().yAxis.getActiveOption();
    VibrationMonitoringSettings::OptionItem zAxisOption = mVibrationHndlInstance->getTriaxialSettings().zAxis.getActiveOption();

    mVibrationMonitoringSettingsUi->XaxisComboBox->setCurrentIndex( xAxisOption );
    mVibrationMonitoringSettingsUi->YaxisComboBox->setCurrentIndex( yAxisOption );
    mVibrationMonitoringSettingsUi->ZaxisComboBox->setCurrentIndex( zAxisOption );

    mVibrationMonitoringSettingsUi->XGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == xAxisOption );
    mVibrationMonitoringSettingsUi->YGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == yAxisOption );
    mVibrationMonitoringSettingsUi->ZGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == zAxisOption );

    VibrationMonitoringSettings::TransitionZone xTransition = mVibrationHndlInstance->getTriaxialSettings().xAxis.getTransitionIso20816Part1NonRotating();
    mVibrationMonitoringSettingsUi->XvABSpin->setValue( 1000 * xTransition.vFromAtoB );
    mVibrationMonitoringSettingsUi->XvBCSpin->setValue( 1000 * xTransition.vFromBtoC );
    mVibrationMonitoringSettingsUi->XvCDSpin->setValue( 1000 * xTransition.vFromCtoD );

    VibrationMonitoringSettings::TransitionZone yTransition = mVibrationHndlInstance->getTriaxialSettings().yAxis.getTransitionIso20816Part1NonRotating();
    mVibrationMonitoringSettingsUi->YvABSpin->setValue( 1000 * yTransition.vFromAtoB );
    mVibrationMonitoringSettingsUi->YvBCSpin->setValue( 1000 * yTransition.vFromBtoC );
    mVibrationMonitoringSettingsUi->YvCDSpin->setValue( 1000 * yTransition.vFromCtoD );

    VibrationMonitoringSettings::TransitionZone zTransition = mVibrationHndlInstance->getTriaxialSettings().zAxis.getTransitionIso20816Part1NonRotating();
    mVibrationMonitoringSettingsUi->ZvABSpin->setValue( 1000 * zTransition.vFromAtoB );
    mVibrationMonitoringSettingsUi->ZvBCSpin->setValue( 1000 * zTransition.vFromBtoC );
    mVibrationMonitoringSettingsUi->ZvCDSpin->setValue( 1000 * zTransition.vFromCtoD );

    // X-axis boundaries
    mVibrationMonitoringSettingsUi->XvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, xTransition.vFromBtoC ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->XvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, xTransition.vFromAtoB ) - DBL_SPIN_EPS ) );
    mVibrationMonitoringSettingsUi->XvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, xTransition.vFromCtoD ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->XvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, xTransition.vFromBtoC ) - DBL_SPIN_EPS ) );

    // Y-axis boundaries
    mVibrationMonitoringSettingsUi->YvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, yTransition.vFromBtoC ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->YvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, yTransition.vFromAtoB ) - DBL_SPIN_EPS) );
    mVibrationMonitoringSettingsUi->YvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, yTransition.vFromCtoD ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->YvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, yTransition.vFromBtoC ) - DBL_SPIN_EPS ) );

    // Z-axis boundaries
    mVibrationMonitoringSettingsUi->ZvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, zTransition.vFromBtoC ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->ZvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, zTransition.vFromAtoB ) - DBL_SPIN_EPS ) );
    mVibrationMonitoringSettingsUi->ZvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, zTransition.vFromCtoD ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->ZvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, zTransition.vFromBtoC ) - DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsDlg.exec();
}


//!************************************************************************
//! Handle for nullifying the values of the accelerometer
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleNullifyValuesAccel()
{
    if( QMessageBox::Yes == QMessageBox::question( this, APP_NAME,
                           "Detected accelerations will be subtracted for all axes\
                           \nso that new values will ideally become zero.\
                           \n\nIt is highly recommended to proceed only if there\
                           \nare no dynamic accelerations acting, and the sensor\
                           \nis in a still and rigid position.\
                           \n\nContinue?",
                           QMessageBox::Yes | QMessageBox::No ) )
    {
        if( nullifyValuesAccel() )
        {
            QMessageBox::information( this, APP_NAME, "Nullifying accelerations succeeded.", QMessageBox::Ok );
        }
        else
        {
            QMessageBox::critical( this, APP_NAME, "Nullifying accelerations failed.", QMessageBox::Ok );
        }
    }
}


//!************************************************************************
//! Open a file with a configuration
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleOpenConfiguration()
{
    if( mIsConfigSetupRunning )
    {
        if( QMessageBox::Yes == QMessageBox::question( this, APP_NAME,
                               "A configuration setup is already running.\
                               \n\nStop running the current setup, so that a new one can be loaded?",
                               QMessageBox::Yes | QMessageBox::No ) )
        {
            stopConfiguration();
        }
    }

    if( !mIsConfigSetupRunning )
    {
        QString selectedFilter;
        QString fileName = QFileDialog::getOpenFileName( this,
                                                         "Open configuration file",
                                                         "",
                                                         "Configuration files (*.cfg);;All files (*)",
                                                         &selectedFilter,
                                                         QFileDialog::DontUseNativeDialog
                                                        );

        mConfigFilename = fileName.toStdString();
        std::ifstream inputFile( mConfigFilename );

        if( inputFile.is_open() )
        {
            bool parseOk = true;
            const std::string DELIM = "=";
            std::string currentLine;
            size_t currentLineIndex = 0;

            while( std::getline( inputFile, currentLine ) )
            {
                std::vector<std::string> substringsVec;
                size_t pos = 0;

                while( ( pos = currentLine.find( DELIM ) ) != std::string::npos )
                {
                    substringsVec.push_back( currentLine.substr( 0, pos ) );
                    currentLine.erase( 0, pos + DELIM.length() );
                }

                if( currentLine.size() )
                {
                    substringsVec.push_back( currentLine );
                }

                if( currentLineIndex < CONFIG_FIELD_NAMES.size() )
                {
                    if( 2 == substringsVec.size() )
                    {
                        if( substringsVec.at( 0 ) == CONFIG_FIELD_NAMES.at( static_cast<ConfigField>( currentLineIndex ) ) )
                        {
                            switch( currentLineIndex )
                            {
                                case CONFIG_FIELD_TYPE:
                                    if( compareIstr( ACCEL_NAME.toStdString(), substringsVec.at( 1 ) ) )
                                    {
                                        mConfig.accelerometerType = ACCEL_NAME;
                                    }
                                    else
                                    {
                                        parseOk = false;
                                    }
                                    break;

                                case CONFIG_FIELD_RANGE:
                                    if( compareIstr( "ADXL355", mConfig.accelerometerType.toStdString() ) )
                                    {
                                        if( compareIstr( "2g", substringsVec.at( 1 ) ) )
                                        {
                                            mConfig.range = Adxl355Adxl357Common::ACCELERATION_RANGE_2G;
                                        }
                                        else if( compareIstr( "4g", substringsVec.at( 1 ) ) )
                                        {
                                            mConfig.range = Adxl355Adxl357Common::ACCELERATION_RANGE_4G;
                                        }
                                        else if( compareIstr( "8g", substringsVec.at( 1 ) ) )
                                        {
                                            mConfig.range = Adxl355Adxl357Common::ACCELERATION_RANGE_8G;
                                        }
                                        else
                                        {
                                            parseOk = false;
                                        }
                                    }
                                    else if( compareIstr( "ADXL357", mConfig.accelerometerType.toStdString() ) )
                                    {
                                        if( compareIstr( "10g", substringsVec.at( 1 ) ) )
                                        {
                                            mConfig.range = Adxl355Adxl357Common::ACCELERATION_RANGE_10G;
                                        }
                                        else if( compareIstr( "20g", substringsVec.at( 1 ) ) )
                                        {
                                            mConfig.range = Adxl355Adxl357Common::ACCELERATION_RANGE_20G;
                                        }
                                        else if( compareIstr( "40g", substringsVec.at( 1 ) ) )
                                        {
                                            mConfig.range = Adxl355Adxl357Common::ACCELERATION_RANGE_40G;
                                        }
                                        else
                                        {
                                            parseOk = false;
                                        }
                                    }
                                    else
                                    {
                                        parseOk = false;
                                    }
                                    break;

                                case CONFIG_FIELD_ODR:
                                    {
                                        QString qs = QString::fromStdString( substringsVec.at( 1 ) );
                                        double odrHz = qs.toDouble();

                                        if( 4000 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_4000;
                                        }
                                        else if( 2000 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_2000;
                                        }
                                        else if( 1000 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_1000;
                                        }
                                        else if( 500 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_500;
                                        }
                                        else if( 250 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_250;
                                        }
                                        else if( 125 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_125;
                                        }
                                        else if( 62.5 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_62_5;
                                        }
                                        else if( 31.25 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_31_25;
                                        }
                                        else if( 15.625 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_15_625;
                                        }
                                        else if( 7.8125 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_7_8125;
                                        }
                                        else if( 3.90625 == odrHz )
                                        {
                                            mConfig.odrSetting = Adxl355Adxl357Common::ODR_SETTING_3_90625;
                                        }
                                        else
                                        {
                                            parseOk = false;
                                        }
                                    }
                                    break;

                                case CONFIG_FIELD_NULLIFY:
                                    if( compareIstr( "yes", substringsVec.at( 1 ) ) )
                                    {
                                        mConfig.nullifyActive = true;
                                    }
                                    else if( compareIstr( "no", substringsVec.at( 1 ) ) )
                                    {
                                        mConfig.nullifyActive = false;
                                    }
                                    else
                                    {
                                        parseOk = false;
                                    }
                                    break;

                                case CONFIG_FIELD_NULLIFY_SAMPLES:
                                    {
                                        uint16_t nullifySamples = std::stoul( substringsVec.at( 1 ) );

                                        if( nullifySamples >= 1 )
                                        {
                                            mConfig.nullifySamples = nullifySamples;
                                        }
                                        else
                                        {
                                            parseOk = false;
                                        }
                                    }
                                    break;

                                case CONFIG_FIELD_BATCHES_COUNT:
                                    {
                                        uint32_t batchesCount = std::stoul( substringsVec.at( 1 ) );

                                        if( batchesCount >= 1 )
                                        {
                                            mConfig.batchesCount = batchesCount;
                                        }
                                        else
                                        {
                                            parseOk = false;
                                        }
                                    }
                                    break;

                                case CONFIG_FIELD_SAMPLES_PER_BATCH:
                                    {
                                        uint32_t samplesCount = std::stoul( substringsVec.at( 1 ) );

                                        if( samplesCount >= 1 )
                                        {
                                            mConfig.samplesCount = samplesCount;
                                        }
                                        else
                                        {
                                            parseOk = false;
                                        }
                                    }
                                    break;

                                case CONFIG_FIELD_BATCHES_PERIOD:
                                    {
                                        double batchesPeriod = std::stod( substringsVec.at( 1 ) );

                                        if( batchesPeriod > 0 )
                                        {
                                            mConfig.batchesPeriod = batchesPeriod;
                                        }
                                        else
                                        {
                                            parseOk = false;
                                        }
                                    }
                                    break;

                                default:
                                    break;
                            }
                        }
                        else
                        {
                            parseOk = false;
                            break;
                        }
                    }
                    else
                    {
                        parseOk = false;
                        break;
                    }
                }
                else
                {
                    // already passed all lines of interest
                    break;
                }

                currentLineIndex++;
            }

            inputFile.close();
            QString msg;
            QMessageBox msgBox;

            if( parseOk )
            {
                double singleBatchDuration = static_cast<double>( mConfig.samplesCount ) / mAccelInstance->getOdrFrequency( mConfig.odrSetting );

                if( singleBatchDuration < mConfig.batchesPeriod )
                {
                    double totalDaqDuration = ( mConfig.batchesCount - 1 ) * mConfig.batchesPeriod + singleBatchDuration;
                    msg = "The configuration was parsed successfully.\n";
                    msg += "Total DAQ duration will be ";
                    msg += convertSeconds2String( totalDaqDuration );
                    msg += "\n\nDo you want to run this setup?";

                    if( QMessageBox::Yes == QMessageBox::question( this, APP_NAME, msg, QMessageBox::Yes | QMessageBox::No ) )
                    {
                        runConfiguration();
                    }
                }
                else
                {
                    msg = "The batch period is smaller than a single DAQ duration.\n";
                    msg += "The batch period must be > ";
                    msg += QString::number( singleBatchDuration );
                    msg += "\n\nNo DAQ will be started.";
                    msgBox.setText( msg );
                    msgBox.exec();
                }
            }
            else
            {
                msg = "The selected file does not contain any valid configuration.";
                msgBox.setText( msg );
                msgBox.exec();
            }
        }
        else if( fileName.size() )
        {
            QString msg = "Could not open file \"" + fileName + "\".";
            QMessageBox msgBox;
            msgBox.setText( msg );
            msgBox.exec();
        }
    }
}


//!************************************************************************
//! Handle for closing the 2D plot options
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot2dOptionsClosed()
{
    //////////////////////////
    /// X axis
    //////////////////////////
    if( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) )
    {
        if( !mPlot2dXtransient )
        {
            mPlot2dXtransient = new Plot2d( this, Plot2d::PLOT_2D_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot2dXtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlot2dXtransient, SLOT( receiveNewData() ) );
        }

        if( mPlot2dXtransient->isHidden() )
        {
            mPlot2dXtransient->show();
        }
    }
    else if( mPlot2dXtransient )
    {
        mPlot2dXtransient->close();
    }


    if( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) )
    {
        if( !mPlot2dXfft )
        {
            mPlot2dXfft = new Plot2d( this, Plot2d::PLOT_2D_TYPE_FFT, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot2dXfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlot2dXfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlot2dXfft->isHidden() )
        {
            mPlot2dXfft->show();
        }
    }
    else if( mPlot2dXfft )
    {
        mPlot2dXfft->close();
    }


    if( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) )
    {
        if( !mPlot2dXperiodogram )
        {
            mPlot2dXperiodogram = new Plot2d( this, Plot2d::PLOT_2D_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot2dXperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlot2dXperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlot2dXperiodogram->isHidden() )
        {
            mPlot2dXperiodogram->show();
        }
    }
    else if( mPlot2dXperiodogram )
    {
        mPlot2dXperiodogram->close();
    }


    if( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) )
    {
        if( !mPlot2dXsrs )
        {
            mPlot2dXsrs = new Plot2d( this, Plot2d::PLOT_2D_TYPE_SRS, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot2dXsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlot2dXsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlot2dXsrs->isHidden() )
        {
            mPlot2dXsrs->show();
        }
    }
    else if( mPlot2dXsrs )
    {
        mPlot2dXsrs->close();
    }


    if( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) )
    {
        if( !mPlot2dXcepstrum )
        {
            mPlot2dXcepstrum = new Plot2d( this, Plot2d::PLOT_2D_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot2dXcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlot2dXcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlot2dXcepstrum->isHidden() )
        {
            mPlot2dXcepstrum->show();
        }
    }
    else if( mPlot2dXcepstrum )
    {
        mPlot2dXcepstrum->close();
    }


    //////////////////////////
    /// Y axis
    //////////////////////////
    if( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) )
    {
        if( !mPlot2dYtransient )
        {
            mPlot2dYtransient = new Plot2d( this, Plot2d::PLOT_2D_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot2dYtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlot2dYtransient, SLOT( receiveNewData() ) );
        }

        if( mPlot2dYtransient->isHidden() )
        {
            mPlot2dYtransient->show();
        }
    }
    else if( mPlot2dYtransient )
    {
        mPlot2dYtransient->close();
    }


    if( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) )
    {
        if( !mPlot2dYfft )
        {
            mPlot2dYfft = new Plot2d( this, Plot2d::PLOT_2D_TYPE_FFT, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot2dYfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlot2dYfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlot2dYfft->isHidden() )
        {
            mPlot2dYfft->show();
        }
    }
    else if( mPlot2dYfft )
    {
        mPlot2dYfft->close();
    }


    if( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) )
    {
        if( !mPlot2dYperiodogram )
        {
            mPlot2dYperiodogram = new Plot2d( this, Plot2d::PLOT_2D_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot2dYperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlot2dYperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlot2dYperiodogram->isHidden() )
        {
            mPlot2dYperiodogram->show();
        }
    }
    else if( mPlot2dYperiodogram )
    {
        mPlot2dYperiodogram->close();
    }


    if( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) )
    {
        if( !mPlot2dYsrs )
        {
            mPlot2dYsrs = new Plot2d( this, Plot2d::PLOT_2D_TYPE_SRS, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot2dYsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlot2dYsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlot2dYsrs->isHidden() )
        {
            mPlot2dYsrs->show();
        }
    }
    else if( mPlot2dYsrs )
    {
        mPlot2dYsrs->close();
    }


    if( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) )
    {
        if( !mPlot2dYcepstrum )
        {
            mPlot2dYcepstrum = new Plot2d( this, Plot2d::PLOT_2D_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot2dYcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlot2dYcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlot2dYcepstrum->isHidden() )
        {
            mPlot2dYcepstrum->show();
        }
    }
    else if( mPlot2dYcepstrum )
    {
        mPlot2dYcepstrum->close();
    }


    //////////////////////////
    /// Z axis
    //////////////////////////
    if( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) )
    {
        if( !mPlot2dZtransient )
        {
            mPlot2dZtransient = new Plot2d( this, Plot2d::PLOT_2D_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot2dZtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlot2dZtransient, SLOT( receiveNewData() ) );
        }

        if( mPlot2dZtransient->isHidden() )
        {
            mPlot2dZtransient->show();
        }
    }
    else if( mPlot2dZtransient )
    {
        mPlot2dZtransient->close();
    }


    if( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) )
    {
        if( !mPlot2dZfft )
        {
            mPlot2dZfft = new Plot2d( this, Plot2d::PLOT_2D_TYPE_FFT, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot2dZfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlot2dZfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlot2dZfft->isHidden() )
        {
            mPlot2dZfft->show();
        }
    }
    else if( mPlot2dZfft )
    {
        mPlot2dZfft->close();
    }


    if( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) )
    {
        if( !mPlot2dZperiodogram )
        {
            mPlot2dZperiodogram = new Plot2d( this, Plot2d::PLOT_2D_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot2dZperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlot2dZperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlot2dZperiodogram->isHidden() )
        {
            mPlot2dZperiodogram->show();
        }
    }
    else if( mPlot2dZperiodogram )
    {
        mPlot2dZperiodogram->close();
    }


    if( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) )
    {
        if( !mPlot2dZsrs )
        {
            mPlot2dZsrs = new Plot2d( this, Plot2d::PLOT_2D_TYPE_SRS, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot2dZsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlot2dZsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlot2dZsrs->isHidden() )
        {
            mPlot2dZsrs->show();
        }
    }
    else if( mPlot2dZsrs )
    {
        mPlot2dZsrs->close();
    }


    if( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) )
    {
        if( !mPlot2dZcepstrum )
        {
            mPlot2dZcepstrum = new Plot2d( this, Plot2d::PLOT_2D_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot2dZcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot2d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlot2dZcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlot2dZcepstrum->isHidden() )
        {
            mPlot2dZcepstrum->show();
        }
    }
    else if( mPlot2dZcepstrum )
    {
        mPlot2dZcepstrum->close();
    }


    uint16_t delayUs = getDaqDelayUs();

    if( mDaqThread && mDaqThread->isRunning() )
    {
        mDaqThread->pause();
        mDaqThread->usleep( delayUs );
    }

    updatePlotsVerticalMaxTransient();

    if( mDaqThread && mDaqThread->isRunning() )
    {
        mDaqThread->updateDelay( delayUs );
        mDaqThread->resume();
    }

    updatePlotsFrequencyParams();

    mPlot2dOptionsDlg.close();
}


//!************************************************************************
//! Handle for closing the 3D plot options
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot3dOptionsClosed()
{
    //////////////////////////
    /// X axis
    //////////////////////////
    if( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) )
    {
        if( !mPlot3dXtransient )
        {
            mPlot3dXtransient = new Plot3d( this, Plot3d::PLOT_3D_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot3dXtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlot3dXtransient, SLOT( receiveNewData() ) );
        }

        if( mPlot3dXtransient->isHidden() )
        {
            mPlot3dXtransient->show();
        }
    }
    else if( mPlot3dXtransient )
    {
        mPlot3dXtransient->close();
    }


    if( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) )
    {
        if( !mPlot3dXfft )
        {
            mPlot3dXfft = new Plot3d( this, Plot3d::PLOT_3D_TYPE_FFT, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot3dXfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlot3dXfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlot3dXfft->isHidden() )
        {
            mPlot3dXfft->show();
        }
    }
    else if( mPlot3dXfft )
    {
        mPlot3dXfft->close();
    }


    if( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) )
    {
        if( !mPlot3dXperiodogram )
        {
            mPlot3dXperiodogram = new Plot3d( this, Plot3d::PLOT_3D_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot3dXperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlot3dXperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlot3dXperiodogram->isHidden() )
        {
            mPlot3dXperiodogram->show();
        }
    }
    else if( mPlot3dXperiodogram )
    {
        mPlot3dXperiodogram->close();
    }


    if( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) )
    {
        if( !mPlot3dXsrs )
        {
            mPlot3dXsrs = new Plot3d( this, Plot3d::PLOT_3D_TYPE_SRS, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot3dXsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlot3dXsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlot3dXsrs->isHidden() )
        {
            mPlot3dXsrs->show();
        }
    }
    else if( mPlot3dXsrs )
    {
        mPlot3dXsrs->close();
    }


    if( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) )
    {
        if( !mPlot3dXcepstrum )
        {
            mPlot3dXcepstrum = new Plot3d( this, Plot3d::PLOT_3D_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_X );
            connect( mPlot3dXcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlot3dXcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlot3dXcepstrum->isHidden() )
        {
            mPlot3dXcepstrum->show();
        }
    }
    else if( mPlot3dXcepstrum )
    {
        mPlot3dXcepstrum->close();
    }


    //////////////////////////
    /// Y axis
    //////////////////////////
    if( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) )
    {
        if( !mPlot3dYtransient )
        {
            mPlot3dYtransient = new Plot3d( this, Plot3d::PLOT_3D_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot3dYtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlot3dYtransient, SLOT( receiveNewData() ) );
        }

        if( mPlot3dYtransient->isHidden() )
        {
            mPlot3dYtransient->show();
        }
    }
    else if( mPlot3dYtransient )
    {
        mPlot3dYtransient->close();
    }


    if( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) )
    {
        if( !mPlot3dYfft )
        {
            mPlot3dYfft = new Plot3d( this, Plot3d::PLOT_3D_TYPE_FFT, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot3dYfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlot3dYfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlot3dYfft->isHidden() )
        {
            mPlot3dYfft->show();
        }
    }
    else if( mPlot3dYfft )
    {
        mPlot3dYfft->close();
    }


    if( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) )
    {
        if( !mPlot3dYperiodogram )
        {
            mPlot3dYperiodogram = new Plot3d( this, Plot3d::PLOT_3D_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot3dYperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlot3dYperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlot3dYperiodogram->isHidden() )
        {
            mPlot3dYperiodogram->show();
        }
    }
    else if( mPlot3dYperiodogram )
    {
        mPlot3dYperiodogram->close();
    }


    if( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) )
    {
        if( !mPlot3dYsrs )
        {
            mPlot3dYsrs = new Plot3d( this, Plot3d::PLOT_3D_TYPE_SRS, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot3dYsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlot3dYsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlot3dYsrs->isHidden() )
        {
            mPlot3dYsrs->show();
        }
    }
    else if( mPlot3dYsrs )
    {
        mPlot3dYsrs->close();
    }


    if( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) )
    {
        if( !mPlot3dYcepstrum )
        {
            mPlot3dYcepstrum = new Plot3d( this, Plot3d::PLOT_3D_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlot3dYcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlot3dYcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlot3dYcepstrum->isHidden() )
        {
            mPlot3dYcepstrum->show();
        }
    }
    else if( mPlot3dYcepstrum )
    {
        mPlot3dYcepstrum->close();
    }


    //////////////////////////
    /// Z axis
    //////////////////////////
    if( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) )
    {
        if( !mPlot3dZtransient )
        {
            mPlot3dZtransient = new Plot3d( this, Plot3d::PLOT_3D_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot3dZtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlot3dZtransient, SLOT( receiveNewData() ) );
        }

        if( mPlot3dZtransient->isHidden() )
        {
            mPlot3dZtransient->show();
        }
    }
    else if( mPlot3dZtransient )
    {
        mPlot3dZtransient->close();
    }


    if( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) )
    {
        if( !mPlot3dZfft )
        {
            mPlot3dZfft = new Plot3d( this, Plot3d::PLOT_3D_TYPE_FFT, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot3dZfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlot3dZfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlot3dZfft->isHidden() )
        {
            mPlot3dZfft->show();
        }
    }
    else if( mPlot3dZfft )
    {
        mPlot3dZfft->close();
    }


    if( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) )
    {
        if( !mPlot3dZperiodogram )
        {
            mPlot3dZperiodogram = new Plot3d( this, Plot3d::PLOT_3D_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot3dZperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlot3dZperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlot3dZperiodogram->isHidden() )
        {
            mPlot3dZperiodogram->show();
        }
    }
    else if( mPlot3dZperiodogram )
    {
        mPlot3dZperiodogram->close();
    }


    if( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) )
    {
        if( !mPlot3dZsrs )
        {
            mPlot3dZsrs = new Plot3d( this, Plot3d::PLOT_3D_TYPE_SRS, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot3dZsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlot3dZsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlot3dZsrs->isHidden() )
        {
            mPlot3dZsrs->show();
        }
    }
    else if( mPlot3dZsrs )
    {
        mPlot3dZsrs->close();
    }


    if( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) )
    {
        if( !mPlot3dZcepstrum )
        {
            mPlot3dZcepstrum = new Plot3d( this, Plot3d::PLOT_3D_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlot3dZcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot3d(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlot3dZcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlot3dZcepstrum->isHidden() )
        {
            mPlot3dZcepstrum->show();
        }
    }
    else if( mPlot3dZcepstrum )
    {
        mPlot3dZcepstrum->close();
    }


    uint16_t delayUs = getDaqDelayUs();

    if( mDaqThread && mDaqThread->isRunning() )
    {
        mDaqThread->pause();
        mDaqThread->usleep( delayUs );
    }

    updatePlotsVerticalMaxTransient();

    if( mDaqThread && mDaqThread->isRunning() )
    {
        mDaqThread->updateDelay( delayUs );
        mDaqThread->resume();
    }

    updatePlotsFrequencyParams();

    mPlot3dOptionsDlg.close();
}


//!************************************************************************
//! Handle for deselecting all 2D plots
//! *** X axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot2dOptionsXDeselectAll()
{
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = false;
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = false;
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = false;
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = false;
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = false;

    updatePlot2dOptionsCheckBoxesX();
}


//!************************************************************************
//! Handle for selecting all 2D plots
//! *** X axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot2dOptionsXSelectAll()
{
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = true;
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = true;
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = true;
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = true;

    updatePlot2dOptionsCheckBoxesX();
}


//!************************************************************************
//! Handle for deselecting all 2D plots
//! *** Y axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot2dOptionsYDeselectAll()
{
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = false;
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = false;
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = false;
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = false;
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = false;

    updatePlot2dOptionsCheckBoxesY();
}


//!************************************************************************
//! Handle for selecting all 2D plots
//! *** Y axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot2dOptionsYSelectAll()
{
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = true;
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = true;
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = true;
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = true;

    updatePlot2dOptionsCheckBoxesY();
}


//!************************************************************************
//! Handle for deselecting all 2D plots
//! *** Z axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot2dOptionsZDeselectAll()
{
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = false;
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = false;
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = false;
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = false;
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = false;

    updatePlot2dOptionsCheckBoxesZ();
}


//!************************************************************************
//! Handle for selecting all 2D plots
//! *** Z axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot2dOptionsZSelectAll()
{
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) = true;
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) = true;
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) = true;
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) = true;

    updatePlot2dOptionsCheckBoxesZ();
}


//!************************************************************************
//! Handle for deselecting all 3D plots
//! *** X axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot3dOptionsXDeselectAll()
{
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = false;
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = false;
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = false;
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = false;
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = false;

    updatePlot3dOptionsCheckBoxesX();
}


//!************************************************************************
//! Handle for selecting all 3D plots
//! *** X axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot3dOptionsXSelectAll()
{
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = true;
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = true;
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = true;
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = true;

    updatePlot3dOptionsCheckBoxesX();
}


//!************************************************************************
//! Handle for deselecting all 3D plots
//! *** Y axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot3dOptionsYDeselectAll()
{
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = false;
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = false;
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = false;
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = false;
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = false;

    updatePlot3dOptionsCheckBoxesY();
}


//!************************************************************************
//! Handle for selecting all 3D plots
//! *** Y axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot3dOptionsYSelectAll()
{
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = true;
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = true;
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = true;
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = true;

    updatePlot3dOptionsCheckBoxesY();
}


//!************************************************************************
//! Handle for deselecting all 3D plots
//! *** Z axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot3dOptionsZDeselectAll()
{
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = false;
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = false;
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = false;
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = false;
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = false;

    updatePlot3dOptionsCheckBoxesZ();
}


//!************************************************************************
//! Handle for selecting all 3D plots
//! *** Z axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlot3dOptionsZSelectAll()
{
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) = true;
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) = true;
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) = true;
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) = true;

    updatePlot3dOptionsCheckBoxesZ();
}


//!************************************************************************
//! Handle for resetting the accelerometer
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleResetAccel()
{
    if( QMessageBox::Yes == QMessageBox::question( this, APP_NAME,
                           "This will determine a new configuration\
                           \nsimilar to a power-on reset.\
                           \n\nContinue?",
                           QMessageBox::Yes | QMessageBox::No ) )
    {
        if( resetAccel() )
        {
            QMessageBox::information( this, APP_NAME, "Reset succeeded.", QMessageBox::Ok );
        }
        else
        {
            QMessageBox::critical( this, APP_NAME, "Reset failed.", QMessageBox::Ok );
        }
    }
}


//!************************************************************************
//! Handle for restoring default settings for vibration monitoring
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleRestoreDefaultsVibMonSettings()
{
    VibrationMonitoringSettings vibMonSettings;
    mVibMonSettingsTemporary.xAxis = vibMonSettings;
    mVibMonSettingsTemporary.yAxis = vibMonSettings;
    mVibMonSettingsTemporary.zAxis = vibMonSettings;

    VibrationMonitoringSettings::OptionItem xAxisOption = mVibMonSettingsTemporary.xAxis.getActiveOption();
    VibrationMonitoringSettings::OptionItem yAxisOption = mVibMonSettingsTemporary.yAxis.getActiveOption();
    VibrationMonitoringSettings::OptionItem zAxisOption = mVibMonSettingsTemporary.zAxis.getActiveOption();

    mVibrationMonitoringSettingsUi->XaxisComboBox->setCurrentIndex( xAxisOption );
    mVibrationMonitoringSettingsUi->YaxisComboBox->setCurrentIndex( yAxisOption );
    mVibrationMonitoringSettingsUi->ZaxisComboBox->setCurrentIndex( zAxisOption );

    mVibrationMonitoringSettingsUi->XGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == xAxisOption );
    mVibrationMonitoringSettingsUi->YGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == yAxisOption );
    mVibrationMonitoringSettingsUi->ZGroupBox->setEnabled( VibrationMonitoringSettings::OPTION_ITEM_ISO_20816_1 == zAxisOption );

    VibrationMonitoringSettings::TransitionZone xTransition = mVibMonSettingsTemporary.xAxis.getTransitionIso20816Part1NonRotating();
    mVibrationMonitoringSettingsUi->XvABSpin->setValue( 1000 * xTransition.vFromAtoB );
    mVibrationMonitoringSettingsUi->XvBCSpin->setValue( 1000 * xTransition.vFromBtoC );
    mVibrationMonitoringSettingsUi->XvCDSpin->setValue( 1000 * xTransition.vFromCtoD );

    VibrationMonitoringSettings::TransitionZone yTransition = mVibMonSettingsTemporary.yAxis.getTransitionIso20816Part1NonRotating();
    mVibrationMonitoringSettingsUi->YvABSpin->setValue( 1000 * yTransition.vFromAtoB );
    mVibrationMonitoringSettingsUi->YvBCSpin->setValue( 1000 * yTransition.vFromBtoC );
    mVibrationMonitoringSettingsUi->YvCDSpin->setValue( 1000 * yTransition.vFromCtoD );

    VibrationMonitoringSettings::TransitionZone zTransition = mVibMonSettingsTemporary.zAxis.getTransitionIso20816Part1NonRotating();
    mVibrationMonitoringSettingsUi->ZvABSpin->setValue( 1000 * zTransition.vFromAtoB );
    mVibrationMonitoringSettingsUi->ZvBCSpin->setValue( 1000 * zTransition.vFromBtoC );
    mVibrationMonitoringSettingsUi->ZvCDSpin->setValue( 1000 * zTransition.vFromCtoD );

    // X-axis boundaries
    mVibrationMonitoringSettingsUi->XvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, xTransition.vFromBtoC ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->XvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, xTransition.vFromAtoB ) - DBL_SPIN_EPS ) );
    mVibrationMonitoringSettingsUi->XvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, xTransition.vFromCtoD ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->XvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, xTransition.vFromBtoC ) - DBL_SPIN_EPS ) );

    // Y-axis boundaries
    mVibrationMonitoringSettingsUi->YvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, yTransition.vFromBtoC ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->YvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, yTransition.vFromAtoB ) - DBL_SPIN_EPS) );
    mVibrationMonitoringSettingsUi->YvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, yTransition.vFromCtoD ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->YvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, yTransition.vFromBtoC ) - DBL_SPIN_EPS ) );

    // Z-axis boundaries
    mVibrationMonitoringSettingsUi->ZvABSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_AB_MAX, zTransition.vFromBtoC ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->ZvBCSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MIN, zTransition.vFromAtoB ) - DBL_SPIN_EPS ) );
    mVibrationMonitoringSettingsUi->ZvBCSpin->setMaximum( 1000 * ( std::min( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_BC_MAX, zTransition.vFromCtoD ) + DBL_SPIN_EPS ) );

    mVibrationMonitoringSettingsUi->ZvCDSpin->setMinimum( 1000 * ( std::max( VibrationMonitoringSettings::ISO_20816_1_TRANSITION_CD_MIN, zTransition.vFromBtoC ) - DBL_SPIN_EPS ) );
}


//!************************************************************************
//! Handle for saving the settings for vibration monitoring
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleSaveVibMonSettings()
{
    mVibrationHndlInstance->getTriaxialSettings().xAxis = mVibMonSettingsTemporary.xAxis;
    mVibrationHndlInstance->getTriaxialSettings().yAxis = mVibMonSettingsTemporary.yAxis;
    mVibrationHndlInstance->getTriaxialSettings().zAxis = mVibMonSettingsTemporary.zAxis;

    mCbmCanvas->update();

    mVibrationMonitoringSettingsDlg.close();
}


#if BUILD_CUDA
    //!************************************************************************
    //! Handle for actions related to TensorRT engine build
    //!
    //! @returns nothing
    //!************************************************************************
    /* slot */ void SpectralViewer::handleTrtBuildDone()
    {   
        mTrt.engineTried = true;

        mMainUi->actionExit->setEnabled( true );
        mMainUi->actionExit->setText( "&Exit" );

        mTrt.engineBuilt = mTrt.cbmOnnx.isEngineBuilt();

        if( mTrt.engineBuilt )
        {
            mTrt.feedData.zData.resize( mTrt.feedBufferSize, 0 );

            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), this, SLOT( receiveNewFft(int) ) );
            connect( mTrtTimer, SIGNAL( timeout() ), this, SLOT( startTrtInference() ) );

            if( mTrt.isEnabled )
            {
                mTrtTimer->start( 1000 * INFER_PERIOD );
            }

            mTrtMenuInfer->setEnabled( true );
        }
        else
        {
            QMessageBox::critical( this, APP_NAME,
                                   "Building the Deep Learning TensorRT engine failed.",
                                   QMessageBox::Ok );
        }
    }
#endif


//!************************************************************************
//! Initialize the controls from the accelerometer settings dialog
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::initAccelDialogControls()
{
    mAccelerometerDlg.setWindowTitle( ACCEL_SETTINGS_STR );

    std::list<QString> optionsRange =
    {
#if BUILD_ADXL355
        PLUS_MINUS + "2 g",
        PLUS_MINUS + "4 g",
        PLUS_MINUS + "8 g"
#elif BUILD_ADXL357
        PLUS_MINUS + "10 g",
        PLUS_MINUS + "20 g",
        PLUS_MINUS + "40 g"
#endif
    };

    for( const auto& i : optionsRange )
    {
        mAccelerometerUi->RangeComboBox->addItem( i );
    }

    // -> see I2C_HIGH_SPEED_SUPPORTED, I2C_FAST_PLUS_SUPPORTED, and I2C_FAST_SUPPORTED in CMakeLists.txt
    // -> see Adxl355Adxl357Common::setI2cMode()
    //
    //                              ********* I2C speed ********
    std::list<QString> odrRange =   // High   Fast    fast   std
    {                               // speed  plus
        "4000",                     //   Y      -      -      -
        "2000",                     //   Y      Y      -      -
        "1000",                     //   Y      Y      -      -
         "500",                     //   Y      Y      Y      -
         "250",                     //   Y      Y      Y      -
         "125",                     //   Y      Y      Y      Y
          "62.5",                   //   Y      Y      Y      Y
          "31.25",                  //   Y      Y      Y      Y
          "15.625",                 //   Y      Y      Y      Y
           "7.8125",                //   Y      Y      Y      Y
           "3.90625"                //   Y      Y      Y      Y
    };

    for( const auto& i : odrRange )
    {
        mAccelerometerUi->OdrComboBox->addItem( i );
    }

    QStandardItemModel* odrItemModel = qobject_cast< QStandardItemModel* >( mAccelerometerUi->OdrComboBox->model() );

    for( size_t i = 0; i < odrRange.size(); i++ )
    {
        odrItemModel->item( i )->setEnabled( false );
    }

#ifdef BUILD_I2C_HIGH_SPEED
    odrItemModel->item( 0 )->setEnabled( true );
#endif

#ifdef BUILD_I2C_FAST_PLUS
    odrItemModel->item( 1 )->setEnabled( true );
    odrItemModel->item( 2 )->setEnabled( true );
#endif

#if BUILD_I2C_FAST
    odrItemModel->item( 3 )->setEnabled( true );
    odrItemModel->item( 4 )->setEnabled( true );
#endif

    for( size_t i = 5; i < odrRange.size(); i++ )
    {
        odrItemModel->item( i )->setEnabled( true );
    }

    updateAccelDialogControls();

    connect( mAccelerometerUi->RangeComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleChangedAccelRange(int) ) );;
    connect( mAccelerometerUi->OdrComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleChangedAccelOdr(int) ) );;

    connect( mAccelerometerUi->NullifyValuesButton, SIGNAL( clicked() ), this, SLOT( handleNullifyValuesAccel() ) );
    connect( mAccelerometerUi->ResetButton, SIGNAL( clicked() ), this, SLOT( handleResetAccel() ) );

    connect( mAccelerometerUi->CloseButton, SIGNAL( clicked() ), &mAccelerometerDlg, SLOT( close() ) );
}


//!************************************************************************
//! Initialize the controls from the frequency analysis settings dialog
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::initFreqAnalysisDialogControls()
{
    if( mFreqAnalysisInstance )
    {
        //////////////
        /// FFT
        //////////////
        for( const auto& i : FrequencyAnalysis::FFT_SIZE_VALUES )
        {
            mFrequencyAnalysisUi->FftSizeComboBox->addItem( QString::number( i.second ) );
        }       

        std::map<WindowFunction::WindowFunctionType, WindowFunction::ParameterData> paramWinFuncMap = mFreqAnalysisInstance->getWindowFunction().getParametersMap();

        for( const auto& i : WindowFunction::WINDOW_FUNCTION_NAMES )
        {
            mFrequencyAnalysisUi->WinFuncComboBox->addItem( i.second );

            if( paramWinFuncMap.at( i.first ).isEmphasized )
            {
                mFrequencyAnalysisUi->WinFuncComboBox->setItemData( static_cast<int>( i.first ), QBrush( Qt::white ), Qt::TextColorRole );
                mFrequencyAnalysisUi->WinFuncComboBox->setItemData( static_cast<int>( i.first ), QBrush( Qt::darkCyan ), Qt::BackgroundColorRole );
                QStandardItemModel* itemModel = qobject_cast< QStandardItemModel* >( mFrequencyAnalysisUi->WinFuncComboBox->model() );
                QStandardItem* rowItem = itemModel->item( i.first );
                auto crtFont = rowItem->font();
                crtFont.setBold( true );
                rowItem->setFont( crtFont );
            }

            mFrequencyAnalysisUi->WinFuncComboBox->setItemData( static_cast<int>( i.first ),
                                                              QString::fromStdWString( mFreqAnalysisInstance->getWindowFunction().getTypeDescription( i.first ) ),
                                                              Qt::ToolTipRole );
        }

        //////////////
        /// SRS
        //////////////
        bool isSrsRunning = mFreqAnalysisInstance->getSrsIsRunning();
        mFrequencyAnalysisUi->SrsInUsecheckBox->setChecked( isSrsRunning );

        for( const auto& i : FrequencyAnalysis::SRS_METHOD_NAMES )
        {
            mFrequencyAnalysisUi->SrsMethodComboBox->addItem( QString::fromStdString( i.second ) );
        }

        FrequencyAnalysis::SrsZeta zeta = mFreqAnalysisInstance->getSrsParameters().zeta;
        mFrequencyAnalysisUi->SrsZetaSpinBox->setValue( zeta.crtValue );

        mFrequencyAnalysisUi->SrsMethodComboBox->setEnabled( isSrsRunning );
        mFrequencyAnalysisUi->SrsZetaSpinBox->setEnabled( isSrsRunning );
    }

    //////////////
    /// FFT
    //////////////
    connect( mFrequencyAnalysisUi->FftSizeComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleChangedFreqAnalysisFftSize(int) ) );
    connect( mFrequencyAnalysisUi->WinFuncComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleChangedFreqAnalysisWinFuncType(int) ) );
    connect( mFrequencyAnalysisUi->WinFuncParamSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedFreqAnalysisWinFuncParam(double) ) );

    //////////////
    /// SRS
    //////////////
    connect( mFrequencyAnalysisUi->SrsInUsecheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedFreqAnalysisSrsInUse(bool) ) );
    connect( mFrequencyAnalysisUi->SrsMethodComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleChangedFreqAnalysisSrsModel(int) ) );
    connect( mFrequencyAnalysisUi->SrsZetaSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedFreqAnalysisSrsZeta(double) ) );

    connect( mFrequencyAnalysisUi->CloseButton, SIGNAL( clicked() ), &mFrequencyAnalysisDlg, SLOT( close() ) );
}


//!************************************************************************
//! Initialize the controls from the 2D plot options dialog
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::initPlot2dOptionsControls()
{
    mPlot2dOptions.xAxis =
    {
        { Plot2d::PLOT_2D_TYPE_TRANSIENT,   false },
        { Plot2d::PLOT_2D_TYPE_FFT,         false },
        { Plot2d::PLOT_2D_TYPE_PERIODOGRAM, false },
        { Plot2d::PLOT_2D_TYPE_SRS,         false },
        { Plot2d::PLOT_2D_TYPE_CEPSTRUM,    false }
    };

    mPlot2dOptions.yAxis = mPlot2dOptions.xAxis;
    mPlot2dOptions.zAxis = mPlot2dOptions.xAxis;

    updatePlot2dOptionsButtonsX();
    updatePlot2dOptionsButtonsY();
    updatePlot2dOptionsButtonsZ();

    //////////////////////////
    /// X axis
    //////////////////////////
    connect( mPlot2dOptionsUi->XaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionXtransient( bool ) ) );
    connect( mPlot2dOptionsUi->XaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionXfft( bool ) ) );
    connect( mPlot2dOptionsUi->XaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionXperiodogram( bool ) ) );
    connect( mPlot2dOptionsUi->XaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionXsrs( bool ) ) );
    connect( mPlot2dOptionsUi->XaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionXcepstrum( bool ) ) );

    connect( mPlot2dOptionsUi->XaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot2dOptionsXSelectAll() ) );
    connect( mPlot2dOptionsUi->XaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot2dOptionsXDeselectAll() ) );

    //////////////////////////
    /// Y axis
    //////////////////////////
    connect( mPlot2dOptionsUi->YaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionYtransient( bool ) ) );
    connect( mPlot2dOptionsUi->YaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionYfft( bool ) ) );
    connect( mPlot2dOptionsUi->YaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionYperiodogram( bool ) ) );
    connect( mPlot2dOptionsUi->YaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionYsrs( bool ) ) );
    connect( mPlot2dOptionsUi->YaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionYcepstrum( bool ) ) );

    connect( mPlot2dOptionsUi->YaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot2dOptionsYSelectAll() ) );
    connect( mPlot2dOptionsUi->YaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot2dOptionsYDeselectAll() ) );

    //////////////////////////
    /// Z axis
    //////////////////////////
    connect( mPlot2dOptionsUi->ZaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionZtransient( bool ) ) );
    connect( mPlot2dOptionsUi->ZaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionZfft( bool ) ) );
    connect( mPlot2dOptionsUi->ZaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionZperiodogram( bool ) ) );
    connect( mPlot2dOptionsUi->ZaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionZsrs( bool ) ) );
    connect( mPlot2dOptionsUi->ZaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot2dOptionZcepstrum( bool ) ) );

    connect( mPlot2dOptionsUi->ZaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot2dOptionsZSelectAll() ) );
    connect( mPlot2dOptionsUi->ZaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot2dOptionsZDeselectAll() ) );

    connect( mPlot2dOptionsUi->OkButton, SIGNAL( clicked() ), this, SLOT( handlePlot2dOptionsClosed() ) );
}


//!************************************************************************
//! Initialize the controls from the 3D plot options dialog
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::initPlot3dOptionsControls()
{
    mPlot3dOptions.xAxis =
    {
        { Plot3d::PLOT_3D_TYPE_TRANSIENT,   false },
        { Plot3d::PLOT_3D_TYPE_FFT,         false },
        { Plot3d::PLOT_3D_TYPE_PERIODOGRAM, false },
        { Plot3d::PLOT_3D_TYPE_SRS,         false },
        { Plot3d::PLOT_3D_TYPE_CEPSTRUM,    false }
    };

    mPlot3dOptions.yAxis = mPlot3dOptions.xAxis;
    mPlot3dOptions.zAxis = mPlot3dOptions.xAxis;

    updatePlot3dOptionsButtonsX();
    updatePlot3dOptionsButtonsY();
    updatePlot3dOptionsButtonsZ();

    //////////////////////////
    /// X axis
    //////////////////////////
    connect( mPlot3dOptionsUi->XaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionXtransient( bool ) ) );
    connect( mPlot3dOptionsUi->XaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionXfft( bool ) ) );
    connect( mPlot3dOptionsUi->XaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionXperiodogram( bool ) ) );
    connect( mPlot3dOptionsUi->XaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionXsrs( bool ) ) );
    connect( mPlot3dOptionsUi->XaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionXcepstrum( bool ) ) );

    connect( mPlot3dOptionsUi->XaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot3dOptionsXSelectAll() ) );
    connect( mPlot3dOptionsUi->XaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot3dOptionsXDeselectAll() ) );

    //////////////////////////
    /// Y axis
    //////////////////////////
    connect( mPlot3dOptionsUi->YaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionYtransient( bool ) ) );
    connect( mPlot3dOptionsUi->YaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionYfft( bool ) ) );
    connect( mPlot3dOptionsUi->YaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionYperiodogram( bool ) ) );
    connect( mPlot3dOptionsUi->YaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionYsrs( bool ) ) );
    connect( mPlot3dOptionsUi->YaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionYcepstrum( bool ) ) );

    connect( mPlot3dOptionsUi->YaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot3dOptionsYSelectAll() ) );
    connect( mPlot3dOptionsUi->YaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot3dOptionsYDeselectAll() ) );

    //////////////////////////
    /// Z axis
    //////////////////////////
    connect( mPlot3dOptionsUi->ZaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionZtransient( bool ) ) );
    connect( mPlot3dOptionsUi->ZaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionZfft( bool ) ) );
    connect( mPlot3dOptionsUi->ZaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionZperiodogram( bool ) ) );
    connect( mPlot3dOptionsUi->ZaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionZsrs( bool ) ) );
    connect( mPlot3dOptionsUi->ZaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlot3dOptionZcepstrum( bool ) ) );

    connect( mPlot3dOptionsUi->ZaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot3dOptionsZSelectAll() ) );
    connect( mPlot3dOptionsUi->ZaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlot3dOptionsZDeselectAll() ) );

    connect( mPlot3dOptionsUi->OkButton, SIGNAL( clicked() ), this, SLOT( handlePlot3dOptionsClosed() ) );
}


#if BUILD_CUDA
    //!************************************************************************
    //! Initialize the controls from the TensorRT settings dialog
    //!
    //! @returns: nothing
    //!************************************************************************
    void SpectralViewer::initTrtSettingsDialogControls()
    {
        //////////////////////////
        /// Window title
        //////////////////////////
        mTrtSettingsDlg.setWindowTitle( APP_NAME + " - TensorRT Settings" );

        //////////////////////////
        /// Plot for 1 s inference
        //////////////////////////
        mTrtInfer01Canvas = new PlotInferCanvas( &mTrtSettingsDlg );
        auto layout01Sec = new QVBoxLayout();
        layout01Sec->setMargin( 0 );
        layout01Sec->addWidget( mTrtInfer01Canvas );
        mTrtSettingsUi->Frame01Sec->setLayout( layout01Sec );
        mTrtInfer01Canvas->adjustSize();
        mTrtInfer01Canvas->setThreshold( INFER_LOSS_THD );
        // elements are 1s apart => 600 * 1s = 10m span
        mTrtInfer01Canvas->getDataVector().resize( mTrtInfer01Size, 0 );
        mTrtInfer01Canvas->setHorizontalMax( mTrtInfer01Size );
        mTrtInfer01Canvas->update();

        //////////////////////////
        /// Plot for averaged 10 s inference
        //////////////////////////
        mTrtInfer10Canvas = new PlotInferCanvas( &mTrtSettingsDlg );
        auto layout10Sec = new QVBoxLayout();
        layout10Sec->setMargin( 0 );
        layout10Sec->addWidget( mTrtInfer10Canvas );
        mTrtSettingsUi->Frame10Sec->setLayout( layout10Sec );
        mTrtInfer10Canvas->adjustSize();
        mTrtInfer10Canvas->setThreshold( INFER_LOSS_THD );
        // elements are 10s apart => 360 * 10s = 1h span
        mTrtInfer10Canvas->getDataVector().resize( mTrtInfer10Size, 0 );
        mTrtInfer10Canvas->setHorizontalMax( mTrtInfer10Size * 10 );
        mTrtInfer10Canvas->update();

        //////////////////////////
        /// Plot for averaged 60 s inference
        //////////////////////////
        mTrtInfer60Canvas = new PlotInferCanvas( &mTrtSettingsDlg );
        auto layout60Sec = new QVBoxLayout();
        layout60Sec->setMargin( 0 );
        layout60Sec->addWidget( mTrtInfer60Canvas );
        mTrtSettingsUi->Frame60Sec->setLayout( layout60Sec );
        mTrtInfer60Canvas->adjustSize();
        mTrtInfer60Canvas->setThreshold( INFER_LOSS_THD );
        // elements are 1m apart => 300 * 1m = 5h span
        mTrtInfer60Canvas->getDataVector().resize( mTrtInfer60Size, 0 );
        mTrtInfer60Canvas->setHorizontalMax( mTrtInfer60Size * 60 );
        mTrtInfer60Canvas->update();


        //////////////////////////
        /// Common updates
        //////////////////////////
        updateTrtSettingsDialogControls();

        connect( mTrtSettingsUi->LossThdSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedTrtInferZloss(double) ) );

        connect( mTrtSettingsUi->CloseButton, SIGNAL( clicked() ), &mTrtSettingsDlg, SLOT( close() ) );
    }
#endif


//!************************************************************************
//! Initialize the controls from the vibration monitoring settings dialog
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::initVibMonDialogControls()
{
    mVibrationMonitoringSettingsUi->AccelerometerGroupBox->setTitle( ACCEL_NAME );

    // option lists are common for all three axes
    std::map<VibrationMonitoringSettings::OptionItem, std::string> optionsXaxis = mVibrationHndlInstance->getTriaxialSettings().xAxis.getOptionsMap();

    for( const auto& i : optionsXaxis )
    {
        mVibrationMonitoringSettingsUi->XaxisComboBox->addItem( QString::fromStdString( i.second ) );
        mVibrationMonitoringSettingsUi->XaxisComboBox->setItemData( static_cast<int>( i.first ),
            QString::fromStdString( mVibrationHndlInstance->getTriaxialSettings().xAxis.getOptionDescription( i.first ) ),
            Qt::ToolTipRole );

        mVibrationMonitoringSettingsUi->YaxisComboBox->addItem( QString::fromStdString( i.second ) );
        mVibrationMonitoringSettingsUi->YaxisComboBox->setItemData( static_cast<int>( i.first ),
            QString::fromStdString( mVibrationHndlInstance->getTriaxialSettings().xAxis.getOptionDescription( i.first ) ),
            Qt::ToolTipRole );

        mVibrationMonitoringSettingsUi->ZaxisComboBox->addItem( QString::fromStdString( i.second ) );
        mVibrationMonitoringSettingsUi->ZaxisComboBox->setItemData( static_cast<int>( i.first ),
            QString::fromStdString( mVibrationHndlInstance->getTriaxialSettings().xAxis.getOptionDescription( i.first ) ),
            Qt::ToolTipRole );
    }

    connect( mVibrationMonitoringSettingsUi->XaxisComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleChangedVibMonXaxisItem(int) ) );
    connect( mVibrationMonitoringSettingsUi->YaxisComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleChangedVibMonYaxisItem(int) ) );
    connect( mVibrationMonitoringSettingsUi->ZaxisComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleChangedVibMonZaxisItem(int) ) );

    connect( mVibrationMonitoringSettingsUi->XvABSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonXvAB(double) ) );
    connect( mVibrationMonitoringSettingsUi->XvBCSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonXvBC(double) ) );
    connect( mVibrationMonitoringSettingsUi->XvCDSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonXvCD(double) ) );

    connect( mVibrationMonitoringSettingsUi->YvABSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonYvAB(double) ) );
    connect( mVibrationMonitoringSettingsUi->YvBCSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonYvBC(double) ) );
    connect( mVibrationMonitoringSettingsUi->YvCDSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonYvCD(double) ) );

    connect( mVibrationMonitoringSettingsUi->ZvABSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonZvAB(double) ) );
    connect( mVibrationMonitoringSettingsUi->ZvBCSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonZvBC(double) ) );
    connect( mVibrationMonitoringSettingsUi->ZvCDSpin, SIGNAL( valueChanged(double) ), this, SLOT( handleChangedVibMonZvCD(double) ) );

    connect( mVibrationMonitoringSettingsUi->SaveButton, SIGNAL( clicked() ), this, SLOT( handleSaveVibMonSettings() ) );
    connect( mVibrationMonitoringSettingsUi->CancelButton, SIGNAL( clicked() ), &mVibrationMonitoringSettingsDlg, SLOT( close() ) );
    connect( mVibrationMonitoringSettingsUi->RestoreDefaultsButton, SIGNAL( clicked() ), this, SLOT( handleRestoreDefaultsVibMonSettings() ) );
}


//!************************************************************************
//! Nullify the values of the accelerometer
//! This is useful if static accelerations, e.g. due to Earth's gravity,
//! need to be subtracted for all following readings. After this action only
//! dynamic accelerations will be read.
//!
//! @returns true for success
//!************************************************************************
bool SpectralViewer::nullifyValuesAccel()
{
    bool statusOk = false;

    if( mAccelInstance )
    {
        uint16_t delayUs = getDaqDelayUs();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->pause();
            mDaqThread->usleep( delayUs );
        }

        double xAxisAvg = 0;
        double yAxisAvg = 0;
        double zAxisAvg = 0;
        double sps = mAccelInstance->getOdrFrequency();
        uint16_t samplesCount = NULLIFY_SAMPLES_DEFAULT;

        if( mConfig.nullifyActive )
        {
            samplesCount = mConfig.nullifySamples;
        }
        else
        {
            const uint16_t NULLIFY_SAMPLES_MIN = 10;
            const double NULLIFY_DURATION = 0.05; // seconds
            samplesCount = static_cast<uint16_t>( NULLIFY_DURATION * sps );

            if( samplesCount < NULLIFY_SAMPLES_MIN )
            {
                samplesCount = NULLIFY_SAMPLES_MIN;
            }
        }

        statusOk = mAccelInstance->setAxisOffset( Adxl355Adxl357Common::AXIS_X, 0 );

        if( statusOk )
        {
            mAccelInstance->setAxisOffset( Adxl355Adxl357Common::AXIS_Y, 0 );
        }

        if( statusOk )
        {
            mAccelInstance->setAxisOffset( Adxl355Adxl357Common::AXIS_Z, 0 );
        }

        if( statusOk )
        {
            bool readOk = true;

            for( uint16_t i = 0; i < samplesCount; i++ )
            {
                double xAxisValue = 0;
                double yAxisValue = 0;
                double zAxisValue = 0;
                usleep( 1.e6 / sps );
                readOk = mAccelInstance->getAccelerationsOnAllAxes( &xAxisValue, &yAxisValue, &zAxisValue );

                if( readOk )
                {
                    xAxisAvg += xAxisValue;
                    yAxisAvg += yAxisValue;
                    zAxisAvg += zAxisValue;
                }
                else
                {
                    statusOk = false;
                    break;
                }
            }
        }

        if( statusOk )
        {
            xAxisAvg /= samplesCount;
            yAxisAvg /= samplesCount;
            zAxisAvg /= samplesCount;

            statusOk = mAccelInstance->setAxisOffset( Adxl355Adxl357Common::AXIS_X, xAxisAvg );

            if( statusOk )
            {
                statusOk = mAccelInstance->setAxisOffset( Adxl355Adxl357Common::AXIS_Y, yAxisAvg );
            }

            if( statusOk )
            {
                statusOk = mAccelInstance->setAxisOffset( Adxl355Adxl357Common::AXIS_Z, zAxisAvg );
            }
        }

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->resume();
        }
    }

    return statusOk;
}


//!************************************************************************
//! Read the temperature from the on-chip sensor in the GUI thread
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::readTemperature()
{
    if( mAccelInstance )
    {
        double tempCelsius = 0;

        if( mAccelInstance->getTemperatureValue( &tempCelsius ) )
        {
            mTemperature = tempCelsius;

            if( mAccelerometerDlg.isVisible() )
            {
                mAccelerometerUi->TemperatureValue->setText( QString::number( mTemperature, 'f', 1 ) + " [" +  DEG + "C]" );
            }
        }
    }
}


//!************************************************************************
//! Receive closed event from a 2D plot window
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::receiveClosedPlot2d
    (
    int aType,          //!< type of closed window
    int aAxis           //!< axis of closed window
    )
{
    Plot2d::Plot2dType type = static_cast<Plot2d::Plot2dType>( aType );
    Adxl355Adxl357Common::Axis axis = static_cast<Adxl355Adxl357Common::Axis>( aAxis );

    switch( axis )
    {
        case Adxl355Adxl357Common::AXIS_X:
            mPlot2dOptions.xAxis.at( type ) = false;
            break;

        case Adxl355Adxl357Common::AXIS_Y:
            mPlot2dOptions.yAxis.at( type ) = false;
            break;

        case Adxl355Adxl357Common::AXIS_Z:
            mPlot2dOptions.zAxis.at( type ) = false;
            break;

        default:
            break;
    }
}


//!************************************************************************
//! Receive closed event from a 3D plot window
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::receiveClosedPlot3d
    (
    int aType,          //!< type of closed window
    int aAxis           //!< axis of closed window
    )
{
    Plot3d::Plot3dType type = static_cast<Plot3d::Plot3dType>( aType );
    Adxl355Adxl357Common::Axis axis = static_cast<Adxl355Adxl357Common::Axis>( aAxis );

    switch( axis )
    {
        case Adxl355Adxl357Common::AXIS_X:
            mPlot3dOptions.xAxis.at( type ) = false;
            break;

        case Adxl355Adxl357Common::AXIS_Y:
            mPlot3dOptions.yAxis.at( type ) = false;
            break;

        case Adxl355Adxl357Common::AXIS_Z:
            mPlot3dOptions.zAxis.at( type ) = false;
            break;

        default:
            break;
    }
}


//!************************************************************************
//! Receive new accelerometer data sent by the DAQ thread
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::receiveNewData
    (
    double  aXaccel,    //!< acceleration on X axis
    double  aYaccel,    //!< acceleration on Y axis
    double  aZaccel     //!< acceleration on Z axis
    )
{
    if( mIsConfigSetupRunning && mDumpRemainingSamples )
    {
        if( mDumpCsvFile.is_open() )
        {
            double accelXms2 = this->mVibrationHndlInstance->convertAccelG2Ms2( aXaccel );
            double accelYms2 = this->mVibrationHndlInstance->convertAccelG2Ms2( aYaccel );
            double accelZms2 = this->mVibrationHndlInstance->convertAccelG2Ms2( aZaccel );

            QString qs = QString::number( accelXms2, 'f', 10 );
            qs += ",";
            qs += QString::number( accelYms2, 'f', 10 );
            qs += ",";
            qs += QString::number( accelZms2, 'f', 10 );
            qs += "\n";

            mDumpCsvFile << qs.toStdString();
        }

        mDumpRemainingSamples--;

        if( 0 == mDumpRemainingSamples )
        {
            mDumpRemainingBatches--;
            stopBatch();
        }

        if( 0 == mDumpRemainingBatches )
        {
            stopConfiguration();
        }
    }
}


#if BUILD_CUDA
    //!************************************************************************
    //! Signal receiver for new FFT bins
    //!
    //! @returns nothing
    //!************************************************************************
    /* slot */ void SpectralViewer::receiveNewFft
        (
        int  aAxis   //!< axis
        )
    {
        if( mTrt.isEnabled )
        {
            mTrtMutex.lock();
                Adxl355Adxl357Common::Axis axis = static_cast<Adxl355Adxl357Common::Axis>( aAxis );
                VibrationHandler* vh = VibrationHandler::getInstance();
                std::vector<VibrationHandler::FftBin> binsVec = vh->getFftBinsOnAxis( axis );
                size_t dataLen = binsVec.size();

                if( dataLen == mTrt.feedBufferSize )
                {
                    int i = 0;

                    switch( axis )
                    {
                        case Adxl355Adxl357Common::AXIS_Z:
                            for( i = 0; i < dataLen; i++ )
                            {
                                mTrt.feedData.zData.at( i ) = binsVec.at( i ).value / ( 2.0 * mTrt.feedBufferSize );
                            }
                            break;

                        default:
                            break;
                    }
                }
            mTrtMutex.unlock();
        }
    }
#endif


//!************************************************************************
//! Receive the SPS value sent by the DAQ thread
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::receiveNewSps
    (
    int aSps    //!< SPS
    )
{
    mReceivedSps = aSps > 0 ? aSps : 0;
    mSpsStatusbarLabel.setText( ACCEL_NAME + ": " + QString::number( mReceivedSps ) + " SPS" );
}


//!************************************************************************
//! Receive a new value read from the temperature read thread
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::receiveNewTemperature
    (
    double aValue   //!< value
    )
{
    if( mAccelInstance )
    {
        mTemperature = aValue;

        if( mAccelerometerDlg.isVisible() )
        {
            mAccelerometerUi->TemperatureValue->setText( QString::number( mTemperature, 'f', 1 ) + " [" +  DEG + "C]" );
        }
    }
}


//!************************************************************************
//! Reset the accelerometer
//!
//! @returns: true at success
//!************************************************************************
bool SpectralViewer::resetAccel()
{
    bool statusOk = false;

    if( mAccelInstance )
    {
        uint16_t delayUs = getDaqDelayUs();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->pause();
            mDaqThread->usleep( delayUs );
        }

        statusOk = mAccelInstance->reset();

        if( statusOk )
        {
#if BUILD_I2C_HIGH_SPEED
            mAccelInstance->setOdr( Adxl355Adxl357Common::ODR_SETTING_4000 );
#elif BUILD_I2C_FAST_PLUS
            mAccelInstance->setOdr( Adxl355Adxl357Common::ODR_SETTING_2000 );
#elif BUILD_I2C_FAST
            mAccelInstance->setOdr( Adxl355Adxl357Common::ODR_SETTING_500 );
#else
            mAccelInstance->setOdr( Adxl355Adxl357Common::ODR_SETTING_125 );
#endif

            mFreqAnalysisInstance->setFftSps( mAccelInstance->getOdrFrequency() );
            updatePlotsFrequencyParams();
            updatePlotsVerticalMaxTransient();

            mAccelInstance->enableStandbyMode( false );
        }

        if( mDaqThread && mDaqThread->isRunning() )
        {
            delayUs = getDaqDelayUs();
            mDaqThread->updateDelay( delayUs );
            mDaqThread->resume();
        }

        if( statusOk )
        {
            if( mAccelerometerDlg.isVisible() )
            {
                updateAccelDialogControls();
            }
        }
    }

    return statusOk;
}


//!************************************************************************
//! Run the parsed configuration
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::runConfiguration()
{
    bool statusOk = false;

    if( mAccelInstance )
    {
        const uint16_t DELAY_US = 1000;
        statusOk = resetAccel();
        usleep( DELAY_US );

        if( !statusOk )
        {
            QMessageBox::critical( this, APP_NAME, "Could not reset the accelerometer.", QMessageBox::Ok );
        }

        if( statusOk )
        {
            statusOk = changeAccelRange( mConfig.range );
            usleep( DELAY_US );

            if( !statusOk )
            {
                QMessageBox::critical( this, APP_NAME, "Could not change acceleration range.", QMessageBox::Ok );
            }
        }

        if( statusOk )
        {
            if( mConfig.nullifyActive )
            {                               
                statusOk = nullifyValuesAccel();
                usleep( DELAY_US );

                if( !statusOk )
                {
                    QMessageBox::critical( this, APP_NAME, "Could not nullify the accelerations.", QMessageBox::Ok );
                }
            }
        }

        if( statusOk )
        {
            statusOk = changeAccelOdr( mConfig.odrSetting );
            usleep( DELAY_US );

            if( !statusOk )
            {
                QMessageBox::critical( this, APP_NAME, "Could not change the ODR.", QMessageBox::Ok );
            }
        }

        if( statusOk )
        {
            statusOk = ( mConfig.batchesCount != 0 );

            if( !statusOk )
            {
                QMessageBox::critical( this, APP_NAME, "Could not start with zero batches.", QMessageBox::Ok );
            }
            else
            {
                mDumpRemainingBatches = mConfig.batchesCount;
            }
        }

        if( statusOk )
        {
            statusOk = ( mConfig.samplesCount != 0 );

            if( !statusOk )
            {
                QMessageBox::critical( this, APP_NAME, "Could not start with zero samples/batch.", QMessageBox::Ok );
            }
            else
            {
                mDumpRemainingSamples = mConfig.samplesCount;
            }
        }

        if( statusOk )
        {
            statusOk = ( mConfig.batchesPeriod > 0 );

            if( !statusOk )
            {
                QMessageBox::critical( this, APP_NAME, "Could not start with zero batch period.", QMessageBox::Ok );
            }
        }

        if( statusOk )
        {
            mMainUi->actionAccelerometer->setEnabled( false );
            const uint16_t DELAY_MS = 500;
            QTimer::singleShot( DELAY_MS, this, SLOT( runBatches() ) );
        }
    }
}


//!************************************************************************
//! Run the configuration batches for dumping data into CSV files
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::runBatches()
{
    connect( mConfigBatchesTimer, SIGNAL( timeout() ), this, SLOT( startBatch() ) );
    startBatch();
    mConfigBatchesTimer->start( 1000 * mConfig.batchesPeriod );
}


//!************************************************************************
//! Start running the current batch
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::startBatch()
{
    mDumpCsvFile.open( createCsvDataFilename( mConfigFilename,
                                              mConfig.batchesCount - mDumpRemainingBatches,
                                              std::to_string( mConfig.batchesCount ).length() ) );

    if( mDumpCsvFile.is_open() )
    {
        mDumpCsvFile << "accX,accY,accZ\n";
    }

    mIsConfigSetupRunning = true;
}


#if BUILD_CUDA
    //!************************************************************************
    //! Start running the TensorRT inference
    //!
    //! @returns: nothing
    //!************************************************************************
    /* slot */ void SpectralViewer::startTrtInference()
    {        
        mTrtMutex.lock();
            mTrt.cbmOnnx.feedInputData( mTrt.feedData.zData );
        mTrtMutex.unlock();

        mTrt.cbmOnnx.infer();
        double loss = 0;
        bool zInferSuccessful = mTrt.cbmOnnx.isInferenceSuccessful( loss );
        mTrt.lossCalc.zAxis = zInferSuccessful ? loss : 0;

        mVibrationHndlInstance->shiftLeft( mTrt.lossHistoryData.zAxis );
        mTrt.lossHistoryData.zAxis.back() = mTrt.lossCalc.zAxis;

        static uint16_t inferIt = 0;
        std::vector<double>::iterator itFirst;
        std::vector<double>::iterator itLast;
        size_t vecSize = 0;

        inferIt++;

        // 1 second
        if( mTrtInfer01Canvas )
        {
            vecSize = mTrtInfer01Canvas->getDataVector().size();
            itFirst = mTrt.lossHistoryData.zAxis.end() - 1 - vecSize;
            itLast = mTrt.lossHistoryData.zAxis.end() - 1;
            mTrtInfer01Canvas->getDataVector().assign( itFirst, itLast );
            mTrtInfer01Canvas->setVerticalMax( *std::max_element( std::begin( mTrtInfer01Canvas->getDataVector() ),
                                                                  std::end( mTrtInfer01Canvas->getDataVector() ),
                                                                  compareInferFnc ) );
            mTrtInfer01Canvas->update();

            uint16_t eventsDetected = checkInferEventsAboveThd();
            mTrtSettingsUi->EventsLabel->setText( QString::number( eventsDetected ) + " events above threshold" );
        }

        // 10 seconds avg.
        if( mTrtInfer10Canvas )
        {
            vecSize = mTrtInfer10Canvas->getDataVector().size();
            const uint8_t AVG_LEN = 10;
            uint8_t historyCounter = 0;
            uint16_t destCounter = 0;
            double avg = 0;

            for( int i = INFER_HISTORY_LENGTH - 1; i >= INFER_HISTORY_LENGTH - mTrtInfer10Size * AVG_LEN; i-- )
            {
                historyCounter++;
                avg += mTrt.lossHistoryData.zAxis.at( i );

                if( 0 == historyCounter % AVG_LEN )
                {
                    mTrtInfer10Canvas->getDataVector().at( vecSize - 1 - destCounter ) = avg / static_cast<double>( AVG_LEN );
                    destCounter++;
                    historyCounter = 0;
                    avg = 0;
                }
            }

            mTrtInfer10Canvas->setVerticalMax( *std::max_element( std::begin( mTrtInfer10Canvas->getDataVector() ),
                                                                  std::end( mTrtInfer10Canvas->getDataVector() ),
                                                                  compareInferFnc ) );
            if( 0 == inferIt % AVG_LEN )
            {
                mTrtInfer10Canvas->update();
            }
        }

        // 60 seconds avg.
        if( mTrtInfer60Canvas )
        {
            vecSize = mTrtInfer60Canvas->getDataVector().size();
            const uint8_t AVG_LEN = 60;
            uint8_t historyCounter = 0;
            uint16_t destCounter = 0;
            double avg = 0;

            for( int i = INFER_HISTORY_LENGTH - 1; i >= INFER_HISTORY_LENGTH - mTrtInfer60Size * AVG_LEN; i-- )
            {
                historyCounter++;
                avg += mTrt.lossHistoryData.zAxis.at( i );

                if( 0 == historyCounter % AVG_LEN )
                {
                    mTrtInfer60Canvas->getDataVector().at( vecSize - 1 - destCounter ) = avg / static_cast<double>( AVG_LEN );
                    destCounter++;
                    historyCounter = 0;
                    avg = 0;
                }
            }

            mTrtInfer60Canvas->setVerticalMax( *std::max_element( std::begin( mTrtInfer60Canvas->getDataVector() ),
                                                                  std::end( mTrtInfer60Canvas->getDataVector() ),
                                                                  compareInferFnc ) );
            if( 0 == inferIt % AVG_LEN )
            {
                mTrtInfer60Canvas->update();
                inferIt = 0;
            }
        }
    }
#endif


//!************************************************************************
//! Update data when stopping the current batch
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::stopBatch()
{
    mIsConfigSetupRunning = false;
    mDumpRemainingSamples = mConfig.samplesCount;

    if( mDumpCsvFile.is_open() )
    {
        mDumpCsvFile.close();
    }
}


//!************************************************************************
//! Update data when stopping a configuration
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::stopConfiguration()
{
    mConfigBatchesTimer->stop();
    mIsConfigSetupRunning = false;
    mDumpRemainingBatches = 0;
    mDumpRemainingSamples = 0;
    mConfig.batchesCount = 0;
    mConfig.samplesCount = 0;
    mConfig.batchesPeriod = 0;
    mConfig.nullifyActive = false;

    mMainUi->actionAccelerometer->setEnabled( true );

    if( mDumpCsvFile.is_open() )
    {
        mDumpCsvFile.close();
    }
}


//!************************************************************************
//! Update the controls from the accelerometer settings dialog
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updateAccelDialogControls()
{
    int rangeIndex = 0;
    int odrIndex = 0;
    double odrLpfCorner = 0;

    if( mAccelInstance )
    {
        uint16_t delayUs = getDaqDelayUs();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->pause();
            mDaqThread->usleep( delayUs );
        }

        Adxl355Adxl357Common::AccelerationRange crtRange = Adxl355Adxl357Common::ACCELERATION_RANGE_DEFAULT;
        mAccelInstance->getAccelerationRange( &crtRange );

        rangeIndex = static_cast<int>( crtRange -
#if BUILD_ADXL355
            Adxl355Adxl357Common::ACCELERATION_RANGE_2G
#elif BUILD_ADXL357
            Adxl355Adxl357Common::ACCELERATION_RANGE_10G
#endif
            );

        odrIndex = static_cast<int>( mAccelInstance->getOdrSetting() );
        odrLpfCorner = mAccelInstance->getOdrLpfCorner();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->resume();
        }
    }

    mAccelerometerUi->RangeComboBox->setCurrentIndex( rangeIndex );
    mAccelerometerUi->OdrComboBox->setCurrentIndex( odrIndex );
    mAccelerometerUi->LpfFrequencyValue->setText( QString::number( odrLpfCorner ) );
    mAccelerometerUi->TemperatureValue->setText( QString::number( mTemperature, 'f', 1 ) + " [" +  DEG + "C]" );
}


//!************************************************************************
//! Update the date & time information
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::updateDateTime()
{
    QDateTime crtDateTime = QDateTime::currentDateTime();
    QString dateTimeStr = crtDateTime.toString( "dd.MM.yyyy  hh:mm:ss" );
    mDateTimeStatusbarLabel.setText( dateTimeStr );
}


//!************************************************************************
//! Update the 2D plot options buttons for Select/Deselect All
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot2dOptionsButtonsX()
{
    bool allSelected = mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT )
                    && mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_FFT )
                    && mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM )
                    && ( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM );

    bool haveOne = mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT )
                || mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_FFT )
                || mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM )
                || mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_SRS )
                || mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM );

    mPlot2dOptionsUi->XaxisSelectAllButton->setEnabled( !allSelected );
    mPlot2dOptionsUi->XaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the 2D plot options buttons for Select/Deselect All
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot2dOptionsButtonsY()
{
    bool allSelected = mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT )
                    && mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_FFT )
                    && mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM )
                    && ( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM );

    bool haveOne = mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT )
                || mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_FFT )
                || mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM )
                || mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_SRS )
                || mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM );

    mPlot2dOptionsUi->YaxisSelectAllButton->setEnabled( !allSelected );
    mPlot2dOptionsUi->YaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the 2D plot options buttons for Select/Deselect All
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot2dOptionsButtonsZ()
{
    bool allSelected = mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT )
                    && mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_FFT )
                    && mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM )
                    && ( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM );

    bool haveOne = mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT )
                || mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_FFT )
                || mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM )
                || mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_SRS )
                || mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM );

    mPlot2dOptionsUi->ZaxisSelectAllButton->setEnabled( !allSelected );
    mPlot2dOptionsUi->ZaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the 2D plot options checkboxes
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot2dOptionsCheckBoxesX()
{
    mPlot2dOptionsUi->XaxisTransientCheckBox->setChecked( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) );
    mPlot2dOptionsUi->XaxisFftCheckBox->setChecked( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) );
    mPlot2dOptionsUi->XaxisPeriodogramCheckBox->setChecked( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) );
    mPlot2dOptionsUi->XaxisSrsCheckBox->setChecked( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) );
    mPlot2dOptionsUi->XaxisCepstrumCheckBox->setChecked( mPlot2dOptions.xAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update the 2D plot options checkboxes
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot2dOptionsCheckBoxesY()
{
    mPlot2dOptionsUi->YaxisTransientCheckBox->setChecked( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) );
    mPlot2dOptionsUi->YaxisFftCheckBox->setChecked( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) );
    mPlot2dOptionsUi->YaxisPeriodogramCheckBox->setChecked( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) );
    mPlot2dOptionsUi->YaxisSrsCheckBox->setChecked( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) );
    mPlot2dOptionsUi->YaxisCepstrumCheckBox->setChecked( mPlot2dOptions.yAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update the 2D plot options checkboxes
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot2dOptionsCheckBoxesZ()
{
    mPlot2dOptionsUi->ZaxisTransientCheckBox->setChecked( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_TRANSIENT ) );
    mPlot2dOptionsUi->ZaxisFftCheckBox->setChecked( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_FFT ) );
    mPlot2dOptionsUi->ZaxisPeriodogramCheckBox->setChecked( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_PERIODOGRAM ) );
    mPlot2dOptionsUi->ZaxisSrsCheckBox->setChecked( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_SRS ) );
    mPlot2dOptionsUi->ZaxisCepstrumCheckBox->setChecked( mPlot2dOptions.zAxis.at( Plot2d::PLOT_2D_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update the 3D plot options buttons for Select/Deselect All
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot3dOptionsButtonsX()
{
    bool allSelected = mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT )
                    && mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_FFT )
                    && mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM )
                    && ( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM );

    bool haveOne = mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT )
                || mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_FFT )
                || mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM )
                || mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_SRS )
                || mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM );

    mPlot3dOptionsUi->XaxisSelectAllButton->setEnabled( !allSelected );
    mPlot3dOptionsUi->XaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the 3D plot options buttons for Select/Deselect All
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot3dOptionsButtonsY()
{
    bool allSelected = mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT )
                    && mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_FFT )
                    && mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM )
                    && ( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM );

    bool haveOne = mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT )
                || mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_FFT )
                || mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM )
                || mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_SRS )
                || mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM );

    mPlot3dOptionsUi->YaxisSelectAllButton->setEnabled( !allSelected );
    mPlot3dOptionsUi->YaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the 3D plot options buttons for Select/Deselect All
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot3dOptionsButtonsZ()
{
    bool allSelected = mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT )
                    && mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_FFT )
                    && mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM )
                    && ( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM );

    bool haveOne = mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT )
                || mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_FFT )
                || mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM )
                || mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_SRS )
                || mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM );

    mPlot3dOptionsUi->ZaxisSelectAllButton->setEnabled( !allSelected );
    mPlot3dOptionsUi->ZaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the 3D plot options checkboxes
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot3dOptionsCheckBoxesX()
{
    mPlot3dOptionsUi->XaxisTransientCheckBox->setChecked( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) );
    mPlot3dOptionsUi->XaxisFftCheckBox->setChecked( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) );
    mPlot3dOptionsUi->XaxisPeriodogramCheckBox->setChecked( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) );
    mPlot3dOptionsUi->XaxisSrsCheckBox->setChecked( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) );
    mPlot3dOptionsUi->XaxisCepstrumCheckBox->setChecked( mPlot3dOptions.xAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update the 3D plot options checkboxes
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot3dOptionsCheckBoxesY()
{
    mPlot3dOptionsUi->YaxisTransientCheckBox->setChecked( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) );
    mPlot3dOptionsUi->YaxisFftCheckBox->setChecked( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) );
    mPlot3dOptionsUi->YaxisPeriodogramCheckBox->setChecked( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) );
    mPlot3dOptionsUi->YaxisSrsCheckBox->setChecked( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) );
    mPlot3dOptionsUi->YaxisCepstrumCheckBox->setChecked( mPlot3dOptions.yAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update the 3D plot options checkboxes
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlot3dOptionsCheckBoxesZ()
{
    mPlot3dOptionsUi->ZaxisTransientCheckBox->setChecked( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_TRANSIENT ) );
    mPlot3dOptionsUi->ZaxisFftCheckBox->setChecked( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_FFT ) );
    mPlot3dOptionsUi->ZaxisPeriodogramCheckBox->setChecked( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_PERIODOGRAM ) );
    mPlot3dOptionsUi->ZaxisSrsCheckBox->setChecked( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_SRS ) );
    mPlot3dOptionsUi->ZaxisCepstrumCheckBox->setChecked( mPlot3dOptions.zAxis.at( Plot3d::PLOT_3D_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update parameters concerning all 2D and 3D plots
//! - FFT SPS [Hz]
//! - FFT size
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlotsFrequencyParams()
{
    double sps = mFreqAnalysisInstance->getFftSps();
    uint32_t fftSize = mFreqAnalysisInstance->getFftSizeValue();

    //////////////////////////
    /// X axis
    //////////////////////////
    // 2D
    if( mPlot2dXtransient )
    {
        mPlot2dXtransient->setParameters( sps, fftSize );
    }

    if( mPlot2dXfft )
    {
        mPlot2dXfft->setParameters( sps, fftSize );
    }

    if( mPlot2dXperiodogram )
    {
        mPlot2dXperiodogram->setParameters( sps, fftSize );
    }

    if( mPlot2dXsrs )
    {
        mPlot2dXsrs->setParameters( sps, fftSize );
    }

    if( mPlot2dXcepstrum )
    {
        mPlot2dXcepstrum->setParameters( sps, fftSize );
    }

    // 3D
    if( mPlot3dXtransient )
    {
        mPlot3dXtransient->setParameters( sps, fftSize );
    }

    if( mPlot3dXfft )
    {
        mPlot3dXfft->setParameters( sps, fftSize );
    }

    if( mPlot3dXperiodogram )
    {
        mPlot3dXperiodogram->setParameters( sps, fftSize );
    }

    if( mPlot3dXsrs )
    {
        mPlot3dXsrs->setParameters( sps, fftSize );
    }

    if( mPlot3dXcepstrum )
    {
        mPlot3dXcepstrum->setParameters( sps, fftSize );
    }

    //////////////////////////
    /// Y axis
    //////////////////////////
    // 2D
    if( mPlot2dYtransient )
    {
        mPlot2dYtransient->setParameters( sps, fftSize );
    }

    if( mPlot2dYfft )
    {
        mPlot2dYfft->setParameters( sps, fftSize );
    }

    if( mPlot2dYperiodogram )
    {
        mPlot2dYperiodogram->setParameters( sps, fftSize );
    }

    if( mPlot2dYsrs )
    {
        mPlot2dYsrs->setParameters( sps, fftSize );
    }

    if( mPlot2dYcepstrum )
    {
        mPlot2dYcepstrum->setParameters( sps, fftSize );
    }

    // 3D
    if( mPlot3dYtransient )
    {
        mPlot3dYtransient->setParameters( sps, fftSize );
    }

    if( mPlot3dYfft )
    {
        mPlot3dYfft->setParameters( sps, fftSize );
    }

    if( mPlot3dYperiodogram )
    {
        mPlot3dYperiodogram->setParameters( sps, fftSize );
    }

    if( mPlot3dYsrs )
    {
        mPlot3dYsrs->setParameters( sps, fftSize );
    }

    if( mPlot3dYcepstrum )
    {
        mPlot3dYcepstrum->setParameters( sps, fftSize );
    }

    //////////////////////////
    /// Z axis
    //////////////////////////
    // 2D
    if( mPlot2dZtransient )
    {
        mPlot2dZtransient->setParameters( sps, fftSize );
    }

    if( mPlot2dZfft )
    {
        mPlot2dZfft->setParameters( sps, fftSize );
    }

    if( mPlot2dZperiodogram )
    {
        mPlot2dZperiodogram->setParameters( sps, fftSize );
    }

    if( mPlot2dZsrs )
    {
        mPlot2dZsrs->setParameters( sps, fftSize );
    }

    if( mPlot2dZcepstrum )
    {
        mPlot2dZcepstrum->setParameters( sps, fftSize );
    }

    // 3D
    if( mPlot3dZtransient )
    {
        mPlot3dZtransient->setParameters( sps, fftSize );
    }

    if( mPlot3dZfft )
    {
        mPlot3dZfft->setParameters( sps, fftSize );
    }

    if( mPlot3dZperiodogram )
    {
        mPlot3dZperiodogram->setParameters( sps, fftSize );
    }

    if( mPlot3dZsrs )
    {
        mPlot3dZsrs->setParameters( sps, fftSize );
    }

    if( mPlot3dZcepstrum )
    {
        mPlot3dZcepstrum->setParameters( sps, fftSize );
    }
}


//!************************************************************************
//! Update the max vertical limit in transient 2D and 3D Plot objects
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlotsVerticalMaxTransient()
{
    double verticalMax = 0;

    if( mAccelInstance )
    {
        Adxl355Adxl357Common::AccelerationRange accelRange;

        if( mAccelInstance->getAccelerationRange( &accelRange ) )
        {
            switch( accelRange )
            {
                // ADXL355 only
                case Adxl355Adxl357Common::ACCELERATION_RANGE_2G:
                    verticalMax = 2;
                    break;

                case Adxl355Adxl357Common::ACCELERATION_RANGE_4G:
                    verticalMax = 4;
                    break;

                case Adxl355Adxl357Common::ACCELERATION_RANGE_8G:
                    verticalMax = 8;
                    break;

                // ADXL357 only
                case Adxl355Adxl357Common::ACCELERATION_RANGE_10G:
                    verticalMax = 10;
                    break;

                case Adxl355Adxl357Common::ACCELERATION_RANGE_20G:
                    verticalMax = 20;
                    break;

                case Adxl355Adxl357Common::ACCELERATION_RANGE_40G:
                    verticalMax = 40;
                    break;

                default:
                    break;
            }
        }
    }

    //////////////////////////
    /// X axis
    //////////////////////////
    if( mPlot2dXtransient )
    {
        mPlot2dXtransient->setVerticalMaxTransient( verticalMax );
    }

    if( mPlot3dXtransient )
    {
        mPlot3dXtransient->setVerticalMaxTransient( verticalMax );
    }

    //////////////////////////
    /// Y axis
    //////////////////////////
    if( mPlot2dYtransient )
    {
        mPlot2dYtransient->setVerticalMaxTransient( verticalMax );
    }

    if( mPlot3dYtransient )
    {
        mPlot3dYtransient->setVerticalMaxTransient( verticalMax );
    }

    //////////////////////////
    /// Z axis
    //////////////////////////
    if( mPlot2dZtransient )
    {
        mPlot2dZtransient->setVerticalMaxTransient( verticalMax );
    }

    if( mPlot3dZtransient )
    {
        mPlot3dZtransient->setVerticalMaxTransient( verticalMax );
    }
}


#if BUILD_CUDA
    //!************************************************************************
    //! Update the controls from the TensorRT settings dialog
    //!
    //! @returns: nothing
    //!************************************************************************
    void SpectralViewer::updateTrtSettingsDialogControls()
    {
        // inference status
        mTrtSettingsUi->StatusValue->setText( mTrt.isEnabled ? "ENABLED" : "DISABLED" );

        // inference loss threshold
        mTrtSettingsUi->LossThdSpinBox->setValue( mTrt.lossThd.zAxis );

        // full log
        mTrtSettingsUi->LogWidget->clear();

        for( int i = 0; i < mTrt.cbmOnnx.getTrtlog().size(); i++ )
        {
            mTrtSettingsUi->LogWidget->addItem( QString::fromStdString( mTrt.cbmOnnx.getTrtlog().at( i ) ) );
        }
    }
#endif
