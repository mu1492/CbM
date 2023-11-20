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
SpectralViewer.cpp

This file contains the sources for the mechanical vibrations spectral viewer.
*/

#include "SpectralViewer.h"
#include "./ui_SpectralViewer.h"

#include "CbmCanvas.h"
#include "DaqThread.h"
#include "FrequencyAnalysis.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <list>
#include <map>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <QMessageBox>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTime>
#include <QTimer>


const QString SpectralViewer::APP_NAME = "Spectral Viewer";

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
    , mPlotOptionsUi( new Ui::PlotOptionsDialog )
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
#endif
    // paint&draw
    , mCbmCanvas( nullptr )
    // X axis plots
    , mPlotXtransient( nullptr )
    , mPlotXfft( nullptr )
    , mPlotXperiodogram( nullptr )
    , mPlotXsrs( nullptr )
    , mPlotXcepstrum( nullptr )
    // Y axis plots
    , mPlotYtransient( nullptr )
    , mPlotYfft( nullptr )
    , mPlotYperiodogram( nullptr )
    , mPlotYsrs( nullptr )
    , mPlotYcepstrum( nullptr )
    // Z axis plots
    , mPlotZtransient( nullptr )
    , mPlotZfft( nullptr )
    , mPlotZperiodogram( nullptr )
    , mPlotZsrs( nullptr )
    , mPlotZcepstrum( nullptr )
{
    mMainUi->setupUi( this );
    mAccelerometerUi->setupUi( &mAccelerometerDlg );
    mFrequencyAnalysisUi->setupUi( &mFrequencyAnalysisDlg );
    mVibrationMonitoringSettingsUi->setupUi( &mVibrationMonitoringSettingsDlg );
    mPlotOptionsUi->setupUi( &mPlotOptionsDlg );

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
        bool initStatus = mAccelInstance->init( mI2cBusChannel, Adxl355::ADXL355_357_I2C_ADDRESS_PRIMARY );

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
    connect( mMainUi->actionExit, &QAction::triggered, this, &SpectralViewer::handleExit );

    mMainUi->actionAccelerometer->setText( ACCEL_SETTINGS_STR );

    connect( mMainUi->actionAccelerometer, &QAction::triggered, this, &SpectralViewer::handleMenuAccel );
    initAccelDialogControls();

    connect( mMainUi->actionFrequencyAnalysis, &QAction::triggered, this, &SpectralViewer::handleMenuFreqAnalysis );
    initFreqAnalysisDialogControls();

    connect( mMainUi->actionVibrationMonitoring, &QAction::triggered, this, &SpectralViewer::handleMenuVibMon );
    initVibMonDialogControls();

    connect( mMainUi->actionSelectPlots, &QAction::triggered, this, &SpectralViewer::handleMenuSelectPlots );
    initPlotOptionsControls();

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
}


//!************************************************************************
//! Destructor
//!************************************************************************
SpectralViewer::~SpectralViewer()
{
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
    cudaDeviceReset();
#endif

    delete mMainUi;
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

            if( ( CUDA_CC_MIN.major < ccMajor )
             || ( CUDA_CC_MIN.major == ccMajor && CUDA_CC_MIN.minor <= ccMinor ) )
            {
                ccOk = true;
                const int MAX_NAME_LEN = 256;
                char deviceName[MAX_NAME_LEN] = "";
                checkCudaErrors( cuDeviceGetName( deviceName, MAX_NAME_LEN, dev ) );

                mCudaInfo.devId = dev;
                mCudaInfo.cc.major = ccMajor;
                mCudaInfo.cc.minor = ccMinor;
                mCudaInfo.name = deviceName;

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

                msg = "Minimum CUDA requirements were not found on this platform:";
                msg += "\n - minimum CUDA required is " + cudaVerRequiredStr + ", found is " + cudaVerFoundStr;
                msg += "\n - minimum CC required is " + cudaCcRequiredStr + ", found is " + cudaCcFoundStr;
                msg += "\n\nCUDA will not be used and computations will run in the CPU only.";
            }

            QMessageBox::warning( this, "SpectralViewer - CUDA", msg, QMessageBox::Ok );
        }
    }
#endif


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
    if( mAccelInstance )
    {
        Adxl355Adxl357Common::AccelerationRange range = static_cast<Adxl355Adxl357Common::AccelerationRange>( aIndex +
#if BUILD_ADXL355
            Adxl355Adxl357Common::ACCELERATION_RANGE_2G
#elif BUILD_ADXL355
            Adxl355Adxl357Common::ACCELERATION_RANGE_10G
#endif
        );

        uint16_t delayUs = getDaqDelayUs();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->pause();
            mDaqThread->usleep( delayUs );
        }

        mAccelInstance->setAccelerationRange( range );
        updatePlotsVerticalMaxTransient();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->resume();
        }
    }
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
    if( mAccelInstance )
    {
        uint16_t delayUs = getDaqDelayUs();

        if( mDaqThread && mDaqThread->isRunning() )
        {
            mDaqThread->pause();
            mDaqThread->usleep( delayUs );
        }

        mAccelInstance->setOdr( static_cast<Adxl355Adxl357Common::OdrSetting>( aIndex ) );        
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
        if( mPlotXsrs )
        {
            mPlotXsrs->close();
        }

        mPlotOptions.xAxis.at( Plot::PLOT_TYPE_SRS ) = false;

        if( mPlotYsrs )
        {
            mPlotYsrs->close();
        }

        mPlotOptions.yAxis.at( Plot::PLOT_TYPE_SRS ) = false;

        if( mPlotZsrs )
        {
            mPlotZsrs->close();
        }

        mPlotOptions.zAxis.at( Plot::PLOT_TYPE_SRS ) = false;
    }

    updatePlotOptionsButtonsX();
    updatePlotOptionsButtonsY();
    updatePlotOptionsButtonsZ();
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
//! Handle for changing a plot option
//! X axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionXtransient
    (
    bool aEnabled
    )
{
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = aEnabled;
    updatePlotOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a plot option
//! X axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionXfft
    (
    bool aEnabled
    )
{
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_FFT ) = aEnabled;
    updatePlotOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a plot option
//! X axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionXperiodogram
    (
    bool aEnabled
    )
{
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlotOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a plot option
//! X axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionXsrs
    (
    bool aEnabled
    )
{
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_SRS ) = aEnabled;
    updatePlotOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a plot option
//! X axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionXcepstrum
    (
    bool aEnabled
    )
{
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = aEnabled;
    updatePlotOptionsButtonsX();
}


//!************************************************************************
//! Handle for changing a plot option
//! Y axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionYtransient
    (
    bool aEnabled
    )
{
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = aEnabled;
    updatePlotOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a plot option
//! Y axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionYfft
    (
    bool aEnabled
    )
{
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_FFT ) = aEnabled;
    updatePlotOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a plot option
//! Y axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionYperiodogram
    (
    bool aEnabled
    )
{
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlotOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a plot option
//! Y axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionYsrs
    (
    bool aEnabled
    )
{
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_SRS ) = aEnabled;
    updatePlotOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a plot option
//! Y axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionYcepstrum
    (
    bool aEnabled
    )
{
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = aEnabled;
    updatePlotOptionsButtonsY();
}


//!************************************************************************
//! Handle for changing a plot option
//! Z axis / transient
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionZtransient
    (
    bool aEnabled
    )
{
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = aEnabled;
    updatePlotOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a plot option
//! Z axis / FFT
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionZfft
    (
    bool aEnabled
    )
{
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_FFT ) = aEnabled;
    updatePlotOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a plot option
//! Z axis / periodogram
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionZperiodogram
    (
    bool aEnabled
    )
{
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = aEnabled;
    updatePlotOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a plot option
//! Z axis / SRS
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionZsrs
    (
    bool aEnabled
    )
{
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_SRS ) = aEnabled;
    updatePlotOptionsButtonsZ();
}


//!************************************************************************
//! Handle for changing a plot option
//! Z axis / cepstrum
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleChangedPlotOptionZcepstrum
    (
    bool aEnabled
    )
{
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = aEnabled;
    updatePlotOptionsButtonsZ();
}


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
//! Handle for plotting options
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handleMenuSelectPlots()
{
    updatePlotOptionsCheckBoxesX();
    updatePlotOptionsCheckBoxesY();
    updatePlotOptionsCheckBoxesZ();

    bool isSrsRunning = mFreqAnalysisInstance->getSrsIsRunning();
    mPlotOptionsUi->XaxisSrsCheckBox->setEnabled( isSrsRunning );
    mPlotOptionsUi->YaxisSrsCheckBox->setEnabled( isSrsRunning );
    mPlotOptionsUi->ZaxisSrsCheckBox->setEnabled( isSrsRunning );

    mPlotOptionsDlg.exec();
}


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
//! This is useful if static accelerations, e.g. due to Earth's gravity,
//! need to be subtracted for all following readings. After this action only
//! dynamic accelerations will be read.
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
            const uint8_t SAMPLES_COUNT = 10;
            bool readOk = true;

            for( uint8_t i = 0; i < SAMPLES_COUNT; i++ )
            {
                double xAxisValue = 0;
                double yAxisValue = 0;
                double zAxisValue = 0;
                readOk = mAccelInstance->getAccelerationsOnAllAxes( &xAxisValue, &yAxisValue, &zAxisValue );

                if( readOk )
                {
                    xAxisAvg += xAxisValue;
                    yAxisAvg += yAxisValue;
                    zAxisAvg += zAxisValue;
                }
                else
                {
                    break;
                }
            }

            if( readOk )
            {
                xAxisAvg /= SAMPLES_COUNT;
                yAxisAvg /= SAMPLES_COUNT;
                zAxisAvg /= SAMPLES_COUNT;

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

        if( statusOk )
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
//! Handle for closing the plot options
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlotOptionsClosed()
{
    //////////////////////////
    /// X axis
    //////////////////////////
    if( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_TRANSIENT ) )
    {
        if( !mPlotXtransient )
        {
            mPlotXtransient = new Plot( this, Plot::PLOT_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_X );
            connect( mPlotXtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlotXtransient, SLOT( receiveNewData() ) );
        }

        if( mPlotXtransient->isHidden() )
        {
            mPlotXtransient->show();
        }
    }
    else if( mPlotXtransient )
    {
        mPlotXtransient->close();
    }


    if( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_FFT ) )
    {
        if( !mPlotXfft )
        {
            mPlotXfft = new Plot( this, Plot::PLOT_TYPE_FFT, Adxl355Adxl357Common::AXIS_X );
            connect( mPlotXfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlotXfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlotXfft->isHidden() )
        {
            mPlotXfft->show();
        }
    }
    else if( mPlotXfft )
    {
        mPlotXfft->close();
    }


    if( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) )
    {
        if( !mPlotXperiodogram )
        {
            mPlotXperiodogram = new Plot( this, Plot::PLOT_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_X );
            connect( mPlotXperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlotXperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlotXperiodogram->isHidden() )
        {
            mPlotXperiodogram->show();
        }
    }
    else if( mPlotXperiodogram )
    {
        mPlotXperiodogram->close();
    }


    if( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_SRS ) )
    {
        if( !mPlotXsrs )
        {
            mPlotXsrs = new Plot( this, Plot::PLOT_TYPE_SRS, Adxl355Adxl357Common::AXIS_X );
            connect( mPlotXsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlotXsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlotXsrs->isHidden() )
        {
            mPlotXsrs->show();
        }
    }
    else if( mPlotXsrs )
    {
        mPlotXsrs->close();
    }


    if( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) )
    {
        if( !mPlotXcepstrum )
        {
            mPlotXcepstrum = new Plot( this, Plot::PLOT_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_X );
            connect( mPlotXcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlotXcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlotXcepstrum->isHidden() )
        {
            mPlotXcepstrum->show();
        }
    }
    else if( mPlotXcepstrum )
    {
        mPlotXcepstrum->close();
    }


    //////////////////////////
    /// Y axis
    //////////////////////////
    if( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_TRANSIENT ) )
    {
        if( !mPlotYtransient )
        {
            mPlotYtransient = new Plot( this, Plot::PLOT_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlotYtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlotYtransient, SLOT( receiveNewData() ) );
        }

        if( mPlotYtransient->isHidden() )
        {
            mPlotYtransient->show();
        }
    }
    else if( mPlotYtransient )
    {
        mPlotYtransient->close();
    }


    if( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_FFT ) )
    {
        if( !mPlotYfft )
        {
            mPlotYfft = new Plot( this, Plot::PLOT_TYPE_FFT, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlotYfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlotYfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlotYfft->isHidden() )
        {
            mPlotYfft->show();
        }
    }
    else if( mPlotYfft )
    {
        mPlotYfft->close();
    }


    if( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) )
    {
        if( !mPlotYperiodogram )
        {
            mPlotYperiodogram = new Plot( this, Plot::PLOT_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlotYperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlotYperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlotYperiodogram->isHidden() )
        {
            mPlotYperiodogram->show();
        }
    }
    else if( mPlotYperiodogram )
    {
        mPlotYperiodogram->close();
    }


    if( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_SRS ) )
    {
        if( !mPlotYsrs )
        {
            mPlotYsrs = new Plot( this, Plot::PLOT_TYPE_SRS, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlotYsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlotYsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlotYsrs->isHidden() )
        {
            mPlotYsrs->show();
        }
    }
    else if( mPlotYsrs )
    {
        mPlotYsrs->close();
    }


    if( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) )
    {
        if( !mPlotYcepstrum )
        {
            mPlotYcepstrum = new Plot( this, Plot::PLOT_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_Y );
            connect( mPlotYcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlotYcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlotYcepstrum->isHidden() )
        {
            mPlotYcepstrum->show();
        }
    }
    else if( mPlotYcepstrum )
    {
        mPlotYcepstrum->close();
    }


    //////////////////////////
    /// Z axis
    //////////////////////////
    if( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_TRANSIENT ) )
    {
        if( !mPlotZtransient )
        {
            mPlotZtransient = new Plot( this, Plot::PLOT_TYPE_TRANSIENT, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlotZtransient, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewData() ), mPlotZtransient, SLOT( receiveNewData() ) );
        }

        if( mPlotZtransient->isHidden() )
        {
            mPlotZtransient->show();
        }
    }
    else if( mPlotZtransient )
    {
        mPlotZtransient->close();
    }


    if( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_FFT ) )
    {
        if( !mPlotZfft )
        {
            mPlotZfft = new Plot( this, Plot::PLOT_TYPE_FFT, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlotZfft, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewFftBins(int) ), mPlotZfft, SLOT( receiveNewFft(int) ) );
        }

        if( mPlotZfft->isHidden() )
        {
            mPlotZfft->show();
        }
    }
    else if( mPlotZfft )
    {
        mPlotZfft->close();
    }


    if( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) )
    {
        if( !mPlotZperiodogram )
        {
            mPlotZperiodogram = new Plot( this, Plot::PLOT_TYPE_PERIODOGRAM, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlotZperiodogram, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewPeriodogram(int) ), mPlotZperiodogram, SLOT( receiveNewPeriodogram(int) ) );
        }

        if( mPlotZperiodogram->isHidden() )
        {
            mPlotZperiodogram->show();
        }
    }
    else if( mPlotZperiodogram )
    {
        mPlotZperiodogram->close();
    }


    if( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_SRS ) )
    {
        if( !mPlotZsrs )
        {
            mPlotZsrs = new Plot( this, Plot::PLOT_TYPE_SRS, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlotZsrs, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewSrs(int) ), mPlotZsrs, SLOT( receiveNewSrs(int) ) );
        }

        if( mPlotZsrs->isHidden() )
        {
            mPlotZsrs->show();
        }
    }
    else if( mPlotZsrs )
    {
        mPlotZsrs->close();
    }


    if( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) )
    {
        if( !mPlotZcepstrum )
        {
            mPlotZcepstrum = new Plot( this, Plot::PLOT_TYPE_CEPSTRUM, Adxl355Adxl357Common::AXIS_Z );
            connect( mPlotZcepstrum, SIGNAL( closeSignal(int, int) ), this, SLOT( receiveClosedPlot(int, int) ) );
            connect( this->mVibrationHndlInstance, SIGNAL( haveNewCepstrum(int) ), mPlotZcepstrum, SLOT( receiveNewCepstrum(int) ) );
        }

        if( mPlotZcepstrum->isHidden() )
        {
            mPlotZcepstrum->show();
        }
    }
    else if( mPlotZcepstrum )
    {
        mPlotZcepstrum->close();
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

    mPlotOptionsDlg.close();
}


//!************************************************************************
//! Handle for deselecting all plots
//! *** X axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlotOptionsXDeselectAll()
{
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = false;
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_FFT ) = false;
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = false;
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_SRS ) = false;
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = false;

    updatePlotOptionsCheckBoxesX();
}


//!************************************************************************
//! Handle for selecting all plots
//! *** X axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlotOptionsXSelectAll()
{
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = true;
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_FFT ) = true;
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = true;
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlotOptions.xAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = true;

    updatePlotOptionsCheckBoxesX();
}


//!************************************************************************
//! Handle for deselecting all plots
//! *** Y axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlotOptionsYDeselectAll()
{
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = false;
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_FFT ) = false;
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = false;
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_SRS ) = false;
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = false;

    updatePlotOptionsCheckBoxesY();
}


//!************************************************************************
//! Handle for selecting all plots
//! *** Y axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlotOptionsYSelectAll()
{
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = true;
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_FFT ) = true;
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = true;
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlotOptions.yAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = true;

    updatePlotOptionsCheckBoxesY();
}


//!************************************************************************
//! Handle for deselecting all plots
//! *** Z axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlotOptionsZDeselectAll()
{
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = false;
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_FFT ) = false;
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = false;
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_SRS ) = false;
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = false;

    updatePlotOptionsCheckBoxesZ();
}


//!************************************************************************
//! Handle for selecting all plots
//! *** Z axis ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void SpectralViewer::handlePlotOptionsZSelectAll()
{
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_TRANSIENT ) = true;
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_FFT ) = true;
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) = true;
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_SRS ) = mFreqAnalysisInstance->getSrsIsRunning();
    mPlotOptions.zAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) = true;

    updatePlotOptionsCheckBoxesZ();
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
                statusOk = true;

                if( mAccelerometerDlg.isVisible() )
                {
                    updateAccelDialogControls();
                }
            }
        }

        if( statusOk )
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
//! Initialize the controls from the plot options dialog
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::initPlotOptionsControls()
{
    mPlotOptions.xAxis =
    {
        { Plot::PLOT_TYPE_TRANSIENT,   false },
        { Plot::PLOT_TYPE_FFT,         false },
        { Plot::PLOT_TYPE_PERIODOGRAM, false },
        { Plot::PLOT_TYPE_SRS,         false },
        { Plot::PLOT_TYPE_CEPSTRUM,    false }
    };

    mPlotOptions.yAxis = mPlotOptions.xAxis;
    mPlotOptions.zAxis = mPlotOptions.xAxis;

    updatePlotOptionsButtonsX();
    updatePlotOptionsButtonsY();
    updatePlotOptionsButtonsZ();

    //////////////////////////
    /// X axis
    //////////////////////////
    connect( mPlotOptionsUi->XaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionXtransient( bool ) ) );
    connect( mPlotOptionsUi->XaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionXfft( bool ) ) );
    connect( mPlotOptionsUi->XaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionXperiodogram( bool ) ) );
    connect( mPlotOptionsUi->XaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionXsrs( bool ) ) );
    connect( mPlotOptionsUi->XaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionXcepstrum( bool ) ) );

    connect( mPlotOptionsUi->XaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlotOptionsXSelectAll() ) );
    connect( mPlotOptionsUi->XaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlotOptionsXDeselectAll() ) );

    //////////////////////////
    /// Y axis
    //////////////////////////
    connect( mPlotOptionsUi->YaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionYtransient( bool ) ) );
    connect( mPlotOptionsUi->YaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionYfft( bool ) ) );
    connect( mPlotOptionsUi->YaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionYperiodogram( bool ) ) );
    connect( mPlotOptionsUi->YaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionYsrs( bool ) ) );
    connect( mPlotOptionsUi->YaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionYcepstrum( bool ) ) );

    connect( mPlotOptionsUi->YaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlotOptionsYSelectAll() ) );
    connect( mPlotOptionsUi->YaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlotOptionsYDeselectAll() ) );

    //////////////////////////
    /// Z axis
    //////////////////////////
    connect( mPlotOptionsUi->ZaxisTransientCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionZtransient( bool ) ) );
    connect( mPlotOptionsUi->ZaxisFftCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionZfft( bool ) ) );
    connect( mPlotOptionsUi->ZaxisPeriodogramCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionZperiodogram( bool ) ) );
    connect( mPlotOptionsUi->ZaxisSrsCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionZsrs( bool ) ) );
    connect( mPlotOptionsUi->ZaxisCepstrumCheckBox, SIGNAL( toggled(bool) ), this, SLOT( handleChangedPlotOptionZcepstrum( bool ) ) );

    connect( mPlotOptionsUi->ZaxisSelectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlotOptionsZSelectAll() ) );
    connect( mPlotOptionsUi->ZaxisDeselectAllButton, SIGNAL( clicked() ), this, SLOT( handlePlotOptionsZDeselectAll() ) );

    connect( mPlotOptionsUi->OkButton, SIGNAL( clicked() ), this, SLOT( handlePlotOptionsClosed() ) );
}


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
//! Receive closed event from a plot window
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void SpectralViewer::receiveClosedPlot
    (
    int aType,          //!< type of closed window
    int aAxis           //!< axis of closed window
    )
{
    Plot::PlotType type = static_cast<Plot::PlotType>( aType );
    Adxl355Adxl357Common::Axis axis = static_cast<Adxl355Adxl357Common::Axis>( aAxis );

    switch( axis )
    {
        case Adxl355Adxl357Common::AXIS_X:
            mPlotOptions.xAxis.at( type ) = false;
            break;

        case Adxl355Adxl357Common::AXIS_Y:
            mPlotOptions.yAxis.at( type ) = false;
            break;

        case Adxl355Adxl357Common::AXIS_Z:
            mPlotOptions.zAxis.at( type ) = false;
            break;

        default:
            break;
    }
}


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
//! Update the plot options buttons for Select/Deselect All
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlotOptionsButtonsX()
{
    bool allSelected = mPlotOptions.xAxis.at( Plot::PLOT_TYPE_TRANSIENT )
                    && mPlotOptions.xAxis.at( Plot::PLOT_TYPE_FFT )
                    && mPlotOptions.xAxis.at( Plot::PLOT_TYPE_PERIODOGRAM )
                    && ( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlotOptions.xAxis.at( Plot::PLOT_TYPE_CEPSTRUM );

    bool haveOne = mPlotOptions.xAxis.at( Plot::PLOT_TYPE_TRANSIENT )
                || mPlotOptions.xAxis.at( Plot::PLOT_TYPE_FFT )
                || mPlotOptions.xAxis.at( Plot::PLOT_TYPE_PERIODOGRAM )
                || mPlotOptions.xAxis.at( Plot::PLOT_TYPE_SRS )
                || mPlotOptions.xAxis.at( Plot::PLOT_TYPE_CEPSTRUM );

    mPlotOptionsUi->XaxisSelectAllButton->setEnabled( !allSelected );
    mPlotOptionsUi->XaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the plot options buttons for Select/Deselect All
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlotOptionsButtonsY()
{
    bool allSelected = mPlotOptions.yAxis.at( Plot::PLOT_TYPE_TRANSIENT )
                    && mPlotOptions.yAxis.at( Plot::PLOT_TYPE_FFT )
                    && mPlotOptions.yAxis.at( Plot::PLOT_TYPE_PERIODOGRAM )
                    && ( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlotOptions.yAxis.at( Plot::PLOT_TYPE_CEPSTRUM );

    bool haveOne = mPlotOptions.yAxis.at( Plot::PLOT_TYPE_TRANSIENT )
                || mPlotOptions.yAxis.at( Plot::PLOT_TYPE_FFT )
                || mPlotOptions.yAxis.at( Plot::PLOT_TYPE_PERIODOGRAM )
                || mPlotOptions.yAxis.at( Plot::PLOT_TYPE_SRS )
                || mPlotOptions.yAxis.at( Plot::PLOT_TYPE_CEPSTRUM );

    mPlotOptionsUi->YaxisSelectAllButton->setEnabled( !allSelected );
    mPlotOptionsUi->YaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the plot options buttons for Select/Deselect All
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlotOptionsButtonsZ()
{
    bool allSelected = mPlotOptions.zAxis.at( Plot::PLOT_TYPE_TRANSIENT )
                    && mPlotOptions.zAxis.at( Plot::PLOT_TYPE_FFT )
                    && mPlotOptions.zAxis.at( Plot::PLOT_TYPE_PERIODOGRAM )
                    && ( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_SRS ) || !mFreqAnalysisInstance->getSrsIsRunning() )
                    && mPlotOptions.zAxis.at( Plot::PLOT_TYPE_CEPSTRUM );

    bool haveOne = mPlotOptions.zAxis.at( Plot::PLOT_TYPE_TRANSIENT )
                || mPlotOptions.zAxis.at( Plot::PLOT_TYPE_FFT )
                || mPlotOptions.zAxis.at( Plot::PLOT_TYPE_PERIODOGRAM )
                || mPlotOptions.zAxis.at( Plot::PLOT_TYPE_SRS )
                || mPlotOptions.zAxis.at( Plot::PLOT_TYPE_CEPSTRUM );

    mPlotOptionsUi->ZaxisSelectAllButton->setEnabled( !allSelected );
    mPlotOptionsUi->ZaxisDeselectAllButton->setEnabled( haveOne );
}


//!************************************************************************
//! Update the plot options checkboxes
//! *** X axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlotOptionsCheckBoxesX()
{
    mPlotOptionsUi->XaxisTransientCheckBox->setChecked( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_TRANSIENT ) );
    mPlotOptionsUi->XaxisFftCheckBox->setChecked( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_FFT ) );
    mPlotOptionsUi->XaxisPeriodogramCheckBox->setChecked( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) );
    mPlotOptionsUi->XaxisSrsCheckBox->setChecked( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_SRS ) );
    mPlotOptionsUi->XaxisCepstrumCheckBox->setChecked( mPlotOptions.xAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update the plot options checkboxes
//! *** Y axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlotOptionsCheckBoxesY()
{
    mPlotOptionsUi->YaxisTransientCheckBox->setChecked( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_TRANSIENT ) );
    mPlotOptionsUi->YaxisFftCheckBox->setChecked( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_FFT ) );
    mPlotOptionsUi->YaxisPeriodogramCheckBox->setChecked( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) );
    mPlotOptionsUi->YaxisSrsCheckBox->setChecked( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_SRS ) );
    mPlotOptionsUi->YaxisCepstrumCheckBox->setChecked( mPlotOptions.yAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update the plot options checkboxes
//! *** Z axis ***
//!
//! @returns: nothing
//!************************************************************************
void SpectralViewer::updatePlotOptionsCheckBoxesZ()
{
    mPlotOptionsUi->ZaxisTransientCheckBox->setChecked( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_TRANSIENT ) );
    mPlotOptionsUi->ZaxisFftCheckBox->setChecked( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_FFT ) );
    mPlotOptionsUi->ZaxisPeriodogramCheckBox->setChecked( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_PERIODOGRAM ) );
    mPlotOptionsUi->ZaxisSrsCheckBox->setChecked( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_SRS ) );
    mPlotOptionsUi->ZaxisCepstrumCheckBox->setChecked( mPlotOptions.zAxis.at( Plot::PLOT_TYPE_CEPSTRUM ) );
}


//!************************************************************************
//! Update parameters concerning all plots
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
    if( mPlotXtransient )
    {
        mPlotXtransient->setParameters( sps, fftSize );
    }

    if( mPlotXfft )
    {
        mPlotXfft->setParameters( sps, fftSize );
    }

    if( mPlotXperiodogram )
    {
        mPlotXperiodogram->setParameters( sps, fftSize );
    }

    if( mPlotXsrs )
    {
        mPlotXsrs->setParameters( sps, fftSize );
    }

    if( mPlotXcepstrum )
    {
        mPlotXcepstrum->setParameters( sps, fftSize );
    }

    //////////////////////////
    /// Y axis
    //////////////////////////
    if( mPlotYtransient )
    {
        mPlotYtransient->setParameters( sps, fftSize );
    }

    if( mPlotYfft )
    {
        mPlotYfft->setParameters( sps, fftSize );
    }

    if( mPlotYperiodogram )
    {
        mPlotYperiodogram->setParameters( sps, fftSize );
    }

    if( mPlotYsrs )
    {
        mPlotYsrs->setParameters( sps, fftSize );
    }

    if( mPlotYcepstrum )
    {
        mPlotYcepstrum->setParameters( sps, fftSize );
    }

    //////////////////////////
    /// Z axis
    //////////////////////////
    if( mPlotZtransient )
    {
        mPlotZtransient->setParameters( sps, fftSize );
    }

    if( mPlotZfft )
    {
        mPlotZfft->setParameters( sps, fftSize );
    }

    if( mPlotZperiodogram )
    {
        mPlotZperiodogram->setParameters( sps, fftSize );
    }

    if( mPlotZsrs )
    {
        mPlotZsrs->setParameters( sps, fftSize );
    }

    if( mPlotZcepstrum )
    {
        mPlotZcepstrum->setParameters( sps, fftSize );
    }
}


//!************************************************************************
//! Update the max vertical limit in transient Plot objects
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
    if( mPlotXtransient )
    {
        mPlotXtransient->setVerticalMaxTransient( verticalMax );
    }

    //////////////////////////
    /// Y axis
    //////////////////////////
    if( mPlotYtransient )
    {
        mPlotYtransient->setVerticalMaxTransient( verticalMax );
    }

    //////////////////////////
    /// Z axis
    //////////////////////////
    if( mPlotZtransient )
    {
        mPlotZtransient->setVerticalMaxTransient( verticalMax );
    }
}
