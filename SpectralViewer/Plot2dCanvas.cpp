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
Plot2dCanvas.cpp

This file contains the sources for drawing 2D plots.
*/


#include "Plot2dCanvas.h"

#include "Numeric.h"
#include "Plot2d.h"
#include "SrsThread.h"

#include <float.h>

#include <algorithm>
#include <cmath>

#include <QPainter>


//!************************************************************************
//! Constructor
//!************************************************************************
Plot2dCanvas::Plot2dCanvas
    (
    QWidget* aParent,           //!< parent widget
    Plot2d&  aParentPlot2d      //!< parent Plot2d object
    )
    : QWidget( aParent )
    , mParentPlot2d( aParentPlot2d )
    // widget size
    , mSizeW( width() )
    , mSizeH( height() )
    // grid lines
    , mGridLinesH( 3 )
    , mGridLinesV( 3 )
    // horizontal axis
    , mSps( 0 )
    , mFftSize( 0 )
    , mMaxFreq( 0 )
    , mMaxFreqLog( 0 )
    , mMinFreqLog( 0 )
    , mMaxQuefrency( 0 )
    , mBinWidth( 0 )
    , mTimeGate( 0 )
    // vertical axis
    , mMaxVert( -DBL_MAX )
    , mMinVert( DBL_MAX )
    , mHaveVertValues( false )
{
    setBackgroundRole( QPalette::Base );
    setAutoFillBackground( true );

    if( Plot2d::PLOT_2D_TYPE_SRS == mParentPlot2d.getPlot2dType() )
    {
        mGridLinesV = 4;
    }
}


//!************************************************************************
//! Comparison function for FftBin values
//!
//! @returns true if first number is smaller than second
//!************************************************************************
bool Plot2dCanvas::compareBinFnc
    (
    VibrationHandler::FftBin x,   //!< first object
    VibrationHandler::FftBin y    //!< second object
    )
{
    return x.value < y.value;
}


//!************************************************************************
//! Comparison function for FftCepstrum values
//!
//! @returns true if first number is smaller than second
//!************************************************************************
bool Plot2dCanvas::compareCepstrumFnc
    (
    VibrationHandler::FftCepstrum x,   //!< first object
    VibrationHandler::FftCepstrum y    //!< second object
    )
{
    return x.value < y.value;
}


//!************************************************************************
//! Comparison function for FftPsd values
//!
//! @returns true if first number is smaller than second
//!************************************************************************
bool Plot2dCanvas::comparePsdFnc
    (
    VibrationHandler::FftPsd x,   //!< first object
    VibrationHandler::FftPsd y    //!< second object
    )
{
    return x.value < y.value;
}


//!************************************************************************
//! Comparison function for Srs values
//!
//! @returns true if first number is smaller than second
//!************************************************************************
bool Plot2dCanvas::compareSrsFnc
    (
    VibrationHandler::Srs x,   //!< first object
    VibrationHandler::Srs y    //!< second object
    )
{
    return x.value < y.value;
}


//!************************************************************************
//! Draw the horizontal grid lines
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawGridLinesH()
{
    QPainter painter( &mPicture );

    if( mGridLinesH )
    {
        mPen = QPen( Qt::lightGray, 0.5, Qt::DotLine );
        painter.setPen( mPen );
        int barHeight = ( mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE ) / ( mGridLinesH + 1 );

        for( uint8_t i = 1; i <= mGridLinesH; i++ )
        {
            int crtY = TOP_PLOT_SPACE + i * barHeight;
            painter.drawLine( QPoint( LEFT_PLOT_SPACE, crtY ),
                              QPoint( mSizeW - RIGHT_PLOT_SPACE, crtY ) );
        }   
    }

    painter.end();
}


//!************************************************************************
//! Draw the vertical grid lines
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawGridLinesV()
{
    QPainter painter( &mPicture );

    if( mGridLinesV )
    {
        mPen = QPen( Qt::lightGray, 0.5, Qt::DotLine );
        painter.setPen( mPen );

        int barWidth = ( mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE ) / ( mGridLinesV + 1 );

        for( uint8_t i = 1; i <= mGridLinesV; i++ )
        {
            int crtX = LEFT_PLOT_SPACE + barWidth * i;
            painter.drawLine( QPoint( crtX, TOP_PLOT_SPACE ),
                              QPoint( crtX, mSizeH - BOTTOM_PLOT_SPACE ) );
        }

        if( Plot2d::AXIS_TYPE_LOG == mParentPlot2d.getAxisTypeHoriz() )
        {
            mPen = QPen( Qt::lightGray, 0.3, Qt::DashDotLine );
            painter.setPen( mPen );

            for( double f = mMinFreqLog; f < 0.9 * mMaxFreqLog; f *= 10.0 )
            {
                for( uint8_t i = 1; i <= 8; i++ )
                {
                    double freqCrt = ( i + 1 ) * f / mMinFreqLog;
                    int crtX = LEFT_PLOT_SPACE + barWidth * log10( freqCrt );
                    painter.drawLine( QPoint( crtX, TOP_PLOT_SPACE ),
                                      QPoint( crtX, mSizeH - BOTTOM_PLOT_SPACE ) );
                }
            }
        }
    }

    painter.end();
}


//!************************************************************************
//! Draw the outer rectangle for the plot area
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawOuterRectangle()
{
    QPainter painter( &mPicture );

    mPen = QPen( Qt::white, 0.75, Qt::DashLine );
    painter.setPen( mPen );

    painter.drawRect( QRect( QPoint( LEFT_PLOT_SPACE, TOP_PLOT_SPACE ),
                             QPoint( mSizeW - RIGHT_PLOT_SPACE, mSizeH - BOTTOM_PLOT_SPACE ) ) );

    painter.end();
}


//!************************************************************************
//! Draw a transient 2D plot with acceleration data
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawPlot2dTransient()
{
    QPainter painter( &mPicture );

    mPen = QPen( GREEN_COLOR, 0.75, Qt::SolidLine );
    painter.setPen( mPen );

    VibrationHandler* vh = VibrationHandler::getInstance();
    VibrationHandler::AccelerometerDataTriaxial accelDataArrays = vh->getTriaxialAccelArrays();    
    size_t dataLen = accelDataArrays.xArray.size();

    int dataH = mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE;
    int dataW = mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE;
    int crtX = LEFT_PLOT_SPACE + 1;
    double vertScale = dataH / ( mMaxVert - mMinVert );
    int zeroY = TOP_PLOT_SPACE + dataH / 2.0;
    int crtY = 0;
    int nextY = 0;

    switch( mParentPlot2d.getAxis() )
    {
        case Adxl355Adxl357Common::AXIS_X:
            crtY = zeroY - vertScale * accelDataArrays.xArray.at( dataLen / dataW );
            break;

        case Adxl355Adxl357Common::AXIS_Y:
            crtY = zeroY - vertScale * accelDataArrays.yArray.at( dataLen / dataW );
            break;

        case Adxl355Adxl357Common::AXIS_Z:
            crtY = zeroY - vertScale * accelDataArrays.zArray.at( dataLen / dataW );
            break;

        default:
            break;
    }

    if( crtY < zeroY - dataH / 2.0 )
    {
        crtY = zeroY - dataH / 2.0;
    }
    else if( crtY > zeroY + dataH / 2.0 )
    {
        crtY = zeroY + dataH / 2.0;
    }

    for( size_t i = 1; i <= dataW; i++ )
    {        
        int index = ( i + 1 ) * dataLen / dataW;

        if( index > dataLen - 1 )
        {
            index = dataLen - 1;
        }

        switch( mParentPlot2d.getAxis() )
        {
            case Adxl355Adxl357Common::AXIS_X:
                nextY = zeroY - vertScale * accelDataArrays.xArray.at( index );
                break;

            case Adxl355Adxl357Common::AXIS_Y:
                nextY = zeroY - vertScale * accelDataArrays.yArray.at( index );
                break;

            case Adxl355Adxl357Common::AXIS_Z:
                nextY = zeroY - vertScale * accelDataArrays.zArray.at( index );
                break;

            default:
                break;
        }

        if( nextY < zeroY - dataH / 2.0 )
        {
            nextY = zeroY - dataH / 2.0;
        }
        else if( nextY > zeroY + dataH / 2.0 )
        {
            nextY = zeroY + dataH / 2.0;
        }

        painter.drawLine( QPoint( crtX, crtY ), QPoint( crtX + 1, nextY ) );
        crtX++;
        crtY = nextY;
    }

    painter.end();
}


//!************************************************************************
//! Draw a frequency 2D plot with FFT (Fast Fourier Transform)
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawPlot2dFft()
{
    QPainter painter( &mPicture );

    mPen = QPen( YELLOW_COLOR, 0.75, Qt::SolidLine );
    painter.setPen( mPen );

    VibrationHandler* vh = VibrationHandler::getInstance();
    std::vector<VibrationHandler::FftBin> binsVec = vh->getFftBinsOnAxis( mParentPlot2d.getAxis() );
    size_t dataLen = binsVec.size();

    int dataH = mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE;
    int dataW = mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE;
    int crtX = LEFT_PLOT_SPACE + 1;

    VibrationHandler::FftBin binMaxValue = *std::max_element( binsVec.begin(), binsVec.end(), compareBinFnc );
    VibrationHandler::FftBin binMinValue = *std::min_element( binsVec.begin(), binsVec.end(), compareBinFnc );

    if( Plot2d::AXIS_TYPE_LINEAR == mParentPlot2d.getAxisTypeVert() )
    {
        mMaxVert = 1;
        mMinVert = 0;
        mHaveVertValues = true;

        int crtY = TOP_PLOT_SPACE + dataH * ( 1 - ( binsVec.at( dataLen / dataW ).value / binMaxValue.value ) );
        int nextY = 0;

        for( size_t i = 1; i <= dataW; i++ )
        {
            int index = ( i + 1 ) * dataLen / dataW;

            if( index > dataLen - 1 )
            {
                index = dataLen - 1;
            }

            nextY = TOP_PLOT_SPACE + dataH * ( 1 - ( binsVec.at( index ).value / binMaxValue.value ) );

            painter.drawLine( QPoint( crtX, crtY ), QPoint( crtX + 1, nextY ) );
            crtX++;
            crtY = nextY;
        }
    }
    else if( Plot2d::AXIS_TYPE_DB == mParentPlot2d.getAxisTypeVert() )
    {
        static double maxForDbScale = -DBL_MAX;
        static double minForDbScale = DBL_MAX;

        static int32_t maxDb = Numeric::INVALID_MAX_10_DB;
        static int32_t minDb = Numeric::INVALID_MIN_10_DB;

        Numeric* nrInstance = Numeric::getInstance();

        if( binMaxValue.value > maxForDbScale )
        {
            maxForDbScale = binMaxValue.value;
            maxDb = nrInstance->findMagnitudeMaxDbMultipleOf10( maxForDbScale );

            if( Numeric::INVALID_MAX_10_DB != maxDb )
            {
                mMaxVert = maxDb;
            }
            else
            {
                maxDb = mMaxVert;
            }
        }

        if( binMinValue.value < minForDbScale )
        {
            minForDbScale = binMinValue.value;
            minDb = nrInstance->findMagnitudeMinDbMultipleOf10( minForDbScale );

            if( Numeric::INVALID_MIN_10_DB != minDb )
            {
                mMinVert = minDb;
            }
            else
            {
                minDb = mMinVert;
            }
        }

        mMaxVert = maxDb;
        mMinVert = minDb;
        mHaveVertValues = true;

        double logval = 20 * log10( binsVec.at( dataLen / dataW ).value );
        int crtY = TOP_PLOT_SPACE + dataH * ( 1 - ( logval - mMinVert ) / ( mMaxVert - mMinVert ) );
        int nextY = 0;

        for( size_t i = 1; i <= dataW; i++ )
        {
            int index = ( i + 1 ) * dataLen / dataW;

            if( index > dataLen - 1 )
            {
                index = dataLen - 1;
            }

            logval = 20 * log10( binsVec.at( index ).value );
            nextY = TOP_PLOT_SPACE + dataH * ( 1 - ( logval - mMinVert ) / ( mMaxVert - mMinVert ) );

            painter.drawLine( QPoint( crtX, crtY ), QPoint( crtX + 1, nextY ) );
            crtX++;
            crtY = nextY;
        }
    }

    painter.end();
}


//!************************************************************************
//! Draw a frequency 2D plot with periodogram (PSD)
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawPlot2dPeriodogram()
{
    QPainter painter( &mPicture );

    mPen = QPen( MAGENTA_COLOR, 0.75, Qt::SolidLine );
    painter.setPen( mPen );

    VibrationHandler* vh = VibrationHandler::getInstance();
    std::vector<VibrationHandler::FftPsd> psdVec = vh->getFftPsdOnAxis( mParentPlot2d.getAxis() );
    size_t dataLen = psdVec.size();

    int dataH = mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE;
    int dataW = mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE;
    int crtX = LEFT_PLOT_SPACE + 1;

    VibrationHandler::FftPsd psdMaxValue = *std::max_element( psdVec.begin(), psdVec.end(), comparePsdFnc );
    VibrationHandler::FftPsd psdMinValue = *std::min_element( psdVec.begin(), psdVec.end(), comparePsdFnc );

    if( Plot2d::AXIS_TYPE_LINEAR == mParentPlot2d.getAxisTypeVert() )
    {
        mMaxVert = 1;
        mMinVert = 0;
        mHaveVertValues = true;

        int crtY = TOP_PLOT_SPACE + dataH * ( 1 - ( psdVec.at( dataLen / dataW ).value / psdMaxValue.value ) );
        int nextY = 0;

        for( size_t i = 1; i <= dataW; i++ )
        {
            int index = ( i + 1 ) * dataLen / dataW;

            if( index > dataLen - 1 )
            {
                index = dataLen - 1;
            }

            nextY = TOP_PLOT_SPACE + dataH * ( 1 - ( psdVec.at( index ).value / psdMaxValue.value ) );

            painter.drawLine( QPoint( crtX, crtY ), QPoint( crtX + 1, nextY ) );
            crtX++;
            crtY = nextY;
        }
    }
    else if( Plot2d::AXIS_TYPE_DB == mParentPlot2d.getAxisTypeVert() )
    {
        static double maxForDbScale = -DBL_MAX;
        static double minForDbScale = DBL_MAX;

        static int32_t maxDb = Numeric::INVALID_MAX_10_DB;
        static int32_t minDb = Numeric::INVALID_MIN_10_DB;

        Numeric* nrInstance = Numeric::getInstance();

        if( psdMaxValue.value > maxForDbScale )
        {
            maxForDbScale = psdMaxValue.value;
            maxDb = nrInstance->findPowerMaxDbMultipleOf10( maxForDbScale );

            if( Numeric::INVALID_MAX_10_DB != maxDb )
            {
                mMaxVert = maxDb;
            }
            else
            {
                maxDb = mMaxVert;
            }
        }

        if( psdMinValue.value < minForDbScale )
        {
            minForDbScale = psdMinValue.value;
            minDb = nrInstance->findPowerMinDbMultipleOf10( minForDbScale );

            if( Numeric::INVALID_MIN_10_DB != minDb )
            {
                mMinVert = minDb;
            }
            else
            {
                minDb = mMinVert;
            }
        }

        mMaxVert = maxDb;
        mMinVert = minDb;
        mHaveVertValues = true;

        double logval = 10 * log10( psdVec.at( dataLen / dataW ).value );
        int crtY = TOP_PLOT_SPACE + dataH * ( 1 - ( logval - mMinVert ) / ( mMaxVert - mMinVert ) );
        int nextY = 0;

        for( size_t i = 1; i <= dataW; i++ )
        {
            int index = ( i + 1 ) * dataLen / dataW;

            if( index > dataLen - 1 )
            {
                index = dataLen - 1;
            }

            logval = 10 * log10( psdVec.at( index ).value );
            nextY = TOP_PLOT_SPACE + dataH * ( 1 - ( logval - mMinVert ) / ( mMaxVert - mMinVert ) );

            painter.drawLine( QPoint( crtX, crtY ), QPoint( crtX + 1, nextY ) );
            crtX++;
            crtY = nextY;
        }
    }

    painter.end();
}


//!************************************************************************
//! Draw a frequency 2D plot with SRS (Shock Response Spectrum)
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawPlot2dSrs()
{
    QPainter painter( &mPicture );

    mPen = QPen( RED_COLOR, 1.25, Qt::SolidLine );
    painter.setPen( mPen );

    VibrationHandler* vh = VibrationHandler::getInstance();
    std::vector<VibrationHandler::Srs> srsVec = vh->getSrsOnAxis( mParentPlot2d.getAxis() );
    size_t dataLen = srsVec.size();

    VibrationHandler::Srs srsMaxValue = *std::max_element( srsVec.begin() + 1, srsVec.end(), compareSrsFnc );
    VibrationHandler::Srs srsMinValue = *std::min_element( srsVec.begin() + 1, srsVec.end(), compareSrsFnc );

    if( Plot2d::AXIS_TYPE_LOG == mParentPlot2d.getAxisTypeVert() )
    {
        static double maxForLogScale = -DBL_MAX;
        static double minForLogScale = DBL_MAX;

        static double maxLog = Numeric::INVALID_MAX_LOG;
        static double minLog = Numeric::INVALID_MIN_LOG;

        Numeric* nrInstance = Numeric::getInstance();

        if( srsMaxValue.value > maxForLogScale )
        {
            maxForLogScale = srsMaxValue.value;
            maxLog = nrInstance->findLogMaxPwrOf10( maxForLogScale );

            if( Numeric::INVALID_MAX_LOG != maxLog )
            {
                mMaxVert = maxLog;
            }
            else
            {
                maxLog = mMaxVert;
            }
        }

        if( srsMinValue.value < minForLogScale )
        {
            minForLogScale = srsMinValue.value;
            minLog = nrInstance->findLogMinPwrOf10( minForLogScale );

            if( Numeric::INVALID_MIN_LOG != minLog )
            {
                mMinVert = minLog;
            }
            else
            {
                minLog = mMinVert;
            }
        }

        mMaxVert = maxLog;
        mMinVert = minLog;
        mHaveVertValues = true;

        if( mMaxVert > mMinVert )
        {
            mGridLinesH = static_cast<uint8_t>( log10( mMaxVert / mMinVert ) ) - 1;
        }

        size_t srsPoint = 0;

        for( ; srsPoint < dataLen; srsPoint++ )
        {
            if( srsVec.at( srsPoint ).naturalFrequency >= SrsThread::NATURAL_FREQ_MIN )
            {
                break;
            }
        }

        if( srsPoint < dataLen - 1 )
        {
            int dataH = mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE;
            int dataW = mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE;

            double deltaH = dataW / static_cast<double>( 1 + mGridLinesV );
            int crtX = LEFT_PLOT_SPACE + 1;

            double deltaV = dataH / static_cast<double>( 1 + mGridLinesH );
            double ratioV = srsVec.at( srsPoint ).value / mMinVert;
            int crtY = mSizeH - BOTTOM_PLOT_SPACE - static_cast<int>( log10( ratioV ) * deltaV );

            for( ; srsPoint < dataLen; srsPoint++ )
            {
                double ratioH = srsVec.at( srsPoint ).naturalFrequency / SrsThread::NATURAL_FREQ_MIN;
                int nextX = LEFT_PLOT_SPACE + 1 + static_cast<int>( log10( ratioH ) * deltaH );

                if( nextX > mSizeW - RIGHT_PLOT_SPACE )
                {
                    break;
                }

                ratioV = srsVec.at( srsPoint ).value / mMinVert;
                int nextY = mSizeH - BOTTOM_PLOT_SPACE - static_cast<int>( log10( ratioV ) * deltaV );

                painter.drawLine( QPoint( crtX, crtY ), QPoint( nextX, nextY ) );

                crtX = nextX;
                crtY = nextY;
            }
        }
    }

    painter.end();
}


//!************************************************************************
//! Draw a frequency 2D plot with cepstrum
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawPlot2dCepstrum()
{
    QPainter painter( &mPicture );

    mPen = QPen( BLUE_COLOR, 0.75, Qt::SolidLine );
    painter.setPen( mPen );

    VibrationHandler* vh = VibrationHandler::getInstance();
    std::vector<VibrationHandler::FftCepstrum> cepstrumVec = vh->getFftCepstrumOnAxis( mParentPlot2d.getAxis() );
    size_t dataLen = cepstrumVec.size();

    VibrationHandler::FftCepstrum cepstrumMaxValue = *std::max_element( cepstrumVec.begin(), cepstrumVec.end(), compareCepstrumFnc );
    VibrationHandler::FftCepstrum cepstrumMinValue = *std::min_element( cepstrumVec.begin(), cepstrumVec.end(), compareCepstrumFnc );

    if( Plot2d::AXIS_TYPE_DB == mParentPlot2d.getAxisTypeVert() )
    {
        static double maxForDbScale = -DBL_MAX;
        static double minForDbScale = DBL_MAX;

        static int32_t maxDb = Numeric::INVALID_MAX_10_DB;
        static int32_t minDb = Numeric::INVALID_MIN_10_DB;

        Numeric* nrInstance = Numeric::getInstance();

        if( cepstrumMaxValue.value > maxForDbScale )
        {
            maxForDbScale = cepstrumMaxValue.value;
            maxDb = nrInstance->findPowerMaxDbMultipleOf10( maxForDbScale );

            if( Numeric::INVALID_MAX_10_DB != maxDb )
            {
                mMaxVert = maxDb;
            }
            else
            {
                maxDb = mMaxVert;
            }
        }

        if( cepstrumMinValue.value < minForDbScale )
        {
            minForDbScale = cepstrumMinValue.value;
            minDb = nrInstance->findPowerMinDbMultipleOf10( minForDbScale );

            if( Numeric::INVALID_MIN_10_DB != minDb )
            {
                mMinVert = minDb;
            }
            else
            {
                minDb = mMinVert;
            }
        }

        mMaxVert = maxDb;
        mMinVert = minDb;
        mHaveVertValues = true;

        int dataH = mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE;
        int dataW = mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE;
        int crtX = LEFT_PLOT_SPACE + 1;

        double logval = 10 * log10( cepstrumVec.at( dataLen / dataW ).value );
        int crtY = TOP_PLOT_SPACE + dataH * ( 1 - ( logval - mMinVert ) / ( mMaxVert - mMinVert ) );
        int nextY = 0;

        for( size_t i = 1; i <= dataW; i++ )
        {
            int index = ( i + 1 ) * dataLen / dataW;

            if( index > dataLen - 1 )
            {
                index = dataLen - 1;
            }

            logval = 10 * log10( cepstrumVec.at( index ).value );
            nextY = TOP_PLOT_SPACE + dataH * ( 1 - ( logval - mMinVert ) / ( mMaxVert - mMinVert ) );

            painter.drawLine( QPoint( crtX, crtY ), QPoint( crtX + 1, nextY ) );
            crtX++;
            crtY = nextY;
        }
    }

    painter.end();
}


//!************************************************************************
//! Draw the abscissa values
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawValuesAbscissa()
{
    QPainter painter( &mPicture );

    mPen = QPen( Qt::cyan, 1, Qt::SolidLine );
    painter.setPen( mPen );
    painter.setFont( FONT_9 );
    QString s;
    int xOffset = 0;
    int yOffset = static_cast<int>( FONT_SIZE_9 );
    const int TEXT_TOP_MARGIN = mSizeH - BOTTOM_PLOT_SPACE + 2 * yOffset;
    int barWidth = ( mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE ) / ( mGridLinesV + 1 );
    double barDelta = 0;

    switch( mParentPlot2d.getPlot2dType() )
    {
        case Plot2d::PLOT_2D_TYPE_TRANSIENT:
            if( Plot2d::AXIS_TYPE_LINEAR == mParentPlot2d.getAxisTypeHoriz() )
            {
                painter.drawText( LEFT_PLOT_SPACE, TEXT_TOP_MARGIN, "0" );

                barDelta = mTimeGate / ( mGridLinesV + 1.0 );

                for( uint8_t i = 1; i <= mGridLinesV; i++ )
                {
                    s = QString::number( i * barDelta );
                    xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                    painter.drawText( LEFT_PLOT_SPACE + i * barWidth - xOffset, TEXT_TOP_MARGIN, s );
                }

                s = QString::number( mTimeGate );
                xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                painter.drawText( mSizeW - RIGHT_PLOT_SPACE - 2 * xOffset, TEXT_TOP_MARGIN, s );
            }
            break;

        case Plot2d::PLOT_2D_TYPE_FFT:
        case Plot2d::PLOT_2D_TYPE_PERIODOGRAM:
            if( Plot2d::AXIS_TYPE_LINEAR == mParentPlot2d.getAxisTypeHoriz() )
            {
                painter.drawText( LEFT_PLOT_SPACE, TEXT_TOP_MARGIN, "0" );

                barDelta = mMaxFreq / ( mGridLinesV + 1.0 );

                for( uint8_t i = 1; i <= mGridLinesV; i++ )
                {
                    s = formatDecimals( i * barDelta );
                    xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                    painter.drawText( LEFT_PLOT_SPACE + i * barWidth - xOffset, TEXT_TOP_MARGIN, s );
                }

                s = formatDecimals( mMaxFreq );
                xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                painter.drawText( mSizeW - RIGHT_PLOT_SPACE - 2 * xOffset, TEXT_TOP_MARGIN, s );
            }
            else if( Plot2d::AXIS_TYPE_LOG == mParentPlot2d.getAxisTypeHoriz() )
            {
                s = QString::number( mMinFreqLog );
                painter.drawText( LEFT_PLOT_SPACE, TEXT_TOP_MARGIN, s );

                for( uint8_t i = 1; i <= mGridLinesV; i++ )
                {
                    s = QString::number( mMinFreqLog * pow( 10.0, i ) );
                    xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                    painter.drawText( LEFT_PLOT_SPACE + i * barWidth - xOffset, TEXT_TOP_MARGIN, s );
                }

                s = QString::number( mMaxFreqLog );
                xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                painter.drawText( mSizeW - RIGHT_PLOT_SPACE - 2 * xOffset, TEXT_TOP_MARGIN, s );
            }
            break;

        case Plot2d::PLOT_2D_TYPE_SRS:
            if( Plot2d::AXIS_TYPE_LOG == mParentPlot2d.getAxisTypeHoriz() )
            {
                s = formatScientific( mMinFreqLog );
                painter.drawText( LEFT_PLOT_SPACE, TEXT_TOP_MARGIN, s );

                for( uint8_t i = 1; i <= mGridLinesV; i++ )
                {
                    s = formatScientific( mMinFreqLog * pow( 10.0, i ) );
                    xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                    painter.drawText( LEFT_PLOT_SPACE + i * barWidth - xOffset, TEXT_TOP_MARGIN, s );
                }

                s = formatScientific( mMaxFreqLog );
                xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                painter.drawText( mSizeW - RIGHT_PLOT_SPACE - 2 * xOffset, TEXT_TOP_MARGIN, s );
            }
            break;

        case Plot2d::PLOT_2D_TYPE_CEPSTRUM:
            if( Plot2d::AXIS_TYPE_LINEAR == mParentPlot2d.getAxisTypeHoriz() )
            {
                painter.drawText( LEFT_PLOT_SPACE, TEXT_TOP_MARGIN, "0" );

                barDelta = mMaxQuefrency / ( mGridLinesV + 1.0 );

                for( uint8_t i = 1; i <= mGridLinesV; i++ )
                {
                    s = formatDecimals( i * barDelta );
                    xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                    painter.drawText( LEFT_PLOT_SPACE + i * barWidth - xOffset, TEXT_TOP_MARGIN, s );
                }

                s = formatDecimals( mMaxQuefrency );
                xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                painter.drawText( mSizeW - RIGHT_PLOT_SPACE - 2 * xOffset, TEXT_TOP_MARGIN, s );
            }
            break;

        default:
            break;
    }

    painter.end();
}


//!************************************************************************
//! Draw the ordinate values
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::drawValuesOrdinate()
{
    QPainter painter( &mPicture );

    mPen = QPen( Qt::cyan, 1, Qt::SolidLine );
    painter.setPen( mPen );
    painter.setFont( FONT_9 );
    const int TEXT_RIGHT_MARGIN = LEFT_PLOT_SPACE - 5;
    QString s;
    int xOffset = 0;
    int yOffset = static_cast<int>( 0.5 * FONT_SIZE_9 );
    int barHeight = ( mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE ) / ( mGridLinesH + 1 );
    double barDelta = 0;

    switch( mParentPlot2d.getPlot2dType() )
    {
        case Plot2d::PLOT_2D_TYPE_TRANSIENT:
        case Plot2d::PLOT_2D_TYPE_FFT:
        case Plot2d::PLOT_2D_TYPE_PERIODOGRAM:
        case Plot2d::PLOT_2D_TYPE_CEPSTRUM:
            s = formatDecimals( mMaxVert );
            xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
            painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, TOP_PLOT_SPACE + 2 * yOffset, s );

            barDelta = ( mMaxVert - mMinVert ) / ( mGridLinesH + 1.0 );

            for( uint8_t i = 1; i <= mGridLinesH; i++ )
            {
                if( ( 0 == mGridLinesH % 2 )
                  ||( 0 != mGridLinesH % 2 ) && ( i != ( mGridLinesH + 1 ) / 2 ) )
                {
                    s = formatDecimals( mMaxVert - i * barDelta );
                }
                else
                {
                    s = formatDecimals( ( mMaxVert + mMinVert ) / 2.0 );
                }

                xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, TOP_PLOT_SPACE + i * barHeight + yOffset, s );
            }

            s = formatDecimals( mMinVert );
            xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
            painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, mSizeH - BOTTOM_PLOT_SPACE - yOffset, s );
            break;

        case Plot2d::PLOT_2D_TYPE_SRS:
            s = formatScientific( mMaxVert );
            xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
            painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, TOP_PLOT_SPACE + 2 * yOffset, s );

            barDelta = ( mMaxVert - mMinVert ) / ( mGridLinesH + 1.0 );

            for( uint8_t i = 1; i <= mGridLinesH; i++ )
            {
                s = formatScientific( mMaxVert / pow( 10.0, static_cast<double>( i ) ) );
                xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
                painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, TOP_PLOT_SPACE + i * barHeight + yOffset, s );
            }

            s = formatScientific( mMinVert );
            xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
            painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, mSizeH - BOTTOM_PLOT_SPACE - yOffset, s );
            break;

        default:
            break;
    }

    painter.end();
}


//!************************************************************************
//! Convert a real number to string and format it up to two decimals
//!
//! @returns The formatted string with up to two decimals
//!************************************************************************
QString Plot2dCanvas::formatDecimals
    (
    const double aNumber                //!< number to format
    ) const
{
    QString str;
    int i = rint( aNumber * 100.0 );

    if( i % 100 )
    {
        str = QString::number( aNumber, 'f', i % 10 ? 2 : 1 );
    }
    else
    {
        str = QString::number( i / 100 );
    }

    if( "0" == str && 0 != aNumber )
    {
        str = QString::number( aNumber );
    }

    return str;
}


//!************************************************************************
//! Convert a real number to string using scientific format for first digit
//!
//! @returns The formatted string in scientific format
//!************************************************************************
QString Plot2dCanvas::formatScientific
    (
    const double aNumber    //!< number to format
    ) const
{
    int exp = static_cast<int>( floor( log10( aNumber ) ) );
    int digit = static_cast<int>( floor( aNumber / pow( 10.0, static_cast<double>( exp ) ) ) );
    QString str = QString::number( digit ) + "e";

    if( aNumber >= 1 )
    {
        str += "+";
    }

    str += QString::number( exp );

    return str;
}


//!************************************************************************
//! Get the number of grid lines on the horizontal axis
//!
//! @returns The number of vertical grid lines
//!************************************************************************
uint8_t Plot2dCanvas::getGridLinesHorizAxis() const
{
    return mGridLinesV;
}


//!************************************************************************
//! Get the number of grid lines on the vertical axis
//!
//! @returns The number of horizontal grid lines
//!************************************************************************
uint8_t Plot2dCanvas::getGridLinesVertAxis() const
{
    return mGridLinesH;
}


//!************************************************************************
//! Resize minimum size hint
//!
//! @returns A minimum rectangle size to be kept during resize
//!************************************************************************
QSize Plot2dCanvas::minimumSizeHint() const
{
    return QSize( MIN_WIDTH, MIN_HEIGHT );
}


//!************************************************************************
//! Paint event
//! Called whenever something is painted.
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::paintEvent
    (
    QPaintEvent*    /* aEvent */    //!< paint event
    )
{
    QPainter painter( this );

    // use antialiasing
    painter.setRenderHint( QPainter::Antialiasing, true );

    // background
    mPen = QPen( Qt::NoPen );
    mBrush = QBrush( Qt::black, Qt::SolidPattern );
    painter.setPen( mPen );
    painter.setBrush( mBrush );
    painter.drawRect( QRect( 0, 0, mSizeW - 1, mSizeH - 1 ) );
    mPicture = QPicture();

    // outer rectangle
    drawOuterRectangle();
    painter.drawPicture( 0, 0, mPicture );

    // horizontal grid lines
    drawGridLinesH();
    painter.drawPicture( 0, 0, mPicture );

    // vertical grid lines
    drawGridLinesV();
    painter.drawPicture( 0, 0, mPicture );

    // horizontal axis values
    drawValuesAbscissa();
    painter.drawPicture( 0, 0, mPicture );

    // vertical axis values
    if( mHaveVertValues )
    {
        drawValuesOrdinate();
        painter.drawPicture( 0, 0, mPicture );
    }

    // data plots
    switch( mParentPlot2d.getPlot2dType() )
    {
        case Plot2d::PLOT_2D_TYPE_TRANSIENT:
            drawPlot2dTransient();
            break;

        case Plot2d::PLOT_2D_TYPE_FFT:
            drawPlot2dFft();
            break;

        case Plot2d::PLOT_2D_TYPE_PERIODOGRAM:
            drawPlot2dPeriodogram();
            break;

        case Plot2d::PLOT_2D_TYPE_SRS:
            drawPlot2dSrs();
            break;

        case Plot2d::PLOT_2D_TYPE_CEPSTRUM:
            drawPlot2dCepstrum();
            break;

        default:
            break;
    }

    painter.drawPicture( 0, 0, mPicture );
}


//!************************************************************************
//! Resize event
//! Called whenever the drawing surface gets another dimension.
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::resizeEvent
    (
    QResizeEvent*   aEvent      //!< resize event
    )
{
    mSizeW = width();
    mSizeH = height();
    QWidget::resizeEvent( aEvent );
}


//!************************************************************************
//! Set the SPS and FFT size parameters
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::setParameters
    (
    const double   aSps,        //!< sampling rate [Hz]
    const uint32_t aFftSize     //!< FFT size
    )
{
    if( aSps > 0 && aFftSize > 0 )
    {
        mSps = aSps;
        mFftSize = aFftSize;
        mMaxFreq = mSps / 2.0;
        mBinWidth = mSps / mFftSize;
        mTimeGate = 1.0 / mBinWidth;
        mMaxFreqLog = pow( 10.0, static_cast<double>( ceil( log10( mMaxFreq ) ) ) );
        mMaxQuefrency = mTimeGate;
        updateMinFrequencyLog();
    }
}


//!************************************************************************
//! Set the number of grid lines on the horizontal axis
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::setGridLinesHorizAxis
    (
    const uint8_t aGridLinesNumber          //!< number of grid lines
    )
{
    if( aGridLinesNumber > 0 )
    {
        mGridLinesV = aGridLinesNumber;
        updateMinFrequencyLog();
    }
}


//!************************************************************************
//! Set the number of grid lines on the vertical axis
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::setGridLinesVertAxis
    (
    const uint8_t aGridLinesNumber          //!< number of grid lines
    )
{
    if( aGridLinesNumber > 0 )
    {
        mGridLinesH = aGridLinesNumber;
    }
}


//!************************************************************************
//! Set the maximum vertical value for transient plots
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::setVerticalMaxTransient
    (
    const double aVerticalMax           //!< max value for vertical axis [g]
    )
{
    mMaxVert = fabs( aVerticalMax );
    mMinVert = -mMaxVert;

    if( Plot2d::PLOT_2D_TYPE_TRANSIENT == mParentPlot2d.getPlot2dType() )
    {
        mHaveVertValues = true;
    }
}


//!************************************************************************
//! Initialize size hint
//!
//! @returns A rectangle to be used at the first content resize
//!************************************************************************
QSize Plot2dCanvas::sizeHint() const
{
    return QSize( MIN_WIDTH, MIN_HEIGHT );
}


//!************************************************************************
//! Update the minimum frequency for logarithmic horizontal axis
//!
//! The function must be called whenever mMaxFreq or mGridLinesV changed.
//! mMinFreqLog is not used for linear horizontal axis type, where minimum
//! is always zero.
//!
//! @returns nothing
//!************************************************************************
void Plot2dCanvas::updateMinFrequencyLog()
{
    double thd = log10( mMaxFreq );
    mMinFreqLog = 0;

    if( Plot2d::PLOT_2D_TYPE_SRS == mParentPlot2d.getPlot2dType() )
    {
        mMinFreqLog = SrsThread::NATURAL_FREQ_MIN;
        mGridLinesV = static_cast<uint8_t>( log10( mMaxFreqLog / mMinFreqLog ) ) - 1;
    }
    else
    {
        if( thd > 3 )
        {
            mMinFreqLog = pow( 10.0, static_cast<double>( 3 - mGridLinesV ) );
        }
        else if( thd > 2 )
        {
            mMinFreqLog = pow( 10.0, static_cast<double>( 2 - mGridLinesV ) );
        }
        else if( thd > 1 )
        {
            mMinFreqLog = pow( 10.0, static_cast<double>( 1 - mGridLinesV ) );
        }
        else if( thd > 0 )
        {
            mMinFreqLog = pow( 10.0, static_cast<double>( -mGridLinesV ) );
        }
    }
}
