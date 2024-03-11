///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2024 Mihai Ursu                                                 //
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
PlotInferCanvas.cpp

This file contains the sources for drawing TensorRT inference plots.
*/


#include "PlotInferCanvas.h"

#include "Numeric.h"

#include <algorithm>
#include <cmath>

#include <QPainter>


//!************************************************************************
//! Constructor
//!************************************************************************
PlotInferCanvas::PlotInferCanvas
    (
    QWidget* aParent        //!< parent widget
    )
    : QWidget( aParent )
    // widget size
    , mSizeW( width() )
    , mSizeH( height() )
    // grid lines
    , mGridLinesH( 3 )
    , mGridLinesV( 3 )
    // horizontal axis
    , mMaxHoriz( 0 )
    // vertical axis
    , mMaxVert( 0 )
    , mHaveVertValues( false )
    , mDataThreshold( 0 )
    // highlight
    , mHighlightRatio( 0 )
{
    setBackgroundRole( QPalette::Base );
    setAutoFillBackground( true );
}


//!************************************************************************
//! Draw the horizontal grid lines
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::drawGridLinesH()
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
void PlotInferCanvas::drawGridLinesV()
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
    }

    painter.end();
}


//!************************************************************************
//! Draw the highlight area for the events window
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::drawHighlightEventsWindow()
{
    QPainter painter( &mPicture );

    mPen = QPen( Qt::NoPen );
    mBrush = QBrush( QColor( 23, 23, 23 ), Qt::SolidPattern );
    painter.setPen( mPen );
    painter.setBrush( mBrush );
    int dataW = mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE;
    painter.drawRect( QRect( QPoint( mSizeW - RIGHT_PLOT_SPACE - dataW * mHighlightRatio, TOP_PLOT_SPACE ),
                             QPoint( mSizeW - RIGHT_PLOT_SPACE, mSizeH - BOTTOM_PLOT_SPACE ) ) );
    painter.end();
}


//!************************************************************************
//! Draw the outer rectangle for the plot area
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::drawOuterRectangle()
{
    QPainter painter( &mPicture );

    mPen = QPen( Qt::white, 0.75, Qt::DashLine );
    painter.setPen( mPen );

    painter.drawRect( QRect( QPoint( LEFT_PLOT_SPACE, TOP_PLOT_SPACE ),
                             QPoint( mSizeW - RIGHT_PLOT_SPACE, mSizeH - BOTTOM_PLOT_SPACE ) ) );

    painter.end();
}


//!************************************************************************
//! Draw the threshold
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::drawThreshold()
{
    QPainter painter( &mPicture );

    mPen = QPen( QColor( 255, 31, 31 ), 1.0, Qt::SolidLine );
    painter.setPen( mPen );

    int dataH = mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE;
    int dataW = mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE;
    int crtX = LEFT_PLOT_SPACE + 1;
    double vertScale = dataH / mMaxVert;
    int zeroY = TOP_PLOT_SPACE + dataH;
    int crtY = zeroY - vertScale * mDataThreshold;
    painter.drawLine( QPoint( crtX, crtY ), QPoint( crtX + dataW, crtY ) );

    painter.end();
}


//!************************************************************************
//! Draw the inference plot
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::drawPlotInference()
{
    QPainter painter( &mPicture );

    mPen = QPen( QColor( 0, 255, 0 ), 0.75, Qt::SolidLine );
    painter.setPen( mPen );

    size_t dataLen = mDataVector.size();

    int dataH = mSizeH - TOP_PLOT_SPACE - BOTTOM_PLOT_SPACE;
    int dataW = mSizeW - LEFT_PLOT_SPACE - RIGHT_PLOT_SPACE;
    int crtX = LEFT_PLOT_SPACE + 1;
    double vertScale = dataH / mMaxVert;
    int zeroY = TOP_PLOT_SPACE + dataH;
    int crtY = zeroY - vertScale * mDataVector.at( dataLen / dataW );
    int nextY = 0;

    if( crtY < zeroY - dataH )
    {
        crtY = zeroY - dataH;
    }
    else if( crtY > zeroY )
    {
        crtY = zeroY;
    }

    for( size_t i = 1; i <= dataW; i++ )
    {        
        int index = ( i + 1 ) * dataLen / dataW;

        if( index > dataLen - 1 )
        {
            index = dataLen - 1;
        }

        nextY = zeroY - vertScale * mDataVector.at( index );

        if( nextY < zeroY - dataH )
        {
            nextY = zeroY - dataH;
        }
        else if( nextY > zeroY )
        {
            nextY = zeroY;
        }

        painter.drawLine( QPoint( crtX, crtY ), QPoint( crtX + 1, nextY ) );
        crtX++;
        crtY = nextY;
    }

    painter.end();
}


//!************************************************************************
//! Draw the abscissa values
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::drawValuesAbscissa()
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
    double barDelta = mMaxHoriz / ( mGridLinesV + 1.0 );

    painter.drawText( LEFT_PLOT_SPACE, TEXT_TOP_MARGIN, "0" );

    for( uint8_t i = 1; i <= mGridLinesV; i++ )
    {
        s = QString::number( i * barDelta );
        xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
        painter.drawText( LEFT_PLOT_SPACE + i * barWidth - xOffset, TEXT_TOP_MARGIN, s );
    }

    s = QString::number( mMaxHoriz );
    xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
    painter.drawText( mSizeW - RIGHT_PLOT_SPACE - 2 * xOffset, TEXT_TOP_MARGIN, s );

    painter.end();
}


//!************************************************************************
//! Draw the ordinate values
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::drawValuesOrdinate()
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

    s = formatDecimals( mMaxVert );
    xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
    painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, TOP_PLOT_SPACE + 2 * yOffset, s );

    barDelta = mMaxVert / ( mGridLinesH + 1.0 );

    for( uint8_t i = 1; i <= mGridLinesH; i++ )
    {
        if( ( 0 == mGridLinesH % 2 )
          ||( 0 != mGridLinesH % 2 ) && ( i != ( mGridLinesH + 1 ) / 2 ) )
        {
            s = formatDecimals( mMaxVert - i * barDelta );
        }
        else
        {
            s = formatDecimals( mMaxVert / 2.0 );
        }

        xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
        painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, TOP_PLOT_SPACE + i * barHeight + yOffset, s );
    }

    s = "0";
    xOffset = static_cast<int>( s.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
    painter.drawText( TEXT_RIGHT_MARGIN - 2 * xOffset, mSizeH - BOTTOM_PLOT_SPACE - yOffset, s );

    painter.end();
}


//!************************************************************************
//! Convert a real number to string and format it up to two decimals
//!
//! @returns The formatted string with up to two decimals
//!************************************************************************
QString PlotInferCanvas::formatDecimals
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
//! Get the data vector
//!
//! @returns Address of data vector
//!************************************************************************
std::vector<double>& PlotInferCanvas::getDataVector()
{
    return mDataVector;
}


//!************************************************************************
//! Overwrite the maximum vertical value (user action)
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::overwriteVerticalMax
    (
    const double aVerticalMax           //!< max value for vertical axis
    )
{
    if( aVerticalMax > 0 )
    {
        mMaxVert = aVerticalMax;
    }

    mHaveVertValues = true;
}


//!************************************************************************
//! Paint event
//! Called whenever something is painted.
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::paintEvent
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

    mBrush = QBrush( Qt::NoBrush );
    painter.setBrush( mBrush );

    mPicture = QPicture();

    if( mHighlightRatio > 0 )
    {
        // highlight for the events window
        drawHighlightEventsWindow();
        painter.drawPicture( 0, 0, mPicture );
    }

    mBrush = QBrush( Qt::NoBrush );
    painter.setBrush( mBrush );

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

    if( mHaveVertValues )
    {
        // vertical axis values
        drawValuesOrdinate();
        painter.drawPicture( 0, 0, mPicture );

        // threshold
        if( mDataThreshold < mMaxVert )
        {
            drawThreshold();
            painter.drawPicture( 0, 0, mPicture );
        }

        // data plot
        drawPlotInference();
        painter.drawPicture( 0, 0, mPicture );
    }
}


//!************************************************************************
//! Resize event
//! Called whenever the drawing surface gets another dimension.
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::resizeEvent
    (
    QResizeEvent*   aEvent      //!< resize event
    )
{
    mSizeW = width();
    mSizeH = height();
    QWidget::resizeEvent( aEvent );
}


//!************************************************************************
//! Set the highlight ratio of the plot
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::setHighlightRatio
    (
    const double aRatio         //!< highlight ratio
    )
{
    if( aRatio >= 0 && aRatio <= 1 )
    {
        mHighlightRatio = aRatio;
    }
}


//!************************************************************************
//! Set the maximum horizontal value
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::setHorizontalMax
    (
    const double aHorizontalMax         //!< max value for horizontal axis
    )
{
    if( aHorizontalMax > 0 )
    {
        mMaxHoriz = aHorizontalMax;
    }
}


//!************************************************************************
//! Set the threshold
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::setThreshold
    (
    const double aThreshold         //!< threshold
    )
{
    if( aThreshold > 0 )
    {
        mDataThreshold = aThreshold;
    }
}


//!************************************************************************
//! Set the maximum vertical value
//!
//! @returns nothing
//!************************************************************************
void PlotInferCanvas::setVerticalMax
    (
    const double aVerticalMax           //!< max value for vertical axis
    )
{
    if( aVerticalMax > mMaxVert )
    {
        mMaxVert = aVerticalMax;
    }

    mHaveVertValues = true;
}
