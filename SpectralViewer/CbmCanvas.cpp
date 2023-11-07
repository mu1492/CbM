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
CbmCanvas.cpp

This file contains the sources for drawing CbM information.
*/

#include "CbmCanvas.h"

#include "VibrationHandler.h"

#include <QTimer>


//!************************************************************************
//! Constructor
//!************************************************************************
CbmCanvas::CbmCanvas
    (
    QWidget*    aParent     //!< parent widget
    )
    : QWidget( aParent )
    , mSizeW( width() )
    , mSizeH( height() )
{
    setBackgroundRole( QPalette::Base );
    setAutoFillBackground( true );

    QTimer* rmsSpeedLevelsTimer = new QTimer();
    connect( rmsSpeedLevelsTimer, SIGNAL( timeout() ), this, SLOT( updateRmsSpeedLevels() ) );
    rmsSpeedLevelsTimer->start( 1000 );
}


//!************************************************************************
//! Draw the CbM zones A/B/C/D for all three axes
//!
//! @returns nothing
//!************************************************************************
void CbmCanvas::drawCbmZones()
{
    VibrationHandler* vibrHndlInstance = VibrationHandler::getInstance();

    if( vibrHndlInstance )
    {
        QPainter painter( &mPicture );

        const int FONT_SIZE_12 = 12;
        const QFont FONT_12 = QFont( "Courier New", FONT_SIZE_12, QFont::Normal );
        const QString MM_S = " mm/s";
        const int ZONE_SPEED_SPACER = 5;

        const QBrush ZONE_D_BRUSH = RED_BRUSH;
        const QBrush ZONE_C_BRUSH = ORANGE_BRUSH;
        const QBrush ZONE_B_BRUSH = YELLOW_BRUSH;
        const QBrush ZONE_A_BRUSH = GREEN_BRUSH;

        int usableHeight = static_cast<int>( mSizeH * ( 1.0 - FREE_AREA_H_LOW_SCALE - FREE_AREA_H_HIGH_SCALE ) );

        painter.setPen( CYAN_PEN );
        QString str = "Mechanical Vibrations - CbM";
        painter.setFont( FONT_12 );
        int xStrOffset = static_cast<int>( str.size() * FONT_SIZE_12 * COURIER_NEW_RATIO );
        int yStrOffset = static_cast<int>( 0.5 * FONT_SIZE_12 );
        painter.drawText( MIN_WIDTH / 2 - xStrOffset, mSizeH * FREE_AREA_H_LOW_SCALE / 2 + yStrOffset, str );


        VibrationHandler::VibrationMonitoringSettingsTriaxial vmSettings = vibrHndlInstance->getTriaxialSettings();

        //////////////////////////
        /// X axis
        //////////////////////////
        int xLeft = AXIS_LEFT_SPACE;
        int xRight = xLeft + AXIS_ZONE_WIDTH;

        double topSpeedValue = ABOVE_V_CD_SCALE * vmSettings.xAxis.getActiveTransitions().vFromCtoD;

        int heightZoneA = static_cast<int>( usableHeight * vmSettings.xAxis.getActiveTransitions().vFromAtoB / topSpeedValue );
        int heightZoneB = static_cast<int>( usableHeight * ( vmSettings.xAxis.getActiveTransitions().vFromBtoC - vmSettings.xAxis.getActiveTransitions().vFromAtoB ) / topSpeedValue );
        int heightZoneC = static_cast<int>( usableHeight * ( vmSettings.xAxis.getActiveTransitions().vFromCtoD - vmSettings.xAxis.getActiveTransitions().vFromBtoC ) / topSpeedValue );
        int heightZoneD = static_cast<int>( usableHeight * ( ABOVE_V_CD_SCALE - 1.0 ) * vmSettings.xAxis.getActiveTransitions().vFromCtoD / topSpeedValue );

        painter.setPen( CYAN_PEN );
        str = "X axis";
        painter.setFont( FONT_12 );
        xStrOffset = static_cast<int>( str.size() * FONT_SIZE_12 * COURIER_NEW_RATIO );
        yStrOffset = static_cast<int>( 0.5 * FONT_SIZE_12 );
        painter.drawText( ( xLeft + xRight ) / 2 - xStrOffset, mSizeH * ( 1.0 - 0.5 * FREE_AREA_H_LOW_SCALE ) + yStrOffset, str );

        painter.setPen( LIGHT_GRAY_PEN );
        painter.setFont( FONT_9 );
        yStrOffset = static_cast<int>( 0.5 * FONT_SIZE_9 );

        // C-to-D speed
        str = QString::number( 1000 * vmSettings.xAxis.getActiveTransitions().vFromCtoD ) + MM_S;
        int yTop = mSizeH * FREE_AREA_H_HIGH_SCALE + yStrOffset + heightZoneD;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );
        // B-to-C speed
        str = QString::number( 1000 * vmSettings.xAxis.getActiveTransitions().vFromBtoC ) + MM_S;
        yTop += heightZoneC;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );
        // A-to-B speed
        str = QString::number( 1000 * vmSettings.xAxis.getActiveTransitions().vFromAtoB ) + MM_S;
        yTop += heightZoneB;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );

        painter.setPen( NO_PEN );

        painter.setBrush( ZONE_D_BRUSH );
        yTop = mSizeH * FREE_AREA_H_HIGH_SCALE;
        int yBtm = yTop + heightZoneD;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_C_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneC;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_B_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneB;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_A_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneA;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );


        //////////////////////////
        /// Y axis
        //////////////////////////
        xLeft += AXIS_TOTAL_WIDTH;
        xRight = xLeft + AXIS_ZONE_WIDTH;

        topSpeedValue = ABOVE_V_CD_SCALE * vmSettings.yAxis.getActiveTransitions().vFromCtoD;

        heightZoneA = static_cast<int>( usableHeight * vmSettings.yAxis.getActiveTransitions().vFromAtoB / topSpeedValue );
        heightZoneB = static_cast<int>( usableHeight * ( vmSettings.yAxis.getActiveTransitions().vFromBtoC - vmSettings.yAxis.getActiveTransitions().vFromAtoB ) / topSpeedValue );
        heightZoneC = static_cast<int>( usableHeight * ( vmSettings.yAxis.getActiveTransitions().vFromCtoD - vmSettings.yAxis.getActiveTransitions().vFromBtoC ) / topSpeedValue );
        heightZoneD = static_cast<int>( usableHeight * ( ABOVE_V_CD_SCALE - 1.0 ) * vmSettings.yAxis.getActiveTransitions().vFromCtoD / topSpeedValue );

        painter.setPen( CYAN_PEN );
        str = "Y axis";
        painter.setFont( FONT_12 );
        xStrOffset = static_cast<int>( str.size() * FONT_SIZE_12 * COURIER_NEW_RATIO );
        yStrOffset = static_cast<int>( 0.5 * FONT_SIZE_12 );
        painter.drawText( ( xLeft + xRight ) / 2 - xStrOffset, mSizeH * ( 1.0 - 0.5 * FREE_AREA_H_LOW_SCALE ) + yStrOffset, str );

        painter.setPen( LIGHT_GRAY_PEN );
        painter.setFont( FONT_9 );
        yStrOffset = static_cast<int>( 0.5 * FONT_SIZE_9 );

        // C-to-D speed
        str = QString::number( 1000 * vmSettings.yAxis.getActiveTransitions().vFromCtoD ) + MM_S;
        yTop = mSizeH * FREE_AREA_H_HIGH_SCALE + yStrOffset + heightZoneD;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );
        // B-to-C speed
        str = QString::number( 1000 * vmSettings.yAxis.getActiveTransitions().vFromBtoC ) + MM_S;
        yTop += heightZoneC;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );
        // A-to-B speed
        str = QString::number( 1000 * vmSettings.yAxis.getActiveTransitions().vFromAtoB ) + MM_S;
        yTop += heightZoneB;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );

        painter.setPen( NO_PEN );

        painter.setBrush( ZONE_D_BRUSH );
        yTop = mSizeH * FREE_AREA_H_HIGH_SCALE;
        yBtm = yTop + heightZoneD;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_C_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneC;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_B_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneB;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_A_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneA;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );


        //////////////////////////
        /// Z axis
        //////////////////////////
        xLeft += AXIS_TOTAL_WIDTH;
        xRight = xLeft + AXIS_ZONE_WIDTH;

        topSpeedValue = ABOVE_V_CD_SCALE * vmSettings.zAxis.getActiveTransitions().vFromCtoD;

        heightZoneA = static_cast<int>( usableHeight * vmSettings.zAxis.getActiveTransitions().vFromAtoB / topSpeedValue );
        heightZoneB = static_cast<int>( usableHeight * ( vmSettings.zAxis.getActiveTransitions().vFromBtoC - vmSettings.zAxis.getActiveTransitions().vFromAtoB ) / topSpeedValue );
        heightZoneC = static_cast<int>( usableHeight * ( vmSettings.zAxis.getActiveTransitions().vFromCtoD - vmSettings.zAxis.getActiveTransitions().vFromBtoC ) / topSpeedValue );
        heightZoneD = static_cast<int>( usableHeight * ( ABOVE_V_CD_SCALE - 1.0 ) * vmSettings.zAxis.getActiveTransitions().vFromCtoD / topSpeedValue );

        painter.setPen( CYAN_PEN );
        str = "Z axis";
        painter.setFont( FONT_12 );
        xStrOffset = static_cast<int>( str.size() * FONT_SIZE_12 * COURIER_NEW_RATIO );
        yStrOffset = static_cast<int>( 0.5 * FONT_SIZE_12 );
        painter.drawText( ( xLeft + xRight ) / 2 - xStrOffset, mSizeH * ( 1.0 - 0.5 * FREE_AREA_H_LOW_SCALE ) + yStrOffset, str );

        painter.setPen( LIGHT_GRAY_PEN );
        painter.setFont( FONT_9 );
        yStrOffset = static_cast<int>( 0.5 * FONT_SIZE_9 );

        // C-to-D speed
        str = QString::number( 1000 * vmSettings.zAxis.getActiveTransitions().vFromCtoD ) + MM_S;
        yTop = mSizeH * FREE_AREA_H_HIGH_SCALE + yStrOffset + heightZoneD;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );
        // B-to-C speed
        str = QString::number( 1000 * vmSettings.zAxis.getActiveTransitions().vFromBtoC ) + MM_S;
        yTop += heightZoneC;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );
        // A-to-B speed
        str = QString::number( 1000 * vmSettings.zAxis.getActiveTransitions().vFromAtoB ) + MM_S;
        yTop += heightZoneB;
        painter.drawText( xRight + ZONE_SPEED_SPACER, yTop, str );

        painter.setPen( NO_PEN );

        painter.setBrush( ZONE_D_BRUSH );
        yTop = mSizeH * FREE_AREA_H_HIGH_SCALE;
        yBtm = yTop + heightZoneD;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_C_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneC;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_B_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneB;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.setBrush( ZONE_A_BRUSH );
        yTop = yBtm;
        yBtm = yTop + heightZoneA;
        painter.drawRect( QRect( QPoint( xLeft, yTop ), QPoint( xRight, yBtm ) ) );

        painter.end();
    }
}


//!************************************************************************
//! Draw the indicators for the RMS speeds for all three axes
//!
//! @returns nothing
//!************************************************************************
void CbmCanvas::drawRmsSpeedLevels()
{
    VibrationHandler* vibrHndlInstance = VibrationHandler::getInstance();

    if( vibrHndlInstance )
    {
        QPainter painter( &mPicture );

        int usableHeight = static_cast<int>( mSizeH * ( 1.0 - FREE_AREA_H_LOW_SCALE - FREE_AREA_H_HIGH_SCALE ) );
        const int LINE_SPACER = 2;
        const int TRIANGLE_SIDE = 10;
        const QBrush WHITE_BRUSH = QBrush( Qt::white );
        const QBrush DARK_GRAY_BRUSH = QBrush( QColor( 63, 63, 63 ) );

        VibrationHandler::VibrationRmsSpeedsTriaxial rmsSpeeds = vibrHndlInstance->getTriaxialRmsSpeeds();
        VibrationHandler::VibrationMonitoringSettingsTriaxial vmSettings = vibrHndlInstance->getTriaxialSettings();

        //////////////////////////
        /// X axis
        //////////////////////////
        int xLeft = AXIS_LEFT_SPACE + LINE_SPACER;
        int xRight = xLeft + AXIS_ZONE_WIDTH - 2 * LINE_SPACER;
        double topSpeedValue = ABOVE_V_CD_SCALE * vmSettings.xAxis.getActiveTransitions().vFromCtoD;
        double crtSpeedValue = rmsSpeeds.xRmsSpeed > topSpeedValue ? topSpeedValue : rmsSpeeds.xRmsSpeed;
        int yTop = mSizeH * FREE_AREA_H_HIGH_SCALE;
        int yCrt = yTop + static_cast<int>( usableHeight * ( 1.0 - crtSpeedValue / topSpeedValue ) );
        painter.setPen( WHITE_PEN_2 );
        painter.drawLine( QPoint( xLeft, yCrt ), QPoint( xRight, yCrt ) );

        QString str = QString::number( 1000 * rmsSpeeds.xRmsSpeed );
        int xStrOffset = static_cast<int>( str.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );
        int yStrOffset = static_cast<int>( 0.5 * FONT_SIZE_9 );

        painter.setPen( NO_PEN );

        QPainterPath path;
        path.moveTo( xLeft - LINE_SPACER, yCrt - TRIANGLE_SIDE / 2 );
        path.lineTo( xLeft - LINE_SPACER, yCrt + TRIANGLE_SIDE / 2 );
        path.lineTo( xLeft + LINE_SPACER + TRIANGLE_SIDE / 2, yCrt );
        painter.fillPath( path, WHITE_BRUSH );

        path.moveTo( xRight + LINE_SPACER, yCrt - TRIANGLE_SIDE / 2 );
        path.lineTo( xRight + LINE_SPACER, yCrt + TRIANGLE_SIDE / 2 );
        path.lineTo( xRight - LINE_SPACER - TRIANGLE_SIDE / 2, yCrt );
        painter.fillPath( path, WHITE_BRUSH );

        painter.setBrush( DARK_GRAY_BRUSH );
        painter.drawRect( QRect( QPoint( ( xLeft + xRight ) / 2 - xStrOffset - LINE_SPACER, yCrt - 4 * yStrOffset ),
                                 QPoint( ( xLeft + xRight ) / 2 + xStrOffset + LINE_SPACER, yCrt - LINE_SPACER ) ) );

        painter.setPen( WHITE_PEN );
        painter.drawText( ( xLeft + xRight ) / 2 - xStrOffset, yCrt - yStrOffset, str );


        //////////////////////////
        /// Y axis
        //////////////////////////
        xLeft += AXIS_TOTAL_WIDTH;
        xRight = xLeft + AXIS_ZONE_WIDTH - 2 * LINE_SPACER;
        topSpeedValue = ABOVE_V_CD_SCALE * vmSettings.yAxis.getActiveTransitions().vFromCtoD;
        crtSpeedValue = rmsSpeeds.yRmsSpeed > topSpeedValue ? topSpeedValue : rmsSpeeds.yRmsSpeed;
        yCrt = yTop + static_cast<int>( usableHeight * ( 1.0 - crtSpeedValue / topSpeedValue ) );
        painter.setPen( WHITE_PEN_2 );
        painter.drawLine( QPoint( xLeft, yCrt ), QPoint( xRight, yCrt ) );

        str = QString::number( 1000 * rmsSpeeds.yRmsSpeed );
        xStrOffset = static_cast<int>( str.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );

        painter.setPen( NO_PEN );

        path.moveTo( xLeft - LINE_SPACER, yCrt - TRIANGLE_SIDE / 2 );
        path.lineTo( xLeft - LINE_SPACER, yCrt + TRIANGLE_SIDE / 2 );
        path.lineTo( xLeft + LINE_SPACER + TRIANGLE_SIDE / 2, yCrt );
        painter.fillPath( path, WHITE_BRUSH );

        path.moveTo( xRight + LINE_SPACER, yCrt - TRIANGLE_SIDE / 2 );
        path.lineTo( xRight + LINE_SPACER, yCrt + TRIANGLE_SIDE / 2 );
        path.lineTo( xRight - LINE_SPACER - TRIANGLE_SIDE / 2, yCrt );
        painter.fillPath( path, WHITE_BRUSH );

        painter.setBrush( DARK_GRAY_BRUSH );
        painter.drawRect( QRect( QPoint( ( xLeft + xRight ) / 2 - xStrOffset - LINE_SPACER, yCrt - 4 * yStrOffset ),
                                 QPoint( ( xLeft + xRight ) / 2 + xStrOffset + LINE_SPACER, yCrt - LINE_SPACER ) ) );

        painter.setPen( WHITE_PEN );
        painter.drawText( ( xLeft + xRight ) / 2 - xStrOffset, yCrt - yStrOffset, str );


        //////////////////////////
        /// Z axis
        //////////////////////////
        xLeft += AXIS_TOTAL_WIDTH;
        xRight = xLeft + AXIS_ZONE_WIDTH - 2 * LINE_SPACER;
        topSpeedValue = ABOVE_V_CD_SCALE * vmSettings.zAxis.getActiveTransitions().vFromCtoD;
        crtSpeedValue = rmsSpeeds.zRmsSpeed > topSpeedValue ? topSpeedValue : rmsSpeeds.zRmsSpeed;
        yCrt = yTop + static_cast<int>( usableHeight * ( 1.0 - crtSpeedValue / topSpeedValue ) );
        painter.setPen( WHITE_PEN_2 );
        painter.drawLine( QPoint( xLeft, yCrt ), QPoint( xRight, yCrt ) );

        str = QString::number( 1000 * rmsSpeeds.zRmsSpeed );
        xStrOffset = static_cast<int>( str.size() * FONT_SIZE_9 * COURIER_NEW_RATIO );

        painter.setPen( NO_PEN );

        path.moveTo( xLeft - LINE_SPACER, yCrt - TRIANGLE_SIDE / 2 );
        path.lineTo( xLeft - LINE_SPACER, yCrt + TRIANGLE_SIDE / 2 );
        path.lineTo( xLeft + LINE_SPACER + TRIANGLE_SIDE / 2, yCrt );
        painter.fillPath( path, WHITE_BRUSH );

        path.moveTo( xRight + LINE_SPACER, yCrt - TRIANGLE_SIDE / 2 );
        path.lineTo( xRight + LINE_SPACER, yCrt + TRIANGLE_SIDE / 2 );
        path.lineTo( xRight - LINE_SPACER - TRIANGLE_SIDE / 2, yCrt );
        painter.fillPath( path, WHITE_BRUSH );

        painter.setBrush( DARK_GRAY_BRUSH );
        painter.drawRect( QRect( QPoint( ( xLeft + xRight ) / 2 - xStrOffset - LINE_SPACER, yCrt - 4 * yStrOffset ),
                                 QPoint( ( xLeft + xRight ) / 2 + xStrOffset + LINE_SPACER, yCrt - LINE_SPACER ) ) );

        painter.setPen( WHITE_PEN );
        painter.drawText( ( xLeft + xRight ) / 2 - xStrOffset, yCrt - yStrOffset, str );


        painter.end();
    }
}


//!************************************************************************
//! Resize minimum size hint
//!
//! @returns A minimum rectangle size to be kept during resize
//!************************************************************************
QSize CbmCanvas::minimumSizeHint() const
{
    return QSize( MIN_WIDTH, MIN_HEIGHT );
}


//!************************************************************************
//! Paint event
//! Called whenever something is painted.
//!
//! @returns nothing
//!************************************************************************
void CbmCanvas::paintEvent
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

    // CbM zones
    drawCbmZones();
    painter.drawPicture( 0, 0, mPicture );

    // RMS speed levels    
    drawRmsSpeedLevels();
    painter.drawPicture( 0, 0, mPicture );
}


//!************************************************************************
//! Resize event
//! Called whenever the drawing surface gets another dimension.
//!
//! @returns nothing
//!************************************************************************
void CbmCanvas::resizeEvent
    (
    QResizeEvent*   aEvent      //!< resize event
    )
{
    mSizeW = width();
    mSizeH = height();
    QWidget::resizeEvent( aEvent );
}


//!************************************************************************
//! Initialize size hint
//!
//! @returns A rectangle to be used at the first content resize
//!************************************************************************
QSize CbmCanvas::sizeHint() const
{
    return QSize( MIN_WIDTH, MIN_HEIGHT );
}


//!************************************************************************
//! Update RMS speed levels and redraw them
//!
//! @returns nothing
//!************************************************************************
/* slot */ void CbmCanvas::updateRmsSpeedLevels()
{
    update();
}
