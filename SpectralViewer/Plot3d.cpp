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
Plot3d.cpp

This file contains the sources for plotting 3D waveforms.
*/

#include "Plot3d.h"
#include "./ui_Plot3d.h"

#include "Plot3dCanvas.h"
#include "SpectralViewer.h"


const std::map<Plot3d::Plot3dType, QString> Plot3d::PLOT_3D_TYPE_NAMES =
{
    { Plot3d::PLOT_3D_TYPE_TRANSIENT,   "Transient"   },
    { Plot3d::PLOT_3D_TYPE_FFT,         "FFT"         },
    { Plot3d::PLOT_3D_TYPE_PERIODOGRAM, "Periodogram" },
    { Plot3d::PLOT_3D_TYPE_SRS,         "SRS"         },
    { Plot3d::PLOT_3D_TYPE_CEPSTRUM,    "Cepstrum"    }
};

const std::map<Plot3d::AxisType, QString> Plot3d::AXIS_TYPE_NAMES =
{
    { Plot3d::AXIS_TYPE_LINEAR, "Li&near" },
    { Plot3d::AXIS_TYPE_LOG,    "L&og"    },
    { Plot3d::AXIS_TYPE_DB,     "d&B"     }
};

//!************************************************************************
//! Constructor
//!************************************************************************
Plot3d::Plot3d
    (
    QWidget*                    aParent,    //!< parent widget
    Plot3dType                  aType,      //!< plot 3D type
    Adxl355Adxl357Common::Axis  aAxis       //!< accelerometer axis
    )
    : QMainWindow( aParent )
    , mType( aType )
    , mAxis( aAxis )
    // UI
    , mPlot3dUi( new Ui::Plot3d )
    // paint&draw
    , mPlot3dCanvas( this, *this )
    // axes
    , mHorizAxisType( AXIS_TYPE_LINEAR )
    , mVertAxisType( AXIS_TYPE_LINEAR )
{
    mPlot3dUi->setupUi( this );

    this->setWindowTitle( formatTitle() );

    //****************************************
    // menus
    //****************************************
    switch( mType )
    {
        case PLOT_3D_TYPE_TRANSIENT:
            mPlot3dUi->actionHorizLog->setEnabled( false );

            mPlot3dUi->actionVertLog->setEnabled( false );
            mPlot3dUi->actionVertDb->setEnabled( false );
            break;

        case PLOT_3D_TYPE_FFT:
            mPlot3dUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlot3dUi->actionVertLog->setEnabled( false );
            break;

        case PLOT_3D_TYPE_PERIODOGRAM:
            mPlot3dUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlot3dUi->actionVertLog->setEnabled( false );
            break;

        case PLOT_3D_TYPE_SRS:
            mHorizAxisType = AXIS_TYPE_LOG;
            mPlot3dUi->menuHorizontalAxis->setEnabled( false );

            mVertAxisType = AXIS_TYPE_LOG;
            mPlot3dUi->menuVerticalAxis->setEnabled( false );
            break;

        case PLOT_3D_TYPE_CEPSTRUM:
            mPlot3dUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlot3dUi->actionVertLinear->setEnabled( false );
            mPlot3dUi->actionVertLog->setEnabled( false );
            break;

        default:
            break;
    }

    connect( mPlot3dUi->actionHorizLinear, &QAction::triggered, this, &Plot3d::handleHorizAxisLinear );
    connect( mPlot3dUi->actionHorizLog, &QAction::triggered, this, &Plot3d::handleHorizAxisLog );

    connect( mPlot3dUi->actionVertLinear, &QAction::triggered, this, &Plot3d::handleVertAxisLinear );
    connect( mPlot3dUi->actionVertLog, &QAction::triggered, this, &Plot3d::handleVertAxisLog );
    connect( mPlot3dUi->actionVertDb, &QAction::triggered, this, &Plot3d::handleVertAxisDb );


    //****************************************
    // paint&draw
    //****************************************
    setCentralWidget( &mPlot3dCanvas );
    this->adjustSize();

    mPlot3dCanvas.update();

    updateMenuHoriz();
    updateMenuVert();
}


//!************************************************************************
//! Destructor
//!************************************************************************
Plot3d::~Plot3d()
{
    delete mPlot3dUi;
}


//!************************************************************************
//! Close event handler
//!
//! @returns nothing
//!************************************************************************
void Plot3d::closeEvent
    (
    QCloseEvent*    aEvent      //!< close event
    )
{
    emit closeSignal( static_cast<int>( mType ), static_cast<int>( mAxis ) );
    aEvent->accept();
}


//!************************************************************************
//! Format the title string
//!
//! @returns Title string
//!************************************************************************
QString Plot3d::formatTitle()
{
    QString titleStr = SpectralViewer::APP_NAME + " - ";
    titleStr += QChar( 'X' + mAxis - Adxl355Adxl357Common::AXIS_X );
    titleStr += " axis - ";
    titleStr += PLOT_3D_TYPE_NAMES.at( mType );

    return titleStr;
}


//!************************************************************************
//! Get the axis - X, Y, or Z
//!
//! @returns The axis
//!************************************************************************
Adxl355Adxl357Common::Axis Plot3d::getAxis() const
{
    return mAxis;
}


//!************************************************************************
//! Get the horizontal axis type - Linear or Log
//!
//! @returns The horizontal axis type
//!************************************************************************
Plot3d::AxisType Plot3d::getAxisTypeHoriz() const
{
    return mHorizAxisType;
}


//!************************************************************************
//! Get the vertical axis type - Linear, Log, or dB
//!
//! @returns The vertical axis type
//!************************************************************************
Plot3d::AxisType Plot3d::getAxisTypeVert() const
{
    return mVertAxisType;
}


//!************************************************************************
//! Get the 3D plot type
//!
//! @returns The plot type
//!************************************************************************
Plot3d::Plot3dType Plot3d::getPlot3dType() const
{
    return mType;
}


//!************************************************************************
//! Handle for setting linear horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::handleHorizAxisLinear()
{
    mHorizAxisType = AXIS_TYPE_LINEAR;
    updateMenuHoriz();
    mPlot3dCanvas.update();
}


//!************************************************************************
//! Handle for setting logarithmic horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::handleHorizAxisLog()
{
    mHorizAxisType = AXIS_TYPE_LOG;
    updateMenuHoriz();
    mPlot3dCanvas.update();
}


//!************************************************************************
//! Handle for setting linear vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::handleVertAxisLinear()
{
    mVertAxisType = AXIS_TYPE_LINEAR;
    updateMenuVert();
    mPlot3dCanvas.update();
}


//!************************************************************************
//! Handle for setting logarithmic vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::handleVertAxisLog()
{
    mVertAxisType = AXIS_TYPE_LOG;
    updateMenuVert();
    mPlot3dCanvas.update();
}


//!************************************************************************
//! Handle for setting dB vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::handleVertAxisDb()
{
    mVertAxisType = AXIS_TYPE_DB;
    updateMenuVert();
    mPlot3dCanvas.update();
}


//!************************************************************************
//! Key press event handler
//!
//! @returns nothing
//!************************************************************************
void Plot3d::keyPressEvent
    (
    QKeyEvent* aEvent       //!< key event
    )
{
    switch( aEvent->key() )
    {
        case Qt::Key_Escape:
            close();
            break;

        case Qt::Key_L:
            mPlot3dCanvas.toogleLightEnable();
            break;

        case Qt::Key_M:
            mPlot3dCanvas.toogleMeshFill();
            break;

        case Qt::Key_R:
            mPlot3dCanvas.resetView();
            break;

        case Qt::Key_Up:
            mPlot3dCanvas.updateKeyUp();
            break;
        case Qt::Key_Down:
            mPlot3dCanvas.updateKeyDown();
            break;
        case Qt::Key_Right:
            mPlot3dCanvas.updateKeyRight();
            break;
        case Qt::Key_Left:
            mPlot3dCanvas.updateKeyLeft();
            break;

        default:
            QWidget::keyPressEvent( aEvent );
            break;
    }
}


//!************************************************************************
//! Signal receiver for new cepstrum calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::receiveNewCepstrum
    (
    int  aAxis   //!< axis
    )
{
    if( PLOT_3D_TYPE_CEPSTRUM == mType && aAxis == mAxis )
    {
        mPlot3dCanvas.update();
    }
}


//!************************************************************************
//! Signal receiver for new accelerometer data
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::receiveNewData()
{
    if( PLOT_3D_TYPE_TRANSIENT == mType )
    {
        mPlot3dCanvas.update();
    }
}


//!************************************************************************
//! Signal receiver for new FFT calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::receiveNewFft
    (
    int  aAxis   //!< axis
    )
{
    if( PLOT_3D_TYPE_FFT == mType && aAxis == mAxis )
    {
        mPlot3dCanvas.update();
    }
}


//!************************************************************************
//! Signal receiver for new periodogram calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::receiveNewPeriodogram
    (
    int  aAxis   //!< axis
    )
{
    if( PLOT_3D_TYPE_PERIODOGRAM == mType && aAxis == mAxis )
    {
        mPlot3dCanvas.update();
    }
}


//!************************************************************************
//! Signal receiver for new SRS calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3d::receiveNewSrs
    (
    int  aAxis   //!< axis
    )
{
    if( PLOT_3D_TYPE_SRS == mType && aAxis == mAxis )
    {
        mPlot3dCanvas.update();
    }
}


//!************************************************************************
//! Set the SPS and FFT size parameters
//!
//! @returns nothing
//!************************************************************************
void Plot3d::setParameters
    (
    const double   aSps,        //!< sampling rate [Hz]
    const uint32_t aFftSize     //!< FFT size
    )
{
    if( aSps > 0 && aFftSize > 0 )
    {
        mPlot3dCanvas.setParameters( aSps, aFftSize );
        mPlot3dCanvas.update();
    }
}


//!************************************************************************
//! Set the maximum vertical value for transient plots
//!
//! @returns nothing
//!************************************************************************
void Plot3d::setVerticalMaxTransient
    (
    const double aVerticalMax   //!< max value for vertical axis [g]
    )
{
    mPlot3dCanvas.setVerticalMaxTransient( aVerticalMax );
    mPlot3dCanvas.update();
}


//!************************************************************************
//! Update the horizontal menu items
//!
//! @returns nothing
//!************************************************************************
void Plot3d::updateMenuHoriz()
{
    mPlot3dUi->actionHorizLinear->setText( ( AXIS_TYPE_LINEAR == mHorizAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LINEAR ) );
    mPlot3dUi->actionHorizLog->setText( ( AXIS_TYPE_LOG == mHorizAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LOG ) );
}


//!************************************************************************
//! Update the vertical menu items
//!
//! @returns nothing
//!************************************************************************
void Plot3d::updateMenuVert()
{
    mPlot3dUi->actionVertLinear->setText( ( AXIS_TYPE_LINEAR == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LINEAR ) );
    mPlot3dUi->actionVertLog->setText( ( AXIS_TYPE_LOG == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LOG ) );
    mPlot3dUi->actionVertDb->setText( ( AXIS_TYPE_DB == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_DB ) );
}
