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
Plot2d.cpp

This file contains the sources for plotting 2D waveforms.
*/

#include "Plot2d.h"
#include "./ui_Plot2d.h"

#include "Plot2dCanvas.h"
#include "SpectralViewer.h"

#include <QPalette>


const std::map<Plot2d::Plot2dType, QString> Plot2d::PLOT_2D_TYPE_NAMES =
{
    { Plot2d::PLOT_2D_TYPE_TRANSIENT,   "Transient"   },
    { Plot2d::PLOT_2D_TYPE_FFT,         "FFT"         },
    { Plot2d::PLOT_2D_TYPE_PERIODOGRAM, "Periodogram" },
    { Plot2d::PLOT_2D_TYPE_SRS,         "SRS"         },
    { Plot2d::PLOT_2D_TYPE_CEPSTRUM,    "Cepstrum"    }
};

const std::map<Plot2d::AxisType, QString> Plot2d::AXIS_TYPE_NAMES =
{
    { Plot2d::AXIS_TYPE_LINEAR, "Li&near" },
    { Plot2d::AXIS_TYPE_LOG,    "L&og"    },
    { Plot2d::AXIS_TYPE_DB,     "d&B"     }
};

const std::map<Plot2d::GridLines, QString> Plot2d::GRID_LINES_NAMES =
{
    { Plot2d::GRID_LINES_3, "&3 grid lines" },
    { Plot2d::GRID_LINES_4, "&4 grid lines" },
    { Plot2d::GRID_LINES_5, "&5 grid lines" }
};

//!************************************************************************
//! Constructor
//!************************************************************************
Plot2d::Plot2d
    (
    QWidget*                    aParent,    //!< parent widget
    Plot2dType                  aType,      //!< plot 2D type
    Adxl355Adxl357Common::Axis  aAxis       //!< accelerometer axis
    )
    : QMainWindow( aParent )
    , mType( aType )
    , mAxis( aAxis )    
    // UI
    , mPlot2dUi( new Ui::Plot2d )
    // paint&draw
    , mPlot2dCanvas( this, *this )
    // axes
    , mHorizAxisType( AXIS_TYPE_LINEAR )
    , mVertAxisType( AXIS_TYPE_LINEAR )
{
    mPlot2dUi->setupUi( this );

    this->setWindowTitle( formatTitle() );

    //****************************************
    // menus
    //****************************************
    switch( mType )
    {
        case PLOT_2D_TYPE_TRANSIENT:
            mPlot2dUi->actionHorizLog->setEnabled( false );

            mPlot2dUi->actionVertLog->setEnabled( false );
            mPlot2dUi->actionVertDb->setEnabled( false );
            break;

        case PLOT_2D_TYPE_FFT:
            mPlot2dUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlot2dUi->actionVertLog->setEnabled( false );
            break;

        case PLOT_2D_TYPE_PERIODOGRAM:
            mPlot2dUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlot2dUi->actionVertLog->setEnabled( false );
            break;

        case PLOT_2D_TYPE_SRS:
            mHorizAxisType = AXIS_TYPE_LOG;
            mPlot2dUi->menuHorizontalAxis->setEnabled( false );

            mVertAxisType = AXIS_TYPE_LOG;
            mPlot2dUi->menuVerticalAxis->setEnabled( false );
            break;

        case PLOT_2D_TYPE_CEPSTRUM:
            mPlot2dUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlot2dUi->actionVertLinear->setEnabled( false );
            mPlot2dUi->actionVertLog->setEnabled( false );
            break;

        default:
            break;
    }

    connect( mPlot2dUi->actionHorizLinear, &QAction::triggered, this, &Plot2d::handleHorizAxisLinear );
    connect( mPlot2dUi->actionHorizLog, &QAction::triggered, this, &Plot2d::handleHorizAxisLog );

    connect( mPlot2dUi->actionHorizGridLines3, &QAction::triggered, this, &Plot2d::handleHorizGridLines3 );
    connect( mPlot2dUi->actionHorizGridLines4, &QAction::triggered, this, &Plot2d::handleHorizGridLines4 );
    connect( mPlot2dUi->actionHorizGridLines5, &QAction::triggered, this, &Plot2d::handleHorizGridLines5 );

    connect( mPlot2dUi->actionVertLinear, &QAction::triggered, this, &Plot2d::handleVertAxisLinear );
    connect( mPlot2dUi->actionVertLog, &QAction::triggered, this, &Plot2d::handleVertAxisLog );
    connect( mPlot2dUi->actionVertDb, &QAction::triggered, this, &Plot2d::handleVertAxisDb );

    connect( mPlot2dUi->actionVertGridLines3, &QAction::triggered, this, &Plot2d::handleVertGridLines3 );
    connect( mPlot2dUi->actionVertGridLines4, &QAction::triggered, this, &Plot2d::handleVertGridLines4 );
    connect( mPlot2dUi->actionVertGridLines5, &QAction::triggered, this, &Plot2d::handleVertGridLines5 );


    //****************************************
    // paint&draw
    //****************************************
    setCentralWidget( &mPlot2dCanvas );
    this->adjustSize();

    mPlot2dCanvas.update();

    updateMenuHoriz();
    updateMenuVert();
}


//!************************************************************************
//! Destructor
//!************************************************************************
Plot2d::~Plot2d()
{
    delete mPlot2dUi;
}


//!************************************************************************
//! Close event handler
//!
//! @returns nothing
//!************************************************************************
void Plot2d::closeEvent
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
QString Plot2d::formatTitle()
{
    QString titleStr = SpectralViewer::APP_NAME + " - ";
    titleStr += QChar( 'X' + mAxis - Adxl355Adxl357Common::AXIS_X );
    titleStr += " axis - ";
    titleStr += PLOT_2D_TYPE_NAMES.at( mType );

    return titleStr;
}


//!************************************************************************
//! Get the axis - X, Y, or Z
//!
//! @returns The axis
//!************************************************************************
Adxl355Adxl357Common::Axis Plot2d::getAxis() const
{
    return mAxis;
}


//!************************************************************************
//! Get the horizontal axis type - Linear or Log
//!
//! @returns The horizontal axis type
//!************************************************************************
Plot2d::AxisType Plot2d::getAxisTypeHoriz() const
{
    return mHorizAxisType;
}


//!************************************************************************
//! Get the vertical axis type - Linear, Log, or dB
//!
//! @returns The vertical axis type
//!************************************************************************
Plot2d::AxisType Plot2d::getAxisTypeVert() const
{
    return mVertAxisType;
}


//!************************************************************************
//! Get the 2D plot type
//!
//! @returns The plot type
//!************************************************************************
Plot2d::Plot2dType Plot2d::getPlot2dType() const
{
    return mType;
}


//!************************************************************************
//! Handle for setting linear horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleHorizAxisLinear()
{
    mHorizAxisType = AXIS_TYPE_LINEAR;
    updateMenuHoriz();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting logarithmic horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleHorizAxisLog()
{
    mHorizAxisType = AXIS_TYPE_LOG;
    updateMenuHoriz();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting 3 grid lines on the horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleHorizGridLines3()
{
    mPlot2dCanvas.setGridLinesHorizAxis( 3 );
    updateMenuHoriz();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting 4 grid lines on the horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleHorizGridLines4()
{
    mPlot2dCanvas.setGridLinesHorizAxis( 4 );
    updateMenuHoriz();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting 5 grid lines on the horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleHorizGridLines5()
{
    mPlot2dCanvas.setGridLinesHorizAxis( 5 );
    updateMenuHoriz();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting linear vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleVertAxisLinear()
{
    mVertAxisType = AXIS_TYPE_LINEAR;
    updateMenuVert();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting logarithmic vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleVertAxisLog()
{
    mVertAxisType = AXIS_TYPE_LOG;
    updateMenuVert();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting dB vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleVertAxisDb()
{
    mVertAxisType = AXIS_TYPE_DB;
    updateMenuVert();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting 3 grid lines on the vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleVertGridLines3()
{
    mPlot2dCanvas.setGridLinesVertAxis( 3 );
    updateMenuVert();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting 4 grid lines on the vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleVertGridLines4()
{
    mPlot2dCanvas.setGridLinesVertAxis( 4 );
    updateMenuVert();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Handle for setting 5 grid lines on the vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::handleVertGridLines5()
{
    mPlot2dCanvas.setGridLinesVertAxis( 5 );
    updateMenuVert();
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Signal receiver for new cepstrum calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::receiveNewCepstrum
    (
    int  aAxis   //!< axis
    )
{
    if( this->isVisible() )
    {
        if( PLOT_2D_TYPE_CEPSTRUM == mType && aAxis == mAxis )
        {
            mPlot2dCanvas.update();
        }
    }
}


//!************************************************************************
//! Signal receiver for new accelerometer data
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::receiveNewData()
{
    if( this->isVisible() )
    {
        if( PLOT_2D_TYPE_TRANSIENT == mType )
        {
            mPlot2dCanvas.update();
        }
    }
}


//!************************************************************************
//! Signal receiver for new FFT calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::receiveNewFft
    (
    int  aAxis   //!< axis
    )
{
    if( this->isVisible() )
    {
        if( PLOT_2D_TYPE_FFT == mType && aAxis == mAxis )
        {
            mPlot2dCanvas.update();
        }
    }
}


//!************************************************************************
//! Signal receiver for new periodogram calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::receiveNewPeriodogram
    (
    int  aAxis   //!< axis
    )
{
    if( this->isVisible() )
    {
        if( PLOT_2D_TYPE_PERIODOGRAM == mType && aAxis == mAxis )
        {
            mPlot2dCanvas.update();
        }
    }
}


//!************************************************************************
//! Signal receiver for new SRS calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot2d::receiveNewSrs
    (
    int  aAxis   //!< axis
    )
{
    if( this->isVisible() )
    {
        if( PLOT_2D_TYPE_SRS == mType && aAxis == mAxis )
        {
            mPlot2dCanvas.update();
        }
    }
}


//!************************************************************************
//! Set the SPS and FFT size parameters
//!
//! @returns nothing
//!************************************************************************
void Plot2d::setParameters
    (
    const double   aSps,        //!< sampling rate [Hz]
    const uint32_t aFftSize     //!< FFT size
    )
{
    if( aSps > 0 && aFftSize > 0 )
    {
        mPlot2dCanvas.setParameters( aSps, aFftSize );
        mPlot2dCanvas.update();
    }
}


//!************************************************************************
//! Set the maximum vertical value for transient plots
//!
//! @returns nothing
//!************************************************************************
void Plot2d::setVerticalMaxTransient
    (
    const double aVerticalMax   //!< max value for vertical axis [g]
    )
{
    mPlot2dCanvas.setVerticalMaxTransient( aVerticalMax );
    mPlot2dCanvas.update();
}


//!************************************************************************
//! Update the horizontal menu items
//!
//! @returns nothing
//!************************************************************************
void Plot2d::updateMenuHoriz()
{
    mPlot2dUi->actionHorizLinear->setText( ( AXIS_TYPE_LINEAR == mHorizAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LINEAR ) );
    mPlot2dUi->actionHorizLog->setText( ( AXIS_TYPE_LOG == mHorizAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LOG ) );

    int horizAxisGridLines = mPlot2dCanvas.getGridLinesHorizAxis();

    mPlot2dUi->actionHorizGridLines3->setText( ( 3 == horizAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_3 ) );
    mPlot2dUi->actionHorizGridLines4->setText( ( 4 == horizAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_4 ) );
    mPlot2dUi->actionHorizGridLines5->setText( ( 5 == horizAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_5 ) );
}


//!************************************************************************
//! Update the vertical menu items
//!
//! @returns nothing
//!************************************************************************
void Plot2d::updateMenuVert()
{
    mPlot2dUi->actionVertLinear->setText( ( AXIS_TYPE_LINEAR == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LINEAR ) );
    mPlot2dUi->actionVertLog->setText( ( AXIS_TYPE_LOG == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LOG ) );
    mPlot2dUi->actionVertDb->setText( ( AXIS_TYPE_DB == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_DB ) );

    int vertAxisGridLines = mPlot2dCanvas.getGridLinesVertAxis();

    mPlot2dUi->actionVertGridLines3->setText( ( 3 == vertAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_3 ) );
    mPlot2dUi->actionVertGridLines4->setText( ( 4 == vertAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_4 ) );
    mPlot2dUi->actionVertGridLines5->setText( ( 5 == vertAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_5 ) );
}
