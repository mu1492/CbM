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
Plot.cpp

This file contains the sources for plotting waveforms.
*/

#include "Plot.h"
#include "./ui_Plot.h"

#include "PlotCanvas.h"
#include "SpectralViewer.h"

#include <QPalette>


const std::map<Plot::PlotType, QString> Plot::PLOT_TYPE_NAMES =
{
    { Plot::PLOT_TYPE_TRANSIENT,   "Transient"   },
    { Plot::PLOT_TYPE_FFT,         "FFT"         },
    { Plot::PLOT_TYPE_PERIODOGRAM, "Periodogram" },
    { Plot::PLOT_TYPE_SRS,         "SRS"         },
    { Plot::PLOT_TYPE_CEPSTRUM,    "Cepstrum"    }
};

const std::map<Plot::AxisType, QString> Plot::AXIS_TYPE_NAMES =
{
    { Plot::AXIS_TYPE_LINEAR, "Li&near" },
    { Plot::AXIS_TYPE_LOG,    "L&og"    },
    { Plot::AXIS_TYPE_DB,     "d&B"     }
};

const std::map<Plot::GridLines, QString> Plot::GRID_LINES_NAMES =
{
    { Plot::GRID_LINES_3, "&3 grid lines" },
    { Plot::GRID_LINES_4, "&4 grid lines" },
    { Plot::GRID_LINES_5, "&5 grid lines" }
};

//!************************************************************************
//! Constructor
//!************************************************************************
Plot::Plot
    (
    QWidget*                    aParent,    //!< parent widget
    PlotType                    aType,      //!< plot type
    Adxl355Adxl357Common::Axis  aAxis       //!< accelerometer axis
    )
    : QMainWindow( aParent )
    , mType( aType )
    , mAxis( aAxis )    
    // UI
    , mPlotUi( new Ui::Plot )
    // paint&draw
    , mPlotCanvas( this, *this )
    // axes
    , mHorizAxisType( AXIS_TYPE_LINEAR )
    , mVertAxisType( AXIS_TYPE_LINEAR )
{
    mPlotUi->setupUi( this );

    this->setWindowTitle( formatTitle() );

    //****************************************
    // menus
    //****************************************
    switch( mType )
    {
        case PLOT_TYPE_TRANSIENT:
            mPlotUi->actionHorizLog->setEnabled( false );

            mPlotUi->actionVertLog->setEnabled( false );
            mPlotUi->actionVertDb->setEnabled( false );
            break;

        case PLOT_TYPE_FFT:
            mPlotUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlotUi->actionVertLog->setEnabled( false );
            break;

        case PLOT_TYPE_PERIODOGRAM:
            mPlotUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlotUi->actionVertLog->setEnabled( false );
            break;

        case PLOT_TYPE_SRS:
            mHorizAxisType = AXIS_TYPE_LOG;
            mPlotUi->menuHorizontalAxis->setEnabled( false );

            mVertAxisType = AXIS_TYPE_LOG;
            mPlotUi->menuVerticalAxis->setEnabled( false );
            break;

        case PLOT_TYPE_CEPSTRUM:
            mPlotUi->actionHorizLog->setEnabled( false );

            mVertAxisType = AXIS_TYPE_DB;
            mPlotUi->actionVertLinear->setEnabled( false );
            mPlotUi->actionVertLog->setEnabled( false );
            break;

        default:
            break;
    }

    connect( mPlotUi->actionHorizLinear, &QAction::triggered, this, &Plot::handleHorizAxisLinear );
    connect( mPlotUi->actionHorizLog, &QAction::triggered, this, &Plot::handleHorizAxisLog );

    connect( mPlotUi->actionHorizGridLines3, &QAction::triggered, this, &Plot::handleHorizGridLines3 );
    connect( mPlotUi->actionHorizGridLines4, &QAction::triggered, this, &Plot::handleHorizGridLines4 );
    connect( mPlotUi->actionHorizGridLines5, &QAction::triggered, this, &Plot::handleHorizGridLines5 );

    connect( mPlotUi->actionVertLinear, &QAction::triggered, this, &Plot::handleVertAxisLinear );
    connect( mPlotUi->actionVertLog, &QAction::triggered, this, &Plot::handleVertAxisLog );
    connect( mPlotUi->actionVertDb, &QAction::triggered, this, &Plot::handleVertAxisDb );

    connect( mPlotUi->actionVertGridLines3, &QAction::triggered, this, &Plot::handleVertGridLines3 );
    connect( mPlotUi->actionVertGridLines4, &QAction::triggered, this, &Plot::handleVertGridLines4 );
    connect( mPlotUi->actionVertGridLines5, &QAction::triggered, this, &Plot::handleVertGridLines5 );


    //****************************************
    // paint&draw
    //****************************************
    setCentralWidget( &mPlotCanvas );
    this->adjustSize();

    mPlotCanvas.update();

    updateMenuHoriz();
    updateMenuVert();
}


//!************************************************************************
//! Destructor
//!************************************************************************
Plot::~Plot()
{
    delete mPlotUi;
}


//!************************************************************************
//! Close event handler
//!
//! @returns nothing
//!************************************************************************
void Plot::closeEvent
    (
    QCloseEvent*    aEvent      //!< close event
    )
{
    emit closeSignal( static_cast<int>( mType ), static_cast<int>( mAxis ) );
    aEvent->accept();
}


//!************************************************************************
//! Close event handler
//!
//! @returns nothing
//!************************************************************************
QString Plot::formatTitle()
{
    QString titleStr = SpectralViewer::APP_NAME + " - ";
    titleStr += QChar( 'X' + mAxis - Adxl355Adxl357Common::AXIS_X );
    titleStr += " axis - ";
    titleStr += PLOT_TYPE_NAMES.at( mType );

    return titleStr;
}


//!************************************************************************
//! Get the axis - X, Y, or Z
//!
//! @returns The axis
//!************************************************************************
Adxl355Adxl357Common::Axis Plot::getAxis() const
{
    return mAxis;
}


//!************************************************************************
//! Get the horizontal axis type - Linear or Log
//!
//! @returns The horizontal axis type
//!************************************************************************
Plot::AxisType Plot::getAxisTypeHoriz() const
{
    return mHorizAxisType;
}


//!************************************************************************
//! Get the vertical axis type - Linear, Log, or dB
//!
//! @returns The vertical axis type
//!************************************************************************
Plot::AxisType Plot::getAxisTypeVert() const
{
    return mVertAxisType;
}


//!************************************************************************
//! Get the plot type
//!
//! @returns The plot type
//!************************************************************************
Plot::PlotType Plot::getPlotType() const
{
    return mType;
}


//!************************************************************************
//! Handle for setting linear horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleHorizAxisLinear()
{
    mHorizAxisType = AXIS_TYPE_LINEAR;
    updateMenuHoriz();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting logarithmic horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleHorizAxisLog()
{
    mHorizAxisType = AXIS_TYPE_LOG;
    updateMenuHoriz();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting 3 grid lines on the horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleHorizGridLines3()
{
    mPlotCanvas.setGridLinesHorizAxis( 3 );
    updateMenuHoriz();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting 4 grid lines on the horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleHorizGridLines4()
{
    mPlotCanvas.setGridLinesHorizAxis( 4 );
    updateMenuHoriz();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting 5 grid lines on the horizontal axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleHorizGridLines5()
{
    mPlotCanvas.setGridLinesHorizAxis( 5 );
    updateMenuHoriz();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting linear vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleVertAxisLinear()
{
    mVertAxisType = AXIS_TYPE_LINEAR;
    updateMenuVert();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting logarithmic vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleVertAxisLog()
{
    mVertAxisType = AXIS_TYPE_LOG;
    updateMenuVert();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting dB vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleVertAxisDb()
{
    mVertAxisType = AXIS_TYPE_DB;
    updateMenuVert();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting 3 grid lines on the vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleVertGridLines3()
{
    mPlotCanvas.setGridLinesVertAxis( 3 );
    updateMenuVert();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting 4 grid lines on the vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleVertGridLines4()
{
    mPlotCanvas.setGridLinesVertAxis( 4 );
    updateMenuVert();
    mPlotCanvas.update();
}


//!************************************************************************
//! Handle for setting 5 grid lines on the vertical axis
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::handleVertGridLines5()
{
    mPlotCanvas.setGridLinesVertAxis( 5 );
    updateMenuVert();
    mPlotCanvas.update();
}


//!************************************************************************
//! Signal receiver for new cepstrum calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::receiveNewCepstrum
    (
    int  aAxis   //!< axis
    )
{
    if( PLOT_TYPE_CEPSTRUM == mType && aAxis == mAxis )
    {
        mPlotCanvas.update();
    }
}


//!************************************************************************
//! Signal receiver for new accelerometer data
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::receiveNewData()
{
    if( PLOT_TYPE_TRANSIENT == mType )
    {
        mPlotCanvas.update();
    }
}


//!************************************************************************
//! Signal receiver for new FFT calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::receiveNewFft
    (
    int  aAxis   //!< axis
    )
{
    if( PLOT_TYPE_FFT == mType && aAxis == mAxis )
    {
        mPlotCanvas.update();
    }
}


//!************************************************************************
//! Signal receiver for new periodogram calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::receiveNewPeriodogram
    (
    int  aAxis   //!< axis
    )
{
    if( PLOT_TYPE_PERIODOGRAM == mType && aAxis == mAxis )
    {
        mPlotCanvas.update();
    }
}


//!************************************************************************
//! Signal receiver for new SRS calculations
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot::receiveNewSrs
    (
    int  aAxis   //!< axis
    )
{
    if( PLOT_TYPE_SRS == mType && aAxis == mAxis )
    {
        mPlotCanvas.update();
    }
}


//!************************************************************************
//! Set the SPS and FFT size parameters
//!
//! @returns nothing
//!************************************************************************
void Plot::setParameters
    (
    const double   aSps,        //!< sampling rate [Hz]
    const uint32_t aFftSize     //!< FFT size
    )
{
    if( aSps > 0 && aFftSize > 0 )
    {
        mPlotCanvas.setParameters( aSps, aFftSize );
        mPlotCanvas.update();
    }
}


//!************************************************************************
//! Set the maximum vertical value for transient plots
//!
//! @returns nothing
//!************************************************************************
void Plot::setVerticalMaxTransient
    (
    const double aVerticalMax   //!< max value for vertical axis [g]
    )
{
    mPlotCanvas.setVerticalMaxTransient( aVerticalMax );
    mPlotCanvas.update();
}


//!************************************************************************
//! Update the horizontal menu items
//!
//! @returns nothing
//!************************************************************************
void Plot::updateMenuHoriz()
{
    mPlotUi->actionHorizLinear->setText( ( AXIS_TYPE_LINEAR == mHorizAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LINEAR ) );
    mPlotUi->actionHorizLog->setText( ( AXIS_TYPE_LOG == mHorizAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LOG ) );

    int horizAxisGridLines = mPlotCanvas.getGridLinesHorizAxis();

    mPlotUi->actionHorizGridLines3->setText( ( 3 == horizAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_3 ) );
    mPlotUi->actionHorizGridLines4->setText( ( 4 == horizAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_4 ) );
    mPlotUi->actionHorizGridLines5->setText( ( 5 == horizAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_5 ) );
}


//!************************************************************************
//! Update the vertical menu items
//!
//! @returns nothing
//!************************************************************************
void Plot::updateMenuVert()
{
    mPlotUi->actionVertLinear->setText( ( AXIS_TYPE_LINEAR == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LINEAR ) );
    mPlotUi->actionVertLog->setText( ( AXIS_TYPE_LOG == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_LOG ) );
    mPlotUi->actionVertDb->setText( ( AXIS_TYPE_DB == mVertAxisType ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + AXIS_TYPE_NAMES.at( AXIS_TYPE_DB ) );

    int vertAxisGridLines = mPlotCanvas.getGridLinesVertAxis();

    mPlotUi->actionVertGridLines3->setText( ( 3 == vertAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_3 ) );
    mPlotUi->actionVertGridLines4->setText( ( 4 == vertAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_4 ) );
    mPlotUi->actionVertGridLines5->setText( ( 5 == vertAxisGridLines ? MENU_ITEM_MARK : MENU_ITEM_SPACE ) +
                " " + GRID_LINES_NAMES.at( GRID_LINES_5 ) );
}
