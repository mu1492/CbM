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
Plot2dCanvas.h

This file contains the definitions for drawing 2D plots.
*/

#ifndef Plot2dCanvas_h
#define Plot2dCanvas_h

#include "VibrationHandler.h"

#include <cstdint>

#include <QBrush>
#include <QPen>
#include <QPicture>
#include <QString>
#include <QWidget>

class Plot2d;


//************************************************************************
// Class for drawing 2D plots
//************************************************************************
class Plot2dCanvas : public QWidget
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        static const int MIN_WIDTH  = 400;          //!< minimum width of the drawing area
        static const int MIN_HEIGHT = 300;          //!< minimum height of the drawing area

        static const int LEFT_PLOT_SPACE    = 80;   //!< space at the left side of the plot
        static const int BOTTOM_PLOT_SPACE  = 35;   //!< space at the bottom side of the plot

        static const int RIGHT_PLOT_SPACE   = 15;   //!< space at the right side of the plot
        static const int TOP_PLOT_SPACE     = 15;   //!< space at the top side of the plot

        static constexpr float COURIER_NEW_RATIO = 0.423f;      //!< w/h ratio
        const int FONT_SIZE_9 = 9;                              //!< font size 9
        const QFont FONT_9 = QFont( "Courier New", FONT_SIZE_9, QFont::Normal );    //!< Courier New 9

        const QColor RED_COLOR      = QColor( 255, 0, 0 );    //!< red
        const QColor GREEN_COLOR    = QColor( 0, 255, 0 );    //!< green
        const QColor YELLOW_COLOR   = QColor( 255, 255, 0 );  //!< yellow
        const QColor MAGENTA_COLOR  = QColor( 255, 63, 255 ); //!< magenta
        const QColor BLUE_COLOR     = QColor( 95, 95, 255 );  //!< blue


    //************************************************************************
    // functions
    //************************************************************************
    public:
        Plot2dCanvas
            (
            QWidget* aParent,           //!< parent widget
            Plot2d&  aParentPlot2d      //!< parent Plot2d object
            );

        QSize minimumSizeHint() const override;

        uint8_t getGridLinesHorizAxis() const;

        uint8_t getGridLinesVertAxis() const;

        void setParameters
            (
            const double   aSps,                //!< sampling rate [Hz]
            const uint32_t aFftSize             //!< FFT size
            );

        void setGridLinesHorizAxis
            (
            const uint8_t aGridLinesNumber      //!< number of grid lines
            );

        void setGridLinesVertAxis
            (
            const uint8_t aGridLinesNumber      //!< number of grid lines
            );

        void setVerticalMaxTransient
            (
            const double aVerticalMax           //!< max value for vertical axis [g]
            );

        QSize sizeHint() const override;

    protected:
        void paintEvent
            (
            QPaintEvent*    aEvent              //!< paint event
            ) override;

        void resizeEvent
            (
            QResizeEvent*   aEvent              //!< resize event
            ) override;


    private:
        static bool compareBinFnc
            (
            VibrationHandler::FftBin x,     //!< first object
            VibrationHandler::FftBin y      //!< second object
            );

        static bool compareCepstrumFnc
            (
            VibrationHandler::FftCepstrum x,     //!< first object
            VibrationHandler::FftCepstrum y      //!< second object
            );

        static bool comparePsdFnc
            (
            VibrationHandler::FftPsd x,     //!< first object
            VibrationHandler::FftPsd y      //!< second object
            );

        static bool compareSrsFnc
            (
            VibrationHandler::Srs x,        //!< first object
            VibrationHandler::Srs y         //!< second object
            );

        void drawGridLinesH();
        void drawGridLinesV();

        void drawOuterRectangle();

        void drawPlot2dTransient();
        void drawPlot2dFft();
        void drawPlot2dPeriodogram();
        void drawPlot2dSrs();
        void drawPlot2dCepstrum();

        void drawValuesAbscissa();
        void drawValuesOrdinate();

        QString formatDecimals
            (
            const double aNumber    //!< number to format
            ) const;

        QString formatScientific
            (
            const double aNumber    //!< number to format
            ) const;

        void updateMinFrequencyLog();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        Plot2d&             mParentPlot2d;          //!< parent Plot2d object

        int                 mSizeW;                 //!< horiziontal size of the drawing area
        int                 mSizeH;                 //!< vertical size of the drawing area

        QPicture            mPicture;               //!< picture to draw in
        QPen                mPen;                   //!< pen to draw with
        QBrush              mBrush;                 //!< brush to fill with

        uint8_t             mGridLinesH;            //!< number of horizontal grid lines
        uint8_t             mGridLinesV;            //!< number of vertical grid lines

        double              mSps;                   //!< sampling rate [Hz]
        int                 mFftSize;               //!< FFT size

        double              mMaxFreq;               //!< maximum frequency [Hz]
        double              mMaxFreqLog;            //!< maximum frequency for log scale [Hz]
        double              mMinFreqLog;            //!< minimum frequency for log scale [Hz]

        double              mMaxQuefrency;          //!< maximum cepstrum quefrency [s]

        double              mBinWidth;              //!< FFT bin width [Hz/bin]
        double              mTimeGate;              //!< time gate [s]

        double              mMaxVert;               //!< maximum value on vertical axis
        double              mMinVert;               //!< minimum value on vertical axis
        bool                mHaveVertValues;        //!< true if have vertical axis values
};

#endif // Plot2dCanvas_h
