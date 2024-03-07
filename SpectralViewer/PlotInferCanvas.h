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
PlotInferCanvas.h

This file contains the definitions for drawing TensorRT inference plots.
*/

#ifndef PlotInferCanvas_h
#define PlotInferCanvas_h

#include <cstdint>
#include <vector>

#include <QBrush>
#include <QPen>
#include <QPicture>
#include <QString>
#include <QWidget>


//************************************************************************
// Class for drawing inference plots
//************************************************************************
class PlotInferCanvas : public QWidget
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        static const int LEFT_PLOT_SPACE    = 50;   //!< space at the left side of the plot
        static const int BOTTOM_PLOT_SPACE  = 35;   //!< space at the bottom side of the plot

        static const int RIGHT_PLOT_SPACE   = 15;   //!< space at the right side of the plot
        static const int TOP_PLOT_SPACE     = 15;   //!< space at the top side of the plot

        static constexpr float COURIER_NEW_RATIO = 0.423f;      //!< w/h ratio
        const int FONT_SIZE_9 = 9;                              //!< font size 9
        const QFont FONT_9 = QFont( "Courier New", FONT_SIZE_9, QFont::Normal );    //!< Courier New 9


    //************************************************************************
    // functions
    //************************************************************************
    public:
        PlotInferCanvas
            (
            QWidget* aParent        //!< parent widget
            );

        std::vector<double>& getDataVector();

        void setHorizontalMax
            (
            const double aHorizontalMax         //!< max value for horizontal axis
            );

        void setThreshold
            (
            const double aThreshold             //!< threshold
            );

        void setVerticalMax
            (
            const double aVerticalMax           //!< max value for vertical axis
            );

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
        void drawGridLinesH();
        void drawGridLinesV();

        void drawOuterRectangle();

        void drawPlotInference();

        void drawThreshold();

        void drawValuesAbscissa();
        void drawValuesOrdinate();

        QString formatDecimals
            (
            const double aNumber    //!< number to format
            ) const;


    //************************************************************************
    // variables
    //************************************************************************
    private:
        int                 mSizeW;                 //!< horiziontal size of the drawing area
        int                 mSizeH;                 //!< vertical size of the drawing area

        QPicture            mPicture;               //!< picture to draw in
        QPen                mPen;                   //!< pen to draw with
        QBrush              mBrush;                 //!< brush to fill with

        uint8_t             mGridLinesH;            //!< number of horizontal grid lines
        uint8_t             mGridLinesV;            //!< number of vertical grid lines

        double              mMaxHoriz;              //!< maximum value for horizontal axis

        double              mMaxVert;               //!< maximum value on vertical axis
        bool                mHaveVertValues;        //!< true if have vertical axis values
        double              mDataThreshold;         //!< threshold value

        std::vector<double> mDataVector;            //!< inference data
};

#endif // PlotInferCanvas_h
