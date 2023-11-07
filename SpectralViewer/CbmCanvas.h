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
CbmCanvas.h

This file contains the definitions for drawing CbM information.
*/

#ifndef CbmCanvas_h
#define CbmCanvas_h

#include <QBrush>
#include <QPainter>
#include <QPen>
#include <QPicture>
#include <QWidget>


//************************************************************************
// Class for drawing CbM information
//************************************************************************
class CbmCanvas : public QWidget
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        static constexpr float FREE_AREA_H_LOW_SCALE = 0.125f;  //!< non-drawing area at the bottom of the canvas
        static constexpr float FREE_AREA_H_HIGH_SCALE = 0.125f; //!< non-drawing area at the top of the canvas

        static const int AXIS_TOTAL_WIDTH = 160;                //!< content width per axis
        static const int AXIS_LEFT_SPACE = 20;                  //!< left spacer per axis
        static const int AXIS_ZONE_WIDTH = 60;                  //!< D/C/B/A zone rectangle width

        static const int MIN_WIDTH = 3 * AXIS_TOTAL_WIDTH;      //!< minimum width of the drawing area
        static const int MIN_HEIGHT = 360;                      //!< minimum height of the drawing area

        static constexpr float ABOVE_V_CD_SCALE = 1.1f;         //!< draw 10% more for zone D (above vCD)

        static constexpr float COURIER_NEW_RATIO = 0.423f;      //!< w/h ratio
        const int FONT_SIZE_9 = 9;                              //!< font size 9
        const QFont FONT_9 = QFont( "Courier New", FONT_SIZE_9, QFont::Normal );    //!< Courier New 9

        const QBrush RED_BRUSH    = QBrush( QColor( 255, 0, 0 ) );      //!< solid red brush
        const QBrush ORANGE_BRUSH = QBrush( QColor( 255, 165, 0 ) );    //!< solid orange brush
        const QBrush YELLOW_BRUSH = QBrush( QColor( 239, 239, 0 ) );    //!< solid yellow brush
        const QBrush GREEN_BRUSH  = QBrush( QColor( 0, 159, 0 ) );      //!< solid green brush

        const QPen NO_PEN = QPen( Qt::NoPen );                                  //!< no pen
        const QPen CYAN_PEN = QPen( Qt::cyan, 1, Qt::SolidLine );               //!< cyan pen
        const QPen LIGHT_GRAY_PEN = QPen( Qt::lightGray, 1, Qt::SolidLine );    //!< light gray pen
        const QPen WHITE_PEN = QPen( Qt::white, 1, Qt::SolidLine );             //!< white pen
        const QPen WHITE_PEN_2 = QPen( Qt::white, 2, Qt::SolidLine );           //!< white pen, thickness=2


    //************************************************************************
    // functions
    //************************************************************************
    public:
        CbmCanvas
            (
            QWidget*        aParent = nullptr   //!< parent widget
            );

        QSize minimumSizeHint() const override;

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
        void drawCbmZones();

        void drawRmsSpeedLevels();

    private slots:
        void updateRmsSpeedLevels();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        int                 mSizeW;                 //!< horiziontal size of the drawing area
        int                 mSizeH;                 //!< vertical size of the drawing area

        QPicture            mPicture;               //!< picture to draw in
        QPen                mPen;                   //!< pen to draw with
        QBrush              mBrush;                 //!< brush to fill with
};

#endif // CbmCanvas_h
