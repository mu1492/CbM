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
Plot3dCanvas.h

This file contains the definitions for drawing 3D plots.
*/

#ifndef Plot3dCanvas_h
#define Plot3dCanvas_h

#include "VibrationHandler.h"

#include <cstdint>

#include <QFont>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QString>
#include <QTimer>

class Plot3d;


//************************************************************************
// Class for drawing 2D plots
//************************************************************************
class Plot3dCanvas : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        static const int MIN_WIDTH  = 400;          //!< minimum width of the drawing area
        static const int MIN_HEIGHT = 400;          //!< minimum height of the drawing area

        static constexpr float MESH_COLORS[][3] =   //!< mesh colors
        {
            0.0,    0.0,    1.0,    // blue -> z_min
            0.0,    0.125,  1.0,
            0.0,    0.25,   1.0,
            0.0,    0.375,  1.0,
            0.0,    0.5,    1.0,
            0.0,    0.625,  1.0,
            0.0,    0.75,   1.0,
            0.0,    0.875,  1.0,
            0.0,    1.0,    1.0,    // cyan
            0.0,    1.0,    0.875,
            0.0,    1.0,    0.75,
            0.0,    1.0,    0.625,
            0.0,    1.0,    0.5,
            0.0,    1.0,    0.375,
            0.0,    1.0,    0.25,
            0.0,    1.0,    0.125,
            0.0,    1.0,    0.0,    // green
            0.125,  1.0,    0.0,
            0.25,   1.0,    0.0,
            0.375,  1.0,    0.0,
            0.5,    1.0,    0.0,
            0.625,  1.0,    0.0,
            0.75,   1.0,    0.0,
            0.875,  1.0,    0.0,
            1.0,    1.0,    0.0,    // yellow
            1.0,    0.875,  0.0,
            1.0,    0.75,   0.0,
            1.0,    0.625,  0.0,
            1.0,    0.5,    0.0,
            1.0,    0.375,  0.0,
            1.0,    0.25,   0.0,
            1.0,    0.125,  0.0,
            1.0,    0.0,    0.0,    // red
            1.0,    0.0,    0.125,
            1.0,    0.0,    0.25,
            1.0,    0.0,    0.375,
            1.0,    0.0,    0.5,
            1.0,    0.0,    0.625,
            1.0,    0.0,    0.75,
            1.0,    0.0,    0.875,
            1.0,    0.0,    1.0     // magenta -> z_max
        };

        static const uint8_t MESH_COLORS_NR = sizeof( MESH_COLORS ) / ( sizeof( MESH_COLORS[0] ) );   //!< number of mesh colors

        static const uint8_t REFRESH_MS = 100;  //!< refresh period [ms]

    //************************************************************************
    // functions
    //************************************************************************
    public:
        Plot3dCanvas
            (
            QWidget* aParent,           //!< parent widget
            Plot3d&  aParentPlot3d      //!< parent Plot3d object
            );

        ~Plot3dCanvas();

        QSize minimumSizeHint() const override;

        void setParameters
            (
            const double   aSps,                //!< sampling rate [Hz]
            const uint32_t aFftSize             //!< FFT size
            );

        void setVerticalMaxTransient
            (
            const double aVerticalMax           //!< max value for vertical axis [g]
            );

        QSize sizeHint() const override;

        void resetView();

        void stopRefresh();

        void toogleLightEnable();

        void toogleMeshFill();

        void updateAdxlCepstrum();
        void updateAdxlData();
        void updateAdxlFft();
        void updateAdxlPeriodogram();
        void updateAdxlSrs();        

        void updateKeyDown();
        void updateKeyLeft();
        void updateKeyRight();
        void updateKeyUp();

    protected:
        void initializeGL() override;

        void mouseMoveEvent
            (
            QMouseEvent*    aEvent              //!< mouse move event
            ) override;

        void mousePressEvent
            (
            QMouseEvent*    aEvent              //!< mouse press event
            ) override;

        void mouseReleaseEvent
            (
            QMouseEvent*    aEvent              //!< mouse release event
            ) override;

        void paintGL() override;

        void resizeGL
            (
            int aWidth,                         //!< width
            int aHeight                         //!< height
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

        void cleanupAll();

        void cleanupData();
        void cleanupDataBuf();

        void convertToScreenCoords
            (
            const float aProjX,     //!< input x
            const float aProjY,     //!< input y
            int*        aScreenX,   //!< screen x
            int*        aScreenY    //!< screen y
            );

        void init3dMeshList();

        void initColorsList();

        void make3dDataAllocateData();
        void make3dDataAllocateDataBuf();

        void make3dDataAllocateVxsNs();

        void make3dDataAssign();
        void make3dDataCompute();        

        void rotatePt2Vec
            (
            int       aX,           //!< x coordinate
            int       aY,           //!< y coordinate
            double    aVec[3]       //!< vector
            ) const;

        void outputScaledText
            (
            const float     aX,         //!< x coordinate
            const float     aY,         //!< y coordinate
            const float     aScale,     //!< scale
            const QString   aString,    //!< string
            QColor          aColor      //!< color
            );

        void updateMinFrequencyLog();

    private slots:
        void cleanupVxsNs();

        void refresh();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        Plot3d&             mParentPlot3d;          //!< parent Plot3d object

        int                 mSizeW;                 //!< horiziontal size of the drawing area
        int                 mSizeH;                 //!< vertical size of the drawing area

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

        uint32_t            mColorsList;            //!< colors list
        bool                mLightEnabled;          //!< true if light is enabled

        uint32_t            mMeshList;              //!< mesh list
        double**            mMeshData;              //!< mesh data
        double**            mMeshDataBuf;           //!< mesh data buffered
        double***           mMeshVxs;               //!< mesh vertices
        double***           mMeshNs;                //!< mesh normals
        bool                mMeshFill;              //!< true if mesh is filled
        uint16_t            mMeshDeltaX;            //!< X delta mesh
        uint16_t            mMeshDeltaY;            //!< Y delta mesh

        double              mRotAngleDeg;           //!< rotation angle [degrees]
        double              mRotAxis[3];            //!< rotation axis
        double              mRotTransform[4][4];    //!< rotation transform
        double              mRotLastPosition[3];    //!< rotation last position
        int                 mRotMouseButton;        //!< mouse button for plot rotation
        bool                mRotIsActive;           //!< rotation status

        double              mZmin;                  //!< maximum value on z axis
        double              mZmax;                  //!< maximum value on z axis

        QTimer*             mRefreshTimer;          //!< refresh timer
};

#endif // Plot3dCanvas_h
