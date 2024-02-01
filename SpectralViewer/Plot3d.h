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
Plot3d.h

This file contains the definitions for plotting 3D waveforms.
*/

#ifndef Plot3d_h
#define Plot3d_h

#include "Adxl355Adxl357Common.h"
#include "Plot3dCanvas.h"

#include <cstdint>
#include <map>

#include <QCloseEvent>
#include <QMainWindow>
#include <QString>

QT_BEGIN_NAMESPACE
    namespace Ui
    {
        class Plot3d;
    }
QT_END_NAMESPACE


//************************************************************************
// Class for plotting 3D waveforms
//************************************************************************
class Plot3d : public QMainWindow
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        typedef enum : uint8_t
        {
            PLOT_3D_TYPE_TRANSIENT,
            PLOT_3D_TYPE_FFT,
            PLOT_3D_TYPE_PERIODOGRAM,
            PLOT_3D_TYPE_SRS,
            PLOT_3D_TYPE_CEPSTRUM,

            // keep this last
            PLOT_3D_TYPE_COUNT
        }Plot3dType;

        typedef enum : uint8_t
        {
            AXIS_TYPE_LINEAR,
            AXIS_TYPE_LOG,
            AXIS_TYPE_DB
        }AxisType;

    private:
        static const std::map<Plot3dType, QString> PLOT_3D_TYPE_NAMES;

        static const std::map<AxisType, QString> AXIS_TYPE_NAMES;

        const QString MENU_ITEM_MARK = QString::fromUtf8( "\u25CF" );
        const QString MENU_ITEM_SPACE = "   ";


    //************************************************************************
    // functions
    //************************************************************************
    public:
        Plot3d
            (
            QWidget*                    aParent = nullptr,                      //!< parent widget
            Plot3dType                  aType = PLOT_3D_TYPE_TRANSIENT,         //!< plot 3D type
            Adxl355Adxl357Common::Axis  aAxis = Adxl355Adxl357Common::AXIS_X    //!< accelerometer axis
            );

        ~Plot3d();

        Adxl355Adxl357Common::Axis getAxis() const;

        AxisType getAxisTypeHoriz() const;

        AxisType getAxisTypeVert() const;

        Plot3dType getPlot3dType() const;

        void setParameters
            (
            const double   aSps,                //!< sampling rate [Hz]
            const uint32_t aFftSize             //!< FFT size
            );

        void setVerticalMaxTransient
            (
            const double aVerticalMax           //!< max value for vertical axis [g]
            );

    public slots:
        void receiveNewCepstrum
            (
            int aAxis    //!< axis
            );

        void receiveNewData();

        void receiveNewFft
            (
            int aAxis    //!< axis
            );

        void receiveNewPeriodogram
            (
            int aAxis    //!< axis
            );

        void receiveNewSrs
            (
            int aAxis    //!< axis
            );

    protected:
        void closeEvent
            (
            QCloseEvent* aEvent                 //!< close event
            );

        void keyPressEvent
            (
            QKeyEvent*  aEvent                  //!< key event
            ) override;

    private:
        QString formatTitle();

        void updateMenuHoriz();

        void updateMenuVert();

    private slots:
        void handleHorizAxisLinear();
        void handleHorizAxisLog();

        void handleSceneLightToggle();
        void handleSceneMeshToggle();
        void handleSceneResetView();

        void handleVertAxisLinear();
        void handleVertAxisLog();
        void handleVertAxisDb();

    signals:
        void closeSignal
            (
            int aType,                          //!< type of closed window
            int aAxis                           //!< axis of closed window
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        Ui::Plot3d*                 mPlot3dUi;      //!< plot 3D UI

        Plot3dType                  mType;          //!< plot 3D type
        Adxl355Adxl357Common::Axis  mAxis;          //!< axis

        Plot3dCanvas                mPlot3dCanvas;  //!< canvas to draw on

        AxisType                    mHorizAxisType; //!< linear or log
        AxisType                    mVertAxisType;  //!< linear or log
};

#endif // Plot3d_h
