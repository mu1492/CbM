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
GlWidget.h

This file contains the definitions for the OpenGL widget.
*/

#ifndef GlWidget_h
#define GlWidget_h

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>


class GlWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

    //************************************************************************
    // functions
    //************************************************************************
    public:
        GlWidget
            (
            QWidget* aParent = nullptr      //!< parent widget
            );

        ~GlWidget() override;

    public slots:
        void setAngles
            (
            double aAngleX,     //!< angle on X-axis [deg]
            double aAngleY,     //!< angle on Y-axis [deg]
            double aAngleZ      //!< angle on Z-axis [deg]
            );

    protected:
        void initializeGL() override;

        void paintGL() override;

    //************************************************************************
    // variables
    //************************************************************************
    private:
        double     mRotationAngleX;     //!< rotation angle on X axis [deg]
        double     mRotationAngleY;     //!< rotation angle on Y axis [deg]
        double     mRotationAngleZ;     //!< rotation angle on Z axis [deg]
};

#endif // GlWidget_h
