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
GlWidget.cpp

This file contains the sources for the OpenGL widget.
*/

#include "GlWidget.h"

#include <GL/gl.h>

#include <cmath>

#include <QCoreApplication>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QPaintEngine>


//!************************************************************************
//! Constructor
//!************************************************************************
GlWidget::GlWidget
    (
    QWidget* aParent        //!< parent widget
    )
    : QOpenGLWidget( aParent )
    , mRotationAngleX( 0 )
    , mRotationAngleY( 0 )
    , mRotationAngleZ( 0 )
{
}


//!************************************************************************
//! Destructor
//!************************************************************************
GlWidget::~GlWidget()
{
}


//!************************************************************************
//! OpenGL initialize
//!
//! @returns: nothing
//!************************************************************************
/* override */ void GlWidget::initializeGL()
{
    initializeOpenGLFunctions();
}


//!************************************************************************
//! OpenGL paint
//!
//! @returns: nothing
//!************************************************************************
/* override */ void GlWidget::paintGL()
{
    glPushMatrix();
        glRotated(  mRotationAngleX, 1, 0, 0 );
        glRotated( -mRotationAngleY, 0, 0, 1 );
        glRotated(  mRotationAngleZ, 0, 1, 0 );

        const float SCALE = 0.5f;
        glScalef( SCALE, SCALE, SCALE );
        glEnable( GL_LINE_SMOOTH );
        glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
        glLineWidth( 3.0 );

        glBegin( GL_LINES );
            glColor3f( 1, 0, 0 );
            glVertex3f( 0, 0, 0 );
            glVertex3f( 1, 0, 0);

            glColor3f( 0, 0, 1 );
            glVertex3f( 0, 0, 0 );
            glVertex3f( 0, 1, 0 );

            glColor3f( 0, 1, 0 );
            glVertex3f( 0, 0, 0 );
            glVertex3f( 0, 0, -1 );
        glEnd();

            glLineWidth( 1.0 );
            glColor3f( 0.25, 0.25, 0.25 );

        glBegin( GL_LINES );
            glVertex3f( 0, 1, 0 );
            glVertex3f( 1, 1, 0 );

            glVertex3f( 0, 1, -1 );
            glVertex3f( 1, 1, -1 );

            glVertex3f( 0, 0, -1 );
            glVertex3f( 1, 0, -1 );

            glVertex3f( 1, 1, 0 );
            glVertex3f( 1, 0, 0 );

            glVertex3f( 1, 1, -1 );
            glVertex3f( 1, 0, -1 );

            glVertex3f( 0, 1, -1 );
            glVertex3f( 0, 0, -1 );

            glVertex3f( 1, 0, 0 );
            glVertex3f( 1, 0, -1 );

            glVertex3f( 1, 1, 0 );
            glVertex3f( 1, 1, -1 );

            glVertex3f( 0, 1, 0 );
            glVertex3f( 0, 1, -1 );
        glEnd();
    glPopMatrix();
}


//!************************************************************************
//! Set angles on all three axes
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void GlWidget::setAngles
    (
    double aAngleX,     //!< angle on X-axis [deg]
    double aAngleY,     //!< angle on Y-axis [deg]
    double aAngleZ      //!< angle on Z-axis [deg]
    )
{
    mRotationAngleX = aAngleX;
    mRotationAngleY = aAngleY;
    mRotationAngleZ = aAngleZ;
    update();
}
