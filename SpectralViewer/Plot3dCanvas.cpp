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
Plot3dCanvas.cpp

This file contains the sources for drawing 3D plots.
*/

#include "Plot3dCanvas.h"

#include "Numeric.h"
#include "Plot3d.h"
#include "SrsThread.h"

#include <float.h>
#include <GL/gl.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

#include <QMouseEvent>
#include <QPainter>


//!************************************************************************
//! Constructor
//!************************************************************************
Plot3dCanvas::Plot3dCanvas
    (
    QWidget* aParent,           //!< parent widget
    Plot3d&  aParentPlot3d      //!< parent Plot3d object
    )
    : QOpenGLWidget( aParent )
    , mParentPlot3d( aParentPlot3d )
    // widget size
    , mSizeW( width() )
    , mSizeH( height() )
    // horizontal axis
    , mSps( 0 )
    , mFftSize( 0 )
    , mMaxFreq( 0 )
    , mMaxFreqLog( 0 )
    , mMinFreqLog( 0 )
    , mMaxQuefrency( 0 )
    , mBinWidth( 0 )
    , mTimeGate( 0 )
    // vertical axis
    , mMaxVert( -DBL_MAX )
    , mMinVert( DBL_MAX )
    , mHaveVertValues( false )
    // GL
    , mColorsList( 0 )
    , mLightEnabled( true )
    , mMeshList( 0 )
    , mMeshData( nullptr )
    , mMeshVxs( nullptr )
    , mMeshNs( nullptr )
    , mMeshFill( true )
    , mMeshDeltaX( 2 )
    , mMeshDeltaY( 2 )
    // rotation
    , mRotAngleDeg( 0 )
    , mRotMouseButton( Qt::LeftButton )
    , mRotIsActive( false )
    // ranges
    , mXmin( 0 )
    , mXmax( 0 )
    , mYmin( 0 )
    , mYmax( 0 )
    , mZmin( 0 )
    , mZmax( 0 )
{
    memset( &mRotAxis, 0, sizeof( mRotAxis ) );
    memset( &mRotTransform, 0, sizeof( mRotTransform ) );
    memset( &mRotLastPosition, 0, sizeof( mRotLastPosition ) );
}


//!************************************************************************
//! Destructor
//!************************************************************************
Plot3dCanvas::~Plot3dCanvas()
{
}


//!************************************************************************
//! Convert projected to screen coordinates
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::convertToScreenCoords
    (
    const float aProjX,     //!< input x
    const float aProjY,     //!< input y
    int*        aScreenX,   //!< output x
    int*        aScreenY    //!< output y
    )
{
    if( aScreenX && aScreenY )
    {
        *aScreenX = static_cast<int>( 0.5 * mSizeW * ( 1 + aProjX ) );
        *aScreenY = static_cast<int>( 0.5 * mSizeH * ( 1 - aProjY ) );
    }
}


//!************************************************************************
//! Data cleanup - deallocate memory
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3dCanvas::cleanup()
{
    uint16_t i = 0;
    uint16_t j = 0;

    if( mMeshVxs )
    {
        for( i = 0; i < mMeshDeltaX; i++ )
        {
            for( j = 0; j < mMeshDeltaY; j++ )
            {
                delete[] mMeshVxs[i][j];
            }

            delete[] mMeshVxs[i];
        }

        delete[] mMeshVxs;
    }

    if( mMeshNs )
    {
        for( i = 0; i < mMeshDeltaX; i++ )
        {
            for( j = 0; j < mMeshDeltaY; j++ )
            {
                delete[] mMeshNs[i][j];
            }

            delete[] mMeshNs[i];
        }

        delete[] mMeshNs;
    }
}


//!************************************************************************
//! Initialize the 3D mesh list
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::init3dMeshList()
{
    if( mMeshList )
    {
        glDeleteLists( mMeshList, 1 );
    }

    mMeshList = glGenLists( 1 );

    glNewList( mMeshList, GL_COMPILE );
        for( uint16_t i = 0; i < mMeshDeltaX - 1; i++ )
        {
            glPolygonMode( GL_BACK, GL_LINE );
            glBegin( GL_TRIANGLE_STRIP );
                for( uint16_t j = 0; j < mMeshDeltaY; j++ )
                {
                    glColor3fv( MESH_COLORS[ static_cast< int >( MESH_COLORS_NR * ( mMeshVxs[i + 1][j][1] + 0.5 ) ) ] );
                    glNormal3dv( mMeshNs[i + 1][j] );
                    glVertex3dv( mMeshVxs[i + 1][j] );

                    glColor3fv( MESH_COLORS[ static_cast< int >( MESH_COLORS_NR * ( mMeshVxs[i][j][1] + 0.5 ) ) ] );
                    glNormal3dv( mMeshNs[i][j] );
                    glVertex3dv( mMeshVxs[i][j] );
                }
            glEnd();
        }
    glEndList();
}


//!************************************************************************
//! Initialize the colors list
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::initColorsList()
{
    if( mColorsList )
    {
        glDeleteLists( mColorsList, 1 );
    }

    mColorsList = glGenLists( 1 );

    glNewList( mColorsList, GL_COMPILE );
        glPushMatrix();
            glLoadIdentity();
            glTranslated( 0.0, 0.0, -2.0 );
            glPolygonMode( GL_BACK, GL_FILL );
            glNormal3f( 0.5, 0.5, 0.5 );
            const float COLOR_LEGEND_H = 0.5f;
            const float COLOR_INDEX_H = COLOR_LEGEND_H / MESH_COLORS_NR;
            float crtY = -0.75f;

            for( uint8_t i = 0; i < MESH_COLORS_NR; i++ )
            {
                glColor3fv( MESH_COLORS[i] );
                float nextY = -0.75 + ( i + 1 ) * COLOR_INDEX_H;
                glRectf( -0.975, crtY, -0.875, nextY );
                crtY = nextY;
            }
        glPopMatrix();
    glEndList();
}


//!************************************************************************
//! OpenGL initializer
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::initializeGL()
{
    connect( context(), &QOpenGLContext::aboutToBeDestroyed, this, &Plot3dCanvas::cleanup );

    initializeOpenGLFunctions();

    make3dData();
    init3dMeshList();

    initColorsList();

    glPushMatrix();
        glLoadIdentity();
        glGetDoublev( GL_MODELVIEW_MATRIX, reinterpret_cast< double* >( mRotTransform ) );
    glPopMatrix();
}


//!************************************************************************
//! Create the 3D data
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::make3dData()
{
    make3dDataAllocate();
    make3dDataAssign();
    make3dDataCompute();
    make3dDataCleanup();
}


//!************************************************************************
//! Create the 3D data
//! *** allocate memory ***
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::make3dDataAllocate()
{
    mMeshData = new double*[mMeshDeltaX];

    mMeshVxs = new double**[mMeshDeltaX];
    mMeshNs = new double**[mMeshDeltaX];

    for( uint16_t i = 0; i < mMeshDeltaX; i++ )
    {
        mMeshData[i] = new double[mMeshDeltaY];

        mMeshVxs[i] = new double*[mMeshDeltaY];
        mMeshNs[i] = new double*[mMeshDeltaY];

        for( uint16_t j = 0; j < mMeshDeltaY; j++ )
        {
            mMeshVxs[i][j] = new double[3];
            mMeshNs[i][j] = new double[3];
        }
    }
}


//!************************************************************************
//! Create the 3D data
//! *** assign data ***
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::make3dDataAssign()
{
    uint16_t i = 0;
    uint16_t j = 0;

    mZmin = 0;
    mZmax = 0;

    double xStep = ( mXmax - mXmin ) / mMeshDeltaX;
    double yStep = ( mYmax - mYmin ) / mMeshDeltaY;

    for( j = 0; j < mMeshDeltaY; j++ )
    {
        double y_crt = mYmin + j * yStep;

        for( i = 0; i < mMeshDeltaX; i++ )
        {
            double x_crt = mXmin + i * xStep;

            mMeshData[i][j] = sin( x_crt ) * cos( y_crt );

            if( mZmax < mMeshData[i][j] )
            {
                mZmax = mMeshData[i][j];
            }

            if( mZmin > mMeshData[i][j] )
            {
                mZmin = mMeshData[i][j];
            }
        }
    }

    mZmax -= mZmin;

    for( i = 0; i < mMeshDeltaX; i++ )
    {
        for( j = 0; j < mMeshDeltaY; j++ )
        {
            mMeshData[i][j] += -mZmin;
        }
    }
}


//!************************************************************************
//! Create the 3D data
//! *** compute mesh vertices and normals ***
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::make3dDataCompute()
{
    uint16_t i = 0;
    uint16_t j = 0;

    double*** meshFacetNs = new double**[mMeshDeltaX];

    for( i = 0; i < mMeshDeltaX; i++ )
    {
        meshFacetNs[i] = new double*[mMeshDeltaY];

        for( j = 0; j < mMeshDeltaY; j++ )
        {
            meshFacetNs[i][j] = new double[3];
        }
    }

    double u[3] = { 0 };
    double v[3] = { 0 };
    double n[3] = { 0 };
    double xProdNorm = 0;

    for( i = 0; i < mMeshDeltaX - 1; i++ )
    {
        for( j = 0; j < mMeshDeltaY - 1; j++ )
        {
            mMeshVxs[i][j][0] = static_cast< double >( i ) / static_cast< double >( mMeshDeltaX ) - 0.5;
            mMeshVxs[i][j][1] = mMeshData[i][j] / mZmax - 0.5;
            mMeshVxs[i][j][2] = static_cast< double >( j ) / static_cast< double >( mMeshDeltaY ) - 0.5;

            mMeshVxs[i][j+1][0] = mMeshVxs[i][j][0];
            mMeshVxs[i][j+1][1] = mMeshData[i][j + 1] / mZmax - 0.5;
            mMeshVxs[i][j+1][2] = static_cast< double >( j + 1 ) / static_cast< double >( mMeshDeltaY ) - 0.5;

            mMeshVxs[i+1][j][0] = static_cast< double >( i + 1 ) / static_cast< double >( mMeshDeltaX ) - 0.5;
            mMeshVxs[i+1][j][1] = mMeshData[i + 1][j] / mZmax - 0.5;
            mMeshVxs[i+1][j][2] = mMeshVxs[i][j][2];

            u[0] = mMeshVxs[i][j+1][0] - mMeshVxs[i][j][0];
            u[1] = mMeshVxs[i][j+1][1] - mMeshVxs[i][j][1];
            u[2] = mMeshVxs[i][j+1][2] - mMeshVxs[i][j][2];

            v[0] = mMeshVxs[i+1][j][0] - mMeshVxs[i][j][0];
            v[1] = mMeshVxs[i+1][j][1] - mMeshVxs[i][j][1];
            v[2] = mMeshVxs[i+1][j][2] - mMeshVxs[i][j][2];

            n[0] = u[1] * v[2] - u[2] * v[1];
            n[1] = u[2] * v[0] - u[0] * v[2];
            n[2] = u[0] * v[1] - u[1] * v[0];

            xProdNorm = sqrt( n[0] * n[0] + n[1] * n[1] + n[2] * n[2] );

            n[0] /= xProdNorm;
            n[1] /= xProdNorm;
            n[2] /= xProdNorm;

            meshFacetNs[i][j][0] = n[0];
            meshFacetNs[i][j][1] = n[1];
            meshFacetNs[i][j][2] = n[2];
        }
    }

    mMeshVxs[i][j][0] = static_cast< double >( i ) / static_cast< double >( mMeshDeltaX ) - 0.5;
    mMeshVxs[i][j][1] = mMeshData[i][j] / mZmax - 0.5;
    mMeshVxs[i][j][2] = static_cast< double >( j ) / static_cast< double >( mMeshDeltaY ) - 0.5;

    meshFacetNs[i][j][0] = n[0];
    meshFacetNs[i][j][1] = n[1];
    meshFacetNs[i][j][2] = n[2];

    for( i = 1; i < mMeshDeltaX - 1; i++ )
    {
        for( j = 1; j < mMeshDeltaY - 1; j++ )
        {
            n[0] = meshFacetNs[i - 1][j - 1][0];
            n[1] = meshFacetNs[i - 1][j - 1][1];
            n[2] = meshFacetNs[i - 1][j - 1][2];

            n[0] += meshFacetNs[i][j - 1][0];
            n[1] += meshFacetNs[i][j - 1][1];
            n[2] += meshFacetNs[i][j - 1][2];

            n[0] += meshFacetNs[i - 1][j][0];
            n[1] += meshFacetNs[i - 1][j][1];
            n[2] += meshFacetNs[i - 1][j][2];

            n[0] += meshFacetNs[i][j][0];
            n[1] += meshFacetNs[i][j][1];
            n[2] += meshFacetNs[i][j][2];

            xProdNorm = sqrt( n[0] * n[0] + n[1] * n[1] + n[2] * n[2] );

            n[0] /= xProdNorm;
            n[1] /= xProdNorm;
            n[2] /= xProdNorm;

            mMeshNs[i][j][0] = n[0];
            mMeshNs[i][j][1] = n[1];
            mMeshNs[i][j][2] = n[2];
        }
    }

    for( i = 0; i < mMeshDeltaX; i++ )
    {
        mMeshNs[i][0][0] = mMeshNs[i][1][0];
        mMeshNs[i][0][1] = mMeshNs[i][1][1];
        mMeshNs[i][0][2] = mMeshNs[i][1][2];

        mMeshNs[i][mMeshDeltaY - 1][0] = mMeshNs[i][mMeshDeltaY - 2][0];
        mMeshNs[i][mMeshDeltaY - 1][1] = mMeshNs[i][mMeshDeltaY - 2][1];
        mMeshNs[i][mMeshDeltaY - 1][2] = mMeshNs[i][mMeshDeltaY - 2][2];
    }

    for( j = 0; j < mMeshDeltaY; j++ )
    {
        mMeshNs[0][j][0] = mMeshNs[1][j][0];
        mMeshNs[0][j][1] = mMeshNs[1][j][1];
        mMeshNs[0][j][2] = mMeshNs[1][j][2];

        mMeshNs[mMeshDeltaX - 1][j][0] = mMeshNs[mMeshDeltaX - 2][j][0];
        mMeshNs[mMeshDeltaX - 1][j][1] = mMeshNs[mMeshDeltaX - 2][j][1];
        mMeshNs[mMeshDeltaX - 1][j][2] = mMeshNs[mMeshDeltaX - 2][j][2];
    }

    for( i = 0; i < mMeshDeltaX; i++ )
    {
        for( j = 0; j < mMeshDeltaY; j++ )
        {
            delete[] meshFacetNs[i][j];
        }

        delete[] meshFacetNs[i];
    }

    delete[] meshFacetNs;
}


//!************************************************************************
//! Create the 3D data
//! *** deallocate memory ***
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::make3dDataCleanup()
{
    for( uint16_t i = 0; i < mMeshDeltaX; i++ )
    {
        delete[] mMeshData[i];
    }

    delete[] mMeshData;
}


//!************************************************************************
//! Resize minimum size hint
//!
//! @returns A minimum rectangle size to be kept during resize
//!************************************************************************
QSize Plot3dCanvas::minimumSizeHint() const
{
    return QSize( MIN_WIDTH, MIN_HEIGHT );
}


//!************************************************************************
//! Mouse move event handler
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::mouseMoveEvent
    (
    QMouseEvent* aEvent     //!< mouse move event
    )
{
    if( mRotIsActive )
    {
        double crtPosition[3] = { 0 };
        rotatePt2Vec( aEvent->x(), aEvent->y(), crtPosition );

        double dx = crtPosition[0] - mRotLastPosition[0];
        double dy = crtPosition[1] - mRotLastPosition[1];
        double dz = crtPosition[2] - mRotLastPosition[2];

        mRotAngleDeg = 90.0 * sqrt( dx * dx + dy * dy + dz * dz );

        mRotAxis[0] = mRotLastPosition[1] * crtPosition[2] - mRotLastPosition[2] * crtPosition[1];
        mRotAxis[1] = mRotLastPosition[2] * crtPosition[0] - mRotLastPosition[0] * crtPosition[2];
        mRotAxis[2] = mRotLastPosition[0] * crtPosition[1] - mRotLastPosition[1] * crtPosition[0];

        mRotLastPosition[0] = crtPosition[0];
        mRotLastPosition[1] = crtPosition[1];
        mRotLastPosition[2] = crtPosition[2];

        update();
    }
}


//!************************************************************************
//! Mouse press event handler
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::mousePressEvent
    (
    QMouseEvent* aEvent     //!< mouse press event
    )
{   
    if( aEvent->buttons() & mRotMouseButton )
    {
        mRotIsActive = true;
        rotatePt2Vec( aEvent->x(), aEvent->y(), mRotLastPosition );
        update();
    }
}


//!************************************************************************
//! Mouse release event handler
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::mouseReleaseEvent
    (
    QMouseEvent* aEvent     //!< mouse release event
    )
{
    if( aEvent->button() & mRotMouseButton )
    {
        mRotIsActive = false;
        mRotAngleDeg = 0;
        update();
    }
}


//!************************************************************************
//! Output a scaled string
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::outputScaledText
    (
    const float     aX,         //!< x coordinate
    const float     aY,         //!< y coordinate
    const float     aScale,     //!< scale
    const QString   aString,    //!< string
    QColor          aColor      //!< color
    )
{
    QPainter painter( this );
        painter.setPen( QPen( aColor, 1, Qt::SolidLine ) );
        painter.setFont( QFont( "Courier New", 9 * aScale, QFont::Normal ) );
        painter.drawText( aX, aY, aString );
    painter.end();
}


//!************************************************************************
//! OpenGL painter
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::paintGL()
{
    glClearColor( 0.0, 0.0, 0.0, 0.0 );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glPushMatrix();
        glPushMatrix();
            glLoadIdentity();
            glRotated( mRotAngleDeg, mRotAxis[0], mRotAxis[1], mRotAxis[2] );
            glMultMatrixd( reinterpret_cast< double* >( mRotTransform ) );
            glGetDoublev( GL_MODELVIEW_MATRIX, reinterpret_cast< double* >( mRotTransform ) );
        glPopMatrix();

        glMultMatrixd( reinterpret_cast< double* >( mRotTransform ) );

        if( mLightEnabled )
        {
            glEnable( GL_LIGHTING );
            glEnable( GL_LIGHT0 );
        }
        else
        {
            glDisable( GL_LIGHTING );
        }

        glPolygonMode( GL_FRONT_AND_BACK, mMeshFill ? GL_FILL : GL_LINE );
        glEnable( GL_COLOR_MATERIAL );
        glEnable( GL_DEPTH_TEST );
        glCallList( mMeshList );

        glDisable( GL_LIGHTING );
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        glCallList( mColorsList );
    glPopMatrix();

    int screenX = 0;
    int screenY = 0;
    convertToScreenCoords( -0.975, -0.85, &screenX, &screenY );
    outputScaledText( screenX, screenY, 1, QString::number( mZmin ), Qt::cyan );

    convertToScreenCoords( -0.975, -0.20, &screenX, &screenY );
    outputScaledText( screenX, screenY, 1, QString::number( mZmax + mZmin ), Qt::cyan );

    glFlush();
}


//!************************************************************************
//! Reset the view
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::resetView()
{
    glPushMatrix();
        glLoadIdentity();
        glGetDoublev( GL_MODELVIEW_MATRIX, ( double* )mRotTransform );
    glPopMatrix();
}


//!************************************************************************
//! OpenGL resizer
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::resizeGL
    (
    int aWidth,     //!< width
    int aHeight     //!< height
    )
{
    mSizeW = aWidth;
    mSizeH = aHeight;

    glViewport( 0, 0, mSizeW, mSizeH );
    glMatrixMode( GL_PROJECTION );

    glLoadIdentity();
    glFrustum( -0.5, 0.5, -0.5, 0.5, 1.0, 4.0 );
    glMatrixMode( GL_MODELVIEW );

    glLoadIdentity();
    glTranslated( 0.0, 0.0, -2.0 );
    GLfloat lightPosition[] = { 1.0, 1.0, 1.0, 0.0 };
    glLightfv( GL_LIGHT0, GL_POSITION, lightPosition );

    update();
}


//!************************************************************************
//! Convert point rotation to vector
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::rotatePt2Vec
    (
    int       aX,           //!< x coordinate
    int       aY,           //!< y coordinate
    double    aVec[3]       //!< vector
    ) const
{
    if( mSizeW && mSizeH )
    {
        aVec[0] = ( 2.0 * aX - mSizeW ) / mSizeW;
        aVec[1] = ( mSizeH - 2.0 * aY ) / mSizeH;
        double dist = sqrt( aVec[0] * aVec[0] + aVec[1] * aVec[1] );
        aVec[2] = cos( ( Numeric::PI / 2.0 ) * ( ( dist < 1.0 ) ? dist : 1.0 ) );

        double a = 1.0 / sqrt( aVec[0] * aVec[0] + aVec[1] * aVec[1] + aVec[2] * aVec[2] );
        aVec[0] *= a;
        aVec[1] *= a;
        aVec[2] *= a;
    }
}


//!************************************************************************
//! Set the SPS and FFT size parameters
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::setParameters
    (
    const double   aSps,        //!< sampling rate [Hz]
    const uint32_t aFftSize     //!< FFT size
    )
{
    if( aSps > 0 && aFftSize > 0 )
    {
        mSps = aSps;
        mFftSize = aFftSize;
        mMaxFreq = mSps / 2.0;
        mBinWidth = mSps / mFftSize;
        mTimeGate = 1.0 / mBinWidth;
        mMaxFreqLog = pow( 10.0, static_cast<double>( ceil( log10( mMaxFreq ) ) ) );
        mMaxQuefrency = mTimeGate;
        updateMinFrequencyLog();
    }
}


//!************************************************************************
//! Set the maximum vertical value for transient plots
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::setVerticalMaxTransient
    (
    const double aVerticalMax           //!< max value for vertical axis [g]
    )
{
    mMaxVert = fabs( aVerticalMax );
    mMinVert = -mMaxVert;

    if( Plot3d::PLOT_3D_TYPE_TRANSIENT == mParentPlot3d.getPlot3dType() )
    {
        mHaveVertValues = true;
    }
}


//!************************************************************************
//! Initialize size hint
//!
//! @returns A rectangle to be used at the first content resize
//!************************************************************************
QSize Plot3dCanvas::sizeHint() const
{
    return QSize( MIN_WIDTH, MIN_HEIGHT );
}


//!************************************************************************
//! Toggle the lighting
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::toogleLightEnable()
{
    makeCurrent();
        mLightEnabled = !mLightEnabled;
        update();
    doneCurrent();
}


//!************************************************************************
//! Toggle the mesh fill
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::toogleMeshFill()
{
    makeCurrent();
        mMeshFill = !mMeshFill;
        update();
    doneCurrent();
}


//!************************************************************************
//! Update the minimum frequency for logarithmic horizontal axis
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateMinFrequencyLog()
{
    mMinFreqLog = 0;

    if( Plot3d::PLOT_3D_TYPE_SRS == mParentPlot3d.getPlot3dType() )
    {
        mMinFreqLog = SrsThread::NATURAL_FREQ_MIN;
    }
}


void Plot3dCanvas::updateKeyUp()
{
    cleanup();

    mMeshDeltaX += 10;

    makeCurrent();
        make3dData();
        init3dMeshList();
        update();
    doneCurrent();
}

void Plot3dCanvas::updateKeyDown()
{
    if( mMeshDeltaX > 10 )
    {
        cleanup();

        mMeshDeltaX -= 10;

        makeCurrent();
            make3dData();
            init3dMeshList();
            update();
        doneCurrent();
    }
}

void Plot3dCanvas::updateKeyRight()
{
    cleanup();

    mMeshDeltaY += 10;

    makeCurrent();
        make3dData();
        init3dMeshList();
        update();
    doneCurrent();
}

void Plot3dCanvas::updateKeyLeft()
{
    if( mMeshDeltaY > 10 )
    {
        cleanup();

        mMeshDeltaY -= 10;

        makeCurrent();
            make3dData();
            init3dMeshList();
            update();
        doneCurrent();
    }
}
