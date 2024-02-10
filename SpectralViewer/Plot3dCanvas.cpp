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
    // vertical axis
    , mMaxVert( -FLT_MAX )
    , mMinVert( FLT_MAX )
    , mHaveVertValues( false )
    // GL
    , mColorsList( 0 )
    , mLightEnabled( true )
    , mMeshList( 0 )
    , mMeshData( nullptr )
    , mMeshDataBuf( nullptr )
    , mMeshVxs( nullptr )
    , mMeshNs( nullptr )
    , mMeshFill( true )
    , mMeshDeltaX( 100 )
    , mMeshDeltaY( 100 )
    // rotation
    , mRotAngleDeg( 0 )
    , mRotMouseButton( Qt::LeftButton )
    , mRotIsActive( false )
    // ranges
    , mZmin( FLT_MAX )
    , mZmax( -FLT_MAX )
    // refresh
    , mRefreshTimer( new QTimer( this ) )
{
    memset( &mRotAxis, 0, sizeof( mRotAxis ) );
    memset( &mRotTransform, 0, sizeof( mRotTransform ) );
    memset( &mRotLastPosition, 0, sizeof( mRotLastPosition ) );

    connect( mRefreshTimer, SIGNAL( timeout() ), this, SLOT( refresh() ) );
}


//!************************************************************************
//! Destructor
//!************************************************************************
Plot3dCanvas::~Plot3dCanvas()
{
}


//!************************************************************************
//! Data cleanup - deallocate all memory
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::cleanupAll()
{
    cleanupVxsNs();

    cleanupData();
    cleanupDataBuf();
}


//!************************************************************************
//! Data cleanup - deallocate data memory
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::cleanupData()
{
    if( mMeshData )
    {
        for( uint8_t i = 0; i < mMeshDeltaX; i++ )
        {
            delete[] mMeshData[i];
        }

        delete[] mMeshData;
        mMeshData = nullptr;
    }
}


//!************************************************************************
//! Data cleanup - deallocate buffered data memory
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::cleanupDataBuf()
{
    if( mMeshDataBuf )
    {
        for( uint8_t i = 0; i < mMeshDeltaX; i++ )
        {
            delete[] mMeshDataBuf[i];
        }

        delete[] mMeshDataBuf;
        mMeshDataBuf = nullptr;
    }
}


//!************************************************************************
//! Data cleanup - deallocate vertices and normals memory
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3dCanvas::cleanupVxsNs()
{
    uint8_t i = 0;
    uint8_t j = 0;

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
        mMeshVxs = nullptr;
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
        mMeshNs = nullptr;
    }
}


//!************************************************************************
//! Comparison function for FftBin values
//!
//! @returns true if first number is smaller than second
//!************************************************************************
bool Plot3dCanvas::compareBinFnc
    (
    VibrationHandler::FftBin x,   //!< first object
    VibrationHandler::FftBin y    //!< second object
    )
{
    return x.value < y.value;
}


//!************************************************************************
//! Comparison function for FftCepstrum values
//!
//! @returns true if first number is smaller than second
//!************************************************************************
bool Plot3dCanvas::compareCepstrumFnc
    (
    VibrationHandler::FftCepstrum x,   //!< first object
    VibrationHandler::FftCepstrum y    //!< second object
    )
{
    return x.value < y.value;
}


//!************************************************************************
//! Comparison function for FftPsd values
//!
//! @returns true if first number is smaller than second
//!************************************************************************
bool Plot3dCanvas::comparePsdFnc
    (
    VibrationHandler::FftPsd x,   //!< first object
    VibrationHandler::FftPsd y    //!< second object
    )
{
    return x.value < y.value;
}


//!************************************************************************
//! Comparison function for Srs values
//!
//! @returns true if first number is smaller than second
//!************************************************************************
bool Plot3dCanvas::compareSrsFnc
    (
    VibrationHandler::Srs x,   //!< first object
    VibrationHandler::Srs y    //!< second object
    )
{
    return x.value < y.value;
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
//! Initialize the 3D mesh list
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::init3dMeshList
    (
    const bool  aResize     //!< true if resizing
    )
{
    if( mMeshVxs && mMeshNs )
    {
        if( aResize )
        {
            if( mMeshList )
            {
                glDeleteLists( mMeshList, 1 );
            }

            mMeshList = glGenLists( 1 );
        }

        if( mMeshList )
        {
            glNewList( mMeshList, GL_COMPILE );
                for( uint8_t i = 0; i < mMeshDeltaX - 1; i++ )
                {
                    glPolygonMode( GL_BACK, GL_LINE );
                    glBegin( GL_TRIANGLE_STRIP );
                        for( uint8_t j = 0; j < mMeshDeltaY; j++ )
                        {
                            int index = static_cast<int>( MESH_COLORS_NR * ( mMeshVxs[i + 1][j][1] + 0.5 ) );

                            if( index < 0 )
                            {
                                index = 0;
                            }
                            else if( index > MESH_COLORS_NR - 1 )
                            {
                                index = MESH_COLORS_NR - 1;
                            }

                            glColor3fv( MESH_COLORS[index] );
                            glNormal3fv( mMeshNs[i + 1][j] );
                            glVertex3fv( mMeshVxs[i + 1][j] );

                            index = static_cast<int>( MESH_COLORS_NR * ( mMeshVxs[i][j][1] + 0.5 ) );

                            if( index < 0 )
                            {
                                index = 0;
                            }
                            else if( index > MESH_COLORS_NR - 1 )
                            {
                                index = MESH_COLORS_NR - 1;
                            }

                            glColor3fv( MESH_COLORS[index] );
                            glNormal3fv( mMeshNs[i][j] );
                            glVertex3fv( mMeshVxs[i][j] );
                        }
                    glEnd();
                }
            glEndList();
        }
    }
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

    if( mColorsList )
    {
        glNewList( mColorsList, GL_COMPILE );
            glPushMatrix();
                glLoadIdentity();
                glTranslatef( 0.0, 0.0, -2.0 );
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
}


//!************************************************************************
//! OpenGL initializer
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::initializeGL()
{
    connect( context(), &QOpenGLContext::aboutToBeDestroyed, this, &Plot3dCanvas::cleanupAll );
    initializeOpenGLFunctions();

    make3dDataAllocateVxsNs();
    make3dDataAllocateData();
    make3dDataAllocateDataBuf();

    make3dDataAssign();
    make3dDataCompute();

    cleanupData();
    cleanupDataBuf();

    init3dMeshList( true );

    initColorsList();

    glPushMatrix();
        glLoadIdentity();
        glGetFloatv( GL_MODELVIEW_MATRIX, reinterpret_cast< float* >( mRotTransform ) );
    glPopMatrix();
}


//!************************************************************************
//! Create the 3D data
//! *** allocate memory for data ***
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::make3dDataAllocateData()
{
    if( !mMeshData )
    {
        mMeshData = new float*[mMeshDeltaX];

        for( uint8_t i = 0; i < mMeshDeltaX; i++ )
        {
            mMeshData[i] = new float[mMeshDeltaY]();
        }
    }
}


//!************************************************************************
//! Create the 3D data
//! *** allocate memory for data buffered ***
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::make3dDataAllocateDataBuf()
{
    if( !mMeshDataBuf )
    {
        mMeshDataBuf = new float*[mMeshDeltaX];

        for( uint8_t i = 0; i < mMeshDeltaX; i++ )
        {
            mMeshDataBuf[i] = new float[mMeshDeltaY]();
        }
    }
}


//!************************************************************************
//! Create the 3D data
//! *** allocate memory for vertices and normals ***
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::make3dDataAllocateVxsNs()
{
    uint8_t i = 0;
    uint8_t j = 0;

    if( !mMeshVxs )
    {
        mMeshVxs = new float**[mMeshDeltaX];

        for( i = 0; i < mMeshDeltaX; i++ )
        {
            mMeshVxs[i] = new float*[mMeshDeltaY];

            for( j = 0; j < mMeshDeltaY; j++ )
            {
                mMeshVxs[i][j] = new float[3]();
            }
        }
    }

    if( !mMeshNs )
    {
        mMeshNs = new float**[mMeshDeltaX];

        for( i = 0; i < mMeshDeltaX; i++ )
        {
            mMeshNs[i] = new float*[mMeshDeltaY];

            for( j = 0; j < mMeshDeltaY; j++ )
            {
                mMeshNs[i][j] = new float[3]();
            }
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
    if( mMeshData && mMeshDataBuf )
    {
        int16_t i = 0;
        int16_t j = 0;
        mZmin = FLT_MAX;
        mZmax = -FLT_MAX;

        switch( mParentPlot3d.getPlot3dType() )
        {
            case Plot3d::PLOT_3D_TYPE_TRANSIENT:
                {
                    VibrationHandler* vh = VibrationHandler::getInstance();
                    VibrationHandler::AccelerometerDataTriaxial accelDataArrays = vh->getTriaxialAccelArrays();
                    size_t dataLen = accelDataArrays.xArray.size();

                    for( j = 0; j < mMeshDeltaY - 1; j++ )
                    {
                        for( i = 0; i < mMeshDeltaX; i++ )
                        {
                            mMeshData[i][j] = mMeshDataBuf[i][j + 1];
                            mMeshDataBuf[i][j] = mMeshData[i][j];
                        }
                    }

                    for( i = 0; i < mMeshDeltaX; i++ )
                    {
                        size_t index = i * dataLen / mMeshDeltaX;

                        if( index > dataLen - 1 )
                        {
                            index = dataLen - 1;
                        }

                        switch( mParentPlot3d.getAxis() )
                        {
                            case Adxl355Adxl357Common::AXIS_X:
                                mMeshData[i][j] = static_cast<float>( accelDataArrays.xArray.at( index ) );
                                break;

                            case Adxl355Adxl357Common::AXIS_Y:
                                mMeshData[i][j] = static_cast<float>( accelDataArrays.yArray.at( index ) );
                                break;

                            case Adxl355Adxl357Common::AXIS_Z:
                                mMeshData[i][j] = static_cast<float>( accelDataArrays.zArray.at( index ) );
                                break;

                            default:
                                mMeshData[i][j] = 0;
                                break;
                        }

                        mMeshDataBuf[i][j] = mMeshData[i][j];
                    }
                }
                break;

            case Plot3d::PLOT_3D_TYPE_FFT:
                {
                    VibrationHandler* vh = VibrationHandler::getInstance();
                    std::vector<VibrationHandler::FftBin> binsVec = vh->getFftBinsOnAxis( mParentPlot3d.getAxis() );
                    size_t dataLen = binsVec.size();

                    VibrationHandler::FftBin binMaxValue = *std::max_element( binsVec.begin(), binsVec.end(), compareBinFnc );
                    VibrationHandler::FftBin binMinValue = *std::min_element( binsVec.begin(), binsVec.end(), compareBinFnc );

                    if( Plot3d::AXIS_TYPE_LINEAR == mParentPlot3d.getAxisTypeVert() )
                    {
                        mMaxVert = 1;
                        mMinVert = 0;
                        mHaveVertValues = true;

                        for( j = 0; j < mMeshDeltaY - 1; j++ )
                        {
                            for( i = 0; i < mMeshDeltaX; i++ )
                            {
                                mMeshData[i][j] = mMeshDataBuf[i][j + 1];
                                mMeshDataBuf[i][j] = mMeshData[i][j];
                            }
                        }

                        for( i = 0; i < mMeshDeltaX; i++ )
                        {
                            size_t index = i * dataLen / mMeshDeltaX;

                            if( index > dataLen - 1 )
                            {
                                index = dataLen - 1;
                            }

                            mMeshData[i][j] = static_cast<float>( binsVec.at( index ).value / binMaxValue.value );
                            mMeshDataBuf[i][j] = mMeshData[i][j];
                        }
                    }
                    else if( Plot3d::AXIS_TYPE_DB == mParentPlot3d.getAxisTypeVert() )
                    {
                        static float maxForDbScale = -FLT_MAX;
                        static float minForDbScale = FLT_MAX;

                        static int32_t maxDb = Numeric::INVALID_MAX_10_DB;
                        static int32_t minDb = Numeric::INVALID_MIN_10_DB;

                        Numeric* nrInstance = Numeric::getInstance();

                        if( binMaxValue.value > maxForDbScale )
                        {
                            maxForDbScale = static_cast<float>( binMaxValue.value );
                            maxDb = nrInstance->findMagnitudeMaxDbMultipleOf10( maxForDbScale );

                            if( Numeric::INVALID_MAX_10_DB != maxDb )
                            {
                                mMaxVert = maxDb;
                            }
                            else
                            {
                                maxDb = mMaxVert;
                            }
                        }

                        if( binMinValue.value < minForDbScale )
                        {
                            minForDbScale = binMinValue.value;
                            minDb = nrInstance->findMagnitudeMinDbMultipleOf10( minForDbScale );

                            if( Numeric::INVALID_MIN_10_DB != minDb )
                            {
                                mMinVert = minDb;
                            }
                            else
                            {
                                minDb = mMinVert;
                            }
                        }

                        mMaxVert = maxDb;
                        mMinVert = minDb;
                        mHaveVertValues = true;

                        for( j = 0; j < mMeshDeltaY - 1; j++ )
                        {
                            for( i = 0; i < mMeshDeltaX; i++ )
                            {
                                mMeshData[i][j] = mMeshDataBuf[i][j + 1];
                                mMeshDataBuf[i][j] = mMeshData[i][j];
                            }
                        }

                        for( i = 0; i < mMeshDeltaX; i++ )
                        {
                            size_t index = i * dataLen / mMeshDeltaX;

                            if( index > dataLen - 1 )
                            {
                                index = dataLen - 1;
                            }

                            mMeshData[i][j] = static_cast<float>( 20 * log10( binsVec.at( index ).value ) );
                            mMeshDataBuf[i][j] = mMeshData[i][j];
                        }
                    }
                }
                break;

            case Plot3d::PLOT_3D_TYPE_PERIODOGRAM:
                {
                    VibrationHandler* vh = VibrationHandler::getInstance();
                    std::vector<VibrationHandler::FftPsd> psdVec = vh->getFftPsdOnAxis( mParentPlot3d.getAxis() );
                    size_t dataLen = psdVec.size();

                    VibrationHandler::FftPsd psdMaxValue = *std::max_element( psdVec.begin(), psdVec.end(), comparePsdFnc );
                    VibrationHandler::FftPsd psdMinValue = *std::min_element( psdVec.begin(), psdVec.end(), comparePsdFnc );

                    if( Plot3d::AXIS_TYPE_LINEAR == mParentPlot3d.getAxisTypeVert() )
                    {
                        mMaxVert = 1;
                        mMinVert = 0;
                        mHaveVertValues = true;

                        for( j = 0; j < mMeshDeltaY - 1; j++ )
                        {
                            for( i = 0; i < mMeshDeltaX; i++ )
                            {
                                mMeshData[i][j] = mMeshDataBuf[i][j + 1];
                                mMeshDataBuf[i][j] = mMeshData[i][j];
                            }
                        }

                        for( i = 0; i < mMeshDeltaX; i++ )
                        {
                            size_t index = i * dataLen / mMeshDeltaX;

                            if( index > dataLen - 1 )
                            {
                                index = dataLen - 1;
                            }

                            mMeshData[i][j] = static_cast<float>( psdVec.at( index ).value / psdMaxValue.value );
                            mMeshDataBuf[i][j] = mMeshData[i][j];
                        }
                    }
                    else if( Plot3d::AXIS_TYPE_DB == mParentPlot3d.getAxisTypeVert() )
                    {
                        static float maxForDbScale = -FLT_MAX;
                        static float minForDbScale = FLT_MAX;

                        static int32_t maxDb = Numeric::INVALID_MAX_10_DB;
                        static int32_t minDb = Numeric::INVALID_MIN_10_DB;

                        Numeric* nrInstance = Numeric::getInstance();

                        if( psdMaxValue.value > maxForDbScale )
                        {
                            maxForDbScale = static_cast<float>( psdMaxValue.value );
                            maxDb = nrInstance->findPowerMaxDbMultipleOf10( maxForDbScale );

                            if( Numeric::INVALID_MAX_10_DB != maxDb )
                            {
                                mMaxVert = maxDb;
                            }
                            else
                            {
                                maxDb = mMaxVert;
                            }
                        }

                        if( psdMinValue.value < minForDbScale )
                        {
                            minForDbScale = psdMinValue.value;
                            minDb = nrInstance->findPowerMinDbMultipleOf10( minForDbScale );

                            if( Numeric::INVALID_MIN_10_DB != minDb )
                            {
                                mMinVert = minDb;
                            }
                            else
                            {
                                minDb = mMinVert;
                            }
                        }

                        mMaxVert = maxDb;
                        mMinVert = minDb;
                        mHaveVertValues = true;

                        for( j = 0; j < mMeshDeltaY - 1; j++ )
                        {
                            for( i = 0; i < mMeshDeltaX; i++ )
                            {
                                mMeshData[i][j] = mMeshDataBuf[i][j + 1];
                                mMeshDataBuf[i][j] = mMeshData[i][j];
                            }
                        }

                        for( i = 0; i < mMeshDeltaX; i++ )
                        {
                            size_t index = i * dataLen / mMeshDeltaX;

                            if( index > dataLen - 1 )
                            {
                                index = dataLen - 1;
                            }

                            mMeshData[i][j] = static_cast<float>( 10 * log10( psdVec.at( index ).value ) );
                            mMeshDataBuf[i][j] = mMeshData[i][j];
                        }
                    }
                }
                break;

            case Plot3d::PLOT_3D_TYPE_SRS:
                {
                    VibrationHandler* vh = VibrationHandler::getInstance();
                    std::vector<VibrationHandler::Srs> srsVec = vh->getSrsOnAxis( mParentPlot3d.getAxis() );
                    size_t dataLen = srsVec.size();

                    VibrationHandler::Srs srsMaxValue = *std::max_element( srsVec.begin() + 1, srsVec.end(), compareSrsFnc );
                    VibrationHandler::Srs srsMinValue = *std::min_element( srsVec.begin() + 1, srsVec.end(), compareSrsFnc );

                    if( Plot3d::AXIS_TYPE_LOG == mParentPlot3d.getAxisTypeVert() )
                    {
                        static float maxForLogScale = -FLT_MAX;
                        static float minForLogScale = FLT_MAX;

                        static float maxLog = static_cast<float>( Numeric::INVALID_MAX_LOG );
                        static float minLog = static_cast<float>( Numeric::INVALID_MIN_LOG );

                        Numeric* nrInstance = Numeric::getInstance();

                        if( srsMaxValue.value > maxForLogScale )
                        {
                            maxForLogScale = srsMaxValue.value;
                            maxLog = nrInstance->findLogMaxPwrOf10( maxForLogScale );

                            if( Numeric::INVALID_MAX_LOG != maxLog )
                            {
                                mMaxVert = maxLog;
                            }
                            else
                            {
                                maxLog = mMaxVert;
                            }
                        }

                        if( srsMinValue.value < minForLogScale )
                        {
                            minForLogScale = srsMinValue.value;
                            minLog = nrInstance->findLogMinPwrOf10( minForLogScale );

                            if( Numeric::INVALID_MIN_LOG != minLog )
                            {
                                mMinVert = minLog;
                            }
                            else
                            {
                                minLog = mMinVert;
                            }
                        }

                        mMaxVert = maxLog;
                        mMinVert = minLog;
                        mHaveVertValues = true;

                        for( j = 0; j < mMeshDeltaY - 1; j++ )
                        {
                            for( i = 0; i < mMeshDeltaX; i++ )
                            {
                                mMeshData[i][j] = mMeshDataBuf[i][j + 1];
                                mMeshDataBuf[i][j] = mMeshData[i][j];
                            }
                        }

                        size_t srsPoint = 0;

                        for( ; srsPoint < dataLen; srsPoint++ )
                        {
                            if( srsVec.at( srsPoint ).naturalFrequency >= SrsThread::NATURAL_FREQ_MIN )
                            {
                                break;
                            }
                        }

                        for( i = srsPoint * mMeshDeltaX / dataLen; i < mMeshDeltaX; i++ )
                        {
                            size_t index = i * dataLen / mMeshDeltaX;

                            if( index > dataLen - 1 )
                            {
                                index = dataLen - 1;
                            }

                            mMeshData[i][j] = static_cast<float>( log10( srsVec.at( index ).value / pow( 10.0, mMinVert ) ) );
                            mMeshDataBuf[i][j] = mMeshData[i][j];
                        }
                    }
                }
                break;

            case Plot3d::PLOT_3D_TYPE_CEPSTRUM:
                {
                    VibrationHandler* vh = VibrationHandler::getInstance();
                    std::vector<VibrationHandler::FftCepstrum> cepstrumVec = vh->getFftCepstrumOnAxis( mParentPlot3d.getAxis() );
                    size_t dataLen = cepstrumVec.size();

                    VibrationHandler::FftCepstrum cepstrumMaxValue = *std::max_element( cepstrumVec.begin(), cepstrumVec.end(), compareCepstrumFnc );
                    VibrationHandler::FftCepstrum cepstrumMinValue = *std::min_element( cepstrumVec.begin(), cepstrumVec.end(), compareCepstrumFnc );

                    if( Plot3d::AXIS_TYPE_DB == mParentPlot3d.getAxisTypeVert() )
                    {
                        static float maxForDbScale = -FLT_MAX;
                        static float minForDbScale = FLT_MAX;

                        static int32_t maxDb = Numeric::INVALID_MAX_10_DB;
                        static int32_t minDb = Numeric::INVALID_MIN_10_DB;

                        Numeric* nrInstance = Numeric::getInstance();

                        if( cepstrumMaxValue.value > maxForDbScale )
                        {
                            maxForDbScale = static_cast<float>( cepstrumMaxValue.value );
                            maxDb = nrInstance->findPowerMaxDbMultipleOf10( maxForDbScale );

                            if( Numeric::INVALID_MAX_10_DB != maxDb )
                            {
                                mMaxVert = maxDb;
                            }
                            else
                            {
                                maxDb = mMaxVert;
                            }
                        }

                        if( cepstrumMinValue.value < minForDbScale )
                        {
                            minForDbScale = cepstrumMinValue.value;
                            minDb = nrInstance->findPowerMinDbMultipleOf10( minForDbScale );

                            if( Numeric::INVALID_MIN_10_DB != minDb )
                            {
                                mMinVert = minDb;
                            }
                            else
                            {
                                minDb = mMinVert;
                            }
                        }

                        mMaxVert = maxDb;
                        mMinVert = minDb;
                        mHaveVertValues = true;

                        for( j = 0; j < mMeshDeltaY - 1; j++ )
                        {
                            for( i = 0; i < mMeshDeltaX; i++ )
                            {
                                mMeshData[i][j] = mMeshDataBuf[i][j + 1];
                                mMeshDataBuf[i][j] = mMeshData[i][j];
                            }
                        }

                        for( i = 0; i < mMeshDeltaX; i++ )
                        {
                            size_t index = i * dataLen / mMeshDeltaX;

                            if( index > dataLen - 1 )
                            {
                                index = dataLen - 1;
                            }

                            mMeshData[i][j] = static_cast<float>( 10 * log10( cepstrumVec.at( index ).value ) );
                            mMeshDataBuf[i][j] = mMeshData[i][j];
                        }
                    }
                }
                break;

            default:
                break;
        }

        if( mHaveVertValues )
        {
            switch( mParentPlot3d.getPlot3dType() )
            {
                case Plot3d::PLOT_3D_TYPE_TRANSIENT:
                case Plot3d::PLOT_3D_TYPE_FFT:
                case Plot3d::PLOT_3D_TYPE_PERIODOGRAM:
                case Plot3d::PLOT_3D_TYPE_CEPSTRUM:
                    mZmax = mMaxVert;
                    mZmin = mMinVert;
                    break;

                case Plot3d::PLOT_3D_TYPE_SRS:
                    mZmax = log10( mMaxVert );
                    mZmin = log10( mMinVert );
                    break;

                default:
                    break;
            }
        }

        if( mZmin <= mZmax )
        {
            mZmax -= mZmin;

            for( i = 0; i < mMeshDeltaX; i++ )
            {
                for( j = 0; j < mMeshDeltaY; j++ )
                {
                    mMeshData[i][j] -= mZmin;
                }
            }
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
    if( mMeshVxs && mMeshNs && mMeshData )
    {
        uint8_t i = 0;
        uint8_t j = 0;

        float*** meshFacetNs = new float**[mMeshDeltaX];

        for( i = 0; i < mMeshDeltaX; i++ )
        {
            meshFacetNs[i] = new float*[mMeshDeltaY];

            for( j = 0; j < mMeshDeltaY; j++ )
            {
                meshFacetNs[i][j] = new float[3]();
            }
        }

        float u[3] = { 0 };
        float v[3] = { 0 };
        float n[3] = { 0 };
        float xProdNorm = 0;

        for( i = 0; i < mMeshDeltaX - 1; i++ )
        {
            for( j = 0; j < mMeshDeltaY - 1; j++ )
            {
                mMeshVxs[i][j][0] = static_cast<float>( i ) / static_cast<float>( mMeshDeltaX ) - 0.5;
                mMeshVxs[i][j][1] = mMeshData[i][j] / mZmax - 0.5;
                mMeshVxs[i][j][2] = static_cast<float>( j ) / static_cast<float>( mMeshDeltaY ) - 0.5;

                mMeshVxs[i][j+1][0] = mMeshVxs[i][j][0];
                mMeshVxs[i][j+1][1] = mMeshData[i][j + 1] / mZmax - 0.5;
                mMeshVxs[i][j+1][2] = static_cast<float>( j + 1 ) / static_cast<float>( mMeshDeltaY ) - 0.5;

                mMeshVxs[i+1][j][0] = static_cast<float>( i + 1 ) / static_cast<float>( mMeshDeltaX ) - 0.5;
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

        mMeshVxs[i][j][0] = static_cast<float>( i ) / static_cast<float>( mMeshDeltaX ) - 0.5;
        mMeshVxs[i][j][1] = mMeshData[i][j] / mZmax - 0.5;
        mMeshVxs[i][j][2] = static_cast<float>( j ) / static_cast<float>( mMeshDeltaY ) - 0.5;

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
        float crtPosition[3] = { 0 };
        rotatePt2Vec( aEvent->x(), aEvent->y(), crtPosition );

        float dx = crtPosition[0] - mRotLastPosition[0];
        float dy = crtPosition[1] - mRotLastPosition[1];
        float dz = crtPosition[2] - mRotLastPosition[2];

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
    glDisable( GL_LIGHTING );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    int screenX = 0;
    int screenY = 0;
    QString str;

    if( mHaveVertValues )
    {
        convertToScreenCoords( -0.975, -0.85, &screenX, &screenY );
        outputScaledText( screenX, screenY, 1, QString::number( mMinVert ), Qt::cyan );

        convertToScreenCoords( -0.975, -0.20, &screenX, &screenY );
        outputScaledText( screenX, screenY, 1, QString::number( mMaxVert ), Qt::cyan );
    }

    convertToScreenCoords( -0.7, -0.95, &screenX, &screenY );
    str = "Mesh size = ";
    str += QString::number( mMeshDeltaX );
    str += "x";
    str += QString::number( mMeshDeltaY );
    outputScaledText( screenX, screenY, 1, str, Qt::gray );

    convertToScreenCoords( -0.7, -0.89, &screenX, &screenY );
    str = "Mesh fill ";
    str += mMeshFill ? "ON" : "OFF";
    outputScaledText( screenX, screenY, 1, str, Qt::gray );

    convertToScreenCoords( -0.7, -0.83, &screenX, &screenY );
    str = "Light ";
    str += mLightEnabled ? "ON" : "OFF";
    outputScaledText( screenX, screenY, 1, str, Qt::gray );

    glPushMatrix();
        if( mColorsList )
        {
            glPushMatrix();
                glDisable( GL_LIGHTING );
                glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
                glCallList( mColorsList );
            glPopMatrix();
        }

        glPushMatrix();
            glLoadIdentity();
            glRotatef( mRotAngleDeg, mRotAxis[0], mRotAxis[1], mRotAxis[2] );
            glMultMatrixf( reinterpret_cast< float* >( mRotTransform ) );
            glGetFloatv( GL_MODELVIEW_MATRIX, reinterpret_cast< float* >( mRotTransform ) );
        glPopMatrix();

        glMultMatrixf( reinterpret_cast< float* >( mRotTransform ) );

        if( mMeshList )
        {
            if( mLightEnabled )
            {
                glEnable( GL_LIGHTING );
                glEnable( GL_LIGHT0 );
            }
            else
            {
                glDisable( GL_LIGHTING );
            }

            glPushMatrix();
                glPolygonMode( GL_FRONT_AND_BACK, mMeshFill ? GL_FILL : GL_LINE );
                glEnable( GL_COLOR_MATERIAL );
                glEnable( GL_DEPTH_TEST );
                glCallList( mMeshList );
            glPopMatrix();
        }
    glPopMatrix();    

    glFinish();
}


//!************************************************************************
//! Refresh the content
//!
//! @returns nothing
//!************************************************************************
/* slot */ void Plot3dCanvas::refresh()
{
    if( mParentPlot3d.isVisible() )
    {
        makeCurrent();
            make3dDataAllocateData();
            make3dDataAllocateDataBuf();

            make3dDataAssign();
            make3dDataCompute();

            init3dMeshList( false );
            update();
        doneCurrent();
    }
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
        glGetFloatv( GL_MODELVIEW_MATRIX, reinterpret_cast< float* >( mRotTransform ) );
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
    glTranslatef( 0.0, 0.0, -2.0 );
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
    float     aVec[3]       //!< vector
    ) const
{
    if( mSizeW && mSizeH )
    {
        aVec[0] = ( 2.0 * aX - mSizeW ) / mSizeW;
        aVec[1] = ( mSizeH - 2.0 * aY ) / mSizeH;
        float dist = sqrt( aVec[0] * aVec[0] + aVec[1] * aVec[1] );
        aVec[2] = cos( ( Numeric::PI / 2.0 ) * ( ( dist < 1.0 ) ? dist : 1.0 ) );

        float a = 1.0 / sqrt( aVec[0] * aVec[0] + aVec[1] * aVec[1] + aVec[2] * aVec[2] );
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
        makeCurrent();
            make3dDataAllocateData();
            make3dDataAllocateDataBuf();

            make3dDataAssign();
            make3dDataCompute();

            init3dMeshList( true );

            update();
        doneCurrent();
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
    mMaxVert = static_cast<float>( fabs( aVerticalMax ) );
    mMinVert = -mMaxVert;

    if( Plot3d::PLOT_3D_TYPE_TRANSIENT == mParentPlot3d.getPlot3dType() )
    {
        mHaveVertValues = true;

        makeCurrent();
            make3dDataAllocateData();
            make3dDataAllocateDataBuf();

            make3dDataAssign();
            make3dDataCompute();

            init3dMeshList( true );

            update();
        doneCurrent();
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
//! Stop the refresh
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::stopRefresh()
{
    mRefreshTimer->stop();
}


//!************************************************************************
//! Toggle the lighting
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::toogleLightEnable()
{
    mLightEnabled = !mLightEnabled;

    makeCurrent();
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
    mMeshFill = !mMeshFill;

    makeCurrent();
        update();
    doneCurrent();
}


//!************************************************************************
//! Update the content for new cepstrum data
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateAdxlCepstrum()
{
    if( mParentPlot3d.isVisible() && ( Plot3d::PLOT_3D_TYPE_CEPSTRUM == mParentPlot3d.getPlot3dType() ) )
    {
        if( !mRefreshTimer->isActive() )
        {
            mRefreshTimer->start( REFRESH_MS );
        }
    }
}


//!************************************************************************
//! Update the content for new accelerograms
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateAdxlData()
{
    if( mParentPlot3d.isVisible() && ( Plot3d::PLOT_3D_TYPE_TRANSIENT == mParentPlot3d.getPlot3dType() ) )
    {
        if( !mRefreshTimer->isActive() )
        {
            mRefreshTimer->start( REFRESH_MS );
        }
    }
}


//!************************************************************************
//! Update the content for new FFT data
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateAdxlFft()
{
    if( mParentPlot3d.isVisible() && ( Plot3d::PLOT_3D_TYPE_FFT == mParentPlot3d.getPlot3dType() ) )
    {
        if( !mRefreshTimer->isActive() )
        {
            mRefreshTimer->start( REFRESH_MS );
        }
    }
}


//!************************************************************************
//! Update the content for new periodogram data
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateAdxlPeriodogram()
{
    if( mParentPlot3d.isVisible() && ( Plot3d::PLOT_3D_TYPE_PERIODOGRAM == mParentPlot3d.getPlot3dType() ) )
    {
        if( !mRefreshTimer->isActive() )
        {
            mRefreshTimer->start( REFRESH_MS );
        }
    }
}


//!************************************************************************
//! Update the content for new SRS data
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateAdxlSrs()
{
    if( mParentPlot3d.isVisible() && ( Plot3d::PLOT_3D_TYPE_SRS == mParentPlot3d.getPlot3dType() ) )
    {
        if( !mRefreshTimer->isActive() )
        {
            mRefreshTimer->start( REFRESH_MS );
        }
    }
}


//!************************************************************************
//! Update the content for Down key
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateKeyDown()
{
    if( mMeshDeltaX > MESH_X_MIN )
    {
        cleanupVxsNs();
        cleanupData();
        cleanupDataBuf();

        mMeshDeltaX -= MESH_X_STEP;

        makeCurrent();
            make3dDataAllocateVxsNs();
            make3dDataAllocateData();
            make3dDataAllocateDataBuf();

            make3dDataAssign();
            make3dDataCompute();

            init3dMeshList( true );
            update();
        doneCurrent();
    }
}


//!************************************************************************
//! Update the content for Left key
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateKeyLeft()
{
    if( mMeshDeltaY > MESH_Y_MIN )
    {
        cleanupVxsNs();
        cleanupData();
        cleanupDataBuf();

        mMeshDeltaY -= MESH_Y_STEP;

        makeCurrent();
            make3dDataAllocateVxsNs();
            make3dDataAllocateData();
            make3dDataAllocateDataBuf();

            make3dDataAssign();
            make3dDataCompute();

            init3dMeshList( true );
            update();
        doneCurrent();
    }
}


//!************************************************************************
//! Update the content for Right key
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateKeyRight()
{    
    if( mMeshDeltaY < MESH_Y_MAX )
    {
        cleanupVxsNs();
        cleanupData();
        cleanupDataBuf();

        mMeshDeltaY += MESH_Y_STEP;

        makeCurrent();
            make3dDataAllocateVxsNs();
            make3dDataAllocateData();
            make3dDataAllocateDataBuf();

            make3dDataAssign();
            make3dDataCompute();

            init3dMeshList( true );
            update();
        doneCurrent();
    }
}


//!************************************************************************
//! Update the content for Up key
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::updateKeyUp()
{
    if( mMeshDeltaX < MESH_X_MAX )
    {
        cleanupVxsNs();
        cleanupData();
        cleanupDataBuf();

        mMeshDeltaX += MESH_X_STEP;

        makeCurrent();
            make3dDataAllocateVxsNs();
            make3dDataAllocateData();
            make3dDataAllocateDataBuf();

            make3dDataAssign();
            make3dDataCompute();

            init3dMeshList( true );
            update();
        doneCurrent();
    }
}


//!************************************************************************
//! Wipe all 3D data
//!
//! @returns nothing
//!************************************************************************
void Plot3dCanvas::wipeAllData()
{
    uint8_t i = 0;
    uint8_t j = 0;

    if( mMeshData )
    {
        for( i = 0; i < mMeshDeltaX; i++ )
        {
            for( j = 0; j < mMeshDeltaY; j++ )
            {
                mMeshData[i][j] = 0;
            }
        }
    }

    if( mMeshDataBuf )
    {
        for( i = 0; i < mMeshDeltaX; i++ )
        {
            for( j = 0; j < mMeshDeltaY; j++ )
            {
                mMeshDataBuf[i][j] = 0;
            }
        }
    }
}
