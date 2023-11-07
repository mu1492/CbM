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
FftThread.cpp

This file contains the sources for the FFT thread.
*/

#include "FftThread.h"

#if BUILD_CUDA
    #include <algorithm>
    #include <cstring>
#endif


//!************************************************************************
//! Constructor
//!************************************************************************
FftThread::FftThread
    (
    QObject* aParent    //!< parent object
    )
    : QThread( aParent )
    , mIndex( 0 )
    , mFftSize( 0 )
    , mFftSizeChanged( false )
    , mIsRestarting( false )
    , mIsAborting( false )
#if BUILD_CUDA
    , mForceUseCpu( true )
    , mCuFftHandle( 0 )
    , mCuFftStream( nullptr )
    , mCuFftDirectDataInput( nullptr )
    , mCuFftDirectDataOutput( nullptr )
    , mCuFftInverseDataInput( nullptr )
    , mCuFftInverseDataOutput( nullptr )
#endif
{
#if BUILD_CUDA
    bool status = ( CUFFT_SUCCESS == cufftCreate( &mCuFftHandle ) );

    if( status )
    {
        status = ( cudaSuccess == cudaStreamCreateWithFlags( &mCuFftStream, cudaStreamNonBlocking ) );
    }

    if( status )
    {
        status = ( CUFFT_SUCCESS == cufftSetStream( mCuFftHandle, mCuFftStream ) );
    }
#endif
}


//!************************************************************************
//! Destructor
//!************************************************************************
FftThread::~FftThread()
{
    mMutex.lock();
        mIsAborting = true;
        mWaitCondition.wakeOne();
    mMutex.unlock();

    wait();

#if BUILD_CUDA
    if( mCuFftDirectDataOutput )
    {
        cudaFree( mCuFftDirectDataOutput );
    }
    if( mCuFftDirectDataInput )
    {
        cudaFree( mCuFftDirectDataInput );
    }

    if( mCuFftInverseDataOutput )
    {
        cudaFree( mCuFftInverseDataOutput );
    }
    if( mCuFftInverseDataInput )
    {
        cudaFree( mCuFftInverseDataInput );
    }

    cufftDestroy( mCuFftHandle );

    if( mCuFftStream )
    {
        cudaStreamDestroy( mCuFftStream );
    }
#endif
}


//!************************************************************************
//! Compute data loader
//! It provides input data and information for calculating the FFT.
//!
//! @returns nothing
//!************************************************************************
void FftThread::compute
    (
    std::vector<double>         aDataVec,   //!< input data for FFT computing
    FrequencyAnalysis::FftSense aSense,     //!< FFT compute sense - direct or inverse
    int                         aIndex,     //!< subthread index
    bool                        aCpuOnly    //!< true if selecting only CPU/FPU and no GPU
    )
{
    QMutexLocker locker( &mMutex );
    mDataVec = aDataVec;
    mSense = aSense;
    mIndex = aIndex;
#if BUILD_CUDA
    mForceUseCpu = aCpuOnly;
#else
    Q_UNUSED( aCpuOnly );
#endif
    mFftSizeChanged = ( mFftSize != mDataVec.size() - 1 );
    mFftSize = ( mDataVec.size() >= 1 ? mDataVec.size() - 1 : 0 );

    if( !isRunning() )
    {
        start();
    }
    else
    {
        mIsRestarting = true;
        mWaitCondition.wakeOne();
    }
}


//!************************************************************************
//! FFT thread main function
//!
//! @returns nothing
//!************************************************************************
/* virtual */ void FftThread::run()
{
    forever
    {
        mMutex.lock();
            double* dataArray = mDataVec.data();
            const int DATA_LEN = mDataVec.size();
            const int FFT_SIZE = mFftSize;
            const FrequencyAnalysis::FftSense SENSE = mSense;
            const int INDEX = mIndex;

            #if BUILD_CUDA
                const int CX_SIZE = FFT_SIZE / 2 + 1;

                // input for direct FFT, output for inverse FFT
                std::vector<double> cuIoDoubleVec( FFT_SIZE, 0 );
                // output for direct FFT, input for inverse FFT
                std::vector<std::complex<double>> cuIoComplexVec( CX_SIZE, std::complex<double>( 0, 0 ) );

                if( !mForceUseCpu )
                {
                    if( SENSE == FrequencyAnalysis::FFT_SENSE_DIRECT )
                    {
                        std::copy( mDataVec.begin() + 1, mDataVec.end(), cuIoDoubleVec.begin() );
                    }
                    else if( SENSE == FrequencyAnalysis::FFT_SENSE_INVERSE )
                    {
                        size_t cxVecLen = cuIoComplexVec.size();

                        cuIoComplexVec.at( 0 ) = std::complex<double>( mDataVec.at( 1 ), 0 );
                        cuIoComplexVec.at( cxVecLen - 1 ) = std::complex<double>( mDataVec.at( 2 ), 0 );

                        for( size_t i = 1; i <= cxVecLen - 2; i++ )
                        {
                            cuIoComplexVec.at( i ) = std::complex<double>( mDataVec.at( 2 * i + 1 ), -mDataVec.at( 2 * i + 2 ) );
                        }
                    }
                }
            #endif
        mMutex.unlock();

        if( mIsAborting )
        {
            return;
        }

        FrequencyAnalysis* faInstance = FrequencyAnalysis::getInstance();

        #if !BUILD_CUDA
            faInstance->calculateFft( dataArray, FFT_SIZE, SENSE );
        #else
            bool status = true;

            if( mForceUseCpu )
            {               
                faInstance->calculateFft( dataArray, FFT_SIZE, SENSE );
            }
            else
            {
                if( SENSE == FrequencyAnalysis::FFT_SENSE_DIRECT )
                {
                    if( mFftSizeChanged )
                    {
                        mFftSizeChanged = false;

                        if( status && mCuFftDirectDataOutput )
                        {
                            status = ( cudaSuccess == cudaFree( mCuFftDirectDataOutput ) );
                        }
                        if( status && mCuFftDirectDataInput )
                        {
                            status = ( cudaSuccess == cudaFree( mCuFftDirectDataInput ) );
                        }

                        if( status )
                        {
                            status = ( CUFFT_SUCCESS == cufftPlan1d( &mCuFftHandle,
                                                                     FFT_SIZE,
                                                                     CUFFT_D2Z,
                                                                     2 ) );
                        }

                        if( status )
                        {
                            status = ( cudaSuccess == cudaMalloc( static_cast<double**>( &mCuFftDirectDataInput ),
                                                                  sizeof( double ) * FFT_SIZE ) );
                        }
                        if( status )
                        {
                            status = ( cudaSuccess == cudaMalloc( static_cast<cufftDoubleComplex**>( &mCuFftDirectDataOutput ),
                                                                  sizeof( std::complex<double> ) * CX_SIZE ) );
                        }
                    }

                    if( status )
                    {
                        status = ( cudaSuccess == cudaMemcpyAsync( mCuFftDirectDataInput,
                                                                   cuIoDoubleVec.data(),
                                                                   sizeof( double ) * FFT_SIZE,
                                                                   cudaMemcpyHostToDevice,
                                                                   mCuFftStream ) );
                    }

                    if( status )
                    {
                        status = ( CUFFT_SUCCESS == cufftExecD2Z( mCuFftHandle,
                                                                  mCuFftDirectDataInput,
                                                                  mCuFftDirectDataOutput ) );
                    }

                    if( status )
                    {
                        status = ( cudaSuccess == cudaMemcpyAsync( cuIoComplexVec.data(),
                                                                   mCuFftDirectDataOutput,
                                                                   sizeof( std::complex<double> ) * CX_SIZE,
                                                                   cudaMemcpyDeviceToHost,
                                                                   mCuFftStream ) );
                    }

                    if( status )
                    {
                        status = ( cudaSuccess == cudaStreamSynchronize( mCuFftStream ) );
                    }

                    if( status )
                    {
                        size_t cxVecLen = cuIoComplexVec.size();

                        dataArray[1] = cuIoComplexVec.at( 0 ).real();
                        dataArray[2] = cuIoComplexVec.at( cxVecLen - 1 ).real();

                        for( int i = 3; i <= DATA_LEN - 1; i++ )
                        {
                            if( i % 2 )
                            {
                                dataArray[i] = cuIoComplexVec.at( ( i - 1 ) / 2 ).real();
                            }
                            else
                            {
                                dataArray[i] = -cuIoComplexVec.at( ( i - 2 ) / 2 ).imag();
                            }
                        }
                    }
                }
                else if( SENSE == FrequencyAnalysis::FFT_SENSE_INVERSE )
                {
                    if( mFftSizeChanged )
                    {
                        mFftSizeChanged = false;

                        if( status && mCuFftInverseDataOutput )
                        {
                            status = ( cudaSuccess == cudaFree( mCuFftInverseDataOutput ) );
                        }
                        if( status && mCuFftInverseDataInput )
                        {
                            status = ( cudaSuccess == cudaFree( mCuFftInverseDataInput ) );
                        }

                        if( status )
                        {
                            status = ( CUFFT_SUCCESS == cufftPlan1d( &mCuFftHandle,
                                                                     FFT_SIZE,
                                                                     CUFFT_Z2D,
                                                                     2 ) );
                        }

                        if( status )
                        {
                            status = ( cudaSuccess == cudaMalloc( static_cast<cufftDoubleComplex**>( &mCuFftInverseDataInput ),
                                                                  sizeof( std::complex<double> ) * CX_SIZE ) );
                        }
                        if( status )
                        {
                            status = ( cudaSuccess == cudaMalloc( static_cast<double**>( &mCuFftInverseDataOutput ),
                                                                  sizeof( double ) * FFT_SIZE ) );
                        }
                    }

                    if( status )
                    {
                        status = ( cudaSuccess == cudaMemcpyAsync( mCuFftInverseDataInput,
                                                                   cuIoComplexVec.data(),
                                                                   sizeof( std::complex<double> ) * CX_SIZE,
                                                                   cudaMemcpyHostToDevice,
                                                                   mCuFftStream ) );
                    }

                    if( status )
                    {
                        status = ( CUFFT_SUCCESS == cufftExecZ2D( mCuFftHandle,
                                                                  mCuFftInverseDataInput,
                                                                  mCuFftInverseDataOutput ) );
                    }

                    if( status )
                    {
                        status = ( cudaSuccess == cudaMemcpyAsync( cuIoDoubleVec.data(),
                                                                   mCuFftInverseDataOutput,
                                                                   sizeof( double ) * FFT_SIZE,
                                                                   cudaMemcpyDeviceToHost,
                                                                   mCuFftStream ) );
                    }

                    if( status )
                    {
                        status = ( cudaSuccess == cudaStreamSynchronize( mCuFftStream ) );
                    }

                    if( status )
                    {
                        memcpy( &dataArray[1], cuIoDoubleVec.data(), FFT_SIZE * sizeof( double ) );
                    }
                }
            }
        #endif

        if( !mIsRestarting )
        {
            #if !BUILD_CUDA
                emit fftComputeDone( dataArray, DATA_LEN, INDEX );
            #else
                if( mForceUseCpu )
                {
                    emit fftComputeDone( dataArray, DATA_LEN, INDEX );
                }
                else
                {
                    if( status )
                    {
                        emit fftComputeDone( dataArray, DATA_LEN, INDEX );
                    }
                }
            #endif
        }

        mMutex.lock();
            if( !mIsRestarting )
            {
                mWaitCondition.wait( &mMutex );
            }

            mIsRestarting = false;
        mMutex.unlock();
    }
}
