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
TrtEngineBuffer.cpp

This file contains the sources for the TensorRT CUDA engine buffer.
*/

#include "TrtEngineBuffer.h"

#include <cstdint>
#include <numeric>


//!************************************************************************
//! Constructor
//!************************************************************************
TrtEngineBuffer::TrtEngineBuffer
    (
    std::shared_ptr<nvinfer1::ICudaEngine>  aCudaEngine,    //!< CUDA engine
    const nvinfer1::IExecutionContext*      aExeContext     //!< execution context
    )
    : mCudaEngine( aCudaEngine )
{
    for( int i = 0; i < mCudaEngine->getNbIOTensors(); i++ )
    {
        auto tensorName = mCudaEngine->getBindingName( i );
        auto bindingDims = aExeContext ? aExeContext->getTensorShape( tensorName ) : mCudaEngine->getTensorShape( tensorName );
        size_t vol = aExeContext || 1;
        nvinfer1::DataType dataType = mCudaEngine->getTensorDataType( tensorName );
        int vectorizedDim = mCudaEngine->getTensorVectorizedDim( tensorName );

        if( -1 != vectorizedDim )
        {
            int scalarsPerElement = mCudaEngine->getTensorComponentsPerElement( tensorName );
            bindingDims.d[vectorizedDim] = divUp( bindingDims.d[vectorizedDim], scalarsPerElement );
            vol *= scalarsPerElement;
        }

        vol *= std::accumulate( bindingDims.d, bindingDims.d + bindingDims.nbDims, int64_t{ 1 }, std::multiplies<int64_t>{} );

        std::unique_ptr<ManagedCudaBuffer> managedCudaBuffer{ new ManagedCudaBuffer() };
        managedCudaBuffer->deviceBuffer = CudaDeviceBuffer( vol, dataType );
        managedCudaBuffer->hostBuffer = CudaHostBuffer( vol, dataType );

        mDeviceBindings.emplace_back( managedCudaBuffer->deviceBuffer.getData() );
        mManagedCudaBuffer.emplace_back( std::move( managedCudaBuffer ) );
    }
}


//!************************************************************************
//! Copy CUDA D2H or H2D buffers
//!
//! @returns true if the buffers can be copied
//!************************************************************************
bool TrtEngineBuffer::copyCudaBuffers
    (
    const bool aCopyInput,      //!< true for input, false for output
    const bool aD2h             //!< true for D2H, false for H2D
    )
{
    bool status = true;
    const cudaMemcpyKind MEMCPY_TYPE = aD2h ? cudaMemcpyDeviceToHost : cudaMemcpyHostToDevice;

    for( int i = 0; i < mCudaEngine->getNbIOTensors(); i++ )
    {
        void* dstPtr = aD2h ? mManagedCudaBuffer[i]->hostBuffer.getData() :
                              mManagedCudaBuffer[i]->deviceBuffer.getData();

        const void* SRC_PTR = aD2h ? mManagedCudaBuffer[i]->deviceBuffer.getData() :
                                     mManagedCudaBuffer[i]->hostBuffer.getData();

        const size_t BYTE_SIZE = mManagedCudaBuffer[i]->hostBuffer.getBufferSize();

        auto tensorName = mCudaEngine->getBindingName( i );
        bool tensorIsInput = ( nvinfer1::TensorIOMode::kINPUT == mCudaEngine->getTensorIOMode( tensorName ) );

        if( ( aCopyInput &&  tensorIsInput )
        || ( !aCopyInput && !tensorIsInput ) )
        {
            if( cudaSuccess != cudaMemcpy( dstPtr, SRC_PTR, BYTE_SIZE, MEMCPY_TYPE ) )
            {
                status = false;
                break;
            }
        }
    }

    return status;
}


//!************************************************************************
//! Copy the contents of input H2D
//!
//! @returns true if buffers can be copied
//!************************************************************************
bool TrtEngineBuffer::copyCudaInputH2d()
{
    return copyCudaBuffers( true, false );
}


//!************************************************************************
//! Copy the contents of output D2H
//!
//! @returns true if buffers can be copied
//!************************************************************************
bool TrtEngineBuffer::copyCudaOutputD2h()
{
    return copyCudaBuffers( false, true );
}


//!************************************************************************
//! Calculate 1+(a-1)/b
//!
//! @returns The expression value
//!************************************************************************
int32_t TrtEngineBuffer::divUp
    (
    int32_t a,    //!< parameter
    int     b     //!< denominator
    )
{
    return b ? ( a + b - 1 ) / b  : 1;
}


//!************************************************************************
//! Get the vector of device buffers bindings for IExecutionContext
//! (i.e. for the IExecutionContext execute and enqueue methods)
//!
//! @returns The vector of device buffers
//!************************************************************************
std::vector<void*>& TrtEngineBuffer::getDeviceBindings()
{
    return mDeviceBindings;
}


//!************************************************************************
//! Get the host buffer data corresponding to a tensor name
//!
//! @returns The host buffer data
//!************************************************************************
void* TrtEngineBuffer::getHostBufferData
    (
    const std::string& aTensorName      //!< tensor name
    ) const
{
    int index = mCudaEngine->getBindingIndex( aTensorName.c_str() );
    return ( -1 != index ) ? mManagedCudaBuffer[index]->hostBuffer.getData() : nullptr;
}
