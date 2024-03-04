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
TrtGenericBuffer.cpp

This file contains the sources for the TensorRT generic buffer.
*/

#include "TrtGenericBuffer.h"

#include "TrtEngineBuffer.h"

#include <new>
#include <numeric>


template class TrtGenericBuffer<CudaDeviceAllocator, CudaDeviceFree>;
template class TrtGenericBuffer<CudaHostAllocator, CudaHostFree>;

//!************************************************************************
//! Constructor
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
TrtGenericBuffer<AllocFunc, FreeFunc>::TrtGenericBuffer
    (
    nvinfer1::DataType aDataType    //!< data type
    )
    : mSize( 0 )
    , mCapacity( 0 )
    , mType( aDataType )
    , mBuffer( nullptr )
{
}


//!************************************************************************
//! Constructor
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
TrtGenericBuffer<AllocFunc, FreeFunc>::TrtGenericBuffer
    (
    size_t              aSize,      //!< size
    nvinfer1::DataType  aDataType   //!< data type
    )
    : mSize( aSize )
    , mCapacity( aSize )
    , mType( aDataType )
{
    if( !allocFn( &mBuffer, this->getBufferSize() ) )
    {
        throw std::bad_alloc();
    }
}


//!************************************************************************
//! Destructor
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
TrtGenericBuffer<AllocFunc, FreeFunc>::~TrtGenericBuffer()
{
    freeFn( mBuffer );
}


//!************************************************************************
//! Get the buffer size, in bytes
//!
//! @returns The number of bytes
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
size_t TrtGenericBuffer<AllocFunc, FreeFunc>::getBufferSize() const
{
    return this->getElementsCount() * getElementSize( mType );
}


//!************************************************************************
//! Get the buffer data
//!
//! @returns The pointer to buffer data
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
void* TrtGenericBuffer<AllocFunc, FreeFunc>::getData()
{
    return mBuffer;
}


//!************************************************************************
//! Get the buffer data
//!
//! @returns The pointer to buffer data
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
const void* TrtGenericBuffer<AllocFunc, FreeFunc>::getData() const
{
    return mBuffer;
}


//!************************************************************************
//! Get the size of an element
//!
//! @returns The size in bytes
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
uint32_t TrtGenericBuffer<AllocFunc, FreeFunc>::getElementSize
    (
    nvinfer1::DataType aDataType   //!< data type
    ) noexcept
{
    uint32_t elementSize = 0;

    switch( aDataType )
    {
        case nvinfer1::DataType::kINT32:
        case nvinfer1::DataType::kFLOAT:
            elementSize = 4;
            break;

        case nvinfer1::DataType::kHALF:
            elementSize = 2;
            break;

        case nvinfer1::DataType::kBOOL:
        case nvinfer1::DataType::kUINT8:
        case nvinfer1::DataType::kINT8:
            elementSize = 1;
            break;

       default:
            break;
    }

    return elementSize;
}


//!************************************************************************
//! Get the number of elements in the buffer
//!
//! @returns The number of elements
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
size_t TrtGenericBuffer<AllocFunc, FreeFunc>::getElementsCount() const
{
    return mSize;
}


//!************************************************************************
//! Resize the buffer
//!
//! @returns nothing
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
void TrtGenericBuffer<AllocFunc, FreeFunc>::resize
    (
    size_t aSize    //!< new size
    )
{
    mSize = aSize;

    if( mCapacity < aSize )
    {
        freeFn( mBuffer );

        if( !allocFn( &mBuffer, this->getBufferSize() ) )
        {
            throw std::bad_alloc{};
        }

        mCapacity = aSize;
    }
}


//!************************************************************************
//! Resize the buffer
//!
//! @returns nothing
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
void TrtGenericBuffer<AllocFunc, FreeFunc>::resize
    (
    const nvinfer1::Dims& aDimensions   //!< tensor dimensions
    )
{
    return this->resize( std::accumulate( aDimensions.d, aDimensions.d + aDimensions.nbDims, int64_t{ 1 }, std::multiplies<int64_t>{} ) );
}


//!************************************************************************
//! Assignment operator
//!
//! @returns The value of the object
//!************************************************************************
template <typename AllocFunc, typename FreeFunc>
TrtGenericBuffer<AllocFunc, FreeFunc>& TrtGenericBuffer<AllocFunc, FreeFunc>::operator=
    (
    TrtGenericBuffer&& aBuffer      //!< buffer
    )
{
    if( this != &aBuffer )
    {
        freeFn( mBuffer );

        mSize = aBuffer.mSize;
        mCapacity = aBuffer.mCapacity;
        mType = aBuffer.mType;
        mBuffer = aBuffer.mBuffer;

        aBuffer.mSize = 0;
        aBuffer.mCapacity = 0;
        aBuffer.mBuffer = nullptr;
    }

    return *this;
}
