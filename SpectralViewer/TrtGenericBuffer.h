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
TrtGenericBuffer.h

This file contains the definitions for the TensorRT generic buffer.
*/

#ifndef TrtGenericBuffer_h
#define TrtGenericBuffer_h

#include <NvInfer.h>

#include <cstddef>


//************************************************************************
// Class for handling generic TensorRT buffer operations
//************************************************************************
template <typename AllocFunc, typename FreeFunc>
class TrtGenericBuffer
{
    //************************************************************************
    // functions
    //************************************************************************
    public:
        TrtGenericBuffer
            (
            nvinfer1::DataType aDataType = nvinfer1::DataType::kFLOAT   //!< data type
            );

        TrtGenericBuffer
            (
            size_t              aSize,      //!< size
            nvinfer1::DataType  aDataType   //!< data type
            );

        ~TrtGenericBuffer();

        size_t getBufferSize() const;

        void* getData();

        const void* getData() const;

        size_t getElementsCount() const;

        void resize
            (
            size_t aSize                    //!< new size
            );

        void resize
            (
            const nvinfer1::Dims& aDimensions   //!< tensor dimensions
            );

        TrtGenericBuffer& operator=
            (
            TrtGenericBuffer&& aBuffer      //!< buffer
            );

    private:
        static uint32_t getElementSize
            (
            nvinfer1::DataType aDataType    //!< data type
            ) noexcept;


    //************************************************************************
    // variables
    //************************************************************************
    private:
        size_t              mSize;          //!< number of elements in the buffer
        size_t              mCapacity;      //!< capacity, in elements
        nvinfer1::DataType  mType;          //!< data type
        void*               mBuffer;        //!< buffer

        AllocFunc           allocFn;        //!< memory allocation function
        FreeFunc            freeFn;         //!< memory deallocation function
};

#endif // TrtGenericBuffer_h
