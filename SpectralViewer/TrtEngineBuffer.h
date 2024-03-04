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
TrtEngineBuffer.h

This file contains the definitions for the TensorRT CUDA engine buffer.
*/

#ifndef TrtEngineBuffer_h
#define TrtEngineBuffer_h

#include "TrtGenericBuffer.h"

#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>


//************************************************************************
// Class for allocating memory on CUDA device (NVIDIA GPU)
//************************************************************************
class CudaDeviceAllocator
{
    public:
        bool operator()
            (
            void**  aDevPtr,    //!< device pointer
            size_t  aSize       //!< memory size
            ) const
        {
            return cudaSuccess == cudaMalloc( aDevPtr, aSize );
        }
};

//************************************************************************
// Class for deallocating memory on CUDA device (NVIDIA GPU)
//************************************************************************
class CudaDeviceFree
{
    public:
        bool operator()
            (
            void*   aDevPtr     //!< device pointer
            ) const
        {
            return cudaSuccess == cudaFree( aDevPtr );
        }
};

//************************************************************************
// Class for allocating memory on CUDA host (CPU)
//************************************************************************
class CudaHostAllocator
{
    public:
        bool operator()
            (
            void**  aHostPtr,   //!< host pointer
            size_t  aSize       //!< memory size
            ) const
        {
            *aHostPtr = malloc( aSize );
            return *aHostPtr != nullptr;
        }
};

//************************************************************************
// Class for deallocating memory on CUDA host (CPU)
//************************************************************************
class CudaHostFree
{
    public:
        void operator()
            (
            void*   aHostPtr    //!< host pointer
            ) const
        {
            free( aHostPtr );
        }
};


using CudaDeviceBuffer = TrtGenericBuffer<CudaDeviceAllocator, CudaDeviceFree>;
using CudaHostBuffer = TrtGenericBuffer<CudaHostAllocator, CudaHostFree>;

//************************************************************************
// Class for grouping a pair of CUDA host-device buffers
//************************************************************************
class ManagedCudaBuffer
{
    public:
        CudaDeviceBuffer    deviceBuffer;
        CudaHostBuffer      hostBuffer;
};


//************************************************************************
// Class for handling buffer interactions with the CUDA engine
//************************************************************************
class TrtEngineBuffer
{
    //************************************************************************
    // functions
    //************************************************************************
    public:
        TrtEngineBuffer
            (
            std::shared_ptr<nvinfer1::ICudaEngine>  aCudaEngine,            //!< CUDA engine
            const nvinfer1::IExecutionContext*      aExeContext = nullptr   //!< execution context
            );

        ~TrtEngineBuffer() = default;

        bool copyCudaInputH2d();

        bool copyCudaOutputD2h();

        std::vector<void*>& getDeviceBindings();

        void* getHostBufferData
            (
            const std::string& aTensorName      //!< tensor name
            ) const;

    private:
        bool copyCudaBuffers
            (
            const bool          aCopyInput,     //!< true for input, false for output
            const bool          aD2h            //!< true for D2H, false for H2D
            );

        int32_t divUp
            (
            int32_t a,    //!< parameter
            int     b     //!< denominator
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        std::shared_ptr<nvinfer1::ICudaEngine>          mCudaEngine;        //!< pointer to the engine
        std::vector<std::unique_ptr<ManagedCudaBuffer>> mManagedCudaBuffer; //!< vector of pointers to the managed CUDA buffer
        std::vector<void*>                              mDeviceBindings;    //!< vector of device buffers needed for engine execution
};

#endif // TrtEngineBuffer_h
