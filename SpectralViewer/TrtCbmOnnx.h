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
TrtCbmOnnx.h

This file contains the definitions for the CbM ONNX-TensorRT deep learning.
*/

#ifndef TrtCbmOnnx_h
#define TrtCbmOnnx_h

#include "TrtEngineBuffer.h"
#include "TrtLogger.h"

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <QObject>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>


//************************************************************************
// Class for handling CbM ONNX-TensorRT deep learning
//************************************************************************
class TrtCbmOnnx : public QObject
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        static const int32_t INPUT_BUFFER_SIZE = 128;   //!< data feed size

    private:
        typedef struct
        {
            std::string onnxFileName;                   //!< ONNX file name
            std::vector<std::string> inputTensorNames;  //!< input tensors
            std::vector<std::string> outputTensorNames; //!< output tensors
            int32_t inputSize;                          //!< input size
            int32_t outputSize;                         //!< output size
        }OnnxParams;

        typedef struct
        {
            template <typename T>
            void operator()
                (
                T*  aInferObject     //!< inference object
                ) const
            {
                delete aInferObject;
            }
        }InferDeleter;

        template <typename T>
        using InferUniquePtr = std::unique_ptr<T, InferDeleter>;

        static auto constexpr StreamDeleter = []
            (
            cudaStream_t* aStream   //!< CUDA stream
            )
        {
            if( aStream )
            {
                cudaStreamDestroy( *aStream );
                delete aStream;
            }
        };


    //************************************************************************
    // functions
    //************************************************************************
    public:
        TrtCbmOnnx();

        void clearTrtLog();

        bool feedInputData
            (
            const std::vector<float> aDataVector    //!< data
            );

        std::vector<std::string> getTrtlog() const;

        void infer();

        bool isEngineBuilt() const;

        bool isInferenceSuccessful
            (
            double& aLoss   //!< calculated loss
            ) const;

    public slots:
        void startBuild();

    private:
        bool constructNetwork
            (
            InferUniquePtr<nvonnxparser::IParser>& aParser  //!< ONNX parser
            );

        std::unique_ptr<cudaStream_t, decltype( StreamDeleter )> makeCudaStream();

        bool processInput
            (
            const TrtEngineBuffer& aEngineBuffer    //!< engine buffer
            );

        bool processOutput
            (
            const TrtEngineBuffer&  aEngineBuffer,  //!< engine buffer
            double&                 aLoss           //!< calculated loss
            );

    signals:
        void buildFinished();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        OnnxParams                              mOnnxParams;        //!< ONNX parameters
        std::shared_ptr<nvinfer1::ICudaEngine>  mCudaEngine;        //!< TensorRT engine used to run the network
        nvinfer1::Dims                          mInferInputDims;    //!< dimensions of the input to the network
        nvinfer1::Dims                          mInferOutputDims;   //!< dimensions of the output to the network
        TrtLogger                               mTrtLogger;         //!< TensorRT logger
        std::vector<float>                      mFeedDataVec;       //!< vector with feed data
        bool                                    mEngineBuilt;       //!< true if engine is built
        bool                                    mInferSuccessful;   //!< true if inference has no errors
        double                                  mInferLoss;         //!< calculated inference loss
};

#endif // TrtCbmOnnx_h
