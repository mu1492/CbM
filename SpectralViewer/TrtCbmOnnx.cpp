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
TrtCbmOnnx.cpp

This file contains the sources for the CbM ONNX-TensorRT deep learning.
*/

#include "TrtCbmOnnx.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>


//!************************************************************************
//! Constructor
//!************************************************************************
TrtCbmOnnx::TrtCbmOnnx()
    : mCudaEngine( nullptr )
    , mEngineBuilt( false )
    , mInferSuccessful( false )
    , mInferLoss( 0 )
{
    mTrtLogger.setReportSeverity( TrtLogger::Severity::kINFO );

    mOnnxParams.onnxFileName = "./data/cbm.onnx";
    mOnnxParams.inputTensorNames.push_back( "input" );
    mOnnxParams.outputTensorNames.push_back( "output" );
    mOnnxParams.inputSize = 0;
    mOnnxParams.outputSize = 0;
}


//!************************************************************************
//! Clear the full TensorRT log
//!
//! @returns nothing
//!************************************************************************
void TrtCbmOnnx::clearTrtLog()
{
    mTrtLogger.clearReportVector();
}


//!************************************************************************
//! Construct the ONNX network
//!
//! @returns true if the network can be constructed
//!************************************************************************
bool TrtCbmOnnx::constructNetwork
    (
    InferUniquePtr<nvonnxparser::IParser>& aParser      //!< ONNX parser
    )
{
    bool status = aParser->parseFromFile( mOnnxParams.onnxFileName.c_str(),  static_cast<int>( mTrtLogger.getReportSeverity() ) );
    return status;
}


//!************************************************************************
//! Feed the input buffer with new data
//!
//! @returns true data can be loaded to input buffers
//!************************************************************************
bool TrtCbmOnnx::feedInputData
    (
    const std::vector<float> aDataVector    //!< data
    )
{
    bool status = ( aDataVector.size() == mOnnxParams.inputSize
                  && INPUT_BUFFER_SIZE == mOnnxParams.inputSize );

    if( status )
    {
        for( int i = 0; i < mOnnxParams.inputSize; i++ )
        {
            mFeedDataVec[i] = aDataVector[i];
        }
    }

    return status;
}


//!************************************************************************
//! Get the full TensorRT log
//!
//! @returns The log as a strings vector
//!************************************************************************
std::vector<std::string> TrtCbmOnnx::getTrtlog() const
{
    return mTrtLogger.getReportVector();
}


//!************************************************************************
//! Run the TensorRT inference engine
//! Allocate the buffer, set inputs, and execute the engine.
//!
//! @returns nothing
//!************************************************************************
void TrtCbmOnnx::infer()
{
    bool status = false;
    mInferSuccessful = false;
    mInferLoss = 0;

    if( mEngineBuilt )
    {
        TrtEngineBuffer trtEngineBuffer( mCudaEngine );
        auto context = InferUniquePtr<nvinfer1::IExecutionContext>( mCudaEngine->createExecutionContext() );
        status = ( context != nullptr );

        if( status )
        {
            status = ( 1 == mOnnxParams.inputTensorNames.size() );
        }

        if( status )
        {
            status = processInput( trtEngineBuffer );
        }

        if( status )
        {
            status = trtEngineBuffer.copyCudaInputH2d();
        }

        if( status )
        {
            status = context->executeV2( trtEngineBuffer.getDeviceBindings().data() );
        }

        if( status )
        {
            status = trtEngineBuffer.copyCudaOutputD2h();
        }

        if( status )
        {
            status = processOutput( trtEngineBuffer, mInferLoss );
        }

        mInferSuccessful = status;
    }

    emit inferFinished();
}


//!************************************************************************
//! Check if engine is built
//!
//! @returns true if engine is built
//!************************************************************************
bool TrtCbmOnnx::isEngineBuilt() const
{
    return mEngineBuilt;
}


//!************************************************************************
//! Check if inference is error free
//!
//! @returns true if inference is successful
//!************************************************************************
bool TrtCbmOnnx::isInferenceSuccessful
    (
    double& aLoss   //!< calculated loss
    ) const
{
    aLoss = mInferLoss;
    return mInferSuccessful;
}


//!************************************************************************
//! Make a CUDA stream
//!
//! @returns Pointer to the created stream
//!************************************************************************
std::unique_ptr<cudaStream_t, decltype( TrtCbmOnnx::StreamDeleter )> TrtCbmOnnx::makeCudaStream()
{
    std::unique_ptr<cudaStream_t, decltype( StreamDeleter )> pStream( new cudaStream_t, StreamDeleter );

    if( cudaStreamCreateWithFlags( pStream.get(), cudaStreamNonBlocking ) != cudaSuccess )
    {
        pStream.reset( nullptr );
    }

    return pStream;
}


//!************************************************************************
//! Get and feed the inference engine with input data
//!
//! @returns true at success
//!************************************************************************
bool TrtCbmOnnx::processInput
    (
    const TrtEngineBuffer& aEngineBuffer    //!< engine buffer
    )
{
    float* hostDataBuffer = static_cast<float*>( aEngineBuffer.getHostBufferData( mOnnxParams.inputTensorNames[0] ) );

    for( int i = 0; i < mOnnxParams.inputSize; i++ )
    {
        hostDataBuffer[i] = mFeedDataVec[i];
    }

    return true;
}


//!************************************************************************
//! Get the output data from inference and calculate the loss
//!
//! @returns true at success
//!************************************************************************
bool TrtCbmOnnx::processOutput
    (
    const TrtEngineBuffer&  aEngineBuffer,  //!< engine buffer
    double&                 aLoss           //!< calculated loss
    )
{
    bool status = ( mOnnxParams.outputSize == mOnnxParams.inputSize );
    aLoss = 0;

    if( status )
    {
        float* fedData = static_cast<float*>( aEngineBuffer.getHostBufferData( mOnnxParams.inputTensorNames[0] ) );
        float* predictedData = static_cast<float*>( aEngineBuffer.getHostBufferData( mOnnxParams.outputTensorNames[0] ) );
        double sum = 0;

        for( int i = 0; i < mOnnxParams.outputSize; i++ )
        {
            sum += fabs( static_cast<double>( predictedData[i] - fedData[i] ) );
        }

        aLoss = sum;
    }

    return status;
}


//!************************************************************************
//! Creates the ONNX network, configures the builder and creates the TRT
//! network engine
//!
//! @returns nothing
//!************************************************************************
/* slot */ void TrtCbmOnnx::startBuild()
{
    auto builder = InferUniquePtr<nvinfer1::IBuilder>( nvinfer1::createInferBuilder( mTrtLogger.getTrtLogger() ) );
    bool status = ( builder != nullptr );

    if( status )
    {
        const auto explicitBatch = 1U << static_cast<uint32_t>( nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH );
        auto network = InferUniquePtr<nvinfer1::INetworkDefinition>( builder->createNetworkV2( explicitBatch ) );
        status = ( network != nullptr );

        if( status )
        {
            auto config = InferUniquePtr<nvinfer1::IBuilderConfig>( builder->createBuilderConfig() );
            status = ( config != nullptr );

            if( status )
            {
                auto parser = InferUniquePtr<nvonnxparser::IParser>( nvonnxparser::createParser( *network, mTrtLogger.getTrtLogger() ) );
                status = ( parser != nullptr );

                if( status )
                {
                    status = constructNetwork( parser );

                    if( status )
                    {
                        auto profileStream = makeCudaStream();
                        status = ( profileStream != nullptr );

                        if( status )
                        {
                            config->setProfileStream( *profileStream );
                            InferUniquePtr<nvinfer1::IHostMemory> plan{ builder->buildSerializedNetwork( *network, *config ) };
                            status = ( plan != nullptr );

                            if( status )
                            {
                                InferUniquePtr<nvinfer1::IRuntime> runtime{ nvinfer1::createInferRuntime( mTrtLogger.getTrtLogger() ) };
                                status = ( runtime != nullptr );

                                if( status )
                                {
                                    mCudaEngine = std::shared_ptr<nvinfer1::ICudaEngine>( runtime->deserializeCudaEngine( plan->data(), plan->size() ), InferDeleter() );
                                    status = ( mCudaEngine != nullptr );

                                    if( status )
                                    {
                                        status = ( 1 == network->getNbInputs() ) && ( 1 == network->getNbOutputs() );

                                        if( status )
                                        {
                                            mInferInputDims = network->getInput( 0 )->getDimensions();
                                            mInferOutputDims = network->getOutput( 0 )->getDimensions();

                                            mOnnxParams.inputSize = mInferInputDims.d[0];
                                            mOnnxParams.outputSize = mInferOutputDims.d[0];

                                            if( INPUT_BUFFER_SIZE == mOnnxParams.inputSize )
                                            {
                                                mFeedDataVec.resize( INPUT_BUFFER_SIZE, 0 );
                                            }
                                            else
                                            {
                                                status = false;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    mEngineBuilt = status;
    emit buildFinished();
}
