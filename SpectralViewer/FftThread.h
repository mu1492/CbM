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
FftThread.h

This file contains the definitions for the FFT thread.
*/

#ifndef FftThread_h
#define FftThread_h

#include "FrequencyAnalysis.h"

#include <vector>

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#if BUILD_CUDA
    #include <complex>
    #include <cufftXt.h>
#endif


//************************************************************************
// Class for handling the FFT calculation thread
//************************************************************************
class FftThread : public QThread
{
    Q_OBJECT

    //************************************************************************
    // functions
    //************************************************************************
    public:
        FftThread
            (
            QObject* aParent = nullptr              //!< parent object
            );

        ~FftThread();

        void compute
            (
            std::vector<double>         aDataVec,   //!< input data for FFT computing
            FrequencyAnalysis::FftSense aSense,     //!< FFT compute sense - direct or inverse
            int                         aIndex,     //!< subthread index
            bool                        aCpuOnly    //!< true if selecting only CPU/FPU and no GPU
            );

    protected:
        void run() override;

    signals:
        void fftComputeDone
            (
            double*  aDataArray,    //!< output data array with computed FFT
            int      aLength,       //!< array length
            int      aIndex         //!< subthread index
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        QMutex                      mMutex;         //!< mutex
        QWaitCondition              mWaitCondition; //!< wait condition

        std::vector<double>         mDataVec;       //!< FFT data
        FrequencyAnalysis::FftSense mSense;         //!< FFT sense
        int                         mIndex;         //!< subthread index

        uint32_t                    mFftSize;       //!< FFT size
        bool                        mFftSizeChanged;//!< true if the FFT size changed

        bool                        mIsRestarting;  //!< true if restarting
        bool                        mIsAborting;    //!< true if aborting

    #if BUILD_CUDA
        bool                        mForceUseCpu;               //!< true if not computing FFT with CUDA, even if present

        cufftHandle                 mCuFftHandle;               //!< CUDA FFT handle
        cudaStream_t                mCuFftStream;               //!< CUDA stream

        double*                     mCuFftDirectDataInput;      //!< input data for CUDA direct FFT
        cufftDoubleComplex*         mCuFftDirectDataOutput;     //!< output data for CUDA direct FFT

        cufftDoubleComplex*         mCuFftInverseDataInput;     //!< input data for CUDA inverse FFT
        double*                     mCuFftInverseDataOutput;    //!< output data for CUDA inverse FFT
    #endif
};

#endif // FftThread_h
