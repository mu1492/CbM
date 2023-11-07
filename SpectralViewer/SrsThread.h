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
SrsThread.h

This file contains the definitions for the SRS thread.
*/

#ifndef SrsThread_h
#define SrsThread_h

#include "FftThread.h"

#include <vector>

#include <QMutex>
#include <QThread>
#include <QWaitCondition>


//************************************************************************
// Class for handling the SRS calculation thread
//************************************************************************
class SrsThread : public QThread
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        /// ******************************************************************
        /// ISO 18431-4:2007 recommends a number of 20 points per decade for
        /// the natural frequencies (zeta = 0.05, Q = 10).
        /// For a system focusing on 5 decades, this will lead to a number of
        /// 100 points. However, a higher number will provide more frequency
        /// resolution in the same conditions. Because the damping coefficient
        /// is adjustable during runtime, the implementation relies on a number
        /// of points twice as large than recommended in the above mentioned
        /// standard.
        ///
        /// Natural frequency [Hz]  0.01  0.1    1    10    100  1000
        ///                          |     |     |     |     |     |
        ///        Decade #           <-1-> <-2-> <-3-> <-4-> <-5->
        ///
        /// Computing the SRS relies on subthreads for each point per axis,
        /// therefore for all three axes the number of SRS subthreads will be
        /// (NR_OF_POINTS x 3).
        ///
        /// A much higher NR_OF_POINTS won't bring more relevant spectral
        /// information, and the Linux multithreading handler will generate a
        /// message like:
        /// "GLib-ERROR **: [...] Creating pipes for GWakeup: Too many open files"
        ///
        /// For an estimate of the maximum NR_OF_POINTS supported by the system,
        /// use the following command for getting the number of open files:
        ///
        /// ulimit -Sn
        ///
        /// For example, if the result is 1024, and there are already 124 open
        /// file handles, the maximum for NR_OF_POINTS will be (1024-124)/3 = 300.
        /// This is independent of calculating the FFT using the CPU or GPU.
        ///
        /// For ADXL355/ADXL357, the LPF cutoff frequency is half of the Nyquist
        /// frequency and a quarter of the sampling rate. In this case, any SRS
        /// stop natural frequency in (0.25*SPS...0.5*SPS] will provide relevant
        /// spectral information. The current implementation calculates the SRS
        /// for natural frequencies going up to nearly 0.5*SPS, using a constant
        /// number of points per decade.
        ///
        /// A selected list of references is given at the beginning of
        /// FrequencyAnalysis.cpp.
        /// ******************************************************************
        static const int NR_OF_POINTS = 200;                //!< natural frequency points per axis

    public:
        static constexpr double NATURAL_FREQ_MIN = 0.01;    //!< [Hz]


    //************************************************************************
    // functions
    //************************************************************************
    public:
        SrsThread
            (
            QObject* aParent = nullptr              //!< parent object
            );

        ~SrsThread();

        void compute
            (
            std::vector<double>         aDataVec    //!< input data for SRS computing
            );

    protected:
        void run() override;

    private slots:
        void receiveSubthreadFft
            (
            double*  aDataArray, //!< array with computed FFT values
            int      aLength,    //!< array length
            int      aIndex      //!< subthread index
            );

    signals:
        void srsComputeDone
            (
            double*  aNaturalFrequenciesArray,  //!< natural frequencies
            double*  aValuesArray,              //!< computed SRS values
            int      aLength                    //!< length of both arrays
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        QMutex                      mMutex;         //!< mutex
        QWaitCondition              mWaitCondition; //!< wait condition

        std::vector<double>         mDataVec;       //!< SRS data
        uint32_t                    mFftSize;       //!< FFT size

        bool                        mIsRestarting;  //!< true if restarting
        bool                        mIsAborting;    //!< true if aborting

        std::vector<FftThread>      mFftSubthreadsVec;      //!< vector with FFT compute subthreads
        std::vector<bool>           mFftSubthreadsReady;    //!< vector with ready statuses

        std::vector<double>         mNaturalFreqVec;    //!< natural frequencies vector
        std::vector<double>         mSrsValuesVec;      //!< SRS values vector
};

#endif // SrsThread_h
