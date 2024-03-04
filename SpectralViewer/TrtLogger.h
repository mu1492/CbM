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
TrtLogger.h

This file contains the definitions for the TensorRT logger.
*/

#ifndef TrtLogger_h
#define TrtLogger_h

#include <NvInfer.h>

#include <string>
#include <vector>


//************************************************************************
// Class for handling TensorRT logging
//************************************************************************
class TrtLogger : public nvinfer1::ILogger
{
    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        typedef nvinfer1::ILogger::Severity Severity;   //!< report severity


    //************************************************************************
    // functions
    //************************************************************************
    public:
        explicit TrtLogger();

        void clearReportVector();

        Severity getReportSeverity() const;

        std::vector<std::string> getReportVector() const;

        nvinfer1::ILogger& getTrtLogger() noexcept;

        virtual void log
            (
            Severity        aSeverity,      //!< report severity
            const char*     aMessage        //!< message
            ) noexcept override;

        bool setReportSeverity
            (
            const Severity  aSeverity       //!< report severity
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        Severity                    mReportSeverity;    //!< severity level
        std::vector<std::string>    mReportVec;         //!< vector with report messages
};

#endif // TrtLogger_h
