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
TrtLogger.cpp

This file contains the sources for the TensorRT logger.
*/

#include "TrtLogger.h"


//!************************************************************************
//! Constructor
//!************************************************************************
TrtLogger::TrtLogger()
    : mReportSeverity( Severity::kERROR )
{
}


//!************************************************************************
//! Clear the vector with report messages
//!
//! @returns nothing
//!************************************************************************
void TrtLogger::clearReportVector()
{
    mReportVec.clear();
}


//!************************************************************************
//! Get the report severity
//!
//! @returns The severity level
//!************************************************************************
TrtLogger::Severity TrtLogger::getReportSeverity() const
{
    return mReportSeverity;
}


//!************************************************************************
//! Get the vector with report messages
//!
//! @returns The strings vector
//!************************************************************************
std::vector<std::string> TrtLogger::getReportVector() const
{
    return mReportVec;
}


//!************************************************************************
//! Retrieve the nvinfer::ILogger associated with this Logger
//!
//! @returns The associated nvinfer1::ILogger
//!************************************************************************
nvinfer1::ILogger& TrtLogger::getTrtLogger() noexcept
{
    return *this;
}


//!************************************************************************
//! Log messages according to the specified severity
//! Implementation of the nvinfer1::ILogger::log() virtual method
//!
//! @returns nothing
//!************************************************************************
/* virtual */ void TrtLogger::log
    (
    Severity        aSeverity,      //!< report severity
    const char*     aMessage        //!< message
    ) noexcept
{
    if( aSeverity <= mReportSeverity )
    {
        std::string msg;

        switch( aSeverity )
        {
            case Severity::kINTERNAL_ERROR:
                msg = "[N]";
                break;

            case Severity::kERROR:
                msg = "[E]";
                break;

            case Severity::kWARNING:
                msg = "[W]";
                break;

            case Severity::kINFO:
                msg = "[I]";
                break;

            case Severity::kVERBOSE:
            default:
                msg = "[V]";
                break;
        }

        msg.append( " " );
        msg.append( aMessage );
        mReportVec.push_back( msg );
    }
}


//!************************************************************************
//! Set the report severity
//!
//! @returns true if the severity can be set
//!************************************************************************
bool TrtLogger::setReportSeverity
    (
    const Severity  aSeverity       //!< report severity
    )
{
    bool status = ( aSeverity >= Severity::kINTERNAL_ERROR ) && ( aSeverity <= Severity::kVERBOSE );

    if( status )
    {
        mReportSeverity = aSeverity;
    }

    return status;
}
