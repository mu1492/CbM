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
WindowFunction.h

This file contains the definitions for the FFT window functions.
*/

#ifndef WindowFunction_h
#define WindowFunction_h

#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>

#include <QString>


//************************************************************************
// Class for handling vibrations monitoring
//************************************************************************
class WindowFunction
{
    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        //************************************************************************
        // Defined window functions
        //************************************************************************
        typedef enum : uint8_t
        {
            WINDOW_FUNCTION_TYPE_BARTLETT,
            WINDOW_FUNCTION_TYPE_BARTLETT_HANN,
            WINDOW_FUNCTION_TYPE_BLACKMAN,
            WINDOW_FUNCTION_TYPE_BLACKMAN_EXACT,
            WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M3,
            WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_3,
            WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M4,
            WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_4,
            WINDOW_FUNCTION_TYPE_BLACKMAN_NUTTALL,
            WINDOW_FUNCTION_TYPE_BOHMAN,
            WINDOW_FUNCTION_TYPE_CAUCHY,
            WINDOW_FUNCTION_TYPE_DE_LA_VALLEE_POUSSIN,
            WINDOW_FUNCTION_TYPE_FLAT_TOP_5,
            WINDOW_FUNCTION_TYPE_GAUSSIAN,
            WINDOW_FUNCTION_TYPE_HAMMING,
            WINDOW_FUNCTION_TYPE_HANN,
            WINDOW_FUNCTION_TYPE_HANN_POISSON,
            WINDOW_FUNCTION_TYPE_KAISER_BESSEL,
            WINDOW_FUNCTION_TYPE_NUTTALL,
            WINDOW_FUNCTION_TYPE_PLANCK_TAPER,
            WINDOW_FUNCTION_TYPE_POISSON,
            WINDOW_FUNCTION_TYPE_RECTANGULAR,
            WINDOW_FUNCTION_TYPE_RIEMANN,
            WINDOW_FUNCTION_TYPE_RIESZ,
            WINDOW_FUNCTION_TYPE_SINE,
            WINDOW_FUNCTION_TYPE_TUKEY,
            WINDOW_FUNCTION_TYPE_WELCH
        }WindowFunctionType;

        static const std::map<WindowFunctionType, QString> WINDOW_FUNCTION_NAMES;

        typedef struct
        {
            bool   isEmphasized;    //!< true if emphasized for mechanical vibrations
            bool   needsParameter;  //!< true if the type needs a parameter
            double minValue;        //!< parameter minimum allowed value
            double crtValue;        //!< parameter current value
            double maxValue;        //!< parameter maximum allowed value
            QString paramSymbol;    //!< symbol for the parameter
        }ParameterData;

    private:
        const QString ALPHA_SMALL   = QString::fromUtf8( "\u03B1" );   //!< small alpha
        const QString EPSILON_SMALL = QString::fromUtf8( "\u03B5" );   //!< small epsilon


    //************************************************************************
    // functions
    //************************************************************************
    public:
        WindowFunction();

        ~WindowFunction();

        WindowFunctionType getActiveType();

        std::map<WindowFunctionType, ParameterData> getParametersMap() const;

        std::wstring getTypeDescription
            (
            const WindowFunctionType aType      //!< type
            ) const;

        bool getWindowValue
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool setActiveType
            (
            const WindowFunctionType aType      //!< type
            );

        bool setFftSize
            (
            const uint32_t  aSize               //!< FFT size
            );

        bool setParameter
            (
            const WindowFunctionType aWindowType,   //!< window type
            const double             aValue         //!< parameter value
            );

    private:
        std::wstring formatDescription
            (
            const double    aHighestSidelobedB,     //!< highest sidelobe [dB]
            const double    aFalloffRateDbOct,      //!< fall off rate [dB/oct]
            const double    a3DbBwBins = 0,         //!< 3dB bandwidth [bins]
            const double    a6DbBwBins = 0,         //!< 6dB bandwidth [bins]
            const double    aEquivNoiseBwBins = 0   //!< equivalent noise bandwidth [bins]
            ) const;

        std::wstring formatDescription
            (
            const std::vector<double>   aHighestSidelobedBVec,  //!< highest sidelobe vector [dB]
            const double                aFalloffRateDbOct,      //!< fall off rate [dB/oct]
            const std::vector<double>   a3DbBwBinsVec,          //!< 3dB bandwidth vector [bins]
            const std::vector<double>   a6DbBwBinsVec,          //!< 6dB bandwidth vector [bins]
            const std::vector<double>   aEquivNoiseBwBinsVec    //!< equivalent noise bandwidth vector[bins]
            ) const;

        bool getWindowValueBartlett
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBartlettHann
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBlackman
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBlackmanExact
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBlackmanHarrisMin3
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBlackmanHarris3
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBlackmanHarrisMin4
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBlackmanHarris4
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBlackmanNuttall
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueBohman
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueCauchy
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueDeLaValleePoussin
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueFlatTop5
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueGaussian
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueHamming
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueHann
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueHannPoisson
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueKaiserBessel
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueNuttall
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValuePlanckTaper
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValuePoisson
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueRectangular
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueRiemann
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueRiesz
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueSine
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueTukey
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;

        bool getWindowValueWelch
            (
            const uint32_t aIndex,              //!< index
            double*        aValue               //!< value
            ) const;


    //************************************************************************
    // variables
    //************************************************************************
    private:
        uint32_t                                    mFftSize;               //!< FFT size
        std::map<WindowFunctionType, ParameterData> mParametersMap;         //!< available parameters
        WindowFunctionType                          mActiveType;            //!< active window function
};

#endif // WindowFunction_h
