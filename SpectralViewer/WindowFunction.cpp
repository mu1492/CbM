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
WindowFunction.cpp

This file contains the sources for the FFT window functions.
*/

/*
References (selected list):
===========================
[1] Gade, S., Herulfsen, H. - Use of Weighting Functions in DFT/FFT Analysis, Part I,
            Bruel&Kjaer Technical Review 3, 1987, pp. 1-28

[2] Gade, S., Herulfsen, H. - Use of Weighting Functions in DFT/FFT Analysis, Part II,
            Bruel&Kjaer Technical Review 4, 1987, pp. 1-35

[3] Harris, F.J. - On the Use of Windows for Harmonic Analysis with the Discrete Fourier Transform,
            Proc. IEEE 66, pp. 51-83, 1978

[4] Nutall, A.H. - Some Windows with Very Good Sidelobe Behavior,
            IEEE Trans. Acoustics, Speech, Sig. Proc. 29 (1), pp. 84-91, 1981

[5] Oppenheim, A.V., Schafer, R.W. - Digital Signal Processing, Prentice-Hall, 1975

[6] ISO 18431-2:2004. Mechanical vibration and shock - Signal processing -
            Part 2: Time domain windows for Fourier Transform analysis

[7] ISO 18431-2:2004/Cor 1:2008. Mechanical vibration and shock - Signal processing -
            Part 2: Time domain windows for Fourier Transform analysis - Technical Corrigendum 1
*/

#include "WindowFunction.h"

#include "Numeric.h"


const std::map<WindowFunction::WindowFunctionType, QString> WindowFunction::WINDOW_FUNCTION_NAMES =
{
    { WindowFunction::WINDOW_FUNCTION_TYPE_BARTLETT,             "Bartlett (triangular)" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BARTLETT_HANN,        "Bartlett-Hann" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BLACKMAN,             "Blackman" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BLACKMAN_EXACT,       "Blackman exact" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M3,   "Blackman-Harris min. 3-term" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_3,    "Blackman-Harris 3-term" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M4,   "Blackman-Harris min. 4-term" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_4,    "Blackman-Harris 4-term" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BLACKMAN_NUTTALL,     "Blackman-Nuttall" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_BOHMAN,               "Bohman" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_CAUCHY,               "Cauchy" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_DE_LA_VALLEE_POUSSIN, "De la Vallee-Poussin (Parzen)" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_FLAT_TOP_5,           "Flat top 5th order" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_GAUSSIAN,             "Gaussian (Weierstrass)" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_HAMMING,              "Hamming" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_HANN,                 "Hann" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_HANN_POISSON,         "Hann-Poisson" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_KAISER_BESSEL,        "Kaiser-Bessel" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_NUTTALL,              "Nuttall" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_PLANCK_TAPER,         "Planck taper" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_POISSON,              "Poisson (exponential)" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_RECTANGULAR,          "Rectangular (uniform)" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_RIEMANN,              "Riemann" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_RIESZ,                "Riesz" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_SINE,                 "Sine" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_TUKEY,                "Tukey (tapered cosine)" },
    { WindowFunction::WINDOW_FUNCTION_TYPE_WELCH,                "Welch" }
};


//!************************************************************************
//! Constructor
//!************************************************************************
WindowFunction::WindowFunction()
    : mFftSize( 1 )
    , mActiveType( WINDOW_FUNCTION_TYPE_HANN )
{
    mParametersMap =
    {
        /////////////////////////////////////////////////////////////////////
        /// Boundaries in the following list can be modified as convenient.
        /// Table I (pp. 55) and Fig 12 (pp. 58) from [3] may provide more insight.
        /////////////////////////////////////////////////////////////////////

        { WINDOW_FUNCTION_TYPE_BARTLETT,             { true,   false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BARTLETT_HANN,        { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BLACKMAN,             { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BLACKMAN_EXACT,       { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M3,   { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_3,    { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M4,   { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_4,    { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BLACKMAN_NUTTALL,     { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_BOHMAN,               { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_CAUCHY,               { false,  true,  3.0, 4.0, 5.0, ALPHA_SMALL } },
        { WINDOW_FUNCTION_TYPE_DE_LA_VALLEE_POUSSIN, { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_FLAT_TOP_5,           { true,   false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_GAUSSIAN,             { false,  true,  2.0, 2.5, 5.0, ALPHA_SMALL } },
        { WINDOW_FUNCTION_TYPE_HAMMING,              { true,   false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_HANN,                 { true,   false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_HANN_POISSON,         { false,  true,  0.5, 1.0, 2.0, ALPHA_SMALL } },
        { WINDOW_FUNCTION_TYPE_KAISER_BESSEL,        { false,  true,  2.0, 3.0, 3.5, ALPHA_SMALL } },
        { WINDOW_FUNCTION_TYPE_NUTTALL,              { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_PLANCK_TAPER,         { false,  true,  0.01, 0.1, 1.0, EPSILON_SMALL } },
        { WINDOW_FUNCTION_TYPE_POISSON,              { false,  true,  2.0, 2.5, 4.0, ALPHA_SMALL } },
        { WINDOW_FUNCTION_TYPE_RECTANGULAR,          { true,   false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_RIEMANN,              { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_RIESZ,                { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_SINE,                 { false,  false, 0, 0, 0, "N/A" } },
        { WINDOW_FUNCTION_TYPE_TUKEY,                { false,  true,  0.01, 0.5, 1.0, ALPHA_SMALL } },
        { WINDOW_FUNCTION_TYPE_WELCH,                { true,   false, 0, 0, 0, "N/A" } }
    };
}


//!************************************************************************
//! Destructor
//!************************************************************************
WindowFunction::~WindowFunction()
{
}


//!************************************************************************
//! Format the wstring for the window function description
//!
//! @returns The description as wide string
//!************************************************************************
std::wstring WindowFunction::formatDescription
    (
    const double    aHighestSidelobedB,     //!< highest sidelobe [dB]
    const double    aFalloffRateDbOct,      //!< fall off rate [dB/oct]
    const double    a3DbBwBins,             //!< 3dB bandwidth [bins]
    const double    a6DbBwBins,             //!< 6dB bandwidth [bins]
    const double    aEquivNoiseBwBins       //!< equivalent noise bandwidth [bins]
    ) const
{
    std::wstring description;
    const size_t LEN = 10;
    wchar_t tmpWstr[LEN] = L"";

    swprintf( tmpWstr, LEN, L"%.2lf", aHighestSidelobedB );
    description = L"Highest sidelobe = " + std::wstring( tmpWstr ) + L" dB";

    swprintf( tmpWstr, LEN, L"%.0lf", aFalloffRateDbOct );
    description += L"\nFall-off rate = " + std::wstring( tmpWstr ) + L" dB/oct";

    if( a3DbBwBins )
    {
        swprintf( tmpWstr, LEN, L"%.2lf", a3DbBwBins );
        description += L"\n3dB BW = " + std::wstring( tmpWstr ) + L" bins";
    }

    if( a6DbBwBins )
    {
        swprintf( tmpWstr, LEN, L"%.2lf", a6DbBwBins );
        description += L"\n6dB BW = "  + std::wstring( tmpWstr ) + L" bins";
    }

    if( aEquivNoiseBwBins )
    {
        swprintf( tmpWstr, LEN, L"%.2lf", aEquivNoiseBwBins );
        description += L"\nEquivalent noise BW = " + std::wstring( tmpWstr ) + L" bins";
    }

    return description;
}


//!************************************************************************
//! Format the wstring for the window function description
//!
//! @returns The description as wide string
//!************************************************************************
std::wstring WindowFunction::formatDescription
    (
    const std::vector<double>   aHighestSidelobedBVec,  //!< highest sidelobe vector [dB]
    const double                aFalloffRateDbOct,      //!< fall off rate [dB/oct]
    const std::vector<double>   a3DbBwBinsVec,          //!< 3dB bandwidth vector [bins]
    const std::vector<double>   a6DbBwBinsVec,          //!< 6dB bandwidth vector [bins]
    const std::vector<double>   aEquivNoiseBwBinsVec    //!< equivalent noise bandwidth vector[bins]
    ) const
{
    std::wstring description;
    const std::wstring APPROX = L"\x2248";
    const std::wstring ALPHA_SMALL = L"\x03B1";
    const std::wstring SQUARED = L"\x00B2";

    const size_t LEN = 20;
    wchar_t tmpWstr[LEN] = L"";

    description = L"Highest sidelobe (" + ALPHA_SMALL + L") " + APPROX + L" ";
    for( size_t i = 0; i < aHighestSidelobedBVec.size(); i++ )
    {
        if( i >= 3 )
        {
            break;
        }

        if( aHighestSidelobedBVec[i] )
        {
            if( aHighestSidelobedBVec[i] > 0 && i )
            {
                description += L"+";
            }

            swprintf( tmpWstr, LEN, L"%lg", aHighestSidelobedBVec[i] );
            description += std::wstring( tmpWstr );

            if( i )
            {
                description += ALPHA_SMALL;
            }

            if( 2 == i  )
            {
                description += SQUARED;
            }
        }
    }
    description += L" dB";

    swprintf( tmpWstr, LEN, L"%lg", aFalloffRateDbOct );
    description += L"\nFall-off rate = " + std::wstring( tmpWstr ) + L" dB/oct";

    description += L"\n3dB BW (" + ALPHA_SMALL + L") " + APPROX + L" ";
    for( size_t i = 0; i < a3DbBwBinsVec.size(); i++ )
    {
        if( i >= 2 )
        {
            break;
        }

        if( a3DbBwBinsVec[i] )
        {
            if( a3DbBwBinsVec[i] > 0 && i )
            {
                description += L"+";
            }

            swprintf( tmpWstr, LEN, L"%lg", a3DbBwBinsVec[i] );
            description += std::wstring( tmpWstr );

            if( i )
            {
                description += ALPHA_SMALL;
            }
        }
    }
    description += L" bins";


    description += L"\n6dB BW (" + ALPHA_SMALL + L") " + APPROX + L" ";
    for( size_t i = 0; i < a6DbBwBinsVec.size(); i++ )
    {
        if( i >= 2 )
        {
            break;
        }

        if( a6DbBwBinsVec[i] )
        {
            if( a6DbBwBinsVec[i] > 0 && i )
            {
                description += L"+";
            }

            swprintf( tmpWstr, LEN, L"%lg", a6DbBwBinsVec[i] );
            description += std::wstring( tmpWstr );

            if( i )
            {
                description += ALPHA_SMALL;
            }
        }
    }
    description += L" bins";


    description += L"\nEquivalent noise BW (" + ALPHA_SMALL + L") " + APPROX + L" ";
    for( size_t i = 0; i < aEquivNoiseBwBinsVec.size(); i++ )
    {
        if( i >= 2 )
        {
            break;
        }

        if( aEquivNoiseBwBinsVec[i] )
        {
            if( aEquivNoiseBwBinsVec[i] > 0 && i )
            {
                description += L"+";
            }

            swprintf( tmpWstr, LEN, L"%lg", aEquivNoiseBwBinsVec[i] );
            description += std::wstring( tmpWstr );

            if( i )
            {
                description += ALPHA_SMALL;
            }
        }
    }
    description += L" bins";

    return description;
}


//!************************************************************************
//! Get the active type
//!
//! @returns The active type
//!************************************************************************
WindowFunction::WindowFunctionType WindowFunction::getActiveType()
{
    return mActiveType;
}


//!************************************************************************
//! Get the map with parameters for each type
//!
//! @returns The parameters map
//!************************************************************************
std::map<WindowFunction::WindowFunctionType, WindowFunction::ParameterData> WindowFunction::getParametersMap() const
{
    return mParametersMap;
}


//!************************************************************************
//! Get the description of the specified type
//!
//! @returns The description string
//!************************************************************************
std::wstring WindowFunction::getTypeDescription
    (
    const WindowFunctionType aType        //!< type
    ) const
{
    std::wstring description = L" ";
    std::vector<double> hslVec;
    std::vector<double> bw3dbVec;
    std::vector<double> bw6dbVec;
    std::vector<double> enbwVec;

    switch( aType )
    {
        case WINDOW_FUNCTION_TYPE_BARTLETT:
            description = formatDescription( -26.45, 12, 1.28, 1.78, 1.33 );
            break;

        case WINDOW_FUNCTION_TYPE_BARTLETT_HANN:
            description = formatDescription( -35.97, 12 );
            break;

        case WINDOW_FUNCTION_TYPE_BLACKMAN:
            description = formatDescription( -58.11, 18, 1.68, 2.35, 1.73 );
            break;

        case WINDOW_FUNCTION_TYPE_BLACKMAN_EXACT:
            description = formatDescription( -68.24, 6, 1.52, 2.13, 1.57 );
            break;

        case WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M3:
            description = formatDescription( -70.83, 6, 1.66, 2.27, 1.71 );
            break;

        case WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_3:
            description = formatDescription( -62.05, 6, 1.56, 2.19, 1.61 );
            break;

        case WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M4:
            description = formatDescription( -92.01, 6, 1.90, 2.72, 2.00 );
            break;

        case WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_4:
            description = formatDescription( -74.39, 6, 1.74, 2.44, 1.79 );
            break;

        case WINDOW_FUNCTION_TYPE_BLACKMAN_NUTTALL:
            description = formatDescription( -98.0, 6 );
            break;

        case WINDOW_FUNCTION_TYPE_BOHMAN:
            description = formatDescription( -46.0, 24, 1.71, 2.38, 1.79 );
            break;

        case WINDOW_FUNCTION_TYPE_CAUCHY:
            hslVec = { 35, -35.5, 4.5 };
            bw3dbVec = { 0.827, 0.17 };
            bw6dbVec = { 0.95, 0.315 };
            enbwVec = { 0.607, 0.29 };
            description = formatDescription( hslVec, 6, bw3dbVec, bw6dbVec, enbwVec );
            break;

        case WINDOW_FUNCTION_TYPE_DE_LA_VALLEE_POUSSIN:
            description = formatDescription( -53.04, 24, 1.82, 2.55, 1.92 );
            break;

        case WINDOW_FUNCTION_TYPE_FLAT_TOP_5:
            description = formatDescription( -86.52, 6, 3.72, 4.58, 3.77 );
            break;

        case WINDOW_FUNCTION_TYPE_GAUSSIAN:
            hslVec = { 25.67, -27 };
            bw3dbVec = { 0.117, 0.46 };
            bw6dbVec = { 0.207, 0.66 };
            enbwVec = { 0.113, 0.51 };
            description = formatDescription( hslVec, 6, bw3dbVec, bw6dbVec, enbwVec );
            break;

        case WINDOW_FUNCTION_TYPE_HAMMING:
            description = formatDescription( -43.19, 6, 1.30, 1.81, 1.36 );
            break;

        case WINDOW_FUNCTION_TYPE_HANN:
            description = formatDescription( -31.47, 18, 1.44, 2.0, 1.50 );
            break;

        case WINDOW_FUNCTION_TYPE_HANN_POISSON:
            hslVec = { -31, -8, 0 };
            bw3dbVec = { 1.425, 0.221 };
            bw6dbVec = { 1.965, 0.341 };
            enbwVec = { 1.465, 0.276 };
            description = formatDescription( hslVec, 18, bw3dbVec, bw6dbVec, enbwVec );
            break;

        case WINDOW_FUNCTION_TYPE_KAISER_BESSEL:
            hslVec = { -13.1, -16.8, 0 };
            bw3dbVec = { 1.071, 0.188 };
            bw6dbVec = { 1.478, 0.27 };
            enbwVec = { 1.114, 0.202 };
            description = formatDescription( hslVec, 6, bw3dbVec, bw6dbVec, enbwVec );
            break;

        case WINDOW_FUNCTION_TYPE_NUTTALL:
            description = formatDescription( -93.0, 18, 0, 0, 1.98 );
            break;

        case WINDOW_FUNCTION_TYPE_POISSON:
            hslVec = { -15, 0, -1 };
            bw3dbVec = { 0.66, 0.27 };
            bw6dbVec = { 0.782, 0.445 };
            enbwVec = { 0.507, 0.39 };
            description = formatDescription( hslVec, 6, bw3dbVec, bw6dbVec, enbwVec );
            break;

        case WINDOW_FUNCTION_TYPE_RECTANGULAR:
            description = formatDescription( -13.25, 6, 0.89, 1.21, 1.00 );
            break;

        case WINDOW_FUNCTION_TYPE_RIEMANN:
            description = formatDescription( -26.37, 12, 1.26, 1.74, 1.30 );
            break;

        case WINDOW_FUNCTION_TYPE_RIESZ:
            description = formatDescription( -21.26, 12, 1.16, 1.59, 1.20 );
            break;

        case WINDOW_FUNCTION_TYPE_TUKEY:
            hslVec = { -16, 14, -24 };
            bw3dbVec = { 0.857, 0.6 };
            bw6dbVec = { 1.163, 0.84 };
            enbwVec = { 0.967, 0.52 };
            description = formatDescription( hslVec, 18, bw3dbVec, bw6dbVec, enbwVec );
            break;

        default:
            break;
    }

    return description;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValue
    (
    const uint32_t aIndex,      //!< index
    double*        aValue       //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        *aValue = 0;

        switch( mActiveType )
        {
            case WINDOW_FUNCTION_TYPE_BARTLETT:
                status = getWindowValueBartlett( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BARTLETT_HANN:
                status = getWindowValueBartlettHann( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BLACKMAN:
                status = getWindowValueBlackman( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BLACKMAN_EXACT:
                status = getWindowValueBlackmanExact( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M3:
                status = getWindowValueBlackmanHarrisMin3( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_3:
                status = getWindowValueBlackmanHarris3( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_M4:
                status = getWindowValueBlackmanHarrisMin4( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BLACKMAN_HARRIS_4:
                status = getWindowValueBlackmanHarris4( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BLACKMAN_NUTTALL:
                status = getWindowValueBlackmanNuttall( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_BOHMAN:
                status = getWindowValueBohman( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_CAUCHY:
                status = getWindowValueCauchy( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_DE_LA_VALLEE_POUSSIN:
                status = getWindowValueDeLaValleePoussin( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_FLAT_TOP_5:
                status = getWindowValueFlatTop5( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_GAUSSIAN:
                status = getWindowValueGaussian( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_HAMMING:
                status = getWindowValueHamming( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_HANN:
                status = getWindowValueHann( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_HANN_POISSON:
                status = getWindowValueHannPoisson( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_KAISER_BESSEL:
                status = getWindowValueKaiserBessel( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_NUTTALL:
                status = getWindowValueNuttall( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_PLANCK_TAPER:
                status = getWindowValuePlanckTaper( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_POISSON:
                status = getWindowValuePoisson( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_RECTANGULAR:
                status = getWindowValueRectangular( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_RIEMANN:
                status = getWindowValueRiemann( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_RIESZ:
                status = getWindowValueRiesz( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_SINE:
                status = getWindowValueSine( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_TUKEY:
                status = getWindowValueTukey( aIndex, aValue );
                break;

            case WINDOW_FUNCTION_TYPE_WELCH:
                status = getWindowValueWelch( aIndex, aValue );
                break;

            default:
                break;
        }
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Bartlett ***
//! See (23) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBartlett
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        *aValue = 1.0 - fabs( ( 2.0 * static_cast<double>( aIndex ) / mFftSize - 1.0 ) );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Bartlett-Hann ***
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBartlettHann
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.62;
        const double A1 = 0.48;
        const double A2 = 0.38;
        double q = static_cast<double>( aIndex ) / mFftSize - 0.5;
        *aValue = A0 - A1 * fabs( q ) + A2 * cos( Numeric::TWO_PI * q );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Blackman ***
//! See (19) in [4]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBlackman
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.42;
        const double A1 = 0.50;
        const double A2 = 0.08;
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Blackman exact ***
//! See (20) in [4]
//! See (32) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBlackmanExact
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double DENOM = 18608.0;
        const double A0 = 7938.0 / DENOM;   // =0.426590713671539
        const double A1 = 9240.0 / DENOM;   // =0.496560619088564
        const double A2 = 1430.0 / DENOM;   // =0.076848667239897
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Blackman-Harris "minimum 3-term" ***
//! See (21) in [4]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBlackmanHarrisMin3
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.42323;
        const double A1 = 0.49755;
        const double A2 = 0.07922;
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Blackman-Harris "3-term" ***
//! See (22) in [4]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBlackmanHarris3
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.44959;
        const double A1 = 0.49364;
        const double A2 = 0.05677;
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Blackman-Harris "minimum 4-term" ***
//! See (23) in [4]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBlackmanHarrisMin4
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.35875;
        const double A1 = 0.48829;
        const double A2 = 0.14128;
        const double A3 = 0.01168;
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q ) - A3 * cos( 3.0 * q );
    }

    return status;
}

//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Blackman-Harris "4-term" ***
//! See (24) in [4]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBlackmanHarris4
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.40217;
        const double A1 = 0.49703;
        const double A2 = 0.09892;
        const double A3 = 0.00188;
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q ) - A3 * cos( 3.0 * q );
    }

    return status;
}

//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Blackmann-Nuttall ***
//! See (37) in [4].
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBlackmanNuttall
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.3635819;
        const double A1 = 0.4891775;
        const double A2 = 0.1365995;
        const double A3 = 0.0106411;
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q ) - A3 * cos( 3.0 * q );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Bohman ***
//! See (39) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueBohman
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        int32_t n = aIndex - mFftSize / 2;
        double q = 2.0 * fabs( static_cast<double>( n ) ) / mFftSize;
        *aValue = ( 1.0 - q ) * cos( Numeric::PI * q ) + sin( Numeric::PI * q ) / Numeric::PI;
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Cauchy ***
//! See (42) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueCauchy
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double ALPHA = mParametersMap.at( WINDOW_FUNCTION_TYPE_CAUCHY ).crtValue;
        int32_t n = aIndex - mFftSize / 2;
        *aValue = 1.0 / ( 1.0 + pow( 2.0 * ALPHA * static_cast<double>( n ) / mFftSize, 2.0 ) );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** de la Vallee-Poussin (Parzen) ***
//! See (37) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueDeLaValleePoussin
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        uint32_t L = mFftSize + 1;
        int32_t n = aIndex - mFftSize / 2;
        double q = 2.0 * fabs( static_cast<double>( n ) ) / L;

        if( fabs( static_cast<double>( n ) ) <= L / 4.0 )
        {
            *aValue = 1.0 - 6.0 * pow( q, 2.0 ) * ( 1.0 - q );
        }
        else
        {
            *aValue = 2.0 * pow( 1.0 - q, 3.0 );
        }
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Flat top - 5th order ***
//! See (5) in [1]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueFlatTop5
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.21557895;
        const double A1 = 0.41663158;
        const double A2 = 0.277263158;
        const double A3 = 0.083578947;
        const double A4 = 0.006947368;
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q ) - A3 * cos( 3.0 * q ) + A4 * cos( 4.0 * q );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Gaussian (Weierstrass) ***
//! See (44) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueGaussian
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double ALPHA = mParametersMap.at( WINDOW_FUNCTION_TYPE_GAUSSIAN ).crtValue;
        *aValue = exp( -0.5 * pow( ( 2.0 * static_cast<double>( aIndex ) - mFftSize ) / ( mFftSize / ALPHA ), 2.0 ) );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Hamming ***
//! See (30) in [3]
//! See (E.4) in [2]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueHamming
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 25.0 / 46.0;
        *aValue = A0 - ( 1.0 - A0 ) * cos( Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Hann ***
//! See (27) in [3]
//! See (3) in [1]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueHann
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        *aValue = 0.5 * ( 1.0 - cos( Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize ) );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Hann-Poisson ***
//! See (41) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueHannPoisson
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double ALPHA = mParametersMap.at( WINDOW_FUNCTION_TYPE_HANN_POISSON ).crtValue;
        int32_t n = aIndex - mFftSize / 2;
        *aValue = 0.5 * ( 1.0 + cos( Numeric::TWO_PI * static_cast<double>( n ) / mFftSize ) ) * exp( -2.0 * ALPHA * fabs( static_cast<double>( n ) ) / mFftSize );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Kaiser-Bessel ***
//! See (46) in [3]
//! See (4) in [1]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueKaiserBessel
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double ALPHA = mParametersMap.at( WINDOW_FUNCTION_TYPE_KAISER_BESSEL ).crtValue;
        const double PIAL = Numeric::PI * ALPHA;
        *aValue = std::cyl_bessel_i( 0, PIAL * sqrt( 1.0 - pow( 2.0 * static_cast<double>( aIndex ) / mFftSize - 1.0, 2.0 ) ) );
        *aValue /= std::cyl_bessel_i( 0, PIAL );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Nuttall ***
//! See (34) in [4].
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueNuttall
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double A0 = 0.355768;
        const double A1 = 0.487396;
        const double A2 = 0.144232;
        const double A3 = 0.012604;
        double q = Numeric::TWO_PI * static_cast<double>( aIndex ) / mFftSize;
        *aValue = A0 - A1 * cos( q ) + A2 * cos( 2.0 * q ) - A3 * cos( 3.0 * q );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Planck taper ***
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValuePlanckTaper
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double EPSILON = mParametersMap.at( WINDOW_FUNCTION_TYPE_PLANCK_TAPER ).crtValue;
        const double EPS_N = EPSILON * mFftSize;
        int32_t n = 0;

        if( aIndex <= mFftSize / 2 )
        {
            n = aIndex;

            if( 0 == aIndex )
            {
                *aValue = 0;
            }
            else if( aIndex < EPS_N )
            {
                *aValue = 1.0 / ( 1.0 + exp( EPS_N * ( EPS_N - 2.0 * n ) / ( EPS_N - n ) / n ) );
            }
            else
            {
                *aValue = 1;
            }
        }
        else
        {
            n = mFftSize - aIndex;

            if( aIndex < mFftSize - EPS_N )
            {
                *aValue = 1;
            }
            else if( aIndex < mFftSize )
            {
                *aValue = 1.0 / ( 1.0 + exp( EPS_N * ( EPS_N - 2.0 * n ) / ( EPS_N - n ) / n ) );
            }
            else
            {
                *aValue = 0;
            }
        }
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Poisson (exponential) ***
//! See (40) in [3]
//! See (8) in [1]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValuePoisson
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double ALPHA = mParametersMap.at( WINDOW_FUNCTION_TYPE_POISSON ).crtValue;
        int32_t n = aIndex - mFftSize / 2;
        *aValue = exp( -2.0 * ALPHA * fabs( static_cast<double>( n ) ) / mFftSize );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** rectangular (boxcar, uniform, Dirichlet) ***
//! See (21) in [3]
//! See (2) in [1]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueRectangular
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        *aValue = 1.0;
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Riemann ***
//! See (36) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueRiemann
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        int32_t n = aIndex - mFftSize / 2;
        double q = Numeric::TWO_PI * static_cast<double>( n ) / mFftSize;
        *aValue = sin( q ) / q;
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Riesz ***
//! See (35) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueRiesz
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        int32_t n = aIndex - mFftSize / 2;
        *aValue = 1.0 - pow( fabs( 2.0 * static_cast<double>( n ) / mFftSize ), 2.0 );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** sine ***
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueSine
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        *aValue = sin( Numeric::PI * static_cast<double>( aIndex ) / mFftSize );
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Tukey (tapered cosine) ***
//! See (38) in [3]
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueTukey
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        const double ALPHA = mParametersMap.at( WINDOW_FUNCTION_TYPE_TUKEY ).crtValue;
        const double ALPHA_N = ALPHA * mFftSize / 2.0;
        int32_t n = 0;

        if( aIndex <= mFftSize / 2 )
        {
            n = aIndex;

            if( aIndex < ALPHA_N )
            {
                *aValue = 0.5 * ( 1.0 - cos( Numeric::PI * n / ALPHA_N ) );
            }
            else
            {
                *aValue = 1;
            }
        }
        else
        {
            n = mFftSize - aIndex;

            if( aIndex < mFftSize - ALPHA_N )
            {
                *aValue = 1;
            }
            else
            {
                *aValue = 0.5 * ( 1.0 - cos( Numeric::PI * n / ALPHA_N ) );
            }
        }
    }

    return status;
}


//!************************************************************************
//! Get the value of the window function for a specified index
//! *** Welch ***
//!
//! @returns The function value
//!************************************************************************
bool WindowFunction::getWindowValueWelch
    (
    const uint32_t aIndex,              //!< index
    double*        aValue               //!< value
    ) const
{
    bool status = aValue != nullptr && aIndex <= mFftSize;

    if( status )
    {
        *aValue = 1.0 - pow( 2.0 * static_cast<double>( aIndex ) / mFftSize - 1.0, 2.0 );
    }

    return status;
}


//!************************************************************************
//! Set the active type
//!
//! @returns true if the type could be set
//!************************************************************************
bool WindowFunction::setActiveType
    (
    const WindowFunctionType aType         //!< type
    )
{
    bool status = aType >= WINDOW_FUNCTION_TYPE_BARTLETT
               && aType <= WINDOW_FUNCTION_TYPE_WELCH;

    if( status )
    {
        mActiveType = aType;
    }

    return status;
}


//!************************************************************************
//! Set the FFT size
//!
//! @returns true if the value could be set
//!************************************************************************
bool WindowFunction::setFftSize
    (
    const uint32_t  aSize               //!< FFT size
    )
{
    bool status = aSize > 0;

    if( status )
    {
        mFftSize = aSize;
    }

    return status;
}


//!************************************************************************
//! Set a new value for the window function parameter.
//! Not all window functions have a parameter defined.
//!
//! @returns true if the value could be set
//!************************************************************************
bool WindowFunction::setParameter
    (
    const WindowFunctionType aWindowType,   //!< window type
    const double             aValue         //!< parameter value
    )
{
    bool status = mParametersMap.at( aWindowType ).needsParameter;

    if( status )
    {
        if( mParametersMap.at( aWindowType ).minValue > aValue
         || mParametersMap.at( aWindowType ).maxValue < aValue )
        {
            status = false;
        }
    }

    if( status )
    {
        mParametersMap.at( aWindowType ).crtValue = aValue;
    }

    return status;
}
