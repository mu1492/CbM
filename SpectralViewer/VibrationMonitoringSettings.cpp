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
VibrationMonitoringSettings.cpp

This file contains the sources for the monitoring settings of mechanical vibrations.
*/

#include "VibrationMonitoringSettings.h"


//!************************************************************************
//! Constructor
//!************************************************************************
VibrationMonitoringSettings::VibrationMonitoringSettings()
    : mActiveOption( OPTION_ITEM_DEFAULT )
{    
    memset( &mIso20816Part1NonRotatingTransition, 0, sizeof( mIso20816Part1NonRotatingTransition) );

    mIso20816Part1NonRotatingTransition.vFromAtoB = 0.5 * ( ISO_20816_1_TRANSITION_AB_MIN + ISO_20816_1_TRANSITION_AB_MAX );
    mIso20816Part1NonRotatingTransition.vFromBtoC = 0.5 * ( ISO_20816_1_TRANSITION_BC_MIN + ISO_20816_1_TRANSITION_BC_MAX );
    mIso20816Part1NonRotatingTransition.vFromCtoD = 0.5 * ( ISO_20816_1_TRANSITION_CD_MIN + ISO_20816_1_TRANSITION_CD_MAX );

    updateActiveTransitions();

    mOptionsMap =
    {
        { OPTION_ITEM_ISO_10816_1_CLASS_1,          "ISO 10816-1 Class I" },
        { OPTION_ITEM_ISO_10816_1_CLASS_2,          "ISO 10816-1 Class II" },
        { OPTION_ITEM_ISO_10816_1_CLASS_3,          "ISO 10816-1 Class III" },
        { OPTION_ITEM_ISO_10816_1_CLASS_4,          "ISO 10816-1 Class IV" },

        { OPTION_ITEM_ISO_20816_1,                  "ISO 20816-1" },

        { OPTION_ITEM_ISO_20816_3_GROUP_1_RIGID,    "ISO 20816-3 Group I rigid mount" },
        { OPTION_ITEM_ISO_20816_3_GROUP_1_FLEXIBLE, "ISO 20816-3 Group I flexible mount" },
        { OPTION_ITEM_ISO_20816_3_GROUP_2_RIGID,    "ISO 20816-3 Group II rigid mount" },
        { OPTION_ITEM_ISO_20816_3_GROUP_2_FLEXIBLE, "ISO 20816-3 Group II flexible mount" },

        { OPTION_ITEM_ISO_20816_21_MAIN_BEARING,    "ISO 20816-21 Main bearing" },
        { OPTION_ITEM_ISO_20816_21_GEARBOX,         "ISO 20816-21 Gearbox" },
        { OPTION_ITEM_ISO_20816_21_GENERATOR,       "ISO 20816-21 Generator" },
        { OPTION_ITEM_ISO_20816_21_TOWER,           "ISO 20816-21 Tower" }
    };
}


//!************************************************************************
//! Destructor
//!************************************************************************
VibrationMonitoringSettings::~VibrationMonitoringSettings()
{
}


//!************************************************************************
//! Get the active option
//!
//! @returns The active option
//!************************************************************************
VibrationMonitoringSettings::OptionItem VibrationMonitoringSettings::getActiveOption()
{
    return mActiveOption;
}


//!************************************************************************
//! Get the active transitions
//!
//! @returns The active transition zones
//!************************************************************************
VibrationMonitoringSettings::TransitionZone VibrationMonitoringSettings::getActiveTransitions()
{
    return mActiveTransitions;
}


//!************************************************************************
//! Get the description of the specified option item
//!
//! @returns The description string
//!************************************************************************
std::string VibrationMonitoringSettings::getOptionDescription
    (
    const OptionItem aOptionItem        //!< option
    ) const
{
    std::string description;

    switch( aOptionItem )
    {
        case OPTION_ITEM_ISO_10816_1_CLASS_1:
            description = "Class I - Small machines\
                           \n\t- electrical motors P < 15kW";
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_2:
            description = "Class II - Medium-sized machines\
                           \n\t-electrical motors with no special foundation P <= 75kW\
                           \n\t-rigidly mounted machines P <= 300kW";
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_3:
            description = "Class III - Large machines\
                           \n\t- large rotating assemblies mounted on rigid foundation P > 300kW";
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_4:
            description = "Class IV - Large machines\
                           \n\t- large rotating assemblies mounted on soft foundation\
                           \n\t- gas turbines P > 10MW";
            break;

        case OPTION_ITEM_ISO_20816_1:
            description = "Non-rotating parts for small and large machines";
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_1_RIGID:
            description = "Group I - Large machines with power (0.3..40] MW\
                           \n\tOR\
                           \n\t-Electrical machines with shaft height larger than 315mm\
                           \nRigid mount";
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_1_FLEXIBLE:
            description = "Group I - Large machines with power (0.3..40] MW\
                           \n\tOR\
                           \n\t-Electrical machines with shaft height larger than 315mm\
                           \nFlexible/elastic mount";
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_2_RIGID:
            description = "Group II - Medium-sized machines with power (15..300] kW\
                           \n\tOR\
                           \n\t-Electrical machines with shaft height [160..315] mm\
                           \nRigid mount";
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_2_FLEXIBLE:
            description = "Group II - Medium-sized machines with power (15..300] kW\
                           \n\tOR\
                           \n\t-Electrical machines with shaft height [160..315] mm\
                           \nFlexible/elastic mount";
            break;

        case OPTION_ITEM_ISO_20816_21_MAIN_BEARING:
            description = "Horizontal axis wind turbines with gearbox\
                           \n\t-Main bearing";
            break;

        case OPTION_ITEM_ISO_20816_21_GEARBOX:
            description = "Horizontal axis wind turbines with gearbox\
                           \n\t-Gearbox";
            break;

        case OPTION_ITEM_ISO_20816_21_GENERATOR:
            description = "Horizontal axis wind turbines with gearbox\
                           \n\t-Generator";
            break;

        case OPTION_ITEM_ISO_20816_21_TOWER:
            description = "Horizontal axis wind turbines with gearbox\
                           \n\t-Tower";
            break;

        default:
            break;
    }

    return description;
}


//!************************************************************************
//! Get the map with available options
//!
//! @returns The options map
//!************************************************************************
std::map<VibrationMonitoringSettings::OptionItem, std::string> VibrationMonitoringSettings::getOptionsMap() const
{
    return mOptionsMap;
}


//!************************************************************************
//! Get the current transitions corresponding to ISO 20816-1 for non-rotating
//! parts
//!
//! @returns The transition zone definitions
//!************************************************************************
VibrationMonitoringSettings::TransitionZone VibrationMonitoringSettings::getTransitionIso20816Part1NonRotating() const
{
    return mIso20816Part1NonRotatingTransition;
}


//!************************************************************************
//! Get the evaluation zone corresponding to the currently selected option
//!
//! @returns The vibration evaluation zone - A, B, C, or D
//!************************************************************************
VibrationMonitoringSettings::EvaluationZone VibrationMonitoringSettings::getZone
    (
    const double aRmsSpeed      //!< RMS speed [m/s]
    ) const
{
    EvaluationZone zone = EVALUATION_ZONE_D;

    switch( mActiveOption )
    {
        case OPTION_ITEM_ISO_10816_1_CLASS_1:
            zone = getZoneIso10816Part1( ISO_10816_1_CLASS_1, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_2:
            zone = getZoneIso10816Part1( ISO_10816_1_CLASS_2, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_3:
            zone = getZoneIso10816Part1( ISO_10816_1_CLASS_3, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_4:
            zone = getZoneIso10816Part1( ISO_10816_1_CLASS_4, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_1:
            zone = getZoneIso20816Part1( aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_1_RIGID:
            zone = getZoneIso20816Part3( ISO_20816_3_GROUP_1_RIGID, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_1_FLEXIBLE:
            zone = getZoneIso20816Part3( ISO_20816_3_GROUP_1_FLEXIBLE, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_2_RIGID:
            zone = getZoneIso20816Part3( ISO_20816_3_GROUP_2_RIGID, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_2_FLEXIBLE:
            zone = getZoneIso20816Part3( ISO_20816_3_GROUP_2_FLEXIBLE, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_21_MAIN_BEARING:
            zone = getZoneIso10816Part21( ISO_20816_21_MAIN_BEARING, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_21_GEARBOX:
            zone = getZoneIso10816Part21( ISO_20816_21_GEARBOX, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_21_GENERATOR:
            zone = getZoneIso10816Part21( ISO_20816_21_GENERATOR, aRmsSpeed );
            break;

        case OPTION_ITEM_ISO_20816_21_TOWER:
            zone = getZoneIso10816Part21( ISO_20816_21_TOWER, aRmsSpeed );
            break;

        default:
            break;
    }

    return zone;
}


//!************************************************************************
//! Get the evaluation zone corresponding to ISO 10816-1 for a RMS speed
//!
//! @returns The vibration evaluation zone - A, B, C, or D
//!************************************************************************
VibrationMonitoringSettings::EvaluationZone VibrationMonitoringSettings::getZoneIso10816Part1
    (
    const Iso10816Part1Class aMachineClass,     //!< machine class
    const double             aRmsSpeed          //!< RMS speed [m/s]
    ) const
{
    EvaluationZone zone = EVALUATION_ZONE_D;

    switch( aMachineClass )
    {
        case ISO_10816_1_CLASS_1:
            if( aRmsSpeed < ISO_10816_1_CLASS_1_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_10816_1_CLASS_1_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_10816_1_CLASS_1_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_10816_1_CLASS_2:
            if( aRmsSpeed < ISO_10816_1_CLASS_2_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_10816_1_CLASS_2_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_10816_1_CLASS_2_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_10816_1_CLASS_3:
            if( aRmsSpeed < ISO_10816_1_CLASS_3_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_10816_1_CLASS_3_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_10816_1_CLASS_3_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_10816_1_CLASS_4:
            if( aRmsSpeed < ISO_10816_1_CLASS_4_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_10816_1_CLASS_4_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_10816_1_CLASS_4_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        default:
            break;
    }

    return zone;
}


//!************************************************************************
//! Get the evaluation zone corresponding to ISO 20816-1 for a RMS speed
//!
//! @returns The vibration evaluation zone - A, B, C, or D
//!************************************************************************
VibrationMonitoringSettings::EvaluationZone VibrationMonitoringSettings::getZoneIso20816Part1
    (
    const double aRmsSpeed      //!< RMS speed [m/s]
    ) const
{
    EvaluationZone zone = EVALUATION_ZONE_D;

    if( aRmsSpeed < mIso20816Part1NonRotatingTransition.vFromAtoB )
    {
        zone = EVALUATION_ZONE_A;
    }
    else if( aRmsSpeed < mIso20816Part1NonRotatingTransition.vFromBtoC )
    {
        zone = EVALUATION_ZONE_B;
    }
    else if( aRmsSpeed < mIso20816Part1NonRotatingTransition.vFromCtoD )
    {
        zone = EVALUATION_ZONE_C;
    }

    return zone;
}


//!************************************************************************
//! Get the evaluation zone corresponding to ISO 20816-3 for a RMS speed
//!
//! @returns The vibration evaluation zone - A, B, C, or D
//!************************************************************************
VibrationMonitoringSettings::EvaluationZone VibrationMonitoringSettings::getZoneIso20816Part3
    (
    const Iso20816Part3Group aMachineGroup,     //!< machine group
    const double             aRmsSpeed          //!< RMS speed [m/s]
    ) const
{
    EvaluationZone zone = EVALUATION_ZONE_D;

    switch( aMachineGroup )
    {
        case ISO_20816_3_GROUP_1_RIGID:
            if( aRmsSpeed < ISO_20816_3_GROUP_1_RIGID_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_20816_3_GROUP_1_RIGID_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_20816_3_GROUP_1_RIGID_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_20816_3_GROUP_1_FLEXIBLE:
            if( aRmsSpeed < ISO_20816_3_GROUP_1_FLEXIBLE_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_20816_3_GROUP_1_FLEXIBLE_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_20816_3_GROUP_1_FLEXIBLE_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_20816_3_GROUP_2_RIGID:
            if( aRmsSpeed < ISO_20816_3_GROUP_2_RIGID_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_20816_3_GROUP_2_RIGID_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_20816_3_GROUP_2_RIGID_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_20816_3_GROUP_2_FLEXIBLE:
            if( aRmsSpeed < ISO_20816_3_GROUP_2_FLEXIBLE_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_20816_3_GROUP_2_FLEXIBLE_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_20816_3_GROUP_2_FLEXIBLE_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        default:
            break;
    }

    return zone;
}


//!************************************************************************
//! Get the evaluation zone corresponding to ISO 20816-21 for a RMS speed
//!
//! @returns The vibration evaluation zone - A, B, C, or D
//!************************************************************************
VibrationMonitoringSettings::EvaluationZone VibrationMonitoringSettings::getZoneIso10816Part21
    (
    const Iso20816Part21Component aComponent,       //!< component
    const double                  aRmsSpeed         //!< RMS speed [m/s]
    ) const
{
    EvaluationZone zone = EVALUATION_ZONE_D;

    switch( aComponent )
    {
        case ISO_20816_21_MAIN_BEARING:
            if( aRmsSpeed < ISO_20816_21_MAIN_BEARING_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_20816_21_MAIN_BEARING_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_20816_21_MAIN_BEARING_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_20816_21_GEARBOX:
            if( aRmsSpeed < ISO_20816_21_GEARBOX_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_20816_21_GEARBOX_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_20816_21_GEARBOX_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_20816_21_GENERATOR:
            if( aRmsSpeed < ISO_20816_21_GENERATOR_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_20816_21_GENERATOR_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_20816_21_GENERATOR_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        case ISO_20816_21_TOWER:
            if( aRmsSpeed < ISO_20816_21_TOWER_TRANSITION.vFromAtoB )
            {
                zone = EVALUATION_ZONE_A;
            }
            else if( aRmsSpeed < ISO_20816_21_TOWER_TRANSITION.vFromBtoC )
            {
                zone = EVALUATION_ZONE_B;
            }
            else if( aRmsSpeed < ISO_20816_21_TOWER_TRANSITION.vFromCtoD )
            {
                zone = EVALUATION_ZONE_C;
            }
            break;

        default:
            break;
    }

    return zone;
}


//!************************************************************************
//! Set the active option
//!
//! @returns true if the option could be set
//!************************************************************************
bool VibrationMonitoringSettings::setActiveOption
    (
    const OptionItem aOptionItem         //!< option
    )
{
    bool status = aOptionItem >= OPTION_ITEM_ISO_10816_1_CLASS_1
               && aOptionItem <= OPTION_ITEM_ISO_20816_21_TOWER;

    if( status )
    {
        mActiveOption = aOptionItem;
        updateActiveTransitions();
    }

    return status;
}


//!************************************************************************
//! Set A<->B zones transition RMS speed corresponding to ISO 20816-1 for
//! non-rotating parts
//!
//! @returns true if the RMS speed could be set
//!************************************************************************
bool VibrationMonitoringSettings::setIso20816Part1NonRotatingTransitionAB
    (
    const double aRmsSpeed      //!< RMS speed [m/s]
    )
{
    bool status = aRmsSpeed >= ISO_20816_1_TRANSITION_AB_MIN
               && aRmsSpeed <= ISO_20816_1_TRANSITION_AB_MAX
               && aRmsSpeed <= mIso20816Part1NonRotatingTransition.vFromBtoC;

    if( status )
    {
        mIso20816Part1NonRotatingTransition.vFromAtoB = aRmsSpeed;

        if( OPTION_ITEM_ISO_20816_1 == mActiveOption )
        {
            mActiveTransitions.vFromAtoB = mIso20816Part1NonRotatingTransition.vFromAtoB;
        }
    }

    return status;
}


//!************************************************************************
//! Set B<->C zones transition RMS speed corresponding to ISO 20816-1 for
//! non-rotating parts
//!
//! @returns true if the RMS speed could be set
//!************************************************************************
bool VibrationMonitoringSettings::setIso20816Part1NonRotatingTransitionBC
    (
    const double aRmsSpeed      //!< RMS speed [m/s]
    )
{
    bool status = aRmsSpeed >= ISO_20816_1_TRANSITION_BC_MIN
               && aRmsSpeed <= ISO_20816_1_TRANSITION_BC_MAX
               && aRmsSpeed >= mIso20816Part1NonRotatingTransition.vFromAtoB
               && aRmsSpeed <= mIso20816Part1NonRotatingTransition.vFromCtoD;

    if( status )
    {
        mIso20816Part1NonRotatingTransition.vFromBtoC = aRmsSpeed;

        if( OPTION_ITEM_ISO_20816_1 == mActiveOption )
        {
            mActiveTransitions.vFromBtoC = mIso20816Part1NonRotatingTransition.vFromBtoC;
        }
    }

    return status;
}


//!************************************************************************
//! Set C<->D zones transition RMS speed corresponding to ISO 20816-1 for
//! non-rotating parts
//!
//! @returns true if the RMS speed could be set
//!************************************************************************
bool VibrationMonitoringSettings::setIso20816Part1NonRotatingTransitionCD
    (
    const double aRmsSpeed      //!< RMS speed [m/s]
    )
{
    bool status = aRmsSpeed >= ISO_20816_1_TRANSITION_CD_MIN
               && aRmsSpeed <= ISO_20816_1_TRANSITION_CD_MAX
               && aRmsSpeed >= mIso20816Part1NonRotatingTransition.vFromBtoC;

    if( status )
    {
        mIso20816Part1NonRotatingTransition.vFromCtoD = aRmsSpeed;        

        if( OPTION_ITEM_ISO_20816_1 == mActiveOption )
        {
            mActiveTransitions.vFromCtoD = mIso20816Part1NonRotatingTransition.vFromCtoD;
        }
    }

    return status;
}


//!************************************************************************
//! Assignment operator for an VibrationMonitoring object
//!
//! @returns the VibrationMonitoring object
//!************************************************************************
VibrationMonitoringSettings& VibrationMonitoringSettings::operator =
    (
    VibrationMonitoringSettings const& aObject      //!< object
    )
{
    memcpy( &this->mIso20816Part1NonRotatingTransition, &aObject.mIso20816Part1NonRotatingTransition, sizeof( TransitionZone ) );
    this->mOptionsMap = aObject.mOptionsMap;
    this->mActiveOption = aObject.mActiveOption;
    this->mActiveTransitions = aObject.mActiveTransitions;
    return *this;
}


//!************************************************************************
//! Update the values for active transitions
//!
//! @returns nothing
//!************************************************************************
void VibrationMonitoringSettings::updateActiveTransitions()
{
    switch( mActiveOption )
    {
        case OPTION_ITEM_ISO_10816_1_CLASS_1:
            memcpy( &mActiveTransitions, &ISO_10816_1_CLASS_1_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_2:
            memcpy( &mActiveTransitions, &ISO_10816_1_CLASS_2_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_3:
            memcpy( &mActiveTransitions, &ISO_10816_1_CLASS_3_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_10816_1_CLASS_4:
            memcpy( &mActiveTransitions, &ISO_10816_1_CLASS_4_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_1:
            memcpy( &mActiveTransitions, &mIso20816Part1NonRotatingTransition, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_1_RIGID:
            memcpy( &mActiveTransitions, &ISO_20816_3_GROUP_1_RIGID_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_1_FLEXIBLE:
            memcpy( &mActiveTransitions, &ISO_20816_3_GROUP_1_FLEXIBLE_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_2_RIGID:
            memcpy( &mActiveTransitions, &ISO_20816_3_GROUP_2_RIGID_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_3_GROUP_2_FLEXIBLE:
            memcpy( &mActiveTransitions, &ISO_20816_3_GROUP_2_FLEXIBLE_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_21_MAIN_BEARING:
            memcpy( &mActiveTransitions, &ISO_20816_21_MAIN_BEARING_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_21_GEARBOX:
            memcpy( &mActiveTransitions, &ISO_20816_21_GEARBOX_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_21_GENERATOR:
            memcpy( &mActiveTransitions, &ISO_20816_21_GENERATOR_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        case OPTION_ITEM_ISO_20816_21_TOWER:
            memcpy( &mActiveTransitions, &ISO_20816_21_TOWER_TRANSITION, sizeof( mActiveTransitions ) );
            break;

        default:
            break;
    }
}
