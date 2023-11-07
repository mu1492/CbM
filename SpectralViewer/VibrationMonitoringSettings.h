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
VibrationMonitoringSettings.h

This file contains the definitions for the monitoring settings of mechanical vibrations.
*/

#ifndef VibrationMonitoringSettings_h
#define VibrationMonitoringSettings_h

#include <cstdint>
#include <cstring>
#include <map>
#include <string>


//************************************************************************
// Class for vibrations monitoring settings
//************************************************************************
class VibrationMonitoringSettings
{
    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        //************************************************************************
        // Evaluation zones, as defined by ISO 10816 and 20816
        //************************************************************************
        typedef enum : uint8_t
        {
            EVALUATION_ZONE_A,              //!< newly commissioned machines (ideal installations, no faults)
            EVALUATION_ZONE_B,              //!< acceptable for unrestricted long-term operation
            EVALUATION_ZONE_C,              //!< unsatisfactory for long-term operation, only short-time allowed
            EVALUATION_ZONE_D               //!< severity that causes damage to the machine
        }EvaluationZone;

        typedef struct
        {
            double vFromAtoB;       //!< RMS speed for transitioning Zone A <-> B [m/s]
            double vFromBtoC;       //!< RMS speed for transitioning Zone B <-> C [m/s]
            double vFromCtoD;       //!< RMS speed for transitioning Zone C <-> D [m/s]
        }TransitionZone;

        //************************************************************************
        // Selected list of ISO standards on measurement and evaluation of machine vibration
        //
        // ISO 10816-1 - General guidelines (1995)
        // ISO 20816-1 - General guidelines (2016)
        // ISO 20816-3 - Industrial machinery with P>15 kW and w in 120..30000 rpm (2022)
        // ISO 20816-21 - Horizontal axis wind turbines with gearbox (2015)
        //************************************************************************

        //************************************************************************
        // ISO 10816-1:1995
        // Mechanical vibration - Evaluation of machine vibration by measurements on non-rotating parts
        // Part 1: General guidelines
        //
        // Standard has been revised by ISO 201816-1:2016, but is still widely used in commercial applications
        //
        // Class I - Small machines
        //              - electrical motors P < 15kW
        //
        // Class II - Medium-sized machines
        //              - electrical motors with no special foundation P <= 75kW
        //              - rigidly mounted machines P <= 300kW
        //
        // Class III - Large machines
        //              - large rotating assemblies mounted on rigid foundation P > 300kW
        //
        // Class IV - Large machines
        //              - large rotating assemblies mounted on soft foundation
        //              - gas turbines P > 10MW
        //************************************************************************
        typedef enum : uint8_t
        {
            ISO_10816_1_CLASS_1,    //!< ISO 10816-1, Class I machines
            ISO_10816_1_CLASS_2,    //!< ISO 10816-1, Class II machines
            ISO_10816_1_CLASS_3,    //!< ISO 10816-1, Class III machines
            ISO_10816_1_CLASS_4     //!< ISO 10816-1, Class IV machines
        }Iso10816Part1Class;

        const TransitionZone ISO_10816_1_CLASS_1_TRANSITION =
        {
            0.71e-3,    //!< Zone A <-> B [m/s]
            1.80e-3,    //!< Zone B <-> C [m/s]
            4.50e-3     //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_10816_1_CLASS_2_TRANSITION =
        {
            1.12e-3,    //!< Zone A <-> B [m/s]
            2.80e-3,    //!< Zone B <-> C [m/s]
            7.10e-3     //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_10816_1_CLASS_3_TRANSITION =
        {
            1.80e-3,    //!< Zone A <-> B [m/s]
            4.50e-3,    //!< Zone B <-> C [m/s]
            11.2e-3     //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_10816_1_CLASS_4_TRANSITION =
        {
            2.80e-3,    //!< Zone A <-> B [m/s]
            7.10e-3,    //!< Zone B <-> C [m/s]
            18.0e-3     //!< Zone C <-> D [m/s]
        };

        static constexpr double ISO_20816_1_TRANSITION_AB_MIN = 0.71e-3;   //!< min RMS speed for A <-> B [m/s]
        static constexpr double ISO_20816_1_TRANSITION_AB_MAX = 4.5e-3;    //!< max RMS speed for A <-> B [m/s]

        static constexpr double ISO_20816_1_TRANSITION_BC_MIN = 1.8e-3;    //!< min RMS speed for B <-> C [m/s]
        static constexpr double ISO_20816_1_TRANSITION_BC_MAX = 9.3e-3;    //!< max RMS speed for B <-> C [m/s]

        static constexpr double ISO_20816_1_TRANSITION_CD_MIN = 4.5e-3;    //!< min RMS speed for C <-> D [m/s]
        static constexpr double ISO_20816_1_TRANSITION_CD_MAX = 14.7e-3;   //!< max RMS speed for C <-> D [m/s]


        //************************************************************************
        // ISO 20816-3:2022
        // Mechanical vibration - Measurement and evaluation of machine vibration
        // Part 3: Industrial machinery with a power rating above 15 kW and operating speeds
        //         between 120 r/min and 30 000 r/min
        //
        // Group I - Large machines 300kW < P <= 40MW
        //           OR
        //           Electrical machines with shaft height  h >= 315mm
        //
        // Group II - Medium-sized machines 15kW < P <= 300kW
        //            OR
        //            Electrical machines with shaft height  160mm <= h <= 315mm
        //************************************************************************
        typedef enum : uint8_t
        {
            ISO_20816_3_GROUP_1_RIGID,      //!< ISO 20816-3, Group I machines, rigid mount
            ISO_20816_3_GROUP_1_FLEXIBLE,   //!< ISO 20816-3, Group I machines, flexible mount
            ISO_20816_3_GROUP_2_RIGID,      //!< ISO 20816-3, Group II machines, rigid mount
            ISO_20816_3_GROUP_2_FLEXIBLE    //!< ISO 20816-3, Group II machines, flexible mount
        }Iso20816Part3Group;

        const TransitionZone ISO_20816_3_GROUP_1_RIGID_TRANSITION =
        {
            2.3e-3,     //!< Zone A <-> B [m/s]
            4.5e-3,     //!< Zone B <-> C [m/s]
            7.1e-3      //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_20816_3_GROUP_1_FLEXIBLE_TRANSITION =
        {
            3.5e-3,     //!< Zone A <-> B [m/s]
            7.1e-3,     //!< Zone B <-> C [m/s]
            11.0e-3     //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_20816_3_GROUP_2_RIGID_TRANSITION =
        {
            1.4e-3,     //!< Zone A <-> B [m/s]
            2.8e-3,     //!< Zone B <-> C [m/s]
            4.5e-3      //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_20816_3_GROUP_2_FLEXIBLE_TRANSITION =
        {
            2.3e-3,     //!< Zone A <-> B [m/s]
            4.5e-3,     //!< Zone B <-> C [m/s]
            7.1e-3      //!< Zone C <-> D [m/s]
        };

        //************************************************************************
        // ISO 20816-21:2015
        // Mechanical vibration - Measurement and evaluation of machine vibration
        // Part 21: Horizontal axis wind turbines with gearbox
        //************************************************************************
        typedef enum : uint8_t
        {
            ISO_20816_21_MAIN_BEARING,      //!< ISO 20816-21, main bearing
            ISO_20816_21_GEARBOX,           //!< ISO 20816-21, gearbox
            ISO_20816_21_GENERATOR,         //!< ISO 20816-21, generator
            ISO_20816_21_TOWER              //!< ISO 20816-21, tower
        }Iso20816Part21Component;

        const TransitionZone ISO_20816_21_MAIN_BEARING_TRANSITION =
        {
            0,          //!< Zone A <-> B [m/s]
            2.0e-3,     //!< Zone B <-> C [m/s]
            3.2e-3      //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_20816_21_GEARBOX_TRANSITION =
        {
            0,          //!< Zone A <-> B [m/s]
            3.5e-3,     //!< Zone B <-> C [m/s]
            5.6e-3      //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_20816_21_GENERATOR_TRANSITION =
        {
            0,          //!< Zone A <-> B [m/s]
            6e-3,       //!< Zone B <-> C [m/s]
            10e-3       //!< Zone C <-> D [m/s]
        };

        const TransitionZone ISO_20816_21_TOWER_TRANSITION =
        {
            0,          //!< Zone A <-> B [m/s]
            60e-3,      //!< Zone B <-> C [m/s]
            100e-3      //!< Zone C <-> D [m/s]
        };


        //************************************************************************
        // List of available options
        //************************************************************************
        typedef enum : uint8_t
        {
            OPTION_ITEM_ISO_10816_1_CLASS_1,
            OPTION_ITEM_ISO_10816_1_CLASS_2,
            OPTION_ITEM_ISO_10816_1_CLASS_3,
            OPTION_ITEM_ISO_10816_1_CLASS_4,

            OPTION_ITEM_ISO_20816_1,

            OPTION_ITEM_ISO_20816_3_GROUP_1_RIGID,
            OPTION_ITEM_ISO_20816_3_GROUP_1_FLEXIBLE,
            OPTION_ITEM_ISO_20816_3_GROUP_2_RIGID,
            OPTION_ITEM_ISO_20816_3_GROUP_2_FLEXIBLE,

            OPTION_ITEM_ISO_20816_21_MAIN_BEARING,
            OPTION_ITEM_ISO_20816_21_GEARBOX,
            OPTION_ITEM_ISO_20816_21_GENERATOR,
            OPTION_ITEM_ISO_20816_21_TOWER
        }OptionItem;

        const OptionItem OPTION_ITEM_DEFAULT = OPTION_ITEM_ISO_10816_1_CLASS_1;


    //************************************************************************
    // functions
    //************************************************************************
    public:
        VibrationMonitoringSettings();

        ~VibrationMonitoringSettings();

        OptionItem getActiveOption();

        TransitionZone getActiveTransitions();

        std::string getOptionDescription
            (
            const OptionItem                aOptionItem         //!< option
            ) const;

        std::map<OptionItem, std::string> getOptionsMap() const;

        TransitionZone getTransitionIso20816Part1NonRotating() const;

        EvaluationZone getZone
            (
            const double                    aRmsSpeed           //!< RMS speed [m/s]
            ) const;

        bool setActiveOption
            (
            const OptionItem                aOptionItem         //!< option
            );

        bool setIso20816Part1NonRotatingTransitionAB
            (
            const double                    aRmsSpeed           //!< RMS speed [m/s]
            );

        bool setIso20816Part1NonRotatingTransitionBC
            (
            const double                    aRmsSpeed           //!< RMS speed [m/s]
            );

        bool setIso20816Part1NonRotatingTransitionCD
            (
            const double                    aRmsSpeed           //!< RMS speed [m/s]
            );

        VibrationMonitoringSettings& operator =
            (
            VibrationMonitoringSettings const& aObject          //!< object
            );

    private:
        EvaluationZone getZoneIso10816Part1
            (
            const Iso10816Part1Class        aMachineClass,      //!< machine class
            const double                    aRmsSpeed           //!< RMS speed [m/s]
            ) const;

        EvaluationZone getZoneIso20816Part1
            (
            const double                    aRmsSpeed           //!< RMS speed [m/s]
            ) const;

        EvaluationZone getZoneIso20816Part3
            (
            const Iso20816Part3Group        aMachineGroup,      //!< machine group
            const double                    aRmsSpeed           //!< RMS speed [m/s]
            ) const;

        EvaluationZone getZoneIso10816Part21
            (
            const Iso20816Part21Component   aComponent,         //!< component
            const double                    aRmsSpeed           //!< RMS speed [m/s]
            ) const;

        void updateActiveTransitions();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        TransitionZone                      mIso20816Part1NonRotatingTransition;    //!< transition data for non-rotating, ISO 20816-1
        std::map<OptionItem, std::string>   mOptionsMap;                            //!< available options

        OptionItem                          mActiveOption;                          //!< active option
        TransitionZone                      mActiveTransitions;                     //!< active transitions
};

#endif // VibrationMonitoringSettings_h
