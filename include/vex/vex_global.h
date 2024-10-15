/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_global.h                                                */
/*    Author:     James Pearman                                               */
/*    Created:    5 July 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_GLOBAL_H
#define   VEX_GLOBAL_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_global.h
  * @brief   Globals for C++ API
*//*---------------------------------------------------------------------------*/

// more natural language based globals
namespace vex {
    extern const rotationUnits         degrees;
    extern const rotationUnits         turns;
    extern const percentUnits          percent;
    extern const timeUnits             seconds;
    extern const distanceUnits         inches;
    extern const distanceUnits         mm;
    extern const directionType         forward;
    extern const directionType         reverse;
    extern const turnType              left;
    extern const turnType              right;

    extern const axisType              xaxis;
    extern const axisType              yaxis;
    extern const axisType              zaxis;

    extern const orientationType       roll;
    extern const orientationType       pitch;
    extern const orientationType       yaw;

    extern const fontType              monoM;
    extern const fontType              monoL;
    extern const fontType              monoXL;
    extern const fontType              monoXXL;
    extern const fontType              monoS;
    extern const fontType              monoXS;
    extern const fontType              propM;
    extern const fontType              propL;
    extern const fontType              propXL;
    extern const fontType              propXXL;

    extern const controllerType        primary;
    extern const controllerType        partner;

    extern const char *                rumbleLong;
    extern const char *                rumbleShort;
    extern const char *                rumblePulse;

    extern const cylinderType          cylinder1;
    extern const cylinderType          cylinder2;
    extern const cylinderType          cylinder3;
    extern const cylinderType          cylinder4;
    extern const cylinderType          cylinderAll;

    extern void wait( double time, timeUnits units );
};

namespace vex {
    extern const int32_t               PORT1;
    extern const int32_t               PORT2;
    extern const int32_t               PORT3;
    extern const int32_t               PORT4;
    extern const int32_t               PORT5;
    extern const int32_t               PORT6;
    extern const int32_t               PORT7;
    extern const int32_t               PORT8;
    extern const int32_t               PORT9;
    extern const int32_t               PORT10;
    extern const int32_t               PORT11;
    extern const int32_t               PORT12;
    extern const int32_t               PORT13;
    extern const int32_t               PORT14;
    extern const int32_t               PORT15;
    extern const int32_t               PORT16;
    extern const int32_t               PORT17;
    extern const int32_t               PORT18;
    extern const int32_t               PORT19;
    extern const int32_t               PORT20;
    extern const int32_t               PORT21;
    extern const int32_t               PORT22;
    
    extern const percentUnits          pct;
    extern const timeUnits             sec;
    extern const timeUnits             msec;
    extern const voltageUnits          volt;
    extern const currentUnits          amp;
    extern const powerUnits            watt;
    extern const torqueUnits           Nm;
    extern const torqueUnits           InLb;
    extern const rotationUnits         deg;
    extern const rotationUnits         rev;
    extern const velocityUnits         rpm;
    extern const velocityUnits         dps;
    extern const temperatureUnits      celsius;
    extern const temperatureUnits      fahrenheit;

    extern const directionType         fwd;
    extern const brakeType             coast;
    extern const brakeType             brake;
    extern const brakeType             hold;
    extern const gearSetting           ratio36_1;
    extern const gearSetting           ratio18_1;
    extern const gearSetting           ratio6_1;

    extern const color &               black;
    extern const color &               white;
    extern const color &               red;
    extern const color &               green;
    extern const color &               blue;
    extern const color &               yellow;
    extern const color &               orange;
    extern const color &               purple;
    extern const color &               cyan;
    extern const color &               transparent;

    extern const fontType              mono20;
    extern const fontType              mono30;
    extern const fontType              mono40;
    extern const fontType              mono60;
    extern const fontType              mono15;
    extern const fontType              mono12;
    extern const fontType              prop20;
    extern const fontType              prop30;
    extern const fontType              prop40;
    extern const fontType              prop60;

    extern const analogUnits           range8bit;
    extern const analogUnits           range10bit;
    extern const analogUnits           range12bit;
    extern const analogUnits           mV;
};

#endif // VEX_GLOBAL_H