/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_units.h                                                 */
/*    Author:     James Pearman                                               */
/*    Created:    5 July 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_UNITS_H
#define   VEX_UNITS_H

// fix issue with some builds (windows simulation build)
#if defined(small)
#undef small
#endif

/*-----------------------------------------------------------------------------*/
/** @file    vex_units.h
  * @brief   Header for global units
*//*---------------------------------------------------------------------------*/

namespace vex {
    /**
     @brief The measurement units for percentage values.
    */
    enum class percentUnits {
      /** @brief A percentage unit that represents a value from 0% to 100% */
      pct = 0
    };
    /**
     @brief The measurement units for time values.
    */
    enum class timeUnits {
      /** @brief A time unit that is measured in seconds.*/
      sec,
      /** @brief A time unit that is measured in milliseconds.*/
      msec
    };
    /**
     @brief The measurement units for current values.
    */
    enum class currentUnits {
      /** @brief A current unit that is measured in amps.*/
      amp
    };
    /**
     @brief The measurement units for voltage values.
    */
    enum class voltageUnits {
      /** @brief A voltage unit that is measured in volts.*/
      volt,
      /** @brief A voltage unit that is measured in millivolts.*/
      mV
    };
    /**
     @brief The measurement units for power values.
    */
    enum class powerUnits {
      /** @brief A power unit that is measured in watts.*/
      watt
    };
    /**
     @brief The measurement units for torque values.
    */
    enum class torqueUnits {
      /** @brief A torque unit that is measured in Newton Meters.*/
      Nm,
      /** @brief A torque unit that is measured in Inch Pounds.*/
      InLb
    };
    /**
     @brief The measurement units for rotation values.
    */
    enum class rotationUnits {
      /** @brief A rotation unit that is measured in degrees.*/
      deg,
      /** @brief A rotation unit that is measured in revolutions.*/
      rev,
      /** @brief A rotation unit that is measured in raw data form.*/
      raw = 99
    };
    /**
     @brief The measurement units for velocity values.
    */
    enum class velocityUnits {
      /** @brief A velocity unit that is measured in percentage.*/
      pct = (int)percentUnits::pct,
      /** @brief A velocity unit that is measured in rotations per minute.*/
      rpm,
      /** @brief A velocity unit that is measured in degrees per second.*/
      dps
    };
    /**
     @brief The measurement units for distance values.
    */
    enum class distanceUnits {
      /** @brief A distance unit that is measured in millimeters.*/
      mm,
      /** @brief A distance unit that is measured in inches.*/
      in,
      /** @brief A distance unit that is measured in centimeters.*/
      cm
    };
    /**
     @brief The measurement units for analog values.
    */
    enum class analogUnits {
      /** @brief An analog unit that is measured in percentage.*/
      pct = (int)percentUnits::pct,
      /** @brief An analog unit that is measured in an 8-bit analog value (a value with 256 possible states).*/
      range8bit,
      /** @brief An analog unit that is measured in a 10-bit analog value (a value with 1024 possible states).*/
      range10bit,
      /** @brief An analog unit that is measured in a 12-bit analog value (a value with 4096 possible states).*/
      range12bit,
      /** @brief An analog unit that is measured in millivolts.*/
      mV
    };
    /**
     @brief The measurement units for temperature values.
    */
    enum class temperatureUnits {
      /** @brief A temperature unit that is measured in celsius.*/
      celsius,
      /** @brief A temperature unit that is measured in fahrenheit.*/
      fahrenheit
    };
    
    // Motor related
    /**
     @brief The defined units for direction values.
    */
    enum class directionType {
      /** @brief A direction unit that is defined as forward.*/
      fwd = 0,
      /** @brief A direction unit that is defined as backward.*/
      rev,
      undefined
    };

    /**
     @brief The defined units for turn values.
    */
    enum class turnType {
      /** @brief A turn unit that is defined as left turning.*/
      left,
      /** @brief A turn unit that is defined as right turning.*/
      right
      };
        
    /**
     @brief The defined units for brake values.
    */
    enum class brakeType {
      /** @brief A brake unit that is defined as coast.*/
      coast = kV5MotorBrakeModeCoast,
      /** @brief A brake unit that is defined as brake.*/
      brake = kV5MotorBrakeModeBrake,
      /** @brief A brake unit that is defined as hold.*/
      hold  = kV5MotorBrakeModeHold,
      undefined
    } ;
    /**
     @brief The defined units for gear values.
    */
    enum class gearSetting {
      /** @brief A gear unit that is defined as the red 36:1 gear cartridge used in V5 Smart Motors.*/
      ratio36_1 = kMotorGearSet_36,
      /** @brief A gear unit that is defined as the green 18:1 gear cartridge used in V5 Smart Motors.*/
      ratio18_1 = kMotorGearSet_18,
      /** @brief A gear unit that is defined as the blue 6:1 gear cartridge used in V5 Smart Motors.*/
      ratio6_1  = kMotorGearSet_06
    };
    
    // Brain/LCD related
    /**
     @brief The defined units for font values.
    */
    enum class fontType {
      /** @brief A font unit that is defined as mono20.*/
      mono20 = 0,
      /** @brief A font unit that is defined as mono30.*/
      mono30,
      /** @brief A font unit that is defined as mono40.*/
      mono40,
      /** @brief A font unit that is defined as mono60.*/
      mono60,
      /** @brief A font unit that is defined as prop20.*/
      prop20,
      /** @brief A font unit that is defined as prop30.*/
      prop30,
      /** @brief A font unit that is defined as prop40.*/
      prop40,
      /** @brief A font unit that is defined as prop60.*/
      prop60,

      // not in spec
      /** @brief A font unit that is defined as mono15.*/
      mono15,
      /** @brief A font unit that is defined as mono12.*/
      mono12,
      /** @brief A font unit that is defined as cjk16.*/      
      cjk16
    };                    

    // triport
    /**
     @brief The defined units for triport devices.
    */
    enum class triportType {
      /** @brief A triport unit that is defined as an analog input.*/
      analogInput,
      /** @brief A triport unit that is defined as an analog output.*/
      analogOutput,
      /** @brief A triport unit that is defined as a digital input.*/
      digitalInput,
      /** @brief A triport unit that is defined as a digital output.*/
      digitalOutput,

      /** @brief A triport unit that is defined as a button.*/
      button,
      /** @brief A triport unit that is defined as a potentiometer.*/
      potentiometer,
      /** @brief A triport unit that is defined as a line sensor.*/
      lineSensor,
      /** @brief A triport unit that is defined as a light sensor.*/
      lightSensor,   
      /** @brief A triport unit that is defined as a gyro.*/
      gyro, 
      /** @brief A triport unit that is defined as an accelerometer.*/
      accelerometer,
      
      /** @brief A triport unit that is defined as a motor.*/
      motor,
      /** @brief A triport unit that is defined as a servo.*/
      servo,
      
      /** @brief A triport unit that is defined as a quadrature encoder.*/
      quadEncoder,
      /** @brief A triport unit that is defined as a sonar.*/
      sonar,
      
      /** @brief A triport unit that is defined as a motor with slew rate control.*/
      motorS,
    };

    /**
     @brief The defined units for controller devices.
    */
    enum class controllerType {
      /** @brief A controller unit defined as a primary controller.*/
      primary = kControllerMaster,
      /** @brief A controller unit defined as a partner controller.*/
      partner = kControllerPartner
    };

    /**
     @brief The defined units for inertial sensor axis.
    */
    enum class axisType {
      xaxis,
      yaxis,
      zaxis
    };

    /**
     @brief The defined units for inertial sensor orientation.
    */
    enum class orientationType {
      roll,
      pitch,
      yaw
    };

    /**
     @brief The defined units for distance sensor object size.
    */
    enum class sizeType {
      none,
      small,
      medium,
      large
    };
    
    /**
     @brief The defined units for optical sensor led state.
    */
    enum class ledState {
      off,
      on
    };
    
    /**
     @brief The defined units for optical sensor gesture types.
    */
    enum class gestureType {
      none,
      up,
      down,
      left,
      right
    };
    
    /**
     @brief The defined units for vexlink types.
    */
    enum class linkType {
      undefined,
      /** @brief A vexlink type that is defined as the manager radio. */
      manager,
      /** @brief A vexlink type that is defined as the worker radio. */
      worker,
      /** @brief A vexlink type that is defined as a raw unmanaged link. */      
      raw
    };

    /**
     @brief The defined types for pneumatic cylinders.
    */
    enum class cylinderType {
      cylinder1 = 0,
      cylinder2 = 1,
      cylinder3 = 2,
      cylinder4 = 3,
      cylinderAll = 0xff
    };    
};

#endif // VEX_UNITS_H