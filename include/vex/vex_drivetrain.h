/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_drivetrain.h                                            */
/*    Author:     James Pearman                                               */
/*    Created:    7 April 2019                                                */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_DRIVETRAIN_CLASS_H
#define   VEX_DRIVETRAIN_CLASS_H

#include "vex_motorgroup.h"

/*-----------------------------------------------------------------------------*/
/** @file    vex_drivetrain.h
  * @brief   Drivetrain class header
*//*---------------------------------------------------------------------------*/

namespace vex {

  class drivetrain  {
    protected:
      vex::motor_group  lm;
      vex::motor_group  rm;

    private:
      double _wheel_circumference;
      double _wheel_track;
      double _wheel_base;
      double _wheel_motor_gear_ratio;

      int32_t _timeout;
      bool   _waitForCompletionAll();

    public:
      drivetrain( motor_group &l, motor_group &r, double wheelTravel=320, double trackWidth=320, double wheelBase=130, distanceUnits unit=distanceUnits::mm, double externalGearRatio = 1.0 );
      drivetrain( vex::motor &l, vex::motor &r, double wheelTravel=320, double trackWidth=320, double wheelBase=130, distanceUnits unit=distanceUnits::mm, double externalGearRatio = 1.0 );

      virtual ~drivetrain();
      
      void  setGearRatio( double ratio );

      /** 
       * @brief Sets the velocity of the drive based on the parameters set in the command. This command will not run the motor.  Any subsequent call that does not contain a specified motor velocity will use this value.
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value. 
       */
      void  setDriveVelocity( double velocity, velocityUnits units );

      void  setDriveVelocity( double velocity, percentUnits units );

      /** 
       * @brief Sets the turn velocity of the drive based on the parameters set in the command. This command will not run the motor.
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value. 
       */
      void  setTurnVelocity( double velocity, velocityUnits units );

      void  setTurnVelocity( double velocity, percentUnits units );

      /** 
       * @brief Sets the timeout for the motor group. If the motor group does not reach its' commanded position prior to the completion of the timeout, the motors will stop.
       * @param time Sets the amount of time.
       * @param units The measurement unit for the time value.
       */
      void  setTimeout( int32_t time, timeUnits units );

      /** 
       * @brief Sets the stopping mode of the motor group by passing a brake mode as a parameter.
       * @param mode The stopping mode can be set to coast, brake, or hold.  
       */
      void  setStopping( brakeType mode );

      /** 
       * @brief Turns the motors on, and drive in the specified direction
       * @param dir The direction to drive.
       */
      void 	drive( directionType dir ) ;

      /**
       * @brief Turn on the motors and drive in the specified direction and a specified velocity.
       * @param dir The direction to drive. 
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value. 
      */
      void 	drive( directionType dir, double velocity, velocityUnits units );

      /**
       * @brief Turn on the motors and drive a distance at the default velocity.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param distance Sets the distance to drive.
       * @param units The measurement unit for the distance value.
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
      */
      virtual bool 	driveFor( double distance, distanceUnits units, bool waitForCompletion=true );

      virtual bool 	driveFor( directionType dir, double distance, distanceUnits units, bool waitForCompletion=true );

      /**
       * @brief Turn on the motors and drive a distance at a specified velocity.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param distance Sets the distance to drive.
       * @param units The measurement unit for the distance value.
       * @param velocity Sets the amount of velocity.
       * @param units_v The measurement unit for the velocity value.
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
      */
      virtual bool 	driveFor( double distance, distanceUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      virtual bool 	driveFor( directionType dir, double distance, distanceUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      /** 
       * @brief Turns the motors on, and rotate in the specified direction
       * @param dir The direction to rotate the robot.
       */
      void 	turn( turnType dir );

      /** 
       * @brief Turns the motors on, and rotate in the specified direction and a specified velocity.
       * @param dir The direction to rotate the robot.
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value. 
       */
      void 	turn( turnType dir, double velocity, velocityUnits units );

      /**
       * @brief Turn on the motors and rotate an angle at the default velocity.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param angle Sets the angle to turn.
       * @param units The measurement unit for the angle value.
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
      */
      virtual bool 	turnFor( double angle, rotationUnits units, bool waitForCompletion=true );

      virtual bool 	turnFor( turnType dir, double angle, rotationUnits units, bool waitForCompletion=true );

      /**
       * @brief Turn on the motors and rotate an angle at a specified velocity.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param angle Sets the angle to turn.
       * @param units The measurement unit for the angle value.
       * @param velocity Sets the amount of velocity.
       * @param units_v The measurement unit for the velocity value.
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
      */
      virtual bool 	turnFor( double angle, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      virtual bool 	turnFor( turnType dir, double angle, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      /** 
       * @brief Checks to see if any of the motors are rotating to a specific target.
       * @return Returns a true Boolean if the motor is on and is rotating to a target. Returns a false Boolean if the motor is done rotating to a target.
       */
      virtual bool 	isMoving( void );

      /** 
       * @brief Checks to see if all the motor are done rotating to a specific target.
       * @return Returns a false Boolean if the motor is on and is rotating to a target. Returns a true Boolean if the motor is done rotating to a target.
       */
      bool 	isDone( void );

      /** 
       * @brief Stops the drive using the default brake mode.
       */
      void  stop();

      /** 
       * @brief Stops the drive using a specified brake mode.
       * @param mode The brake mode can be set to coast, brake, or hold. 
       */
      void  stop( brakeType mode );

      void 	arcade( double drivePower, double turnPower, percentUnits units=percentUnits::pct );

      /** 
       * @brief Gets the average velocity of the drivetrain;
       * @return Returns a double that represents the current velocity of the motor in the units defined in the parameter.
       * @param units The measurement unit for the velocity.
       */
      double  velocity( velocityUnits units );

      double  velocity( percentUnits units ) {
          return velocity( static_cast<velocityUnits>(units) );
      };

      /** 
       * @brief Gets the total current of the drivetrain;
       * @return Returns a double that represents the electrical current of the motor in the units defined in the parameter.
       * @param units The measurement unit for the current.
       */
      double  current( currentUnits units = currentUnits::amp );

      /** 
       * @brief Gets the electrical current of the drivetrain as percentage of maximum.
       * @return Returns a double that represents the electrical current of the drivetrain as percentage of max current.
       * @param units The measurement unit for the current.
       */
      double  current( percentUnits units );

      /** 
       * @brief Gets the average voltage of the drivetrain;
       * @return Returns a double that represents the voltage of the motor in the units defined in the parameter.
       * @param units The measurement unit for the voltage.
       */
      double  voltage( voltageUnits units = voltageUnits::volt );

      /** 
       * @brief Gets the average power of the drivetrain;
       * @return Returns a double that represents the power of the motor in the units defined in the parameter.
       * @param units The measurement unit for the power.
       */
      double  power( powerUnits units = powerUnits::watt );

      /** 
       * @brief Gets the average torque of the drivetrain;
       * @return Returns a double that represents the torque of the motor in the units defined in the parameter.
       * @param units The measurement unit for the torque.
       */
      double  torque( torqueUnits units = torqueUnits::Nm );

      /** 
       * @brief Gets the average efficiency of the drivetrain;
       * @return Returns the efficiency of the motor in the units defined in the parameter.
       * @param units (Optional) The measurement unit for the efficiency. By default, this parameter is a percentage.
       */
      double  efficiency( percentUnits units = percentUnits::pct );

      /** 
       * @brief Gets the average temperature of the drivetrain;
       * @return Returns the temperature of the motor in the units defined in the parameter.
       * @param units The measurement unit for the temperature.
       */
      double  temperature(  percentUnits units );
     
    protected: 
      // Helper functions that can used by sub classes
      double  distanceToMm( double distance, distanceUnits units );
      double  angleToDeg( double angle, rotationUnits units );
      int32_t timeoutGet();
      double  distanceToRevs( double distance, distanceUnits units );
      double  angleToRevs( double angle, rotationUnits units );
      double  getMaxVelocity( velocityUnits units = velocityUnits::rpm );

      double        _turnvelocity;
      velocityUnits _turnvelocityUnits;
      double        _drivevelocity;
      velocityUnits _drivevelocityUnits;
      uint8_t       _turnmode;
  };
};

#endif // VEX_DRIVETRAIN_CLASS_H