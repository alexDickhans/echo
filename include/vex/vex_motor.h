/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_motor.h                                                 */
/*    Author:     James Pearman                                               */
/*    Created:    2 June 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_MOTOR_CLASS_H
#define   VEX_MOTOR_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_motor.h
  * @brief   Motor device class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
    * @brief Use the motor class to control motor devices.
  */
  class motor : public device {
    public:
     /** 
      * @brief Creates a new motor object on the port specified.
      * @param index The port index for this motor. The index is zero-based.
      */
      motor( int32_t index );  
      ~motor();
    
      bool            installed();
      int32_t         value();

      /** 
       * @brief Creates a new motor object on the port specified and sets the reversed flag.
       * @param index The port index for this motor. The index is zero-based.
       * @param reverse Sets the reverse flag for the new motor object.
       */
      motor( int32_t index, bool reverse );

      /** 
       * @brief Creates a new motor object on the port specified and sets the gear setting
       * @param index The port index for this motor. The index is zero-based.
       * @param gears Sets the gear's setting for the new motor object.
       */
      motor( int32_t index, gearSetting gears );

      /** 
       * @brief Creates a new motor object on the port specified Sets the reversed flag and the gear setting for the new motor object.
       * @param index The port index for this motor. The index is zero-based.
       * @param gears Sets the gear's setting for the new motor object.
       * @param reverse Sets the reverse flag for the new motor object.
       */  
      motor( int32_t index, gearSetting gears, bool reverse );

      /** 
       * @brief Sets the motor mode to "reverse", which will make motor commands spin the motor in the opposite direction.
       * @param value If set to true, motor commands spin the motor in the opposite direction.
       */
      void            setReversed( bool value );

      /** 
       * @brief Sets the velocity of the motor based on the parameters set in the command. This command will not run the motor.  Any subsequent call that does not contain a specified motor velocity will use this value.
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value. 
       */
      void            setVelocity( double velocity, velocityUnits units );

      /** 
       * @brief Sets the velocity of the motor based on the parameters set in the command. This command will not run the motor.  Any subsequent call that does not contain a specified motor velocity will use this value.
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value.
       */
      void            setVelocity( double velocity, percentUnits units ){
          setVelocity( velocity, static_cast<velocityUnits>(units) );
      };

      //Legacy 
      void            setBrake( brakeType mode );

      /** 
       * @brief Sets the stopping mode of the motor by passing a brake mode as a parameter.
       * @param mode The stopping mode can be set to coast, brake, or hold.  
       */
      void            setStopping( brakeType mode );

      /** 
       * @brief Resets the motor's encoder to the value of zero. 
       */   
      void            resetPosition( void );

      /** 
       * @brief Sets the value of the motor's encoder to the value specified in the parameter.
       * @param value Sets the current position of the motor.
       * @param units The measurement unit for the position value.
       */
      void            setPosition( double value, rotationUnits units );

      /** 
       * @brief Sets the timeout for the motor. If the motor does not reach its' commanded position prior to the completion of the timeout, the motor will stop.
       * @param time Sets the amount of time.
       * @param units The measurement unit for the time value.
       */
      void            setTimeout( int32_t time, timeUnits units );

      //Actions

      /** 
       * @brief Turns the motor on, and spins it in the specified direction.
       * @param dir The direction to spin the motor.
       */
      void            spin( directionType dir );

      /**
       * @brief Turns on the motor and spins it in a specified direction and a specified velocity.
       * @param dir The direction to spin the motor. 
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value. 
       */
      void            spin( directionType dir, double velocity, velocityUnits units );

      void            spin( directionType dir, double velocity, percentUnits units ){
          spin( dir, velocity, static_cast<velocityUnits>(units) );
      }

      /**
       * @brief Turns on the motor and spins it in a specified direction and a specified voltage.
       * @param dir The direction to spin the motor. 
       * @param voltage Sets the amount of volts.
       * @param units The measurement unit for the voltage value. 
       */
      void            spin( directionType dir, double voltage, voltageUnits units );

      /**
       * @brief Turns on the motor and spins it to an absolute target rotation value at a specified velocity.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param rotation Sets the amount of rotation.
       * @param units The measurement unit for the rotation value.
       * @param velocity Sets the amount of velocity.
       * @param units_v The measurement unit for the velocity value. 
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
       */
      bool            spinTo( double rotation, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      bool            spinToPosition( double rotation, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      /**
       * @brief Turns on the motor and spins it to an absolute target rotation value.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param rotation Sets the amount of rotation.
       * @param units The measurement unit for the rotation value.        
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
       */
      bool            spinTo( double rotation, rotationUnits units, bool waitForCompletion=true );

      bool            spinToPosition( double rotation, rotationUnits units, bool waitForCompletion=true );

      /**
       * @brief Turns on the motor and spins it to a relative target rotation value at a specified velocity.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param rotation Sets the amount of rotation.
       * @param units The measurement unit for the rotation value.
       * @param velocity Sets the amount of velocity.
       * @param units_v The measurement unit for the velocity value.
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
       */
      bool            spinFor( double rotation, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      bool            spinFor( directionType dir, double rotation, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      /**
       * @brief Turns on the motor and spins it to a relative target rotation value.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param rotation Sets the amount of rotation.
       * @param units The measurement unit for the rotation value.        
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
       */
      bool            spinFor( double rotation, rotationUnits units, bool waitForCompletion=true );

      bool            spinFor( directionType dir, double rotation, rotationUnits units, bool waitForCompletion=true );
      
      /**
       * @brief Turns on the motor and spins it to a relative target time value at a specified velocity.
       * @return true on success, false if parameter error
       * @param time Sets the amount of time.
       * @param units The measurement unit for the time value.
       * @param velocity Sets the amount of velocity.
       * @param units_v The measurement unit for the velocity value.       
       */
      bool            spinFor( double time, timeUnits units, double velocity, velocityUnits units_v );

      bool            spinFor( directionType dir, double time, timeUnits units, double velocity, velocityUnits units_v );

      /**
       * @brief Turns on the motor and spins it to a relative target time value.
       * @return true on success, false if parameter error
       * @param time Sets the amount of time.
       * @param units The measurement unit for the time value.              
       */
      bool            spinFor( double time, timeUnits units );

      bool            spinFor( directionType dir, double time, timeUnits units );

      /** 
       * @brief Checks to see if the motor is rotating to a specific target.
       * @return Returns a true Boolean if the motor is on and is rotating to a target. Returns a false Boolean if the motor is done rotating to a target.
       */
      bool            isSpinning( void );

      /** 
       * @brief Checks to see if the motor is done rotating to a specific target.
       * @return Returns a false Boolean if the motor is on and is rotating to a target. Returns a true Boolean if the motor is done rotating to a target.
       */
      bool            isDone( void );

      bool            isSpinningMode( void );

      /** 
       * @brief Stops the motor using the default brake mode.
       */
      void            stop( void );

      /** 
       * @brief Stops the motor using a specified brake mode.
       * @param mode The brake mode can be set to coast, brake, or hold. 
       */
      void            stop( brakeType mode );

      /** 
       * @brief Sets the max torque of the motor.
       * @param value Sets the amount of torque.
       * @param units The measurement unit for the torque value.
       */
      void            setMaxTorque( double value, percentUnits units );

      /** 
       * @brief Sets the max torque of the motor.
       * @param value Sets the amount of torque.
       * @param units The measurement unit for the torque value.
       */
      void            setMaxTorque( double value, torqueUnits units );
      
      /** 
       * @brief Sets the max torque of the motor.
       * @param value Sets the amount of torque.
       * @param units The measurement unit for the torque value.
       */
      void            setMaxTorque( double value, currentUnits units );
      
      // sensing

      /** 
       * @brief Gets which direction the motor is spinning.
       * @return Returns the direction that the motor is spinning.
       */
      directionType   direction( void );

      /** 
       * @brief Gets the current position of the motor's encoder.
       * @returns Returns a double that represents the current position of the motor in the units defined in the parameter.
       * @param units The measurement unit for the position.
       */
      double          position( rotationUnits units );

      /** 
       * @brief Gets the current velocity of the motor.
       * @return Returns a double that represents the current velocity of the motor in the units defined in the parameter.
       * @param units The measurement unit for the velocity.
       */
      double          velocity( velocityUnits units );

      double          velocity( percentUnits units ) {
          return velocity( static_cast<velocityUnits>(units) );
      };

      /** 
       * @brief Gets the electrical current of the motor.
       * @return Returns a double that represents the electrical current of the motor in the units defined in the parameter.
       * @param units The measurement unit for the current.
       */
      double          current( currentUnits units = currentUnits::amp );

      /** 
       * @brief Gets the electrical current of the motor in percentage of maximum.
       * @return Returns a double that represents the electrical current of the motor as percentage of max current.
       * @param units The measurement unit for the current.
       */
      double          current( percentUnits units );

      /** 
       * @brief Gets the electrical voltage of the motor.
       * @return Returns a double that represents the electrical voltage of the motor in the units defined in the parameter.
       * @param units The measurement unit for the voltage.
       */
      double          voltage( voltageUnits units = voltageUnits::volt );

      /** 
       * @brief Gets the power of the motor.
       * @return Returns a double that represents the power of the motor in the units defined in the parameter.
       * @param units The measurement unit for the power.
       */
      double          power( powerUnits units = powerUnits::watt );

      /** 
       * @brief Gets the torque of the motor.
       * @return Returns a double that represents the torque of the motor in the units defined in the parameter.
       * @param units The measurement unit for the torque.
       */
      double          torque( torqueUnits units = torqueUnits::Nm );

      /** 
       * @brief Gets the efficiency of the motor.
       * @return Returns the efficiency of the motor in the units defined in the parameter.
       * @param units (Optional) The measurement unit for the efficiency. By default, this parameter is a percentage.
       */
      double          efficiency( percentUnits units = percentUnits::pct );

      /** 
       * @brief Gets the temperature  of the motor.
       * @return Returns the temperature of the motor in the units defined in the parameter.
       * @param units The measurement unit for the temperature.
       */
      double          temperature( percentUnits units = percentUnits::pct );
      
      /** 
       * @brief Gets the temperature  of the motor.
       * @return Returns the temperature of the motor in the units defined in the parameter.
       * @param units The measurement unit for the temperature.
       */
      double          temperature( temperatureUnits units );

      /** 
       * @brief Get the type of the motor, 11W or 5.5W.
       * @return Returns the type of the motor, 0 is 11W, 1 is 5.5W.
       */
      int32_t         getMotorType();            

      /** 
       * @brief Convert velocity in given units to rpm based on motor gearing
       * @return Returns velocity converted to velocity in output units (default rpm);
       * @param velocity the velocity to convert
       * @param units The measurement unit for the velocity value.       
       * @param unitsout The measurement unit for the returned velocity value.       
       */
      double          convertVelocity( double velocity, velocityUnits units, velocityUnits unitsout = velocityUnits::rpm );

      /** 
       * @brief get the gear cartridge setting for this motor
       * @return Returns the gear cartridge setting.
       */
      gearSetting     getMotorCartridge();

    protected:
      int32_t         getTimeout();
      double          getVelocity( velocityUnits units );
      double          command( velocityUnits units );
    
    private:
      int32_t         _timeout;
      int32_t         _velocity;
      brakeType       _mode;
      brakeType       _brakeMode;
      bool            _spinMode;
            
      void            defaultStopping( brakeType mode );

      void            setRotationUnits( rotationUnits units );
      double          velocityToScaled( double velocity, velocityUnits units );
      int32_t         scaledToVelocity( double value, velocityUnits units );
      double          torqueToCurrent( double torque );
  };
};

#endif // VEX_MOTOR_CLASS_H