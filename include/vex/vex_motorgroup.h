/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_motorgroup.h                                            */
/*    Author:     James Pearman                                               */
/*    Created:    7 April 2019                                                */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_MOTOR_GROUP_CLASS_H
#define   VEX_MOTOR_GROUP_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_motorgroup.h
  * @brief   Motor group class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  
  class motor_group  {
    private:
      class  motor_group_impl;
      class  motor_group_motors {
        friend class vex::motor_group;
        private:
          #define  STATIC_MEMORY  16 // do not change
          uintptr_t _memory[STATIC_MEMORY];
          motor_group_impl  *pimpl;
        public:
          motor_group_motors();
          motor_group_motors(const motor_group_motors&);
          ~motor_group_motors();
      };

      int32_t             _timeout;
      motor_group_motors  _motors;

      void _addMotor();
      void _addMotor( vex::motor &m );

      template <typename... Args>
      void _addMotor(  vex::motor &m1, Args &... m2 ) {
         _addMotor( m1 );
         _addMotor( m2... );
      }

      bool waitForCompletionAll();

    public:      
      motor_group();
      ~motor_group();
      
      template <typename... Args>
      motor_group( vex::motor &m1, Args &... m2 ) : motor_group() {
        _addMotor( m1 );
        _addMotor( m2... );
      }

      template <typename... Args>
      void operator()( vex::motor &m1, Args &... m2 ) {
        _addMotor( m1 );
        _addMotor( m2... );
      }

      /** 
       * @brief return the number of motors in the motor group
       * @return number of motors
       */
      int32_t         count(void);

      /** 
       * @brief Sets the velocity of the motor group based on the parameters set in the command. This command will not run the motor.  Any subsequent call that does not contain a specified motor velocity will use this value.
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value. 
       */
      void            setVelocity( double velocity, velocityUnits units );

      void            setVelocity( double velocity, percentUnits units ){
          setVelocity( velocity, static_cast<velocityUnits>(units) );
      };

      /** 
       * @brief Sets the stopping mode of the motor group by passing a brake mode as a parameter.
       * @param mode The stopping mode can be set to coast, brake, or hold.  
       */
      void            setStopping( brakeType mode );

      /** 
       * @brief Resets the motor's encoder to the value of zero. 
       */   
      void            resetPosition( void );

      /** 
       * @brief Sets the value of all motor encoders to the value specified in the parameter.
       * @param value Sets the current position of the motor.
       * @param units The measurement unit for the position value.
       */
      void            setPosition( double value, rotationUnits units );

      /** 
       * @brief Sets the timeout for the motor group. If the motor group does not reach its' commanded position prior to the completion of the timeout, the motors will stop.
       * @param time Sets the amount of time.
       * @param units The measurement unit for the time value.
       */
      void            setTimeout( int32_t time, timeUnits units );

      /** 
       * @brief Turns the motors on, and spins them in the specified direction.
       * @param dir The direction to spin the motors.
       */
      void            spin( directionType dir );

      /**
       * @brief Turn on the motors and spins them in the specified direction and a specified velocity.
       * @param dir The direction to spin the motors. 
       * @param velocity Sets the amount of velocity.
       * @param units The measurement unit for the velocity value. 
      */
      void            spin( directionType dir, double velocity, velocityUnits units );

      void            spin( directionType dir, double velocity, percentUnits units ){
          spin( dir, velocity, static_cast<velocityUnits>(units) );
      };

      /**
       * @brief Turn on the motors and spins them in the specified direction and a specified voltage.
       * @param dir The direction to spin the motors. 
       * @param voltage Sets the amount of volts.
       * @param units The measurement unit for the voltage value. 
      */
      void            spin( directionType dir, double voltage, voltageUnits units );

      /**
       * @brief Turns on the motors and spin them to an absolute target rotation value at a specified velocity.
       * @return Returns a Boolean that signifies when the motors have reached the target rotation value.
       * @param rotation Sets the amount of rotation.
       * @param units The measurement unit for the rotation value.
       * @param velocity Sets the amount of velocity.
       * @param units_v The measurement unit for the velocity value. 
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
      */
      bool            spinTo( double rotation, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );
      bool            spinToPosition( double rotation, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion=true );

      /**
       * @brief Turn on the motors and spin them to an absolute target rotation value.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param rotation Sets the amount of rotation.
       * @param units The measurement unit for the rotation value.        
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
      */
      bool            spinTo( double rotation, rotationUnits units, bool waitForCompletion=true );
      bool            spinToPosition( double rotation, rotationUnits units, bool waitForCompletion=true );


      /**
       * @brief Turn on the motors and spin them to a relative target rotation value at a specified velocity.
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
       * @brief Turn on the motors and spin them to a relative target rotation value.
       * @return Returns a Boolean that signifies when the motor has reached the target rotation value.
       * @param rotation Sets the amount of rotation.
       * @param units The measurement unit for the rotation value.        
       * @param waitForCompletion (Optional) If true, your program will wait until the motor reaches the target rotational value. If false, the program will continue after calling this function. By default, this parameter is true.
      */
      bool            spinFor( double rotation, rotationUnits units, bool waitForCompletion=true );

      bool            spinFor( directionType dir, double rotation, rotationUnits units, bool waitForCompletion=true );

      /**
       * @brief Turn on the motors and spin them to a relative target time value at a specified velocity.
       * @param time Sets the amount of time.
       * @param units The measurement unit for the time value.
       * @param velocity Sets the amount of velocity.
       * @param units_v The measurement unit for the velocity value.       
      */
      void            spinFor( double time, timeUnits units, double velocity, velocityUnits units_v );
      
      void            spinFor( directionType dir, double time, timeUnits units, double velocity, velocityUnits units_v );

      /**
       * @brief Turn on the motors and spin them to a relative target time value.
       * @param time Sets the amount of time.
       * @param units The measurement unit for the time value.              
      */
      void            spinFor( double time, timeUnits units );
      
      void            spinFor( directionType dir, double time, timeUnits units );

      /** 
       * @brief Checks to see if any of the motors are rotating to a specific target.
       * @return Returns a true Boolean if the motor is on and is rotating to a target. Returns a false Boolean if the motor is done rotating to a target.
       */
      bool            isSpinning( void );

      /** 
       * @brief Checks to see if all the motor are done rotating to a specific target.
       * @return Returns a false Boolean if the motor is on and is rotating to a target. Returns a true Boolean if the motor is done rotating to a target.
       */
      bool            isDone( void );      

      bool            isSpinningMode( void );

      /** 
       * @brief Stops all motors using the default brake mode.
       */
      void            stop( void );

      /** 
       * @brief Stops all motors using a specified brake mode.
       * @param mode The brake mode can be set to coast, brake, or hold. 
       */
      void            stop( brakeType mode );

      /** 
       * @brief Sets the max torque of the motors.
       * @param value Sets the amount of torque.
       * @param units The measurement unit for the torque value.
       */
      void            setMaxTorque( double value, percentUnits units );

      /** 
       * @brief Sets the max torque of the motors.
       * @param value Sets the amount of torque.
       * @param units The measurement unit for the torque value.
       */
      void            setMaxTorque( double value, torqueUnits units );
      
      /** 
       * @brief Sets the max torque of the motors.
       * @param value Sets the amount of torque.
       * @param units The measurement unit for the torque value.
       */
      void            setMaxTorque( double value, currentUnits units );

      /** 
       * @brief Gets which direction the first motor in the group is spinning.
       * @return Returns the direction that the motor is spinning.
       */
      directionType   direction( void );

      /** 
       * @brief Gets the current position of the first motor in the group's encoder.
       * @returns Returns a double that represents the current position of the motor in the units defined in the parameter.
       * @param units The measurement unit for the position.
       */
      double          position( rotationUnits units );

      /** 
       * @brief Gets the current velocity of the first motor in the group.
       * @return Returns a double that represents the current velocity of the motor in the units defined in the parameter.
       * @param units The measurement unit for the velocity.
       */
      double          velocity( velocityUnits units );

      double          velocity( percentUnits units ) {
          return velocity( static_cast<velocityUnits>(units) );
      };

      /** 
       * @brief Gets the sum electrical current for all motors in the group.
       * @return Returns a double that represents the electrical current of the motors in the units defined in the parameter.
       * @param units The measurement unit for the current.
       */
      double          current( currentUnits units = currentUnits::amp );

      /** 
       * @brief Gets the electrical current of the motors in percentage of maximum.
       * @return Returns a double that represents the electrical current of the motors as percentage of max current.
       * @param units The measurement unit for the current.
       */
      double          current( percentUnits units );

      /** 
       * @brief Gets the voltage of the first motor in the group.
       * @return Returns a double that represents the voltage of the motor in the units defined in the parameter.
       * @param units The measurement unit for the voltage.
       */
      double          voltage( voltageUnits units = voltageUnits::volt );

      /** 
       * @brief Gets the power of the first motor in the group.
       * @return Returns a double that represents the power of the motor in the units defined in the parameter.
       * @param units The measurement unit for the power.
       */
      double          power( powerUnits units = powerUnits::watt );

      /** 
       * @brief Gets the torque of the first motor in the group.
       * @return Returns a double that represents the torque of the motor in the units defined in the parameter.
       * @param units The measurement unit for the torque.
       */
      double          torque( torqueUnits units = torqueUnits::Nm );

      /** 
       * @brief Gets the efficiency of the first motor in the group.
       * @return Returns the efficiency of the motor in the units defined in the parameter.
       * @param units (Optional) The measurement unit for the efficiency. By default, this parameter is a percentage.
       */
      double          efficiency( percentUnits units = percentUnits::pct );

      /** 
       * @brief Gets the temperature  of the first motor in the group.
       * @return Returns the temperature of the motor in the units defined in the parameter.
       * @param units The measurement unit for the temperature.
       */
      double          temperature(  percentUnits units = percentUnits::pct );

      /** 
       * @brief Gets the temperature  of the motor.
       * @return Returns the temperature of the motor in the units defined in the parameter.
       * @param units The measurement unit for the temperature.
       */
      double          temperature( temperatureUnits units );

      /** 
       * @brief Convert velocity in given units to rpm based on motor gearing for the first motor in the group.
       * @return Returns velocity converted to velocity in output units (default rpm);
       * @param velocity the velocity to convert
       * @param units The measurement unit for the velocity value.       
       * @param unitsout The measurement unit for the returned velocity value.       
       */
      double          convertVelocity( double velocity, velocityUnits units, velocityUnits unitsout = velocityUnits::rpm );

      /** 
       * @brief get the gear cartridge setting for the first motor in the group.
       * @return Returns the gear cartridge setting.
       */
      gearSetting     getMotorCartridge();

    protected:
      vex::motor *    operator[](int32_t index);
      vex::motor **   begin(); 
      vex::motor **   end();       
  };
}

#endif // VEX_MOTOR_GROUP_CLASS_H