/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2018-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_triport.h                                               */
/*    Author:     James Pearman                                               */
/*    Created:    10 Jan 2018                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @file    vex_triport.h
  * @brief   Triport device class header
*//*---------------------------------------------------------------------------*/

#ifndef   VEX_TRIPORT_CLASS_H
#define   VEX_TRIPORT_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @brief Triport device class                                                */
/*-----------------------------------------------------------------------------*/
namespace vex {
  /** 
    * @brief Use this class to get and set information about devices on Three Wire Ports A through H.
  */
  class triport : public device {
    public:
      enum class tEventType {
        EVENT_DIN_HIGH        =  0,
        EVENT_DIN_LOW         =  1,
        EVENT_AIN_CHANGED     =  2,
        EVENT_AIN_THRESHOLD   =  3
      };

    private:
      uint32_t _id = 0;

      void _setport() {};
      void _configPort( uint32_t id, triportType type );
    
      template <typename first, typename... Args>
      void _setport( first port, Args... args ) {
        _configPort( _id++, port );
        _setport( args... );
      }

      int32_t _getIndex();

      // conversion from C++ enum to C API
      V5_AdiPortConfiguration   _internalType( triportType type );
      triportType               _externalType( V5_AdiPortConfiguration type );
        
    public:
      triport( int32_t index );  
      ~triport();

      template <typename... Args>
      triport( int32_t index, Args... args) : triport(index) {
        _setport( args... );
      }
      /**
       * @brief Gets the status of what is installed on the port.
       * @return Returns true if a 3-Wire Port device is installed in the port, otherwise false.
       */
      bool            installed();
     
      /** 
        * @brief TBD
      */
      class port {
        friend class __tridevice;

        private:
          int32_t   _id;
          triport   *_parent;
          int32_t   _convertAnalog( analogUnits units );
          int32_t   _convertAnalogUnits( int32_t value, analogUnits units, analogUnits units_out );
          int32_t   _virtual_value( int32_t virtual_port );
          void      _virtual_value( int32_t virtual_port, int32_t value );
          
        public:
          port() : _id( -1 ), _parent(NULL) {};
          port( const int32_t id, triport *parent );
          port( const int32_t id, const triportType type, triport *parent );
          ~port() {};          
          
          /**
           * @brief Sets the port to a specific port type.
           * @param type A type of port that you want to set it to.
           */
          void type( const triportType type );

          /**
           * @brief Gets the current port type.
           * @return Returns the Port Type.
           */
          triportType type();

          /**
           * @brief Gets the index of the parent smart port
           */
          int32_t   index();

          /**
           * @brief Gets the id of the port
           * @return Returns an integer in the range 0 to 7, -1 for error
           */
          int32_t   id();

          /**
           * @brief Sets the port's value to the specified integer in the parameter. 
           * @param value An integer number that will be set to the port's value.
           */
          void      value( int32_t value );

          /**
           * @brief Gets the value of a port.
           * @return Returns an integer that represents the port's value.
           */
          int32_t   value();

          /**
           * @brief Sets the value of the port to a Boolean value.
           * @param value A true or false Boolean value that the port will be set to.
           */
          void      set( bool value );


          /**
           * @brief Calls a function when the port is pressed.
           * @param callback A reference to a function.
           */
          void     pressed( void (* callback)(void) );

          /**
           * @brief Calls a function when the port is released.
           * @param callback A reference to a function.
           */
          void     released( void (* callback)(void) );

          /**
           * @brief Calls a function when the port has changed value.
           * @param callback A reference to a function.
           */
          void     changed( void (* callback)(void) );


          /**
           * @brief Calls a function when the port input value has crossed a set threshold.
           * @param callback A reference to a function.
           */
          void     threshold( void (* callback)(void) );
          void     threshold( void (* callback)(void *), void *arg );
               
          void operator()( const triportType type ) {
            this->type( type );
          }
          void operator=( const int32_t value )  {
            this->value( value );
          }
          operator int()  {
            return this->value();
          }          
          operator bool()  {
            return (this->value() > 0) ? true : false;
          }          

          // encoder specific
          /**
           * @brief Resets the rotation of the encoder to zero.
           */
          void    resetRotation( void );

          /**
           * @brief Sets the rotation of the encoder to a specific value.
           * @param val A double that represents the value to set the rotation to.
           * @param units The unit type that the previous value should be treated as.
           */
          void    setRotation( double val, rotationUnits units );

          /**
           * @brief Gets the rotation value of the encoder.
           * @return Returns a double that represents the current rotation of the encoder.
           * @param units A unit type that defines what value of rotation that should be returned.
           */
          double  rotation( rotationUnits units );

          /**
           * @brief Gets the velocity of the encoder.
           * @return Returns a double that represents the current velocity of the encoder.
           * @param units A unit type that defines what value of velocity that should be returned.
           */
          double  velocity( velocityUnits units );

          /**
           * @brief Gets the value of the distance sensor.
           * @return Returns a double that represents the current distance of the sensor.
           * @param units A distance unit type that defines what value of distance that should be returned. 
           */
          double  distance( distanceUnits units );

          /**
           * @brief      Set detection threshold for analog sensors. (for threshold event)
           * @param[in]  threshold the new threshold in specified units
           * @param[in]  units the analog units
           */
          void    setThreshold( int32_t threshold, analogUnits units );
          
          mevent  PRESSED   = { (uint32_t)_parent->_getIndex(), ((uint32_t)tEventType::EVENT_DIN_HIGH    + (_id<<2)) };
          mevent  RELEASED  = { (uint32_t)_parent->_getIndex(), ((uint32_t)tEventType::EVENT_DIN_LOW     + (_id<<2)) };
          mevent  CHANGED   = { (uint32_t)_parent->_getIndex(), ((uint32_t)tEventType::EVENT_AIN_CHANGED + (_id<<2)) };
          
          mevent  &HIGH     = PRESSED;
          mevent  &LOW      = RELEASED;
      };

      // A triport device has 8 ports
      // This creates those instances and associates them with this triport
      port Port[8] = { {0, this},
                       {1, this},
                       {2, this},
                       {3, this},
                       {4, this},
                       {5, this},
                       {6, this},
                       {7, this}
                     };  
      
      // These are the public named references to those ports

      /**
       * @brief A Three Wire Port that is labeled as A on the Brain.
      */               
      port  &A = Port[0];
      /**
       * @brief A Three Wire Port that is labeled as B on the Brain.
      */
      port  &B = Port[1];
      /**
       * @brief A Three Wire Port that is labeled as C on the Brain.
      */
      port  &C = Port[2];
      /**
       * @brief A Three Wire Port that is labeled as D on the Brain.
      */
      port  &D = Port[3];
      /**
       * @brief A Three Wire Port that is labeled as E on the Brain.
      */
      port  &E = Port[4];
      /**
       * @brief A Three Wire Port that is labeled as F on the Brain.
      */
      port  &F = Port[5];
      /**
       * @brief A Three Wire Port that is labeled as G on the Brain.
      */
      port  &G = Port[6];
      /**
       * @brief A Three Wire Port that is labeled as H on the Brain.
      */
      port  &H = Port[7];
  };
  /** 
    * @brief Base class that allows access to Three Wire Port functions and members.
  */
  class __tridevice {
      protected:
        triport::port   &_port;

        int32_t   _convertAnalog( analogUnits units );
        int32_t   _convertAnalogUnits( int32_t value, analogUnits units, analogUnits units_out );
        int32_t   _virtual_value( int32_t virtual_port );
        void      _virtual_value( int32_t virtual_port, int32_t value );
        
        int32_t   _index();
        int32_t   _id();

        mevent &_PRESSED   = _port.PRESSED;
        mevent &_RELEASED  = _port.RELEASED;
        mevent &_CHANGED   = _port.CHANGED;
        mevent &_HIGH      = _port.HIGH;
        mevent &_LOW       = _port.LOW;
                
      public:
        __tridevice( triport::port &port, triportType type );
      protected:
        ~__tridevice();

  };
  /** 
    * @brief Use this class when programming with the limit switch
  */
  class limit : private __tridevice {
      public:

       /**
        * @brief Creates a new limit object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        limit( triport::port &port );
        ~limit();
        
        /**
         * @brief Gets the value of the limit switch
         * @return Returns an integer that represents the value of the limit switch.
        */
        int32_t   value();

        /**
         * @brief Gets the state of the limit switch.
         * @return Returns the state of the limit switch. 
        */
        int32_t   pressing() { return value(); };

        /**
         * @brief Calls a function when the limit switch is pressed.
         * @param callback A reference to a function.
         */
        void      pressed( void (* callback)(void) );

        /**
         * @brief Calls a function when the limit switch is released.
         * @param callback A reference to a function.
         */
        void      released( void (* callback)(void) );

        operator int();
        operator bool();
        
        mevent &PRESSED   = _PRESSED;
        mevent &RELEASED  = _RELEASED;
  };
  /** 
    * @brief Use this class when programming with the bumper device
  */
  class bumper : private __tridevice {
      public:
       /**
        * @brief Creates a new bumper switch object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        bumper( triport::port &port );
        ~bumper();
        
        /**
         * @brief Gets the value of the bumper switch.
         * @return Returns an integer that represents the value of the bumper switch. 
         */
        int32_t   value();

        /**
         * @brief Get the pressed status of the bumper switch.
         * @return Returns the state of the bumper switch. If it is pressed it will return a one. If unpressed, it will return a zero.
         */
        int32_t   pressing() { return value(); };

        /**
         * @brief Calls a function when the bumper switch is pressed.
         * @param callback A reference to a function.
         */
        void      pressed( void (* callback)(void) );

        /**
         * @brief Calls a function when the bumper switch is released.
         * @param callback A reference to a function.
         */
        void      released( void (* callback)(void) );

        operator int();
        operator bool();

        mevent &PRESSED   = _PRESSED;
        mevent &RELEASED  = _RELEASED;
  };  
  /**
    * @brief Use this class when programming with a digital-in device
  */
  class digital_in : private __tridevice {
      public:

        /**
        * @brief Creates a new digital_in object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        digital_in( triport::port &port );
        ~digital_in();
        
        /**
         * @brief Gets the value of the digital_in device.
         * @return Returns an integer that represents the value of the digital-in device.
         */
        int32_t   value();

        /**
         * @brief Calls a function when the digital input goes high.
         * @param callback A reference to a function.
         */
        void      high( void (* callback)(void) );

        /**
         * @brief Calls a function when the digital input goes low.
         * @param callback A reference to a function.
         */
        void      low( void (* callback)(void) );

        operator int();
        operator bool();

        // digital in needs inversion
        // ADI inverts digital inputs so switch press is seen as high
        mevent  &HIGH     = _LOW;
        mevent  &LOW      = _HIGH;
  };
  /**
    * @brief Use this class when programming with a digital-out device.
  */
  class digital_out : private __tridevice {
      public:

        /**
        * @brief Creates a new digital_out object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        digital_out( triport::port &port );
        ~digital_out();
        
        /**
         * @brief Gets the value of the digital_out device.
         * @return Returns an integer that represents the value of the digital-out device.
         */
        int32_t   value();

        /**
         * @brief Sets the digital_out device to a Boolean value.
         * @param value A true or false Boolean value.
         */
        void      set( bool value );
        operator int();
        operator bool();

        /**
         * @brief Sets the output value of the digital_out device.
         * @param value The value to set the digital_out device. Typically 1 or 0.
         */
        void operator=( const int32_t value );
  };

  /**
    * @brief Use this class when programming with a led device.
  */
  class led : public digital_out {
      public:

        /**
        * @brief Creates a new led object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        led( triport::port &port );
        ~led();
        
        /**
         * @brief Sets the led device to the on state.
         */
        void      on();

        /**
         * @brief Sets the led device to the off state.
         */
        void      off();
  };

  /**
    * @brief Use this class when programming with a pneumatics device.
  */
  class pneumatics : public digital_out {
      public:

        /**
        * @brief Creates a new pneumatics object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        pneumatics( triport::port &port );
        ~pneumatics();
        
        /**
         * @brief Sets the pneumatics device to the solenoid open state allowing air to flow into the cylinder.
         */
        void      open();

        /**
         * @brief Sets the pneumatics device to the solenoid close state stopping air flowing into the cylinder.
         */
        void      close();
  };
    
  // Analog sensors
  /** 
    * @brief Use this class when programming with a potentiometer.
  */
  class pot : private __tridevice {
      public:

        /**
        * @brief Creates a new pot object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        pot( triport::port &port );
        ~pot();
        
        /**
         * @brief Gets the value of the potentiometer in the units defined in the parameter.
         * @return Returns an integer that represents the unit value specified by the parameter of the potentiometer device.
         * @param units The measurement unit for the potentiometer device. 
         */
        int32_t   value( analogUnits units );

        /**
         * @brief Gets the value of the potentiometer in the units defined in the parameter.
         * @return Returns an integer that represents the unit value specified by the parameter of the potentiometer device.
         * @param units The measurement unit for the potentiometer device. 
         */
        int32_t   value( percentUnits units ) {
          return value( analogUnits::pct );
        };

        int32_t   angle( percentUnits units = percentUnits::pct ) {
          return value( units );
        };

        /**
         * @brief Gets the value of the potentiometer in the units defined in the parameter.
         * @return Returns an integer that represents the unit value specified by the parameter of the potentiometer device.
         * @param units The measurement unit for the potentiometer device.
         */
        double    value( rotationUnits units );

        double    angle( rotationUnits units ) {
          return value( units );
        };

        /**
         * @brief Calls a function when the potentiometer value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        mevent &CHANGED   = _CHANGED;
  };
  /** 
    * @brief Use this class when programming with a continuous rotation potentiometer.
  */
  class potV2 : private __tridevice {
      public:

       /**
        * @brief Creates a new pot object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        potV2( triport::port &port );
        ~potV2();
        
        /**
         * @brief Gets the value of the potentiometer in the units defined in the parameter.
         * @return Returns an integer that represents the unit value specified by the parameter of the potentiometer device.
         * @param units The measurement unit for the potentiometer device. 
         */
        int32_t   value( analogUnits units );

        /**
         * @brief Gets the value of the potentiometer in the units defined in the parameter.
         * @return Returns an integer that represents the unit value specified by the parameter of the potentiometer device.
         * @param units The measurement unit for the potentiometer device. 
         */
        int32_t   value( percentUnits units ) {
          return value( analogUnits::pct );
        };

        int32_t   angle( percentUnits units = percentUnits::pct ) {
          return value( units );
        };

        /**
         * @brief Gets the value of the potentiometer in the units defined in the parameter.
         * @return Returns an integer that represents the unit value specified by the parameter of the potentiometer device.
         * @param units The measurement unit for the potentiometer device.
         */
        double    value( rotationUnits units );

        double    angle( rotationUnits units ) {
          return value( units );
        };

        /**
         * @brief Calls a function when the potentiometer value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        mevent &CHANGED   = _CHANGED;
  };
  /** 
    * @brief Use this class when programming with a line follower.
  */  
  class line : private __tridevice {
      public:

        /**
        * @brief Creates a new line object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        line( triport::port &port );
        ~line();
        /**
         * @brief Gets the value of the line follower.
         * @return Returns an integer that represents the unit value specified by the parameter of the line follower.
         * @param units The measurement unit for the line device. 
         */
        int32_t   value( analogUnits units );

        int32_t   reflectivity( percentUnits units = percentUnits::pct );
        
        /**
         * @brief Gets the value of the line follower.
         * @return Returns an integer that represents the unit value specified by the parameter of the line follower.
         * @param units The measurement unit for the line device. 
         */
        int32_t   value( percentUnits units ) {
          return value( analogUnits::pct );
        };

        /**
         * @brief Calls a function when the line sensor value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        mevent &CHANGED   = _CHANGED;
  };
  /** 
    * @brief Use this class when programming with an object detector.
  */  
  class objectdetector : private __tridevice {
      private:
        uint32_t _magic;
        bool    _reverse_det;
        bool    _eventAdded;
        void (* _callback_up)(void) ;
        void (* _callback_dn)(void) ;

        static void _threshold_cb( void *arg );

      public:

        /**
         * @brief Creates a new objectdetector on the port specified in the parameter.
         * @param port A reference to a three wire port.
         * @param bReverse A optional boolean to reverse detections, set true to reverse
        */
        objectdetector( triport::port &port, bool bReverse = false );
        ~objectdetector();

        /**
         * @brief Gets the raw value of the objectdetector.
         * @return Returns an integer that represents the unit value specified by the parameter of the objectdetector.
         * @param units The measurement unit for the objectdetector. 
         */
        int32_t   value( analogUnits units );

        /**
         * @brief Gets the reflectivity of the objectdetector.
         * @return Returns an integer that represents the reflectivity value of the objectdetector in the range 0 to 100%
         * @param units The measurement unit for the objectdetector. 
         */
        int32_t   reflectivity( percentUnits units = percentUnits::pct );
        
        /**
         * @brief Gets the value of the objectdetector
         * @return Returns an integer that represents the unit value specified by the parameter of the line follower.
         * @param units The measurement unit for the line device. 
         */
        int32_t   value( percentUnits units = percentUnits::pct ) {
          return value( static_cast<analogUnits>(units) );
        };

        /**
         * @brief Sets the detection threshold for the objectdetector based on reflectivity.
         * @param threshold The threshold for detection of objects.
         * @param units The measurement unit for the threshold value (percent only)
         */
        void      setDetectionThreshold( int32_t threshold, percentUnits units = percentUnits::pct );

        /**
         * @brief Gets the detection status of the objectdetector
         * @return Returns true if an object is currently detected.
         */
        bool      isObjectDetected(void);

        /**
         * @brief Calls a function when the objectdetect raw value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        /**
         * @brief Calls a function when an object is detected.
         * @param callback A reference to a function.
         */
        void      objectLost( void (* callback)(void) );

        /**
         * @brief Calls a function when an object is lost.
         * @param callback A reference to a function.
         */
        void      objectDetected( void (* callback)(void) );
  };



  /**
    * @brief Use this class when programming with a light sensor.
  */  
  class light : private __tridevice {
      public:

        /**
        * @brief Creates a new light object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        light( triport::port &port );
        ~light();

        /**
         * @brief Gets the value of the light sensor.
         * @return Returns an integer that represents the unit value specified by the parameter of the light sensor.
         * @param units The measurement unit for the light device.
         */
        int32_t   value( analogUnits units );

        int32_t   brightness( percentUnits units = percentUnits::pct );

        /**
         * @brief Gets the value of the light sensor.
         * @return Returns an integer that represents the unit value specified by the parameter of the light sensor.
         * @param units The measurement unit for the light device.
         */
        int32_t   value( percentUnits units ) {
          return value( analogUnits::pct );
        };

        /**
         * @brief Calls a function when the light sensor value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        mevent &CHANGED   = _CHANGED;
  };
  /** 
    * @brief Use this class when programming with a gyro sensor.
  */
  class gyro : private __tridevice, public guido {
      private:
        int32_t         _offset_h;
        int32_t         _offset_r;
        uint32_t        _cal_delay;
          
      public:

        /**
        * @brief Creates a new gyro object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        gyro( triport::port &port );
        ~gyro();
        
        /**
         * @brief Gets the value of the gyro sensor.
         * @return Returns an integer that represents the unit value specified by the parameter of the gyro sensor.
         * @param units The measurement unit for the gyro device.
         */
        int32_t   value( analogUnits units );

        /**
         * @brief Gets the value of the gyro sensor.
         * @return Returns an integer that represents the unit value specified by the parameter of the gyro sensor.
         * @param units The measurement unit for the gyro device.
         */
        double    value( rotationUnits units );

        /**
         * @brief Gets the value of the gyro sensor.
         * @return Returns an integer that represents the unit value specified by the parameter of the gyro sensor.
         * @param units The measurement unit for the gyro device.
         */
        int32_t   value( percentUnits units ) {
          return value( analogUnits::pct );
        };

        /**
         * @brief Starts recalibration of the gyro.
         * @param value (Optional) Sets the amount of calibration time. By default, this parameter is zero.
         */
        void      startCalibration( int32_t value = 0 );

        void      calibrate( int32_t value = 0 ) {
          startCalibration( value );
        }

        /**
         * @brief Returns true while the gyro sensor is performing a requested recalibration, changing to false once recalibration has completed.
         * @return Returns true if gyro is still calibrating.
         */
        bool      isCalibrating();
                
        /**
         * @brief Calls a function when the gyro sensor value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        /**
         * @brief reset the gyro sensor angle to 0
         */
        void      resetAngle();

        /**
         * @brief reset the gyro sensor angle to 0
         */
        void      resetHeading();

        /**
         * @brief reset the gyro sensor absolute angle to 0
         */
        void      resetRotation();
        
        /**
         * @brief reset the gyro sensor angle to angle
         * @param value The new angle for the gyro
         * @param units The rotation unit for the angle
         */
        void      resetAngle( double value, rotationUnits units );

        /**
         * @brief reset the gyro sensor angle to angle
         * @param value The new heading for the gyro
         * @param units The rotation unit for the angle
         */
        void      setHeading( double value, rotationUnits units );

        /**
         * @brief Gets the angle of the gyro sensor.
         * @return Returns a double that represents the unit value specified by the parameter of the gyro sensor.
         * @param units The measurement unit for the gyro device.
         */
        double    angle( rotationUnits units = rotationUnits::deg );

        /**
         * @brief Gets the angle of the gyro sensor.
         * @return Returns a double that represents the unit value specified by the parameter of the gyro sensor.
         * @param units The measurement unit for the gyro device.
         */
        double    heading( rotationUnits units = rotationUnits::deg );

        /**
         * @brief Gets the rotation (absolute angle) of the gyro sensor.
         * @return Returns a double that represents the unit value specified by the parameter of the gyro sensor.
         * @param units The measurement unit for the gyro device.
         */
        double    rotation( rotationUnits units = rotationUnits::deg );

        /**
         * @brief set the gyro sensor rotation to angle
         * @param value The new absolute angle for the gyro
         * @param units The rotation unit for the angle
         */
        void      setRotation( double value, rotationUnits units );

        /**
         * @brief Get direction for positive angles
         * @return turnType::right or turnType::left
         */
        turnType        getTurnType();

        mevent &CHANGED   = _CHANGED;
  };
  /** 
    * @brief Use this class when programming with an accelerometer sensor.
  */  
  class accelerometer : private __tridevice {
      public:

        /**
        * @brief Creates a new accelerometer object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        accelerometer( triport::port &port, bool bHighSensitivity = false  );
        ~accelerometer();
        /**
         * @brief Gets the value of the accelerometer.
         * @return Returns an integer that represents the unit value specified by the parameter of the accelerometer sensor.
         * @param units The measurement unit for the accelerometer device.
         */
        int32_t   value( analogUnits units );

        /**
         * @brief Gets the value of the accelerometer.
         * @return Returns an integer that represents the unit value specified by the parameter of the accelerometer sensor.
         * @param units The measurement unit for the accelerometer device. 
         */
        int32_t   value( percentUnits units ) {
          return value( analogUnits::pct );
        };

        /**
         * @brief Gets the value of the accelerometer in G.
         * @return Returns a double that represents the acceleration in G of the accelerometer sensor.
         */
        double    acceleration();
        
        /**
         * @brief Calls a function when the accelerometer value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        mevent &CHANGED   = _CHANGED;
        
        private:
          bool    highSensitivity;
  };
  /** 
    * @brief Use this class when programming with an analog-in device.
  */    
  class analog_in : private __tridevice {
      public:

        /**
        * @brief Creates a new analog-in object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        analog_in( triport::port &port );
        ~analog_in();
        
        /**
         * @brief Gets the value of the analog-in device.
         * @return Returns an integer that represents the unit value specified by the parameter of the analog-in device.
         * @param units The measurement unit for the analog-in device. 
         */
        int32_t   value( analogUnits units );
        /**
         * @brief Gets the value of the analog-in device.
         * @return Returns an integer that represents the unit value specified by the parameter of the analog-in device.
         * @param units The measurement unit for the analog-in device.
         */
        int32_t   value( percentUnits units ) {
          return value( analogUnits::pct );
        };

        /**
         * @brief Calls a function when the analog input value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        mevent &CHANGED   = _CHANGED;
  };  
  /** 
    * @brief Use this class when programming with an encoder.
  */   
  class encoder : private __tridevice {
      public:

        /**
        * @brief Creates a new encoder object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        encoder( triport::port &port );
        ~encoder();
        
        /**
         * @brief Gets the value of the encoder.
         * @return Returns an integer that represents the value of the encoder device.
         */
        int32_t   value();
        operator int();
        
         /**
           * @brief Resets the rotation of the encoder to zero.
           */
        void    resetRotation( void );

        /**
          * @brief Sets the rotation of the encoder to a specific value.
          * @param val Sets the amount of rotation.
          * @param units The measurement unit for the rotation value.
          */
        void    setRotation( double val, rotationUnits units );

        void    setPosition( double val, rotationUnits units );

        /**
         * @brief Gets the rotation value of the encoder.
         * @return Returns a double that represents the current rotation of the encoder.
         * @param units The measurement unit for the encoder device.
         */
        double  rotation( rotationUnits units );

        double  position( rotationUnits units );

        /**
         * @brief Gets the velocity of the encoder.
         * @return Returns a double that represents the current velocity of the encoder.
         * @param units The measurement unit for the encoder device.
         */
        double  velocity( velocityUnits units );

        /**
         * @brief Calls a function when the encoder value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        mevent &CHANGED   = _CHANGED;
  };
  /** 
    * @brief Use this class when programming with a sonar sensor.
  */     
  class sonar : private __tridevice {
      public:

        /**
        * @brief Creates a new sonar object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        sonar( triport::port &port );
        ~sonar();
        
        /**
         * @brief Gets the value of the distance sensor.
         * @return Returns an integer that represents the value of the sonar sensor.  
         */
        int32_t   value();
        operator int();

        /**
         * @brief Gets the value of the sonar sensor.
         * @return Returns an integer that represents the unit value specified by the parameter of the sonar sensor. 
         * @param units The measurement unit for the sonar device.
         */
        double  distance( distanceUnits units );

        /**
         * @brief Calls a function when the sonar value changes.
         * @param callback A reference to a function.
         */
        void      changed( void (* callback)(void) );

        mevent &CHANGED   = _CHANGED;        

        // settings
        void      setMaximum( double distance, distanceUnits units );
        /**
         * @brief Gets an object is present in front of the sonar sensor
         * @return Returns a boolean that will be true if the sonar sensor is detecting an object.
         */
        bool      foundObject();    
        
      private:
        uint32_t  _maxdistance;
        uint32_t  _mindistance;

        void      setMinimum( double distance, distanceUnits units );
  };  
  /** 
    * @brief Use this class when programming with a pwm-out device.
  */
  class pwm_out : private __tridevice {
      public:

        /**
        * @brief Creates a new pwm-out object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        pwm_out( triport::port &port );
        ~pwm_out();
        
        /**
         * @brief Sets the state of the pwm device.
         * @param value Sets the length of the pulse width.
         * @param units The measurement unit for the state value.
         */
        void state( int32_t value, percentUnits units );
  };
  /** 
    * @brief Use this class when programming with a servo device.
  */
  class servo : private __tridevice {
      public:

        /**
        * @brief Creates a new servo object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        servo( triport::port &port );
        ~servo();
        
        /**
         * @brief Sets the position of the servo device.
         * @param value Sets the amount of rotation.
         * @param units The measurement unit for the rotation value.
         */
        void setPosition( int32_t value, percentUnits units );

        /**
         * @brief Sets the position of the servo device.
         * @param value Sets the amount of rotation.
         * @param units The measurement unit for the rotation value.
         */
        void setPosition( double value, rotationUnits units );
    
        /**
        * @brief Sets the position of the servo device.
        * @param value Sets the amount of rotation.
        * @param units The measurement unit for the rotation value.
        */
        void startRotateTo( int32_t value, percentUnits units ){
          setPosition(value, units);
        }

        /**
        * @brief Sets the position of the servo device.
        * @param value Sets the amount of rotation.
        * @param units The measurement unit for the rotation value.
        */
        void startRotateTo( double value, rotationUnits units ){
          setPosition(value, units);
        }
  };
  /** 
    * @brief Use this class when programming a motor connected to a motor controller 29.
  */
  class motor29 : private __tridevice {
      private:
        bool    _reversed;
        int32_t _velocity;
        bool    _spinMode;

      public:

        /**
        * @brief Creates a new motor29 object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        motor29( triport::port &port );

        /**
        * @brief Creates a new motor29 object on the port specified in the parameter.
        * @param port A reference to a three wire port.
        * @param reverse Sets the reverse flag for the new motor29 object.
       */
        motor29( triport::port &port, bool reverse );
        ~motor29();
        
        /**
         * @brief Sets the velocity of the motor based on the parameters set in the command. This command will not run the motor.  Any subsequent call that does not contain a specified motor velocity will use this value.
         * @param velocity Sets the amount of velocity.
         * @param units The measurement unit of the velocity value. 
         */
        void setVelocity( double velocity, percentUnits units );

        /** 
         * @brief Sets the motor mode to "reverse", which will make motor commands spin the motor in the opposite direction.
         * @param value If set to true, the motor will spin in the reversed direction.
         */
        void setReversed( bool value );
        
        /** 
         * @brief Turns the motor on, and spins it in the specified direction.
         * @param dir The direction to spin the motor.
         */
        void spin( directionType dir );

        /**
         * @brief Turns on the motor and spins it in a specified direction and a specified velocity.
         * @param dir The direction to spin the motor. 
         * @param velocity Sets the amount of velocity.
         * @param units The measurement unit of the velocity value. 
        */
        void spin( directionType dir, double velocity, velocityUnits units );

        void spin( directionType dir, double velocity, percentUnits units ){
            spin( dir, velocity, static_cast<velocityUnits>(units) );
        };
        
        /** 
         * @brief Stops the motor using the default brake mode.
         */
        void stop( void );
  };
  /** 
    * @brief Use this class when programming with a motor connected to a victor motor controller.
  */
  class motor_victor : private __tridevice {
      private:
        bool    _reversed;
        int32_t _velocity;
        bool    _spinMode;

      public:

        /**
        * @brief Creates a new motor victor object on the port specified in the parameter.
        * @param port A reference to a three wire port.
       */
        motor_victor( triport::port &port );

        /**
        * @brief Creates a new motor victor object on the port specified in the parameter.
        * @param port A reference to a three wire port.
        * @param reverse Sets the reverse flag for the new motor_victor object.
       */
        motor_victor( triport::port &port, bool reverse );
        ~motor_victor();
        
        /**
         * @brief Sets the velocity of the motor based on the parameters set in the command. This command will not run the motor.  Any subsequent call that does not contain a specified motor velocity will use this value.
         * @param velocity Sets the amount of velocity.
         * @param units The measurement unit of the velocity value. 
         */
        void setVelocity( double velocity, percentUnits units );

        /** 
         * @brief Sets the motor mode to "reverse", which will make motor commands spin the motor in the opposite direction.
         * @param value If set to true, the motor will spin in the reversed direction.
         */
        void setReversed( bool value );
        
        /** 
         * @brief Turns the motor on, and spins it in the specified direction.
         * @param dir The direction to spin the motor.
         */
        void spin( directionType dir );

        /**
         * @brief Turns on the motor and spins it in a specified direction and a specified velocity.
         * @param dir The direction to spin the motor. 
         * @param velocity Sets the amount of velocity.
         * @param units The measurement unit of the velocity value. 
        */
        void spin( directionType dir, double velocity, velocityUnits units );

        void spin( directionType dir, double velocity, percentUnits units ){
            spin( dir, velocity, static_cast<velocityUnits>(units) );
        };
        
        /** 
         * @brief Stops the motor using the default brake mode.
         */
        void stop( void );
    };

};

#endif  // VEX_TRIPORT_CLASS_H
