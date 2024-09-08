/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2021, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_gps.h                                                   */
/*    Author:     James Pearman                                               */
/*    Created:    24 March 2021                                               */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_GPS_CLASS_H
#define   VEX_GPS_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_gps.h
  * @brief   gps sensor device class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
   * @brief Use the gps class to control the gps sensor.
   */
  class gps : public device, public guido {
    private:
      double          _offset_h;
      double          _offset_r;
      uint32_t        _cal_delay;
      turnType        _turn_mode;
      int32_t         _signal_loss_ctr;
      
      double          _scaleRotationValue( double value, rotationUnits units );

      /**
       * @brief Gets the angle (yaw angle) of the gps sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the gps sensor.
       * @param units The measurement unit for the gps device.
       */
      double          angle( rotationUnits units = rotationUnits::deg );
      
    public:
      /** 
       * @brief Creates a new gps object on the port specified.
       * @param index The port index for this gps. The index is zero-based.
       * @param heading_offset The gps sensor rotational offset.
       */
      gps( int32_t index, double heading_offset = 0, turnType dir = turnType::right );

      /** 
       * @brief Creates a new gps object on the port specified.
       * @param index The port index for this gps. The index is zero-based.
       * @param ox the x position for the origin
       * @param oy the y position for the origin
       * @param units The units that ox and oy are using
       * @param heading_offset The gps sensor rotational offset.
       */
      gps( int32_t index, double ox, double oy, distanceUnits units = distanceUnits::mm, double heading_offset = 0, turnType dir = turnType::right );  
      ~gps();
    
      bool            installed();
      int32_t         value();

      /**
       * @brief class to hold gps angle data (proposed)
       * @details 
       *  data can be access either as roll, pitch and yaw or x, y and z
       *  data is stored in degrees.
       */
      class attitude {
        public:
          attitude();  
          attitude( double roll, double pitch, double yaw );
          ~attitude();

          double    roll;
          double    pitch;
          double    yaw;
          
          double  & x = roll;
          double  & y = pitch;
          double  & z = yaw;

          void    transform( double &x, double &y, double &z );
      };  

      /**
       * @brief Starts recalibration of the gps sensor.
       */
      void            startCalibration( int32_t value = 0 );

      void            calibrate( int32_t value = 0 ) {
          startCalibration(value);
      }      
      
      /**
       * @brief Returns true while the gps sensor is performing a requested recalibration, changing to false once recalibration has completed.
       * @return Returns true if gps sensor is still calibrating.
       */
      bool            isCalibrating(void);

      /**
       * @brief reset the heading of the sensor to 0
       */
      void            resetHeading();

      /**
       * @brief reset the rotation angle of the sensor to 0
       */
      void            resetRotation();

      /**
       * @brief set the gps sensor heading to a new value
       * @param value The new heading for the gps sensor
       * @param units The rotation unit for the angle
       */
      void            setHeading( double value, rotationUnits units );

      /**
       * @brief set the gps sensor rotation to angle
       * @param value The new absolute rotation angle for the gps sensor
       * @param units The rotation unit for the angle
       */
      void            setRotation( double value, rotationUnits units );

      /**
       * @brief Gets the roll angle of the gps sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the gps sensor.
       * @param units The measurement unit for the gps sensor.
       */
      double          roll( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets the pitch angle of the gps sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the gps sensor.
       * @param units The measurement unit for the gps device.
       */
      double          pitch( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets the yaw angle of the gps sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the gps sensor.
       * @param units The measurement unit for the gps sensor.
       */
      double          yaw( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets the x position of the gps sensor.
       * @return Returns a double that x positional offset from the center of the competition field.
       */
      double          xPosition( distanceUnits units = distanceUnits::mm );

      /**
       * @brief Gets the y position of the gps sensor.
       * @return Returns a double that y positional offset from the center of the competition field.
       */
      double          yPosition( distanceUnits units = distanceUnits::mm );
      
      /**
       * @brief Gets an orientation angle of the gps sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the gps sensor.
       * @param axis  The orientation axis to recover, roll, pitch or yaw.
       * @param units The measurement unit for the gps sensor.
       */
      double          orientation( orientationType axis, rotationUnits units );            

      /**
       * @brief Gets the heading (yaw angle as 0-360 deg) of the gps sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the gps sensor.
       * @param units The measurement unit for the gps sensor.
       */
      double          heading( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets the absolute angle (yaw angle without limits) of the gps sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the gps sensor.
       * @param units The measurement unit for the gps sensor.
       */
      double          rotation( rotationUnits units = rotationUnits::deg );
            
      /**
       * @brief Gets the gps sensor orientation in rotation form
       * @param a reference to instance of a gps::attitude class
       */
      void            orientation( attitude &a );
      
      /**
       * @brief Gets the gps sensor raw gyro data in specified units
       * @param axis the gps sensor axis to use
       * @param units The units return value
       * @return value of the specified gps sensor axis
       */
      double          gyroRate( axisType axis, velocityUnits units );
      
      /**
       * @brief Gets the gps sensor raw acceleration data in G
       * @param axis the gps sensor axis to use
       * @return value of the specified gps sensor axis
       */
      double          acceleration( axisType axis );

      /**
       * @brief return the perceived signal quality
       * @return signal quality as percent
       */
      int32_t         quality();
      
      /**
       * @brief set the origin to be at the indicated x and y position
       * @param x the x position for the origin
       * @param y the y position for the origin
       * @param units The units that x and y are using
       */
      void            setOrigin( double x, double y, distanceUnits units = distanceUnits::mm );
      
      /**
       * @brief set the origin to the current position
       */
      void            setOrigin();

      /**
       * @brief set the current gps location to be at the indicated x and y position and angle
       * @param x the x position for the current location
       * @param y the y position for the current location
       * @param units The units that x and y are using
       * @param angle the gps heading current location
       * @param units_r The units that angle is using
       */
      void            setLocation( double x, double y, distanceUnits units = distanceUnits::mm, double angle = 0, rotationUnits units_r = rotationUnits::deg );

      /**
       * @brief set the current gps location to be at the indicated x and y position and angle
       * @param x the x position for the current location in mm
       * @param y the y position for the current location in mm
       * @param angle the gps heading current location in degrees
       */
      void            setLocation( double x, double y, double angle );

      enum class tEventType {
        EVENT_HEADING_CHANGED     =  0
      };

      /**
       * @brief Calls a function when the gps sensor heading value changes.
       * @param callback A reference to a function.
       */
      void            changed( void (* callback)(void) );

      //mevent  CHANGED   = { (uint32_t)_index, ((uint32_t)tEventType::EVENT_HEADING_CHANGED) };
      
    protected:
      /**
       * @brief recover gps sensor raw status
       */
      int32_t         status();

      /**
       * @brief recover gps sensor temperature in deg C
       */
      double          temperature();
      
      /**
       * @brief change data rate to user specified value, will be rounded to multiple of 5mS
       */
      void            datarate( uint32_t rate );

      /**
       * @brief Set direction for positive angles
       * @param dir turnType::right or turnType::left
       */
      void            setTurnType( turnType dir );

      /**
       * @brief Get direction for positive angles
       * @return turnType::right or turnType::left
       */
      turnType        getTurnType();

      /**
       * @brief Gets the gps sensor orientation in quaternion form
       * @param q reference to instance of a inertial::quaternion class
       */
      void            orientation( inertial::quaternion &q );

      /**
       * @brief Gets the gps sensor orientation in quaternion form
       * @return inertial::quaternion object
       */
      inertial::quaternion orientation();
      
      /**
       * @brief force bypass mode, this causes location data to be ignored and the unit run as an IMU
       */
      void            bypass( bool value = true );
      
      /**
       * @brief set the sensor rotation offset.
       */
      void            setSensorRotation( double value );
      
      //
      // Most of these are special application.  sub class needs to be used to access them.
      //
      
      /**
       * @brief class to hold gps sensor gyro data
       */
      class gyro {
        friend  vex::gps;
        
        private:
          double    _gx;
          double    _gy;
          double    _gz;
          
        public:
          gyro();  
          ~gyro();

          // set equal to another gyro
          gyro& operator=(const gps::gyro other);

          double  & gx = _gx;
          double  & gy = _gy;
          double  & gz = _gz;
      };

      /**
       * @brief class to hold gps sensor acceleration data
       */
      class accel {
        friend  vex::gps;
        
        private:
          double    _ax;
          double    _ay;
          double    _az;
          
        public:
          accel();  
          ~accel();

          // set equal to another gyro
          accel& operator=(const gps::accel other);

          double  & ax = _ax;
          double  & ay = _ay;
          double  & az = _az;
      };

      /**
       * @brief Gets the gps sensor raw gyro data in dps
       * @return gps::gyro object
       */
      gps::gyro       gyroRate();

      /**
       * @brief Gets the gps sensor raw acceleration data in G
       * @return gps::accel object
       */
      gps::accel      acceleration();
      
      // special application - do not use unless requested
      void            setAccelerometerRange( uint8_t value );         
      void            setCollisionThreshold( double value );   
  };
};

#endif // VEX_IMU_CLASS_H