/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_imu.h                                                   */
/*    Author:     James Pearman                                               */
/*    Created:    3 Oct 2019                                                  */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_IMU_CLASS_H
#define   VEX_IMU_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_imu.h
  * @brief   inertial sensor device class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
   * @brief Use the inertial class to control the inertial sensor.
   */
  class inertial : public device, public guido {
    private:
      double          _offset_h;
      double          _offset_r;
      uint32_t        _cal_delay;
      turnType        _turn_mode;

      double          _scaleRotationValue( double value, rotationUnits units );

      static void     _collisionEventHandler(void *arg);
      void         (* _collisionCallback)( axisType, double, double, double );
      
    public:
      /** 
       * @brief Creates a new inertial object on the port specified.
       * @param index The port index for this inertial. The index is zero-based.
       */
      inertial( int32_t index, turnType dir = turnType::right );  
      ~inertial();
    
      bool            installed();
      int32_t         value();

      /**
       * @brief class to hold an inertial matrix
       * @details
       *  This class wraps a 4x4 matrix that can be used when performing
       *  3d transformations.
       */
      class matrix {
        public:
          matrix();  
          ~matrix();

          double  data[4][4];

          void    transform( double &x, double &y, double &z );
          void    multiply( inertial::matrix &m );
          double  inverse();
      };
      

      /**
       * @brief class to hold inertial quaternion data
       */
      class quaternion {
        public:
          quaternion();  
          quaternion( double a, double b, double c, double d );
          ~quaternion();

          // storage for the quaternion data
          double    a;
          double    b;
          double    c;
          double    d;

          // set equal to another quaternion
          quaternion& operator=(const inertial::quaternion other);

          // Return this quaternion's conjugate.
          quaternion conjugate() const;

          // Return the unit quaternion corresponding to the same rotation as this one.
          quaternion normalized() const;

          // Return the quaternion multiplied by rhs.
          quaternion operator*(const inertial::quaternion& rhs) const;

          // Multiply this quaternion by rhs.
          quaternion& operator*=(const inertial::quaternion& rhs);

          // transform a vector x, y, z using this quaternion
          void        transform( double &x, double &y, double &z );

          // create a rotation quaternion fron an angle in degrees
          static  quaternion  fromAngleX( double angle );
          static  quaternion  fromAngleY( double angle );
          static  quaternion  fromAngleZ( double angle );

          // create a 4x4 matrix representing this quaternions transform
          void        matrix( inertial::matrix &m );         
      };  

      /**
       * @brief class to hold inertial angle data
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
       * @brief Starts recalibration of the inertial sensor.
       */
      void            startCalibration( int32_t value = 0 );

      void            calibrate( int32_t value = 0 ) {
          startCalibration(value);
      }      
      
      /**
       * @brief Returns true while the inertial sensor is performing a requested recalibration, changing to false once recalibration has completed.
       * @return Returns true if inertial sensor is still calibrating.
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
       * @brief set the inertial sensor heading to a new value
       * @param value The new heading for the inertial sensor
       * @param units The rotation unit for the angle
       */
      void            setHeading( double value, rotationUnits units );

      /**
       * @brief set the inertial sensor rotation to angle
       * @param value The new absolute rotation angle for the inertial sensor
       * @param units The rotation unit for the angle
       */
      void            setRotation( double value, rotationUnits units );

      /**
       * @brief Gets the angle (yaw angle) of the inertial sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the inertial sensor.
       * @param units The measurement unit for the inertial device.
       */
      double          angle( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets the roll angle of the inertial sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the inertial sensor.
       * @param units The measurement unit for the inertial sensor.
       */
      double          roll( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets the pitch angle of the inertial sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the inertial sensor.
       * @param units The measurement unit for the inertial device.
       */
      double          pitch( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets the yaw angle of the inertial sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the inertial sensor.
       * @param units The measurement unit for the inertial sensor.
       */
      double          yaw( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets an orientation angle of the inertial sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the inertial sensor.
       * @param axis  The orientation axis to recover, roll, pitch or yaw.
       * @param units The measurement unit for the inertial sensor.
       */
      double          orientation( orientationType axis, rotationUnits units );            

      /**
       * @brief Gets the heading (yaw angle as 0-360 deg) of the inertial sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the inertial sensor.
       * @param units The measurement unit for the inertial sensor.
       */
      double          heading( rotationUnits units = rotationUnits::deg );

      /**
       * @brief Gets the absolute angle (yaw angle without limits) of the inertial sensor.
       * @return Returns a double that represents the unit value specified by the parameter of the inertial sensor.
       * @param units The measurement unit for the inertial sensor.
       */
      double          rotation( rotationUnits units = rotationUnits::deg );
            
      /**
       * @brief Gets the inertial sensor orientation in quaternion form
       * @param q reference to instance of a inertial::quaternion class
       */
      void            orientation( quaternion &q );

      /**
       * @brief Gets the inertial sensor orientation in quaternion form
       * @return inertial::quaternion object
       */
      quaternion      orientation();

      /**
       * @brief Gets the inertial sensor orientation in rotation form
       * @param a reference to instance of a inertial::attitude class
       */
      void            orientation( attitude &a );
      
      /**
       * @brief Gets the inertial sensor raw gyro data in specified units
       * @param axis the inertial sensor axis to use
       * @param units The units return value
       * @return value of the specified inertial sensor axis
       */
      double          gyroRate( axisType axis, velocityUnits units );
      
      /**
       * @brief Gets the inertial sensor raw acceleration data in G
       * @param axis the inertial sensor axis to use
       * @return value of the specified inertial sensor axis
       */
      double          acceleration( axisType axis );

      enum class tEventType {
        EVENT_HEADING_CHANGED     =  0,
        EVENT_COLLISION           =  1,

        // private, not set directly
        EVENT_COLLISION_AXIS_X    =  8,
        EVENT_COLLISION_AXIS_Y    =  9,
        EVENT_COLLISION_AXIS_Z    =  10
      };

      /**
       * @brief Calls a function when the inertial sensor heading value changes.
       * @param callback A reference to a function.
       */
      void            changed( void (* callback)(void) );

      /**
       * @brief Calls a function when the inertial sensor detects a collision
       * @param callback A reference to a function.
       */
      void            collision( void (* callback)(axisType, double, double, double) );

      //mevent  CHANGED   = { (uint32_t)_index, ((uint32_t)tEventType::EVENT_HEADING_CHANGED) };
      //mevent  COLLISION = { (uint32_t)_index, ((uint32_t)tEventType::EVENT_COLLISION) };
      
    protected:
      /**
       * @brief recover inertial sensor raw status
       */
      int32_t         status();

      /**
       * @brief recover inertial sensor temperature in deg C
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

      //
      // Most of these are special application.  sub class needs to be used to access them.
      //
      
      /**
       * @brief class to hold inertial sensor gyro data
       */
      class gyro {
        friend  vex::inertial;
        
        private:
          double    _gx;
          double    _gy;
          double    _gz;
          
        public:
          gyro();  
          ~gyro();

          // set equal to another gyro
          gyro& operator=(const inertial::gyro other);

          double  & gx = _gx;
          double  & gy = _gy;
          double  & gz = _gz;
      };

      /**
       * @brief class to hold inertial sensor acceleration data
       */
      class accel {
        friend  vex::inertial;
        
        private:
          double    _ax;
          double    _ay;
          double    _az;
          
        public:
          accel();  
          ~accel();

          // set equal to another gyro
          accel& operator=(const inertial::accel other);

          double  & ax = _ax;
          double  & ay = _ay;
          double  & az = _az;
      };

      /**
       * @brief Gets the inertial sensor raw gyro data in dps
       * @return inertial::gyro object
       */
      inertial::gyro  gyroRate();

      /**
       * @brief Gets the inertial sensor raw acceleration data in G
       * @return inertial::accel object
       */
      inertial::accel acceleration();

      enum class orientationMode {
          ZUp, ZDown, XUp, XDown, YUp, YDown, Auto
      };

      /**
       * @brief Set the physical orientation of the inertial sensor, this will cause recalibration
       * @param orientation the inertial sensor orientation
       */
      void            setOrientation( inertial::orientationMode orientation );
      
      // special application - do not use unless requested
      void            setAccelerometerRange( uint8_t value );         
      void            setCollisionThreshold( double value );   
  };
};

#endif // VEX_IMU_CLASS_H