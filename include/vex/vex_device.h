/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_device.h                                                */
/*    Author:     James Pearman                                               */
/*    Created:    10 Oct 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_DEVICE_CLASS_H
#define   VEX_DEVICE_CLASS_H

#include "v5_apiprivate.h"

/*-----------------------------------------------------------------------------*/
/** @file    vex_device.h
  * @brief   Header for device class
*//*---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @brief device class                                                        */
/*-----------------------------------------------------------------------------*/
namespace vex {
  /** @brief Use the device class to get information about a vex device plugged into the V5.
  */
  class device  {
    private:
      V5_DeviceT      _ptr;
  
    protected:
      int32_t         _index;
      int32_t         _threadID;
      int32_t         flags();
      
    public:
      device();
      device( int32_t index );  
      virtual ~device();

      V5_DeviceType   type();
      int32_t         index();
      void            init( int32_t index );
      virtual bool    installed();
      virtual int32_t value();
      uint32_t        timestamp();
  };
};

/*-----------------------------------------------------------------------------*/
/** @brief a safe array template                                               */
/*-----------------------------------------------------------------------------*/

namespace vex {
  template <class T, int len> class safearray {
    private:
      int length;
  
    protected:
      T arr[len];
      T outofbounds;
  
    public:
      safearray(){ length = len; outofbounds = 0;
      };
      ~safearray(){};
  
      T &operator[]( int i );
      int getLength() { return length; };
  };

  template <class T, int len> T &safearray<T, len>::operator[](int i)
  {
    // bounds check the array index
    if( i < 0 || i > (length-1) ) {
      return( outofbounds );
    }
    else {
      return( arr[i] );
    }
  }
};


/*-----------------------------------------------------------------------------*/
/** @brief base class with virtual member functions used with IMU and gyro     */
/*-----------------------------------------------------------------------------*/
//
// This class is never directly used except as a ptr to gyro or imu
//
namespace vex {
    class guido {
      public:
        guido(){};  
        virtual ~guido(){};
      
        // pure virtual methods that must be implemented
        virtual double angle( rotationUnits units = rotationUnits::deg ) = 0;
        virtual double heading( rotationUnits units = rotationUnits::deg ) = 0;
        virtual void   calibrate( int32_t value ) = 0;
        virtual bool   isCalibrating(void) = 0;
        virtual void   setHeading( double value, rotationUnits units ) = 0;
        virtual double rotation( rotationUnits units = rotationUnits::deg ) = 0;
        virtual void   setRotation( double value, rotationUnits units ) = 0;
        virtual turnType getTurnType( void ) = 0;
    };
};

/*-----------------------------------------------------------------------------*/
/** @brief All devices class                                                   */
/*-----------------------------------------------------------------------------*/
namespace vex {
  /** @brief Use the device class to get information about all of the vex devices plugged into the V5.
  */
  class devices {
    private:
      safearray<device, V5_MAX_DEVICE_PORTS> data;

    public:
      devices();  
      ~devices();

      device &operator[]( int i ) {
        return( data[i] );
      }
      /**
      * @brief Get the V5 device type plugged into a specific port.      
      * @return Returns a V5 device type.
      * @param index Specifies the index to look at for the device.
      */
      V5_DeviceType   type( int32_t index );

      /**
      * @brief Gets the number of VEX devices that are plugged in.    
      * @return Returns an integer that represent the number of vex devices
      */
      int32_t         number();

      /**
      * @brief Gets the number of specified devices that are plugged into the V5.
      * @return Returns an integer that represents the number of a specific vex devices set by the parameter.
      * @param  type The type of device to look for on the V5.
      */
      int32_t         numberOf( V5_DeviceType type );
  };
};

#endif // VEX_DEVICE_CLASS_H