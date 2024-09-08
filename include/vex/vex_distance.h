/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_distance.h                                              */
/*    Author:     James Pearman                                               */
/*    Created:    26 May 2020                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_DISTANCE_CLASS_H
#define   VEX_DISTANCE_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_distance.h
  * @brief   Distance sensor device class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
   * @brief Use the distance class to control the distance sensor.
   */
  class distance : public device {
    private:
      enum class tEventType {
        EVENT_DISTANCE_CHANGED     =  0,
      };
      
    public:
      /** 
       * @brief Creates a new distance sensor object on the port specified.
       * @param index The port index for this sensor. The index is zero-based.
       */
      distance( int32_t index );  
      ~distance();
    
      bool            installed();
      int32_t         value();

      
      /**
       * @brief Gets the detected object distance
       * @return Returns a double that represents the unit value specified by the parameter of the distance sensor. 
       * @param units The measurement unit for the distance sensor.
       */
      double          objectDistance( distanceUnits units );

      /**
       * @brief Gets an estimate of the detected object size
       * @return Returns the estimated object size
       */
      sizeType        objectSize( void );

      /**
       * @brief Gets an estimate of the detected object size
       * @return Returns a integer that represent a relative object size, 0 to 400 
       */
      int32_t         objectRawSize( void );

      /**
       * @brief Gets an estimate of the detected object velocity
       * @return Returns a double with object velocity in m/s
       */
      double          objectVelocity( void );

      /**
       * @brief check for object in range of the distance sensor
       * @return Returns true if an object is detected
       */
      bool            isObjectDetected( void );
      
      /**
       * @brief Calls a function when the distance value changes.
       * @param callback A reference to a function.
       */
      void            changed( void (* callback)(void) );

  };
};

#endif // VEX_DISTANCE_CLASS_H