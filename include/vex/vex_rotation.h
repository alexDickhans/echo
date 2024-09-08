/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_rotation.h                                              */
/*    Author:     James Pearman                                               */
/*    Created:    3 Oct 2019                                                  */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_ROTATION_CLASS_H
#define   VEX_ROTATION_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_rotation.h
  * @brief   rotation sensor device class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
    * @brief Use the rotation class to control absolute encoder devices.
  */
  class rotation : public device {
    public:
      /** 
       * @brief Creates a new rotation object on the port specified.
       * @param index The port index for this rotation sensor. The index is zero-based.
       */
      rotation( int32_t index, bool reverse = false );  
      ~rotation();
    
      bool            installed();
      int32_t         value();

      /** 
       * @brief Sets the encoder mode to "reverse", position and angle will change in the opposite direction.
       * @param value If set to true, the encoder counts in reverse
       */
      void            setReversed( bool value );

      /**
       * @brief Gets the angle of the absolute encoder.
       * @return Returns a double that represents the unit value specified by the parameter of the encoder sensor.
       * @param units The measurement unit for the encoder device.
       */
      double          angle( rotationUnits units = rotationUnits::deg );

      /** 
       * @brief Resets the position of the absolute encoder to the value of zero. 
       */   
      void            resetPosition( void );

      /** 
       * @brief Sets the value of the absolute encoder to the value specified in the parameter.
       * @param value Sets the current position of the absolute encoder.
       * @param units The measurement unit for the position value.
       */
      void            setPosition( double value, rotationUnits units );
        
      /** 
       * @brief Gets the current position of the absolute encoder.
       * @returns Returns a double that represents the current position of the encoder in the units defined in the parameter.
       * @param units The measurement unit for the position.
       */
      double          position( rotationUnits units );

       /** 
       * @brief Gets the current velocity of the absolute encoder.
       * @return Returns a double that represents the current velocity of the encoder in the units defined in the parameter.
       * @param units The measurement unit for the velocity.
       */
      double          velocity( velocityUnits units );

      enum class tEventType {
        EVENT_ANGLE_CHANGED     =  0
      };

      /**
       * @brief Calls a function when the encoder value changes.
       * @param callback A reference to a function.
       */
      void      changed( void (* callback)(void) );

      //mevent  CHANGED   = { (uint32_t)_index, ((uint32_t)tEventType::EVENT_ANGLE_CHANGED) };
      
    protected:
      /**
       * @brief recover abs encoder raw status
       */
      int32_t         status();
      
      /**
       * @brief recover abs encoder temperature in deg C
       */
      double          temperature();

      /**
       * @brief change data rate to user specified value, will be rounded to multiple of 5mS
       */
      void            datarate( uint32_t rate );
  };
};

#endif // VEX_ROTATION_CLASS_H