/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_optical.h                                               */
/*    Author:     James Pearman                                               */
/*    Created:    26 May 2020                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_OPTICAL_CLASS_H
#define   VEX_OPTICAL_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_optical.h
  * @brief   Optical (color) sensor device class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
   * @brief Use the optical class to control the optical sensor.
   */
  class optical : public device {
    private:
      int32_t   _led_intensity;
      int32_t   _prox_threshold;
      
      enum class tEventType {
        EVENT_OBJECT_FOUND =  0,
        EVENT_OBJECT_LOST  =  1,
        EVENT_GESTURE_UP   =  2,
        EVENT_GESTURE_DOWN  =  3,
        EVENT_GESTURE_LEFT  =  4,
        EVENT_GESTURE_RIGHT =  5,
      };
            
    public:
      /** 
       * @brief Creates a new optical sensor object on the port specified.
       * @param index The port index for this sensor. The index is zero-based.
       * @param enableGesture enable gesture detection
       */
      optical( int32_t index, bool enableGesture = false );  
      ~optical();
    
      bool            installed();
      int32_t         value();

      /**
       * @brief Gets the hue detected by the optical sensor.
       * @return Returns an integer that represents the hue the optical sensor has detected.
       */
      double          hue();

      /**
       * @brief Gets the brightness value detected by the optical sensor.
       * @return Returns an integer that represents the brightness value the optical sensor has detected.
       */
      double          brightness( bool bRaw = false );

      /**
       * @brief Gets the detected color as a fully saturated vex::color
       * @return Returns a vex::color object that represents the hue of the closest color detected to twelve possible values.
       */
      vex::color      color();

      /**
       * @brief Check to see if an object is detected by the optical sensor.
       * @return Returns a boolean that represents whether an object has been detected.
       */
      bool            isNearObject();

      /**
       * @brief Calls a function when the optical sensor proximity sensor detects an object.
       * @param callback A reference to a function.
       */
      void            objectDetected( void (* callback)(void) );

      /**
       * @brief Calls a function when the optical sensor proximity sensor detects an object is missing.
       * @param callback A reference to a function.
       */
      void            objectLost( void (* callback)(void) );

      /**
       * @brief sets the value of the detection threshold
       * @param value The new detection threshold in proximity units (1 to 255).  0 means no change.
       * @return the current value of the proximity threshold
       */
      int32_t         objectDetectThreshold( int32_t value = 0 );
      
      /**
       * @brief Calls a function when the optical sensor gesture engine detects a movement down to up.
       * @param callback A reference to a function.
       */
      void            gestureUp( void (* callback)(void) );

      /**
       * @brief Calls a function when the optical sensor gesture engine detects a movement up to down.
       * @param callback A reference to a function.
       */
      void            gestureDown( void (* callback)(void) );

      /**
       * @brief Calls a function when the optical sensor gesture engine detects a movement right to left.
       * @param callback A reference to a function.
       */
      void            gestureLeft( void (* callback)(void) );

      /**
       * @brief Calls a function when the optical sensor gesture engine detects a movement left to right.
       * @param callback A reference to a function.
       */
      void            gestureRight( void (* callback)(void) );

      /**
       * @brief Turns the led on the optical sensor on or off.
       */
      void            setLight( ledState state );

      /**
       * @brief Set the intensity of the led on the optical sensor in percent.
       */
      void            setLightPower( int32_t intensity, percentUnits units = percentUnits::pct );

      /**
       * @brief A class to collect all gesture detection data.
       */
      class gesture {
          friend  vex::optical;
          
          private:
            uint8_t   _udata;
            uint8_t   _ddata;
            uint8_t   _ldata;
            uint8_t   _rdata;
            uint32_t  _count;
            gestureType _type;
            
          public:
            gesture();
            ~gesture();

            const uint8_t & udata = _udata;
            const uint8_t & ddata = _ddata;
            const uint8_t & ldata = _ldata;
            const uint8_t & rdata = _rdata;
            const uint32_t & count = _count;
            
            const gestureType & type = _type;            
      };
      
      /**
       * @brief Enable gesture detection on the optical sensor.
       */
      void            gestureEnable(void);

      /**
       * @brief Disable gesture detection on the optical sensor.
       */
      void            gestureDisable(void);

      /**
       * @brief get object with the most recent gesture detection data.
       */
      optical::gesture getGesture();        

      /**
       * @brief A class to hold raw or processed rgbc data.
       */
      class rgbc {
          friend  vex::optical;

          public:
            double  red;
            double  green;
            double  blue;
            double  clear;

          rgbc() : red(0), green(0), blue(0), clear(0) {}
          ~rgbc() {}
      };
      
      /**
       * @brief get raw or processed rgbc data. The default is raw.
       */
      optical::rgbc   getRgb( bool bRawColor = true );  
      
      /**
       * @brief set sensor integration time
       */
      void            integrationTime( double timeMs );

      /**
       * @brief get sensor integration time
       */
      double          integrationTime();
      
  };
};

#endif // VEX_OPTICAL_CLASS_H