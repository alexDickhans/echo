/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_signaltower.h                                           */
/*    Author:     James Pearman                                               */
/*    Created:    9 June 2023                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_SIGNALTOWER_CLASS_H
#define   VEX_SIGNALTOWER_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_signaltower.h
  * @brief   light tower device class header
*//*---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @brief signaltower device class                                             */
/*-----------------------------------------------------------------------------*/
namespace vex {
  namespace cte {
    class signaltower : public device {
        private:
          enum class tEventType {
            EVENT_BUTTON_PRESSED     = 1,
            EVENT_BUTTON_RELEASED    = 2
          };
      
          uint8_t _blinkOnTime  = 0;
          uint8_t _blinkOffTime = 0;
        public:
          /**
           * @brief Creates a new signaltower object on the port specified in the parameter.
           * @param index index to the brain port.
          */
          signaltower( int32_t index );  
          ~signaltower();
    
          bool            installed();
        
          enum class ledId {
            red    = 0,
            green  = 1,
            blue   = 2,
            white  = 3,
            yellow = 4,
            none   = 99
          };

          enum class state {
            off,
            on,
            blink
          };

          class color : public vex::color {
            public:
              color() {}
              color( int value ) : vex::color( value ) {}
              ~color() {}
              
              static const color  all;
          };

          /**
           * @brief Sets the signal tower to a color
           * @param color the color to use, red, green etc.
           * @note This is used to set a single led, we can set as red, green, yellow etc. using vex::color
          */
          void            setColor( vex::color color, signaltower::state state = signaltower::state::on );
          /**
           * @brief Sets the signal tower LED to given values
           * @param rgb the brightness of red, green and blue LED using 32 bit value (eg. red on = 0xFF0000)
           * @param yw the brightness of yellow and white LED using 32 bit value (eg. yellow on = 0xFF00)
          */
          void            setColor( uint32_t rgb, uint32_t yw );
          /**
           * @brief Sets a signal tower LED to given value
           * @param id the index of the LED
           * @param value the brightness of the LED (0 to 255)
          */
          void            setColor( ledId id, uint32_t value );
          /**
           * @brief Sets the signal tower LEDs to given values
           * @param r the brightness of the red LED
           * @param y the brightness of the yellow LED
           * @param g the brightness of the green LED
           * @param b the brightness of the blue LED
           * @param w the brightness of the white LED
          */
          void            setColor( uint8_t r, uint8_t y, uint8_t g, uint8_t b, uint8_t w );
          /**
           * @brief Sets the signal tower LEDs to given states
           * @param r the state of the red LED
           * @param y the state of the yellow LED
           * @param g the state of the green LED
           * @param b the state of the blue LED
           * @param w the state of the white LED
          */
          void            setColors( signaltower::state rs, signaltower::state ys, signaltower::state gs, signaltower::state bs, signaltower::state ws );
          /**
           * @brief Sets a signal tower LED to blinking
           * @param id the index of the LED
           * @param bEnable the bink state, true is blinking
          */
          void            setBlink( ledId id, bool bEnable = true );
          /**
           * @brief Set multiple signal tower LED to blinking on
           * @param id0 the index of the first LED
           * @param id1 the index of the second LED (optional)
           * @param id2 the index of the third LED (optional)
           * @param id3 the index of the fourth LED (optional)
           * @param id4 the index of the fifth LED (optional)
          */
          void            enableBlink( ledId id0, ledId id1=ledId::none, ledId id2=ledId::none, ledId id3=ledId::none, ledId id4=ledId::none );
          /**
           * @brief Set multiple signal tower LED to blinking off
           * @param id0 the index of the first LED
           * @param id1 the index of the second LED (optional)
           * @param id2 the index of the third LED (optional)
           * @param id3 the index of the fourth LED (optional)
           * @param id4 the index of the fifth LED (optional)
          */
          void            disableBlink( ledId id0, ledId id1=ledId::none, ledId id2=ledId::none, ledId id3=ledId::none, ledId id4=ledId::none );
          /**
           * @brief Set the blink time for on and off periods
           * @param onTime the period in mS for the LED to be on (max 2500 mS, 0 will use default value of 500mS)
           * @param offTime the period in mS for the LED to be of (max 2500 mS, 0 will use the same as onTime)
          */
          void            setBlinkTime( uint32_t onTime, uint32_t offTime = 0 );


          /**
           * @brief Sets the function to be called when the button is pressed.
           * @param callback A reference to a function.
          */
          void            pressed( void (* callback)(void) );
          /**
           * @brief Sets the function to be called when the button is released.
           * @param callback A reference to a function.
          */
          void            released( void (* callback)(void) );
          /**
           * @brief Gets the status of the signal tower button.
           * @return Returns a Boolean value based on the pressed states of the button. If the button is pressed it will return true.
          */
          bool            pressing( void );        

        protected:
          uint32_t        status();
    };
  }
}

#endif // VEX_SIGNALTOWER_CLASS_H