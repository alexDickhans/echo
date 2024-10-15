/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2018-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_color.h                                                 */
/*    Author:     James Pearman                                               */
/*    Created:    21 May 2018                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_COLOR_CLASS_H
#define   VEX_COLOR_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_color.h
  * @brief   V5 color class header
*//*---------------------------------------------------------------------------*/

namespace vex {
    /**
      * @brief Use the color class to create Color objects.
    */
    class color {
        private:
          uint32_t  _argb;
          bool      _transparent;

          // special constructor only used to create a transparent color
          color( int value, bool transparent );            
          
        public:
          color();          
          /**
          * @brief Creates a color. 
          * @param value The value of the color. 
          */          
          color( int value );
          /**
          * @brief Creates a color using red, green, and blue values. 
          * @param r the color of red on a scale of 0 to 255.
          * @param g the color of green on a scale of 0 to 255.
          * @param b the color of blue on a scale of 0 to 255.
          */
          color( uint8_t r, uint8_t g, uint8_t b );            
          ~color();

          // Assignment
          uint32_t  rgb( uint32_t value );
          uint32_t  rgb( uint8_t r, uint8_t g, uint8_t b );
          void operator=( uint32_t value );

          // Get value
          uint32_t  rgb() const;                        
          operator uint32_t() const;
          
         /**
          * @brief Gets the state of the color's transparency.
          * @return Returns true if the color is transparent.
          */
          bool isTransparent() const;

         /**
          * @brief Creates a color using hue, saturation, and brightness values.         
          * @return Returns a reference to a color.
          * @param hue An integer from 0 to 360 that represents the hue of the color.
          * @param sat A double from 0.0 to 1.0 that represents the saturation of the color.
          * @param value A double from 0.0 to 1.0 that represents the brightness of the color.
          */
          color& hsv( uint32_t hue, double sat, double value );
          
         /**
          * @brief Creates a color using a hexadecimal value.
          * @return Returns a reference to a color.
          * @param color A hexadecimal or web color value that defines a specific color.
          */
          color& web( const char *color );

         /**
          * @brief Gets the hue of the color
          * @return Returns the hue of the color in the range 0 to 360 degrees
          */
          double hue( void ) const;

         /**
          * @brief Gets the saturation of the color
          * @return Returns the saturation of the color in the range 0 to 1.0
          */
          double saturation( void ) const;

         /**
          * @brief Gets the value (brightness) of the color
          * @return Returns the value (brightness) of the color in the range 0 to 1.0
          */
          double brightness( void ) const;

          // predefined colors
          
          /**
           *@brief Represents the color black.
           */
          static const color    black;
          /**
           *@brief Represents the color white.
           */
          static const color    white;
          /**
           *@brief Represents the color red.
           */
          static const color    red;
          /**
           *@brief Represents the color green.
           */
          static const color    green;
          /**
           *@brief Represents the color blue.
           */
          static const color    blue;
          /**
           *@brief Represents the color yellow.
           */
          static const color    yellow;
          /**
           *@brief Represents the color orange.
           */
          static const color    orange;
          /**
           *@brief Represents the color purple.
           */
          static const color    purple;
          /**
           *@brief Represents the color cyan.
           */
          static const color    cyan;
          /**
           *@brief Represents the color transparent.
           */
          static const color    transparent;          
    };
}

#endif // VEX_COLOR_CLASS_H