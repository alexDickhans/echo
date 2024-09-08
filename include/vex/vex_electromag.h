/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_electromag.h                                            */
/*    Author:     James Pearman                                               */
/*    Created:    28 May 2020                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_ELECTROMAG_CLASS_H
#define   VEX_ELECTROMAG_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_electromag.h
  * @brief   Electromagnet device class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
   * @brief Use the distance class to control the distance sensor.
   */
  class electromagnet : public device {
    private:
      uint32_t        _power;
        
    public:
      /** 
       * @brief Creates a new inertial object on the port specified.
       * @param index The port index for this inertial. The index is zero-based.
       */
      electromagnet( int32_t index );  
      ~electromagnet();
    
      bool            installed();
      int32_t         value();
      
      void            setPower( uint32_t power );
      
      /**
       * @brief Energize the electromagnet for a brief time to pickup an object
       */
      void            pickup( void );
      /**
       * @brief Energize the electromagnet for a given time with default power to pickup an object
       */
      void            pickup( double time, timeUnits units = timeUnits::msec);
      /**
       * @brief Energize the electromagnet for a given time with given power to pickup an object
       */
      void            pickup( double time, timeUnits units, int32_t power );

      /**
       * @brief Energize the electromagnet for a brief time to drop an object
       */
      void            drop( void );
      /**
       * @brief Energize the electromagnet for given time with default power to drop an object
       */
      void            drop( double time, timeUnits units = timeUnits::msec );
      /**
       * @brief Energize the electromagnet for given time with given power to drop an object
       */
      void            drop( double time, timeUnits units, int32_t power );

      /**
       * @brief Gets the temperature of the electromagnet
       */
      double          temperature( temperatureUnits units = temperatureUnits::celsius );
  };
};

#endif // VEX_ELECTROMAG_CLASS_H