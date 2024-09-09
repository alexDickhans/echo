/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_pneumatic.h                                             */
/*    Author:     James Pearman                                               */
/*    Created:    1 June 2023                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_PNEUMATIC_CLASS_H
#define   VEX_PNEUMATIC_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_pneumatic.h
  * @brief   pneumatic device class header
*//*---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @brief Pneumatic device class                                              */
/*-----------------------------------------------------------------------------*/
namespace vex {
  namespace cte {
    class pneumatic : public device {
      
        public:
          /**
           * @brief Creates a new pneumatic object on the port specified by the parameter.
           * @param index index to the brain port.
           * @param bPumpEnable optional parameter to enable or disable the air pump
          */
          pneumatic( int32_t index, bool bPumpEnable = true );  
          ~pneumatic();
    
          bool            installed();
        
          /**
           * @brief extend a pneumatic cylinder
           * @param id The cylinder to extend
           */
          void            extend( cylinderType id );
          /**
           * @brief retract a pneumatic cylinder
           * @param id The cylinder to retract
           */
          void            retract( cylinderType id );
          /**
           * @brief Turn on the pneumatic air pump
           */
          void            pumpOn();
          /**
           * @brief Turn off the pneumatic air pump
           */
          void            pumpOff();
          /**
           * @brief Set the pneumatic air pump to on or off
           * @param state true to turn on the air pump, false to turn it off
           */
          void            pump( bool state );

        protected:
          uint32_t        status();
    };
  }
}

#endif // VEX_PNEUMATIC_CLASS_H