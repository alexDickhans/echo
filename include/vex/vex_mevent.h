/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2018-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_mevent.h                                                */
/*    Author:     James Pearman                                               */
/*    Created:    25 Jan 2018                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_MEVENT_CLASS_H
#define   VEX_MEVENT_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_mevent.h
  * @brief   V5 alternate event class header
*//*---------------------------------------------------------------------------*/

namespace vex {
    class mevent {
        private:
          int   _event_id;
          int   _index;
          
        public:
          mevent() : _event_id(-1) {};
          mevent( uint32_t index, uint32_t id );
          ~mevent() {};
                    
          operator int() const;
    };
}

#endif // VEX_MEVENT_CLASS_H