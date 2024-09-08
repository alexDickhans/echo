/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2018-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_timer.h                                                 */
/*    Author:     James Pearman                                               */
/*    Created:    24 May 2018                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_TIMER_CLASS_H
#define   VEX_TIMER_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_timer.h
  * @brief   V5 timer class header
*//*---------------------------------------------------------------------------*/

namespace vex {
    /**
      * @brief Use the timer class to create timers for your program.
    */
    class timer {
        private:
          uint32_t  _offset;
          uint32_t  _initial;
          
        public:
          timer();          
          ~timer();

          // Assignment
          void operator=( uint32_t value );

          // Get value
          operator uint32_t() const;          

         /**
          * @brief Gets the current value of the timer in mS.
          * @return Returns the current value of the timer in mS.
          */
          uint32_t  time() const;
          
         /**
          * @brief Gets the current value of the timer in specified units.
          * @return Returns a double that represents the time in the specified units.
          * @param units The measurement unit for time.
         */
          double    time( timeUnits units ) const;
          
         /**
          * @brief Gets the current value of the timer in seconds.
          * @return Returns the current value of the timer in seconds.
          */
          double    value() const;

         /**
          * @brief Sets the current value of the timer to 0.
          */
          void      clear();

         /**
          * @brief Sets the current value of the timer to 0.
          */
          void      reset();

         /**
          * @brief Gets the current value of the system timer in mS.
          * @return Returns the value of the system timer in mS.
          */
          static uint32_t  system();

         /**
          * @brief Gets the current value of the high-resolution timer (in microseconds).
          * @return Returns the current value of the high-resolution timer (in microseconds).
          */
          static uint64_t  systemHighResolution();

         /**
          * @brief Sets a callback that will be called in the future.
          * @param callback A pointer to a function that will be called when the timer expires.
          * @param value The delay in mS to when the function will be called.
          */
          static void event( void(* callback)(void *), uint32_t value );

         /**
          * @brief Sets a callback that will be called in the future.
          * @param callback A pointer to a function that will be called when the timer expires.
          * @param value The delay in mS to when the function will be called.
          */
          static void event( void(* callback)(void), uint32_t value );
    };
}

#endif // VEX_TIMER_CLASS_H