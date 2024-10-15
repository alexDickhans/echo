/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_thread.h                                                */
/*    Author:     James Pearman                                               */
/*    Created:    8 Nov 2017                                                  */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_THREAD_CLASS_H
#define   VEX_THREAD_CLASS_H

#include <chrono>

/*-----------------------------------------------------------------------------*/
/** @file    vex_thread.h
  * @brief   Header for thread control class (variation on tasks)
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
    * @brief Use this class to create and control threads.
  */
  class thread {
    private:
      static  int   _labelId;
      void         *_callback;
      int           _callbackId;

    public:
      thread() : _callback( NULL ) {};
      /**
      * @brief Creates a thread object.
      * @param callback A reference to a function.
      */
      thread( int  (* callback)(void) );
      /**
      * @brief Creates a thread object.
      * @param callback A reference to a function.
      * @param arg A void pointer that is passed to the callback.
      */
      thread( int  (* callback)(void *), void *arg );
      /**
      * @brief Creates a thread object.
      * @param callback A reference to a function. 
      */
      thread( void (* callback)(void) ) : thread( (int (*)(void)) callback ) {}
      /**
      * @brief Creates a thread object.
      * @param callback A reference to a function.
      * @param arg A void pointer that is passed to the callback.
      */
      thread( void (* callback)(void *), void *arg ) : thread( (int (*)(void *)) callback, arg ) {}
      ~thread();

      static const int32_t threadPrioritylow    =  1;
      static const int32_t threadPriorityNormal =  7;
      static const int32_t threadPriorityHigh   = 15;
      
      /**
      * @brief Gets the ID of the thread.
      * @return Returns an integer that represents the thread's ID.
      */
      int32_t           get_id();

      /**
      * @brief Waits for the other thread to finish its execution. 
      */
      void              join(); 

      /**
      * @brief Permits the thread to execute from the thread handle. 
      */
      void              detach() {};

      /**
      * @brief Checks whether the thread is joinable.
      * @return Returns a true Boolean if the thread is joinable.
      */
      bool              joinable();

      /**
      * @brief Gets the pointer to the native handle of the thread. 
      */
      void             *native_handle();

      /**
      * @brief Swaps the thread IDs with another specified thread in the parameter.
      * @param __t The thread to swap to.
      */
      void              swap( thread& __t );
      
      // non standard
      /**
      * @brief Stops the thread.
      */
      void              interrupt();      

      /**
      * @brief Stop all threads except main.
      */
      static void       interruptAll();      

      /**
      * @brief Sets the priority of the thread.
      * @param priority The priority of the thread to be set to.
      */
      void              setPriority( int32_t priority );

      /**
      * @brief Gets the priority of the thread.
      * @return Returns the priority of the thread as an integer.
      */
      int32_t           priority();


      /**
      * @brief Gets the number of concurrent threads supported by the hardware.
      * @return An integer that represents the amount of concurrent threads supported by the hardware. 
      */      
      static  int32_t   hardware_concurrency();

      /**
      * @brief Swaps two threads specified in the parameters.
      * @param __x A thread to swap with the next thread set in the parameter.
      * @param __y A thread to swap with the previous thread set in the parameter.
      */
      static  void      swap( thread& __x, thread& __y ) { __x.swap(__y); }
  };
  
  namespace this_thread {

      /**
      * @brief Gets the ID of the thread
      * @return Returns an integer that represents the thread's ID.
      */  
      int32_t   get_id();

      /**
      * @brief Suspends the current thread. 
      */  
      void      yield();

      /// sleep_for
      /**
      * @brief Suspends the current thread until the time is expired in milliseconds.
      * @param time The amount of time for the thread to sleep.
      */  
      void      sleep_for( uint32_t time );

      /**
      * @brief Suspends the current thread until the time is expired in milliseconds.
      * @param __rtime The standard chrono::duration object representing the time to sleep.
      */
      template<typename _Rep, typename _Period>
      inline void
      sleep_for(const std::chrono::duration<_Rep, _Period>& __rtime) {
    	  if (__rtime <= __rtime.zero())
    	    return;
    	  
        int __ms = std::chrono::duration_cast<std::chrono::milliseconds>(__rtime).count();
        sleep_for( (uint32_t)__ms );
      }
      
      /// sleep_until
      /**
      * @brief Sleeps until the system timer reaches the time specified in the parameter.
      * @param time The amount of time for the thread to sleep.
      */  
      void      sleep_until( uint32_t time );
      /**
      * @brief Sleeps until the system timer reaches the time specified in the parameter.
      * @param __atime The standard chrono::time_point object representing the time to sleep.
      */
      template<typename _Clock, typename _Duration>
      inline void
      sleep_until(const std::chrono::time_point<_Clock, _Duration>& __atime)
        {
        auto __now = _Clock::now();
        if (__now < __atime)
          sleep_for(__atime - __now);
        }

      // non standard
      /**
      * @brief Sets the priority of the current thread.
      * @param priority The priority of the thread to be set to.
      */
      void              setPriority( int32_t priority );
      /**
      * @brief Gets the priority of the current thread.
      * @return Returns the priority of the current thread as an integer.
      */ 
      int32_t           priority();
    };   
       
  /** 
    * @brief Use this class to create and control mutexes.
  */
  class mutex {
    private:
      uint32_t  _sem;
    
    public:
      mutex();
      ~mutex();
    
      /**
      * @brief Locks the mutex and blocks if the mutex is not available. 
      */
      void  lock();

      /**
      * @brief Try to lock the mutex and returns if the mutex is not available.
      * @return Returns true if successful and false if the mutex is owned by another thread.
      */
      bool  try_lock();

      /**
      * @brief Unlocks the mutex.
      */
      void  unlock();
  };
};


#endif // VEX_THREAD_CLASS_H