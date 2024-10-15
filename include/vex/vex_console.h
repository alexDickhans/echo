/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_console.h                                               */
/*    Author:     James Pearman                                               */
/*    Created:    1 Nov 2017                                                  */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_CONSOLE_CLASS_H
#define   VEX_CONSOLE_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_console.h
  * @brief   V5 console class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  class console : public brain {
    public:
      console();
      ~console();
      
      void    init();
 
    private:
      static  int32_t write(int32_t fd, char* buf, int32_t nbytes);
  };
}

#endif // VEX_CONSOLE_CLASS_H