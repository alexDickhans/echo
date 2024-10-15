/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017-2019, All rights reserved.          */
/*                                                                            */
/*    Module:     vex_controller.h                                            */
/*    Author:     James Pearman                                               */
/*    Created:    10 July 2017                                                */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_CONTROLLER_CLASS_H
#define   VEX_CONTROLLER_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_controller.h
  * @brief   V5 controller class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  /** 
    * @brief Use the controller class to get values from the remote controller as well as write to the controller's screen.
  */
  class controller {
    private:
      // copy if the controller id when this class was instantiated
      controllerType		_controllerId;
      // internal device id for this controller
      int32_t           _index;
      
      int32_t value( V5_ControllerIndex channel );

    protected:
      int32_t _getIndex();
      
    public:

      /** 
        * @brief Creates a new controller object.
      */
      controller();
      /** 
        * @brief Creates a new controller object.
        * @param id The type of controller that is being created. This can be set to primary or partner.
      */
      controller( controllerType id );
      ~controller();

      bool installed();

      /**
       * @brief Rumbles the controller by a pattern defined by the parameter. Dots equal short, dashes equal long and space equals pause.
       * @param str A string that consists of dots and dashes that represent the rumble pattern.
      */ 
      void     rumble( const char *str );

    private:
      // bit masks for events
      enum class tEventType {
        EVENT_5_UP_PRESSED     = 0,
        EVENT_5_UP_RELEASED    = 1,
        EVENT_5_DN_PRESSED     = 2,
        EVENT_5_DN_RELEASED    = 3,
        
        EVENT_6_UP_PRESSED     = 4,
        EVENT_6_UP_RELEASED    = 5,
        EVENT_6_DN_PRESSED     = 6,
        EVENT_6_DN_RELEASED    = 7,
        
        EVENT_7_UP_PRESSED     = 8,
        EVENT_7_UP_RELEASED    = 9,
        EVENT_7_DN_PRESSED     = 10,
        EVENT_7_DN_RELEASED    = 11,

        EVENT_7_L_PRESSED      = 12,
        EVENT_7_L_RELEASED     = 13,
        EVENT_7_R_PRESSED      = 14,
        EVENT_7_R_RELEASED     = 15,

        EVENT_8_UP_PRESSED     = 16,
        EVENT_8_UP_RELEASED    = 17,
        EVENT_8_DN_PRESSED     = 18,
        EVENT_8_DN_RELEASED    = 19,

        EVENT_8_L_PRESSED      = 20,
        EVENT_8_L_RELEASED     = 21,
        EVENT_8_R_PRESSED      = 22,
        EVENT_8_R_RELEASED     = 23,

        EVENT_A_CHANGED        = 24,
        EVENT_B_CHANGED        = 25,
        EVENT_C_CHANGED        = 26,
        EVENT_D_CHANGED        = 27
      };

      enum class tButtonType {
        kButtonL1 = 0,
        kButtonL2,
        kButtonR1,
        kButtonR2,
        kButtonUp,
        kButtonDown,
        kButtonLeft,
        kButtonRight,
        kButtonX,
        kButtonB,
        kButtonY,
        kButtonA,

        kButtonRes1,
        kButtonRes2,

        kButtonUndefined
      };
      /** 
        * @brief Use the button class to get values from the controller's buttons.
      */
    public:
      class button {
        private:
          tButtonType  _id;
          controller   *_parent;
          tEventType _buttonToPressedEvent() const;
          tEventType _buttonToReleasedEvent() const;
          
        public:
          button() : _id( tButtonType::kButtonUndefined), _parent(NULL) {};
          button( const tButtonType id, controller *parent ) : _id( id ), _parent(parent) {};
          ~button() {};
          /**
           * @brief Sets the function to be called when the button is pressed.
           * @param callback A reference to a function.
          */
          void     pressed( void (* callback)(void) ) const;

          /**
           * @brief Sets the function to be called when the button is released.
           * @param callback A reference to a function.
          */
          void     released( void (* callback)(void) ) const;

          /**
           * @brief Gets the status of a button.
           * @return Returns a Boolean value based on the pressed states of the button. If the button is pressed it will return true.
          */
          bool     pressing( void ) const;        

          mevent  PRESSED   = { (uint32_t)_parent->_getIndex(), (uint32_t)_buttonToPressedEvent()  };
          mevent  RELEASED  = { (uint32_t)_parent->_getIndex(), (uint32_t)_buttonToReleasedEvent()  };
      };
      
    public:
      // V5 competition controller buttons
      /**
       * @brief A button that represents the L1 button on the controller
       */
      const button  ButtonL1    = button( tButtonType::kButtonL1   , this );
      /**
       * @brief A button that represents the L2 button on the controller
       */
      const button  ButtonL2    = button( tButtonType::kButtonL2   , this );
      /**
       * @brief A button that represents the R1 button on the controller
       */
      const button  ButtonR1    = button( tButtonType::kButtonR1   , this );
      /**
       * @brief A button that represents the R2 button on the controller.
       */
      const button  ButtonR2    = button( tButtonType::kButtonR2   , this );
      /**
       * @brief A button that represents the up button on the controller.
       */
      const button  ButtonUp    = button( tButtonType::kButtonUp   , this );
      /**
       * @brief A button that represents the down button on the controller.
       */
      const button  ButtonDown  = button( tButtonType::kButtonDown , this );
      /**
       * @brief A button that represents the left button on the controller.
       */
      const button  ButtonLeft  = button( tButtonType::kButtonLeft , this );
      /**
       * @brief A button that represents the right button on the controller.
       */
      const button  ButtonRight = button( tButtonType::kButtonRight, this );
      /**
       * @brief A button that represents the X button on the controller.
       */
      const button  ButtonX     = button( tButtonType::kButtonX    , this );
      /**
       * @brief A button that represents the B button on the controller.
       */
      const button  ButtonB     = button( tButtonType::kButtonB    , this );
      /**
       * @brief A button that represents the Y button on the controller.
       */
      const button  ButtonY     = button( tButtonType::kButtonY    , this );
      /**
       * @brief A button that represents the A button on the controller.
       */
      const button  ButtonA     = button( tButtonType::kButtonA    , this );

    private:
      enum class tAxisType {
        kAxisA = 0,
        kAxisB,
        kAxisC,
        kAxisD,
        
        kAxisUndefined
      };
      
      
    public:
      /** 
        * @brief Use the axis class to get values from one of the controller's joysticks
      */
      class axis {
        private:
          tAxisType  _id;
          controller   *_parent;
          tEventType _joystickToChangedEvent() const;
          
        public:
          axis() : _id( tAxisType::kAxisUndefined), _parent(NULL) {};
          axis( const tAxisType id, controller *parent ) : _id( id ), _parent(parent) {};
          ~axis() {};
          
          /**
           * @brief Sets the function to be called when the joystick axis value changes.
           * @param callback A reference to a function.
          */
          void     changed( void (* callback)(void) ) const;

          /**
           * @brief Gets the value of the joystick axis on a scale from -127 to 127.
           * @return Returns an integer that represents the value of the joystick axis.
          */
          int32_t  value( void ) const;

          /**
           * @brief Gets the position of the joystick axis on a scale from -100 to 100.
           * @return Returns an integer that represents the position of the joystick axis as defined by the unit in the parameter.
           * @param units (Optional) The type of unit that will be returned. By default, this parameter is a percentage.
          */        
          int32_t  position( percentUnits units = percentUnits::pct ) const;        

          mevent  CHANGED   = { (uint32_t)_parent->_getIndex(), (uint32_t)_joystickToChangedEvent()  };
      };
      
     
    private:
      const axis  AxisA = axis( tAxisType::kAxisA, this );
      const axis  AxisB = axis( tAxisType::kAxisB, this );
      const axis  AxisC = axis( tAxisType::kAxisC, this );
      const axis  AxisD = axis( tAxisType::kAxisD, this );
      
    public:
      // V5 joystick names
      /**
       * @brief An axis of a joystick that represents axis 1 on the controller.
       */
      const axis &Axis1 = AxisC;
      /**
       * @brief An axis of a joystick that represents axis 2 on the controller.
       */
      const axis &Axis2 = AxisD;
      /**
       * @brief An axis of a joystick that represents axis 3 on the controller.
       */
      const axis &Axis3 = AxisA;
      /**
       * @brief An axis of a joystick that represents axis 4 on the controller.
       */
      const axis &Axis4 = AxisB;
      
      /** 
        * @brief Use this class to write to the controller's LCD screen.
      */
      class lcd {
        private:
          controller   *_parent;
          int32_t   _row;
          int32_t   _maxrows;
          int32_t   _col;
          int32_t   _maxcols;

          char      _textStr[32];
          
          controllerType  getControllerId();
          
        public:
          lcd();
          lcd(controller *parent);
          ~lcd() {};

        /**
         * @brief Sets the cursor to the row and column number set in the parameters.
         * @param row Sets the row number for where the cursor is placed.
         * @param col Sets the column number for where the cursor is placed.
        */
        void     setCursor( int32_t row, int32_t col );

        /** 
         * @brief An integer that tracks the current cursor position's column, starting at 1.
         * @return Returns an integer that tracks the current cursor position's column, starting at 1.
        */
        int32_t  column();

        /** 
         * @brief An integer that tracks the current cursor position's row, starting at 1.
         * @return Returns an integer that tracks the current cursor position's row, starting at 1.
        */
        int32_t  row();

        /** 
         * @brief Prints a number, string, or Boolean.
         * @param value Information to display on the screen.
         */
        template <class T>
        void     print( T value ) {
          if( std::is_same< T, int >::value )
            print( "%d", (int)value );
          else
          if( std::is_same< T, double >::value )
            print( "%.2f", (double)value );
          else {
            // primarily to handle modkit number
            if( (int)value == value )
              print( "%d", (int)value );
            else
              print( "%.2f", (double)value );
          }
        }

        /** 
           * @brief Prints a number, string, or Boolean.
           * @param format This is a reference to a char format that prints the value of variables.
           * @param ... A variable list of parameters to insert into format string.  
          */           
        void     print( const char *format, ... );
        void     print( char *format, ... );
        
        /**
         * @brief Clears the controller's LCD screen.
        */
        void     clearScreen( void );

        /**
         * @brief Clears the line specified by the parameter.
         * @param number The line number to be cleared.
        */
        void     clearLine( int number );

        /**
         * @brief Clears the current line.
        */
        void     clearLine( void );

        /**
         * @brief Moves the cursor to the next line.
        */
        void     newLine( void );      
      };

      // Controller has an instance of an lcd
      /**
         * @brief Controller's instance of LCD.
      */
      lcd     Screen = lcd(this);
  };
};

#endif  // VEX_CONTROLLER_CLASS_H