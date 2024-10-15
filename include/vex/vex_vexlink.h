/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_vexlink.h                                               */
/*    Author:     James Pearman                                               */
/*    Created:    5 Nov 2019                                                  */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_VEXLINK_CLASS_H
#define   VEX_VEXLINK_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_vexlink.h
  * @brief   Radio/vexlink device class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  class vexlink : public device {
    public:      
      vexlink( int32_t index, const char *name, linkType type, bool isWired = false );  
      virtual ~vexlink();

      bool            installed();

      bool            isLinked();

      bool            isReceiving();

    protected:
      vexlink( int32_t index );  

      enum class tEventType {
        EVENT_DATA_RECEIVED    =  0,
        EVENT_MSG_RECEIVED     =  1
      };
    
      static const uint32_t max_buffer   = 512;
      static const uint32_t max_name_len = 128;

      // configuration of this link
      linkType _type;
      bool     _isWired;
      
      // single event handler
      bool     _eventAdded;
      
      // temporary storage
      uint8_t  _txbuffer[ max_buffer ];
      uint8_t  _rxbuffer[ max_buffer ];

      // storage for link name
      uint8_t  _linkname[ max_name_len ];

      // set these to point at functions that implement
      // the necessary functionality.

      // rx functions
      int32_t (* _avail)( uint32_t index );
      int32_t (* _peek)( uint32_t index );
      int32_t (* _read)( uint32_t index );
      int32_t (* _receive)( uint32_t index, uint8_t *buffer, int32_t length );

      // tx functions
      int32_t (* _free)( uint32_t index );
      int32_t (* _write)( uint32_t index, uint8_t c );
      int32_t (* _transmit)( uint32_t index, uint8_t *buffer, int32_t length );
                  
      int32_t debug( int32_t id );
      
      void    baud( int32_t baudrate );
      
      static  uint32_t _crc32_table[256];
      static  uint32_t  crc32( uint8_t *pData, uint32_t numberOfBytes, uint32_t accumulator );
      static  uint32_t  strcrc32( const char *str );
  };

  /** 
   * @brief Use the serial_link class to send raw data bytes between vexlink devices
  */
  class serial_link : public vexlink {
    public:
      /** 
       * @brief Creates a new serial_link object on the port specified.
       * @param index The port index for this serial_link. The index is zero-based.
       */
      serial_link( int32_t index, const char *name, linkType type, bool isWired = false );  
      ~serial_link();
    
      int32_t     send( uint8_t *buffer, int32_t length );
      int32_t     send( const char *buffer, int32_t length ) {
          return send( (uint8_t *)buffer, length );
      }

      int32_t     receive( uint8_t *buffer, int32_t length, int32_t timeoutMs = 500 );
      
      void        received( void (* callback)(uint8_t *, int32_t) );

    private:
      static void _eventHandler(void *arg);
      void        _eventHandler();

      void     (* _callback)(uint8_t *, int32_t);
  };
 
  /** 
   * @brief Use the message_link class to send strings between vexlink devices
  */
  class message_link : public vexlink {
    public:
      /** 
       * @brief Creates a new message_link object on the port specified.
       * @param index The port index for this message_link. The index is zero-based.
       */
      message_link( int32_t index, const char *name, linkType type, bool isWired = false );  
      ~message_link();
    
      int32_t     send( const char *message );      
      int32_t     send( const char *message, double value );
      int32_t     send( const char *message, int32_t index, double value );

      int32_t     receive( char *message, int32_t length, int32_t timeoutMs = 500 );
            
      void        received( void (* callback)(const char *, const char *, double) );
      void        received( void (* callback)(const char *, const char *, int32_t, double) );

      void        received( const char *message, void (* callback)(const char *, const char *, double) );
      void        received( const char *message, void (* callback)(const char *, const char *, int32_t, double) );
      
    protected:
      class data {
        public:
          const char *linkname;
          double      value;
          int32_t     index;
          uint32_t    time;
          int32_t     count;
      };
      
      void        received( void (* callback)(const char *, data& data) );
    
    private:
      static void _eventHandler(void *arg);
      void        _eventHandler();

      typedef     void (* tCallback_1)(const char *, const char *, double);
      typedef     void (* tCallback_2)(const char *, const char *, int32_t, double);
      typedef     void (* tCallback_3)(const char *, data& data);

      const uint8_t id_dbl      = 1;
      const uint8_t id_idx_dbl  = 2;
      
      void       *_callback;
      int32_t     _callback_type;
      
      data        _data;

      static const int32_t max_message_handlers   = 512;
      
      struct _messageHandlers {
        uint32_t  hash;
        void     *callback;
        int32_t   callback_type;
      } _messages[ max_message_handlers ];
      
      int32_t   _message_index;          

      void      _addMessageHandler( const char *message, void * callback, int32_t type );
  };
};

#endif // VEX_VEXLINK_CLASS_H