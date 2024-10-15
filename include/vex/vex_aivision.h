/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_aivision.h                                              */
/*    Author:     James Pearman                                               */
/*    Created:    18 September 2023                                           */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_AIVISION_CLASS_H
#define   VEX_AIVISION_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_aivision.h
  * @brief   AI Vision class header
*//*---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @brief AI Vision class                                                     */
/*-----------------------------------------------------------------------------*/
namespace vex {
  /**
    * @brief Use this class when programming the AI Vision camera.
  */
  class aivision : public device {
    private:
      #define AIVISION_MAX_OBJECTS               24
      #define AIVISION_DEFAULT_SNAPSHOT_OBJECTS   8

      // Variadic template for color description setting
      void _setObjectDescription();
      template <typename desctype, typename... Args>
      void _setObjectDescription( desctype &desc, Args &... args ) {
        setObjectDescription( desc );
        _setObjectDescription( args... );
      }

    public:

     /**
      * @brief Creates a new AI Vision object on the port specified.
      * @param index The port index for this camera. The index is zero-based.
      */
      aivision( int32_t index );
      ~aivision();

     /**
      * @brief Creates a AI Vision camera object on the port specified.
      * @param index The port index for this AI Vision camera. The index is zero-based.
      * @param desc List of color or code description objects used to setup the detection for this sensor.
      */
      template <typename... Args>
      aivision( int32_t index, Args &... desc  ) : aivision( index ) {
        _setObjectDescription( desc... );
      }

      bool            installed();

      enum class objectType {
        unknownObject  = 0,
        colorObject    = (1 << 0),
        codeObject     = (1 << 1),
        modelObject    = (1 << 2),
        tagObject      = (1 << 3),
        allObject      = (0x3F)
      };

      /**
       * @brief The object class represents an object that the AI Vision camera detects.
      */
      class object {
        friend class aivision;
        friend class safearray<object, AIVISION_MAX_OBJECTS>;

        public:
          class tagcoords {
            public:
              int16_t   x[4];
              int16_t   y[4];
          };

        private:
          #define AIVISION_MAX_CLASS_NAME   20

          int32_t     _id;
          objectType  _type;
          int16_t     _originX;
          int16_t     _originY;
          int16_t     _centerX;
          int16_t     _centerY;
          int16_t     _width;
          int16_t     _height;
          float       _angle;
          bool        _exists;

          tagcoords   _tag;      

          char        _className[AIVISION_MAX_CLASS_NAME];

          vex::color  _color;

          /**
           * @brief Copies all properties of the passed in object into this object.
           * @param obj The object whose properties are to be copied.
           */
          void      set( V5_DeviceAiVisionObject obj, char *cname = NULL );

          /**
           * @brief Sets all properties for this object to default value exceot id;
           */
          object& operator=( int32_t id );
        public:

          /**
           * @brief Creates a new AI Vision camera object with all properties set to default values.
           */
          object();
          ~object();
        
          /**
           * @brief Sets all properties for this object to default values.
           */
          void      clear();
        
          /**
           * @brief Copies an object.
           */
          object& operator=( const object &obj );
 
           /**
           * @brief The unique ID of the object.
           */
          const int32_t &id;
           /**
           * @brief The type of the object.
           */
          const objectType &type;
          /**
           * @brief The top left x position of the object.
           */
          const int16_t  &originX;
          /**
           * @brief The top left y position of the object.
           */
          const int16_t  &originY;
          /**
           * @brief The center x position of the object.
           */
          const int16_t  &centerX;
          /**
           * @brief The center y position of the object.
           */
          const int16_t  &centerY;
          /**
           * @brief The width of the object.
           */
          const int16_t  &width;
          /**
           * @brief The height of the object.
           */
          const int16_t  &height;
          /**
           * @brief The angle of the object.
           */
          const float &angle;
          /**
           * @brief If the AI Vision camera detects the object or not.
           */
          const bool &exists;

          /**
           * @brief The raw coordinates of an apriltag.
           */
          const tagcoords &tag;

          /**
           * @brief Read only pointer to object class name, only valid for model objects.
           */
          const char * const className;
          
          /**
           * @brief The color for this object, only valid for color objects.
           */
          const vex::color &color;
      };

      class objdesc {
        protected:
          uint8_t   _id;
        
        public:
          objdesc();
          objdesc(uint8_t id);

          // read only references to internal variables
          const uint8_t   &id = _id;
      };

      /**
       * @brief Use this class when programming the AI 2D camera.
      */
      class colordesc : public objdesc {
        private:
          uint8_t   _red;
          uint8_t   _green;
          uint8_t   _blue;
          float     _hangle;
          float     _hdsat;

          void      clear();

        public:
          colordesc();
          ~colordesc();
        
         /**
          * @brief Creates a new AI Vision camera color description object.
          * @param id The color description id.
          */
          colordesc( int32_t id, uint8_t red, uint8_t green, uint8_t blue, float hangle, float hdsat );

          // read only references to internal variables
          const uint8_t   &red    = _red;
          const uint8_t   &green  = _green;
          const uint8_t   &blue   = _blue;
          const float     &hangle = _hangle;
          const float     &hdsat  = _hdsat;          
      };

      /**
       * @brief Use this class when programming the AI Vision camera.
       */
      class codedesc : public objdesc {
        friend class vex::aivision;

        private:
          V5_DeviceAiVisionCode _code;
      
        public:
          codedesc(int32_t id, int32_t c1, int32_t c2, int32_t c3=0, int32_t c4=0, int32_t c5=0 );
  
         /**
          * @brief Creates a new AI Vision camera code description object.
          * @param c1 The first color description which is part of the color code.
          * @param c2 The second color description which is part of the color code.
          */
          codedesc( int32_t id, colordesc &c1, colordesc &c2 );
          codedesc( int32_t id, colordesc &c1, colordesc &c2, colordesc &c3 );
          codedesc( int32_t id, colordesc &c1, colordesc &c2, colordesc &c3, colordesc &c4 );
          codedesc( int32_t id, colordesc &c1, colordesc &c2, colordesc &c3, colordesc &c4, colordesc &c5 );
      };

      class tagdesc : public objdesc  {
        public:
          tagdesc(int32_t id);
      };

      class aiobjdesc : public objdesc  {
        public:
          aiobjdesc(int32_t id);
      };

      static const tagdesc     ALL_TAGS;
      static const colordesc   ALL_COLORS;
      static const codedesc    ALL_CODES;
      static const aiobjdesc   ALL_AIOBJS;
      static const objdesc     ALL_OBJECTS;

      static const uint32_t    FLG_COLORMERGE;
      static const uint32_t    FLG_OVLENABLE;

      /**
       * @brief Takes a data sample from the AI Vision camera, and only stores the largest samples of the specified count.
       * @return Returns the number of objects found matching the id andd type passed as parameters.
       * @param id The id of the object to look for.
       * @param type The type of the object to look for.
       * @param count the maximum number of objects to look for.
       */
      int32_t         takeSnapshot( uint32_t id, objectType type, uint32_t count );

      /**
       * @brief Takes a data sample from the AI Vision camera, and only stores the largest samples of the specified count.
       * @return Returns the number of objects found matching the colordesc passed as a parameter.
       * @param desc The color description of the object to look for.
       * @param count the maximum number of objects to look for.
       */
      int32_t         takeSnapshot( const colordesc &desc, int32_t count = AIVISION_DEFAULT_SNAPSHOT_OBJECTS );
      int32_t         takeSnapshot( const codedesc &desc, int32_t count = AIVISION_DEFAULT_SNAPSHOT_OBJECTS );
      int32_t         takeSnapshot( const tagdesc &desc, int32_t count = AIVISION_DEFAULT_SNAPSHOT_OBJECTS);
      int32_t         takeSnapshot( const aiobjdesc &desc, int32_t count = AIVISION_DEFAULT_SNAPSHOT_OBJECTS);
      int32_t         takeSnapshot( const objdesc &desc, int32_t count = AIVISION_DEFAULT_SNAPSHOT_OBJECTS );

      /**
       * @brief enable/disable color and code processing (default state is enabled)
       * @param bEnable Set true to enable color detection
       */
      void            colorDetection( bool bEnable, bool bMerge = false );

      /**
       * @brief enable/disable apriltag processing (default state is disabled)
       * @param bEnable Set true to enable apriltag detection
       */
      void            tagDetection( bool bEnable );

      /**
       * @brief enable/disable AI model processing (default state is disabled)
       * @param bEnable Set true to enable model/object detection
       */
      void            modelDetection( bool bEnable );

      /**
       * @brief reset all to default settings.
       */
      void            reset();

      /**
       * @brief set a color description.
       */
      void            set( const colordesc &desc );
      /**
       * @brief set a code description.
       */
      void            set( const codedesc &desc );

      /**
       * @brief start auto white balance (this takes 2 seconds to run on the sensor)
       */
      void            startAwb();

      /**
       * @brief The amount of objects found in the last data sample.
       */
      int32_t         objectCount;

      /**
       * @brief The largest/first object found in the last data sample.
       */
      object          largestObject;

      /**
       * @brief An array containing the object(s) found in the data sample.
       */
      safearray< object, AIVISION_MAX_OBJECTS > objects;

    protected:
      void            enableTest( uint8_t value );

      void            usbOverlay( bool bEnable );

      void            statusOverlay( bool bEnable );

      enum class tagFamily {
        tagCircle21h7  = 0,
        tag16h5        = 1,
        tag25h9        = 2,
        tag36h11       = 3
      };
      
      void            setTagFamily( tagFamily f );
    
    private:
      bool            setObjectDescription( const colordesc &desc );
      bool            setObjectDescription( const codedesc &desc );
      bool            setObjectDescription( const tagdesc &desc );
      bool            setObjectDescription( const aiobjdesc &desc );
      bool            setObjectDescription( const uint32_t flags );

      bool            _color_enabled;
      bool            _tags_enabled;
      bool            _aiobj_enabled;
      bool            _merge_enabled;
      bool            _ovl_enabled;
      
      // storage for raw data from camera
      V5_DeviceAiVisionObject _objects[ AIVISION_MAX_OBJECTS ];

      // set all objects to 0
      void            _clearObjects( void );
  };
};


#endif // VEX_AIVISION_CLASS_H
