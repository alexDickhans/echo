/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_ctearm.h                                                */
/*    Author:     James Pearman                                               */
/*    Created:    20 June 2023                                                */
/*                                                                            */
/*    Revisions:  V0.1                                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef VEX_CTE_ARM_H_
#define VEX_CTE_ARM_H_

namespace vex {
  namespace cte {
    class arm : public vex::device {
        public: 
            enum class tEventType {
              EVENT_COMMAND_COMPLETE   = 0,
              EVENT_CRASH_DETECT       = 2
            };

            enum class ArmPose {
              z_neg = 1,
              x_pos = 2,
              y_pos = 3,
              y_neg = 4,
              x_neg = 5,
              z_pos = 6,
              pen   = 7
            };

            enum class ArmJoint {
              joint1 = 0,
              joint2,
              joint3,
              joint4,
              joint5,
              joint6
            };

            /**
             @brief The defined units for brake values.
            */
            enum class armBrakeType {
              defaultMode = 0,
              coast = 1,
              brake = 2,
              hold  = 3,
              undefined
            };

            enum class ArmError {
              noError               = 0,
              badCommand            = 1,
              commandFailed         = 2,
              invalidPosition       = 3,
              invalidPoseTransition = 4,
              invalidPath           = 5
            };
          
            #define ARM_SERVO_COUNT 5
          
        public:
          /** 
           * @brief Creates a new cte arm object on the port specified.
           * @param index The port index for this cte arm. The index is zero-based.
           */
          arm( int32_t index );
          ~arm();

          bool            installed();
          bool            isConnected();

          /** 
           * @brief Sets the timeout for the arm. If the arm does not reach commanded positions prior to the completion of the timeout, the arm will stop.
           * @param time Sets the amount of time.
           * @param units The measurement unit for the time value.
           */
          void            setTimeout( int32_t time, timeUnits units );

          /** 
           * @brief Gets the current position of an arm joint
           * @param joint The joint index
           * @return The joint angle in degrees
           */
          float           getJointPosition(ArmJoint joint);
          /** 
           * @brief Gets the current velocity of an arm joint
           * @param joint The joint index
           * @return The joint velocity in deg/sec
           */
          float           getJointVelocity(ArmJoint joint);
          /** 
           * @brief Gets the motor current for an arm joint
           * @param joint The joint index
           * @return The joint current in amps
           */
          float           getJointCurrent(ArmJoint joint);
          /** 
           * @brief Gets the motor voltage for an arm joint (not implemented !)
           * @param joint The joint index
           * @return The joint voltage in volts
           */
          float           getJointVoltage(ArmJoint joint);

          /**
           * @brief      Move tip linearly to x,y,z position
           * @param[in]  x position in mm
           * @param[in]  y position in mm
           * @param[in]  z position in mm
           * @param[in]  relative move is an offset from current position. Default: false
           * @param[in]  waitForCompletion block until move completes. Default: true
           * @return true if successful
           */
          bool            moveToPositionLinear(double x, double y, double z, bool relative = false, bool waitForCompletion = true);
          /**
           * @brief      Move tip to x,y,z position
           * @param[in]  x position in mm
           * @param[in]  y position in mm
           * @param[in]  z position in mm
           * @param[in]  relative move is an offset from current position. Default: false
           * @param[in]  waitForCompletion block until move completes. Default: true
           * @return true if successful
           */
          bool            moveToPositionJoint(double x, double y, double z, bool relative = false, bool waitForCompletion = true);

          /**
           * @brief      Get the angle of the end effector
           * @returns    The angle in degrees.
           */
          float           getEndEffectorAngle();
          /**
           * @brief      Get the X position of the tip
           * @returns    The position in mm.
           */
          float           getX();
          /**
           * @brief      Get the Y position of the tip
           * @returns    The position in mm.
           */
          float           getY();
          /**
           * @brief      Get the Z position of the tip
           * @returns    The position in mm.
           */
          float           getZ();
          /**
           * @brief      Get the roll position of the tip
           * @returns    The position in degrees.
           */
          float           getRoll();
          /**
           * @brief      Get the pitch position of the tip
           * @returns    The position in degrees.
           */
          float           getPitch();
          /**
           * @brief      Get the yaw position of the tip
           * @returns    The position in degrees.
           */
          float           getYaw();

          /**
           * @brief      Set the direction of the end effector
           * @param[in]  pose ArmPose position
           * @param[in]  waitForCompletion block until move completes. Default: true
           * @return true if successful
           */
          bool            setPose(ArmPose pose, bool waitForCompletion = true);
          /**
           * @brief      Sets end effector magnet on or off
           * @param[in]  enabled true to enable magnet
           */
          void            setEndEffectorMagnet(bool enabled); 
          /**
           * @brief      Spin end effector to an angle
           * @param[in]  angle in degrees
           * @param[in]  speed in degrees per second
           * @param[in]  waitForCompletion block until move completes. Default: true
           */
          bool            spinEndEffectorTo(int32_t angle, int32_t speed = 45, bool waitForCompletion = true );
          /**
           * @brief      Spin end effector to an angle (relative to current position)
           * @param[in]  angle in degrees
           * @param[in]  speed in degrees per second
           * @param[in]  waitForCompletion block until move completes. Default: true
           */
          bool            spinEndEffectorFor(int32_t angle, int32_t speed = 45, bool waitForCompletion = true );
          /**
           * @brief      Set the end effector angle
           * @param[in]  angle in degrees  
           */
          void            setEndEffectorAngle(int32_t angle);
          /**
           * @brief      Move tip offset to zOffset position
           * @param[in]  zOffset position in mm, positive is up
           * @return true if successful
           */
          bool            setPenOffset(double zOffset);
          /**
           * @brief      Allows the arm to be manually positioned, disable the profiler
           * @param[in]  angle in degrees  
           */
          void            enableManualMovement();

          /**
           * @brief      Stop the arm and set motors to brake
           * @param[in]  state set true to disable further movement commands
           */
          void            setControlStop(bool state = false);
          /**
           * @brief      Set the linear speed of the arm (Only applies to linear moves)
           * @param[in]  speed speed in mm/s
           */
          void            setLinearMoveSpeed(uint32_t speed);
          /**
           * @brief      Set the speed of the arm (only applies to joint moves)
           * @param[in]  speed speed in mm/s
           */
          void            setJointMoveSpeed(uint32_t speed);
          /**
           * @brief      check if the arm has reached its last commanded position.
           * @returns    true if the arm has reached the command position
           */
          bool            isDone(void);
          /**
           * @brief      check if the arm has been disabled by control stop
           * @returns    true if the arm movement is disabled
           */
          bool            isControlStopEnabled(void);
          /**
           * @brief      check if the arm needs to be zeroed
           * @returns    true if the arm needs to be zeroed
           */
          bool            isZeroNeeded(void);
          /**
           * @brief Sets the function to be called when a move command is completed.
           * @param callback A reference to a function.
          */
          void            commandComplete( void (* callback)(void) );
          /**
           * @brief Sets the function to be called when a crash is detected
           * @param callback A reference to a function.
          */
          void            crashDetect( void (* callback)(void) );

          /**
           * @brief Move the arm to a known safe position when used with the workcell
           * @param[in]  waitForCompletion block until move completes. Default: true
          */
          bool            moveToSafePosition(bool waitForCompletion = true);

          uint32_t        lastError();
        protected:
          int32_t         getTimeout();

          bool            stow(bool waitForCompletion = true);

          bool            moveToPositionLinear(double x, double y, double z, double pitch, double roll, double yaw, bool relative = false, bool waitForCompletion = true);
          bool            moveToPositionJoint(double x, double y, double z, double pitch, double roll, double yaw, bool relative = false, bool waitForCompletion = true);

        private:
          int32_t         _timeout;
          uint16_t        _linearMoveSpeed;
          uint16_t        _jointMoveSpeed;
          ArmPose         _armPose;
          bool            _controlStop;

          void            setVolts( double *voltages) ;
          void            saveZeroValues();
          void            clearZeroValues();
          void            spinJoints(double *velocities);
          void            setPositions( double *new_positions );
          void            stopJoint(ArmJoint joint, armBrakeType brakeMode = armBrakeType::defaultMode);
          void            setJointAngle(ArmJoint joint, int32_t angle, int32_t speed );
          int32_t         getVbat(void);
          uint32_t        getStatus(void);
          uint32_t        getJ6Status(void);
          uint8_t         getJointFlags(ArmJoint joint);
          int             getJointErrors(ArmJoint joint);
      };

      extern const arm::ArmJoint joint1;
      extern const arm::ArmJoint joint2;
      extern const arm::ArmJoint joint3;
      extern const arm::ArmJoint joint4;
      extern const arm::ArmJoint joint5;
      extern const arm::ArmJoint joint6;

      extern const arm::ArmPose  z_neg;
      extern const arm::ArmPose  x_pos;
      extern const arm::ArmPose  y_pos;
      extern const arm::ArmPose  y_neg;
      extern const arm::ArmPose  x_neg;
      extern const arm::ArmPose  z_pos;
  };
};

namespace vex {
  namespace cte {
    class arm_advanced : public vex::cte::arm {
        public: 
          /** 
           * @brief Creates a new cte advanced arm object on the port specified.
           * @param index The port index for this cte arm. The index is zero-based.
           */
          arm_advanced( int32_t index );
          ~arm_advanced();

          using           arm::moveToPositionLinear;
          using           arm::moveToPositionJoint;

          /**
           * @brief      Move tip linearly to x,y,z position and adjust end effector to manual pose
           * @param[in]  x position in mm
           * @param[in]  y position in mm
           * @param[in]  z position in mm
           * @param[in]  pitch end effector position in degrees
           * @param[in]  roll end effector position in degrees
           * @param[in]  yaw end effector position in degrees
           * @param[in]  relative move is an offset from current position. Default: false
           * @param[in]  waitForCompletion block until move completes. Default: true
           * @return true if successful
           */
          bool            moveToPositionLinear(double x, double y, double z, double pitch, double roll, double yaw, bool relative = false, bool waitForCompletion = true);
          /**
           * @brief      Move tip to x,y,z position and adjust end effector to manual pose
           * @param[in]  x position in mm
           * @param[in]  y position in mm
           * @param[in]  z position in mm
           * @param[in]  pitch end effector position in degrees
           * @param[in]  roll end effector position in degrees
           * @param[in]  yaw end effector position in degrees
           * @param[in]  relative move is an offset from current position. Default: false
           * @param[in]  waitForCompletion block until move completes. Default: true
           * @return true if successful
           */
          bool            moveToPositionJoint(double x, double y, double z, double pitch, double roll, double yaw, bool relative = false, bool waitForCompletion = true);
    };
  };
};

#endif // VEX_CTE_ARM_H_