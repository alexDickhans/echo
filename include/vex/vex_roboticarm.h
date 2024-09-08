/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2021, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_roboticarm.h                                            */
/*    Author:     VEX Robotics                                                */
/*    Created:    15 March 2021                                               */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef VEX_ROBOT_ARM_H
#define VEX_ROBOT_ARM_H

namespace vex {
  class RoboticArm
  {
  public:

    struct Position {
      double x, y, z, a;
    };

    struct Angle {
      double j1, j2, j3, j4;
    };

  private:

    // This describes tool tip positions
    struct Quad {
      double  a, b, c, d;
    };

    #define PFLAG_CHANGE_POSITION 1
    #define PFLAG_CHANGE_FILTER   2
    #define PFLAG_CHANGE_VELOCITY 4

    // These are the physical dimensions of the robot arm
    #define ARM_A 7.0
    #define ARM_B 7.0
    #define ARM_C 1.9509
    #define ARM_D 0.0555
    #define ARM_P0r 0.7224
    #define ARM_P0z 5.296
    #define ARM_z3 0.7

    // These variables are for the double boxcar filter used for velocity profiling
    #define FILTERSIZE  1000
    Quad Vfilter[FILTERSIZE];       // Velocity boxcar filter static array
    Quad Afilter[FILTERSIZE];       // Acceleration boxcar filter static array
    Quad VfilterSum, AfilterSum;    // Running sums for the two filters
    int Vhead, Vtail, Ahead, Atail; // Pointer integers into the boxcar filters circular buffers
    Angle Last;                     // Remember the last target angles in order to compute the expected motor velocities

    // Variables used to keep track of position movements and the profiling
    #define LOOPTIME  0.01  // 100 times per second
    double  PosVelocity = 10.0, AngVelocity = 90.0;
    double  TargetX, TargetY, TargetZ, TargetA; // This is the command position coming from the user or user's exective program. This is where we want the motor to go to.
    double  PosX, PosY, PosZ, PosA; // This is the integration of the filtered velocity - the target profile we want the motor to follow
    double  UnfilX, UnfilY, UnfilZ, UnfilA; // This is the instantaneous integrated velocity. This tells us where the motor is going to end up after it goes through its profile
    int     VFilterLen = 50, AFilterLen = 50; // This is the size of the boxcar filters
    int     VFilterBuf, AFilterBuf;
    int     ProfileLoopFlag = 0;
    double  BufX, BufY, BufZ, BufA;
  
    double J1_OFFSET = 0; // This relates the measured values to the mastering values
    double J2_OFFSET = 0;
    double J3_OFFSET = 0;
    double J4_OFFSET = 0;

    double  I1 = 0.0, I2 = 0.0, I3 = 0.0, I4 = 0.0; // Motor integrators

    Position Offset = {.x = 0.0, .y = 0.0, .z = 0.0, .a = 0.0};

    // These are the kinematic constants for the robot arm
    double Arm_dr = 0.0;
    double Arm_dp = 0.0;
    double Arm_dz = 0.0;

    motor &mJ1;
    motor &mJ2;
    motor &mJ3;
    motor &mJ4;
    pot &mJ1_pot;
    pot &mJ2_pot;
    pot &mJ3_pot;
    pot &mJ4_pot;

    bool hasMasterValues = false;
    bool emergencyStopped = false;

    // local changes start
    double joint4Angle = 0.0;
    double linearSpeed = 5;
    double jointSpeed = 25;
    task motionProfileTask;
    bool isLinearMotionMode = true;
    bool filterIsSet = false;
    bool toolOffsetIsSet = false;

    void startProfilerTask();
    void stopProfilerTask();
    bool profilerIsRunning = false;
    bool profilerThreadDisabled = false;
    // local changes end


  public:

    RoboticArm(motor& joint1Motor, pot& joint1Pot, motor& joint2Motor, pot& joint2Pot, motor& joint3Motor, pot& joint3Pot, motor& joint4Motor, pot& joint4Pot);

    void setMasteringValues(int joint1Value, int joint2Value, int joint3Value, int joint4Value);

    // local start
    void moveToPositionLinear(double x, double y, double z);
    void moveToPositionLinear(double x, double y, double z, double a);
    void moveToPositionJoint(double x, double y, double z);
    void moveToPositionJoint(double x, double y, double z, double a);

    void setArmAngle(double angle);

    int getJointValue(int jointNumber);
    double getAxisPosition(axisType axis);

    void setLinearMoveSpeed(double speed);
    void setJointMoveSpeed(double speed);


    void setRelativePosition(double x, double y, double z);

    void enableManualMovement();

    int getInfo();
    // local end

    void getRelativePosition(Position &output);
    void setRelativePosition(double x, double y, double z, double a);
    void moveToRelativePosition(double x, double y, double z, double a);
    void moveToPosition(double x, double y, double z, double a);
    void setProfileVelocity(double pvel, double avel);
    void setToolTipOffset(double dr, double dp, double dz);
    void setProfileFilter(int vlen, int alen);
    int positionArrived(void);
    void stow(void);
    void getPosition(Position &output);
    int positionProfileLoopLinear(void);
    int positionProfileLoopJoint(void);
    void emergencyStop();
    // void readMasteringValues(void);
    void enterMasteringMode(void);

  private:

    void velocityFilter(double vx, double vy, double vz, double va);
    void initializeProfilePosition(void);
    void initializeProfileAngle(void);
    void initializeVelocityFilter(void);
    void motorControlLoop(Angle &angs);
    void complimentaryFilterAngles(Angle &result);
    void readBestAngles(Angle &result);
    void initializeComplimentaryFilter(void);
    void adjustProfileFilter(int vlen, int alen);
    int checkMasteringValue(int masteringValue, int jointNumber, int min, int max);
    double getJ1(void);
    double getJ2(void);
    double getJ3(void);
    double getJ4(void);
    void forwardKinematicSolve(double t, double a1, double b1, double c1, Position &output);
    void inverseKinematicSolve(double x,double y,double z,double a, Angle &output);
    
    static int profilerTaskHelper(void* armPtr);
  };
};

#endif  // VEX_ROBOT_ARM_H

