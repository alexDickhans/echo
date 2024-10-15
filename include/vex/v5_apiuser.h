/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2016, All rights reserved.               */
/*                                                                            */
/*    Module:     v5_apiuser.h                                                */
/*    Author:     James Pearman                                               */
/*    Created:    8 Nov 2016                                                  */
/*                                                                            */
/*    Revisions:  V0.1                                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef V5_APIUSER_H_
#define V5_APIUSER_H_

#include "stdint.h"
#include "stdbool.h"

/*----------------------------------------------------------------------------*/
/** @file    v5_apiuser.h
  * @brief   Header for V5 API device wrapper functions
*//*--------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

void                vexDelay( uint32_t timems );

void                vexLedSet( uint32_t index, V5_DeviceLedColor value );
void                vexLedRgbSet( uint32_t index, uint32_t color);
V5_DeviceLedColor   vexLedGet( uint32_t index );
uint32_t            vexLedRgbGet( uint32_t index );

void                vexAdiPortConfigSet( uint32_t index, uint32_t port, V5_AdiPortConfiguration type );
V5_AdiPortConfiguration vexAdiPortConfigGet( uint32_t index, uint32_t port );
void                vexAdiValueSet( uint32_t index, uint32_t port, int32_t value );
int32_t             vexAdiValueGet( uint32_t index, uint32_t port );

V5_DeviceBumperState  vexBumperGet( uint32_t index );

void                vexGyroReset( uint32_t index );
double              vexGyroHeadingGet( uint32_t index );
double              vexGyroDegreesGet( uint32_t index );

int32_t             vexSonarValueGet( uint32_t index );

int32_t             vexGenericValueGet( uint32_t index );

void                vexMotorVelocitySet( uint32_t index, int32_t velocity );
void                vexMotorVelocityUpdate( uint32_t index, int32_t velocity );
void                vexMotorVoltageSet( uint32_t index, int32_t value );
int32_t             vexMotorVelocityGet( uint32_t index );
int32_t             vexMotorDirectionGet( uint32_t index );
double              vexMotorActualVelocityGet( uint32_t index );
void                vexMotorModeSet( uint32_t index, V5MotorControlMode mode );
V5MotorControlMode  vexMotorModeGet( uint32_t index );
void                vexMotorPwmSet( uint32_t index, int32_t value );
int32_t             vexMotorPwmGet( uint32_t index );
void                vexMotorCurrentLimitSet( uint32_t index, int32_t value );
int32_t             vexMotorCurrentLimitGet( uint32_t index );
void                vexMotorVoltageLimitSet( uint32_t index, int32_t value );
int32_t             vexMotorVoltageLimitGet( uint32_t index );
void                vexMotorPositionPidSet( uint32_t index, V5_DeviceMotorPid *pid );
void                vexMotorVelocityPidSet( uint32_t index, V5_DeviceMotorPid *pid );
int32_t             vexMotorCurrentGet( uint32_t index );
int32_t             vexMotorVoltageGet( uint32_t index );
double              vexMotorPowerGet( uint32_t index );
double              vexMotorTorqueGet( uint32_t index );
double              vexMotorEfficiencyGet( uint32_t index );
double              vexMotorTemperatureGet( uint32_t index );
bool                vexMotorOverTempFlagGet( uint32_t index );
bool                vexMotorCurrentLimitFlagGet( uint32_t index );
uint32_t            vexMotorFaultsGet( uint32_t index );
bool                vexMotorZeroVelocityFlagGet( uint32_t index );
bool                vexMotorZeroPositionFlagGet( uint32_t index );
uint32_t            vexMotorFlagsGet( uint32_t index );
void                vexMotorReverseFlagSet( uint32_t index, bool value );
bool                vexMotorReverseFlagGet( uint32_t index );
void                vexMotorEncoderUnitsSet( uint32_t index, V5MotorEncoderUnits units );
V5MotorEncoderUnits vexMotorEncoderUnitsGet( uint32_t index );
void                vexMotorBrakeModeSet( uint32_t index, V5MotorBrakeMode mode );
V5MotorBrakeMode    vexMotorBrakeModeGet( uint32_t index );
void                vexMotorPositionSet( uint32_t index, double position );
double              vexMotorPositionGet( uint32_t index );
int32_t             vexMotorPositionRawGet( uint32_t index, uint32_t *timestamp );
void                vexMotorPositionReset( uint32_t index );
double              vexMotorTargetGet( uint32_t index );
void                vexMotorServoTargetSet( uint32_t index, double position );
void                vexMotorAbsoluteTargetSet( uint32_t index, double position, int32_t velocity );
void                vexMotorRelativeTargetSet( uint32_t index, double position, int32_t velocity );
void                vexMotorGearingSet( uint32_t index, V5MotorGearset value );
V5MotorGearset      vexMotorGearingGet( uint32_t index );
void                vexMotorExternalProfileSet( uint32_t index, double position, int32_t velocity );

void                vexVisionModeSet( uint32_t index, V5VisionMode mode );
V5VisionMode        vexVisionModeGet( uint32_t index );
int32_t             vexVisionObjectCountGet( uint32_t index );
int32_t             vexVisionObjectGet( uint32_t index, uint32_t indexObj, V5_DeviceVisionObject *pObject );
void                vexVisionSignatureSet( uint32_t index, V5_DeviceVisionSignature *pSignature );
bool                vexVisionSignatureGet( uint32_t index, uint32_t id, V5_DeviceVisionSignature *pSignature );
void                vexVisionBrightnessSet( uint32_t index, uint8_t percent );
uint8_t             vexVisionBrightnessGet( uint32_t index );
void                vexVisionWhiteBalanceModeSet( uint32_t index, V5VisionWBMode mode );
V5VisionWBMode      vexVisionWhiteBalanceModeGet( uint32_t index );
void                vexVisionWhiteBalanceSet( uint32_t index, V5_DeviceVisionRgb color );
V5_DeviceVisionRgb  vexVisionWhiteBalanceGet( uint32_t index );
void                vexVisionLedModeSet( uint32_t index, V5VisionLedMode mode );
V5VisionLedMode     vexVisionLedModeGet( uint32_t index );
void                vexVisionLedBrigntnessSet( uint32_t index, uint8_t percent );
uint8_t             vexVisionLedBrigntnessGet( uint32_t index );
void                vexVisionLedColorSet( uint32_t index, V5_DeviceVisionRgb color);
V5_DeviceVisionRgb  vexVisionLedColorGet( uint32_t index );
void                vexVisionWifiModeSet( uint32_t index, V5VisionWifiMode mode );
V5VisionWifiMode    vexVisionWifiModeGet( uint32_t index );

void                vexImuReset( uint32_t index );
double              vexImuHeadingGet( uint32_t index );
double              vexImuDegreesGet( uint32_t index );
void                vexImuQuaternionGet( uint32_t index, V5_DeviceImuQuaternion *data );
void                vexImuAttitudeGet( uint32_t index, V5_DeviceImuAttitude *data );
void                vexImuRawGyroGet( uint32_t index, V5_DeviceImuRaw *data );
void                vexImuRawAccelGet( uint32_t index, V5_DeviceImuRaw *data );
uint32_t            vexImuStatusGet( uint32_t index );
void                vexImuModeSet( uint32_t index, uint32_t mode );
uint32_t            vexImuModeGet( uint32_t index );
void                vexImuDataRateSet( uint32_t index, uint32_t rate );

int32_t             vexRangeValueGet( uint32_t index );

void                vexAbsEncReset( uint32_t index );
void                vexAbsEncPositionSet( uint32_t index, int32_t position );
int32_t             vexAbsEncPositionGet( uint32_t index );
int32_t             vexAbsEncVelocityGet( uint32_t index );
int32_t             vexAbsEncAngleGet( uint32_t index );
void                vexAbsEncReverseFlagSet( uint32_t index, bool value );
bool                vexAbsEncReverseFlagGet( uint32_t index );
uint32_t            vexAbsEncStatusGet( uint32_t index );
void                vexAbsEncDataRateSet( uint32_t index, uint32_t rate );

double              vexOpticalHueGet( uint32_t index );
double              vexOpticalSatGet( uint32_t index );
double              vexOpticalBrightnessGet( uint32_t index );
int32_t             vexOpticalProximityGet( uint32_t index );
void                vexOpticalRgbGet( uint32_t index, V5_DeviceOpticalRgb *data );
void                vexOpticalLedPwmSet( uint32_t index, int32_t value );
int32_t             vexOpticalLedPwmGet( uint32_t index );
uint32_t            vexOpticalStatusGet( uint32_t index );
void                vexOpticalRawGet( uint32_t index, V5_DeviceOpticalRaw *data );
void                vexOpticalModeSet( uint32_t index, uint32_t mode );
uint32_t            vexOpticalModeGet( uint32_t index );
uint32_t            vexOpticalGestureGet( uint32_t index, V5_DeviceOpticalGesture *pData );
void                vexOpticalGestureEnable( uint32_t index );
void                vexOpticalGestureDisable( uint32_t index );
int32_t             vexOpticalProximityThreshold( uint32_t index, int32_t value );
void                vexOpticalIntegrationTimeSet( uint32_t index, double timems );
double              vexOpticalIntegrationTimeGet( uint32_t index );

void                vexMagnetPowerSet( uint32_t index, int32_t value, int32_t time );
int32_t             vexMagnetPowerGet( uint32_t index );
void                vexMagnetPickup( uint32_t index, V5_DeviceMagnetDuration duration );
void                vexMagnetDrop( uint32_t index, V5_DeviceMagnetDuration duration );
double              vexMagnetTemperatureGet( uint32_t index );
double              vexMagnetCurrentGet( uint32_t index );
uint32_t            vexMagnetStatusGet( uint32_t index );

void                vexLightTowerRgbSet( uint32_t index,  uint32_t rgb_value, uint32_t xyw_value  );
void                vexLightTowerColorSet( uint32_t index, uint32_t color_id, uint32_t value );
uint32_t            vexLightTowerRgbGet( uint32_t index );
uint32_t            vexLightTowerXywGet( uint32_t index );
uint32_t            vexLightTowerStatusGet( uint32_t index );
uint32_t            vexLightTowerDebugGet( uint32_t index, int32_t id );
void                vexLightTowerBlinkSet( uint32_t index, uint8_t select, uint8_t mask, int32_t onTime, int32_t offTime );

uint32_t            vexDistanceDistanceGet( uint32_t index );
uint32_t            vexDistanceConfidenceGet( uint32_t index );
int32_t             vexDistanceObjectSizeGet( uint32_t index );
double              vexDistanceObjectVelocityGet( uint32_t index );
uint32_t            vexDistanceStatusGet( uint32_t index );

void                vexGpsReset( uint32_t index );
double              vexGpsHeadingGet( uint32_t index );
double              vexGpsDegreesGet( uint32_t index );
void                vexGpsQuaternionGet( uint32_t index, V5_DeviceGpsQuaternion *data );
void                vexGpsAttitudeGet( uint32_t index, V5_DeviceGpsAttitude *data, bool bRaw );
void                vexGpsRawGyroGet( uint32_t index, V5_DeviceGpsRaw *data );
void                vexGpsRawAccelGet( uint32_t index, V5_DeviceGpsRaw *data );
uint32_t            vexGpsStatusGet( uint32_t index );
void                vexGpsModeSet( uint32_t index, uint32_t mode );
uint32_t            vexGpsModeGet( uint32_t index );
void                vexGpsDataRateSet( uint32_t index, uint32_t rate );
void                vexGpsOriginSet( uint32_t index, double ox, double oy );
void                vexGpsOriginGet( uint32_t index, double *ox, double *oy );
void                vexGpsRotationSet( uint32_t index, double value );
double              vexGpsRotationGet( uint32_t index );
void                vexGpsInitialPositionSet( uint32_t index, double initial_x, double initial_y, double initial_rotation );
double              vexGpsErrorGet( uint32_t index );

void                vexAiVisionModeSet( uint32_t index, uint32_t mode );
uint32_t            vexAiVisionModeGet( uint32_t index );
int32_t             vexAiVisionObjectCountGet( uint32_t index );
int32_t             vexAiVisionObjectGet( uint32_t index, uint32_t indexObj, V5_DeviceAiVisionObject *pObject );
void                vexAiVisionColorSet( uint32_t index, V5_DeviceAiVisionColor *pColor );
bool                vexAiVisionColorGet( uint32_t index, uint32_t id, V5_DeviceAiVisionColor *pColor );
void                vexAiVisionCodeSet( uint32_t index, V5_DeviceAiVisionCode *pCode );
bool                vexAiVisionCodeGet( uint32_t index, uint32_t id, V5_DeviceAiVisionCode *pCode );
uint32_t            vexAiVisionStatusGet( uint32_t index );
double              vexAiVisionTemperatureGet( uint32_t index );
int32_t             vexAiVisionClassNameGet( uint32_t index, int32_t id, uint8_t *pName );
void                vexAiVisionSensorSet( uint32_t index, double brightness, double contrast );

void                vexPneumaticCompressorSet( uint32_t index, bool bState );
void                vexPneumaticCylinderSet( uint32_t index, uint32_t id, bool bState );
void                vexPneumaticCtrlSet( uint32_t index, V5_DevicePneumaticCtrl *pCtrl );
uint32_t            vexPneumaticStatusGet( uint32_t index );
void                vexPneumaticPwmSet( uint32_t index, uint8_t pwm );
uint32_t            vexPneumaticPwmGet( uint32_t index );
void                vexPneumaticCylinderPwmSet( uint32_t index, uint32_t id, bool bState, uint8_t pwm );
uint32_t            vexPneumaticActuationStatusGet( uint32_t index, uint16_t *ac1, uint16_t *ac2, uint16_t *ac3, uint16_t *ac4 );

void                vexArmPoseSet( uint32_t index, uint8_t pose, uint16_t velocity );
void                vexArmMoveTipCommandLinear( uint32_t index, int32_t x, int32_t y, int32_t z, uint8_t pose, uint16_t velocity, double rotation, uint16_t rot_velocity, bool relative );
void                vexArmMoveTipCommandJoint( uint32_t index, int32_t x, int32_t y, int32_t z, uint8_t pose, uint16_t velocity, double rotation, uint16_t rot_velocity, bool relative );
void                vexArmMoveJointsCommand( uint32_t index, double *positions, uint16_t* velocities, double j6_rotation, uint16_t j6_velocity, double j7_volts, uint16_t j7_timeout, uint16_t j7_i_limit, bool relative );
void                vexArmSpinJoints( uint32_t index, double *velocities );
void                vexArmSetJointPositions( uint32_t index, double *new_positions );
void                vexArmPickUpCommand( uint32_t index );
void                vexArmDropCommand( uint32_t index );
void                vexArmMoveVoltsCommand( uint32_t index, double *voltages );
void                vexArmFullStop( uint32_t index, uint8_t brakeMode );
void                vexArmEnableProfiler( uint32_t index, uint8_t enable );
void                vexArmProfilerVelocitySet(  uint32_t index, uint16_t linear_velocity, uint16_t joint_velocity );
void                vexArmSaveZeroValues( uint32_t index );
void                vexArmForceZeroCommand( uint32_t index );
void                vexArmClearZeroValues( uint32_t index );
void                vexArmBootload( uint32_t index );

void                vexArmTipPositionGet( uint32_t index, int32_t *x, int32_t *y, int32_t *z );
void                vexArmJointInfoGet( uint32_t index, double *positions, double *velocities, int32_t *currents );
double              vexArmJ6PositionGet( uint32_t index );
int32_t             vexArmBatteryGet( uint32_t index );
int32_t             vexArmServoFlagsGet( uint32_t index, uint32_t servoID);
uint32_t            vexArmStatusGet( uint32_t index );
uint32_t            vexArmDebugGet( uint32_t index, int32_t id );
void                vexArmJointErrorsGet( uint32_t index, uint8_t *errors );
void                vexArmJ6PositionSet( uint32_t index, int16_t position );
void                vexArmStopJointsCommand( uint32_t index, int16_t *brakeModes );
void                vexArmReboot( uint32_t index );
void                vexArmTipOffsetSet( uint32_t index, int32_t x, int32_t y, int32_t z );

void                vexArmMoveTipCommandLinearAdv( uint32_t index, V5_DeviceArmTipPosition *position, double j6_rotation, uint16_t j6_velocity, bool relative );
void                vexArmMoveTipCommandJointAdv( uint32_t index, V5_DeviceArmTipPosition *position, double j6_rotation, uint16_t j6_velocity, bool relative );
void                vexArmTipPositionGetAdv( uint32_t index, V5_DeviceArmTipPosition *position );

// Generic serial port comms to any device
void                vexGenericSerialEnable( uint32_t index, int32_t options );
void                vexGenericSerialBaudrate( uint32_t index, int32_t baudrate );
int32_t             vexGenericSerialWriteChar( uint32_t index, uint8_t c );
int32_t             vexGenericSerialWriteFree( uint32_t index );
int32_t             vexGenericSerialTransmit( uint32_t index, uint8_t *buffer, int32_t length );
int32_t             vexGenericSerialReadChar( uint32_t index );
int32_t             vexGenericSerialPeekChar( uint32_t index );
int32_t             vexGenericSerialReceiveAvail( uint32_t index );
int32_t             vexGenericSerialReceive( uint32_t index, uint8_t *buffer, int32_t length );
void                vexGenericSerialFlush( uint32_t index );

// Generic comms using VEXnet radio
void                vexGenericRadioConnection( uint32_t index, char *pName, bool bMaster, bool bAllowRadioOverride );
int32_t             vexGenericRadioWriteChar( uint32_t index, uint8_t c );
int32_t             vexGenericRadioWriteFree( uint32_t index );
int32_t             vexGenericRadioTransmit( uint32_t index, uint8_t *buffer, int32_t length );
int32_t             vexGenericRadioReadChar( uint32_t index );
int32_t             vexGenericRadioPeekChar( uint32_t index );
int32_t             vexGenericRadioReceiveAvail( uint32_t index );
int32_t             vexGenericRadioReceive( uint32_t index, uint8_t *buffer, int32_t length );
void                vexGenericRadioFlush( uint32_t index );
bool                vexGenericRadioLinkStatus( uint32_t index );

#ifdef __cplusplus
}
#endif
#endif /* V5_APIUSER_H_ */