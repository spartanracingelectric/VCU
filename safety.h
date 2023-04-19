#ifndef _SAFETY_H
#define _SAFETY_H

#include "IO_Driver.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "sensors.h"
#include "motorController.h"
#include "bms.h"
#include "serial.h"
#include "wheelSpeeds.h"
#include "AMKdrive.h"

/*
typedef enum { CHECK_tpsOutOfRange    , CHECK_bpsOutOfRange
             , CHECK_tpsOpenOrShort   , CHECK_bpsOpenOrShort
             , CHECK_tpsNotCalibrated , CHECK_bpsNotCalibrated 
             , CHECK_tpsOutOfSync     , CHECK_tpsbpsImplausible
             } SafetyCheck;
*/

typedef struct _SafetyChecker SafetyChecker;

SafetyChecker *SafetyChecker_new(ubyte2 maxChargeAmps, ubyte2 maxDischargeAmps);
void SafetyChecker_update(SafetyChecker *me, MotorController *mcm, BatteryManagementSystem *bms, TorqueEncoder *tps, BrakePressureSensor *bps, Sensor *HVILTermSense, Sensor *LVBattery);
void SafetyChecker_parseCanMessage(SafetyChecker *me, IO_CAN_DATA_FRAME *canMessage);
bool SafetyChecker_allSafe(SafetyChecker *me);
ubyte4 SafetyChecker_getFaults(SafetyChecker *me);
ubyte4 SafetyChecker_getWarnings(SafetyChecker *me);
ubyte4 SafetyChecker_getNotices(SafetyChecker *me);
void SafetyChecker_reduceTorque(SafetyChecker *me, MotorController *mcm, BatteryManagementSystem *bms, WheelSpeeds *wss, _DriveInverter *in1, _DriveInverter *in2, _DriveInverter *in3, _DriveInverter *in4);
//bool SafetyChecker_getError(SafetyChecker* me, SafetyCheck check);
//bool SafetyChecker_getErrorByte(SafetyChecker* me, ubyte1* errorByte);

//ubyte2 checkPowerDraw(BatteryManagementSystem* bms, MotorController* mcm);
void checkBatteryPackTemp(BatteryManagementSystem *bms);
ubyte2 checkPowerDraw(BatteryManagementSystem *bms, MotorController *mcm);

#endif //  _SAFETY_H