#ifndef _SAFETY_H
#define _SAFETY_H

#include "IO_Driver.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "sensors.h"
#include "motorController.h"
#include "bms.h"
#include "wheelSpeeds.h"

typedef struct _SafetyChecker SafetyChecker;

SafetyChecker *SafetyChecker_new(ubyte2 maxChargeAmps, ubyte2 maxDischargeAmps);
void SafetyChecker_update(SafetyChecker *me, MotorController *mcm, BatteryManagementSystem *bms, TorqueEncoder *tps, BrakePressureSensor *bps, Sensor *HVILTermSense, Sensor *LVBattery);
void SafetyChecker_parseCanMessage(SafetyChecker *me, IO_CAN_DATA_FRAME *canMessage);
bool SafetyChecker_allSafe(SafetyChecker *me);
ubyte4 SafetyChecker_getFaults(SafetyChecker *me);
ubyte4 SafetyChecker_getWarnings(SafetyChecker *me);
ubyte4 SafetyChecker_getNotices(SafetyChecker *me);
void SafetyChecker_reduceTorque(SafetyChecker *me, MotorController *mcm, BatteryManagementSystem *bms, WheelSpeeds *wss);
ubyte1 SafetyChecker_getsoftbspd(SafetyChecker *me);
//bool SafetyChecker_getError(SafetyChecker* me, SafetyCheck check);
//bool SafetyChecker_getErrorByte(SafetyChecker* me, ubyte1* errorByte);

//ubyte2 checkPowerDraw(BatteryManagementSystem* bms, MotorController* mcm);
void checkBatteryPackTemp(BatteryManagementSystem *bms);
ubyte2 checkPowerDraw(BatteryManagementSystem *bms, MotorController *mcm);

#endif //  _SAFETY_H