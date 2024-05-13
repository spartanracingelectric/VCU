#ifndef _SAFETY_H
#define _SAFETY_H

#include "IO_Driver.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "sensors.h"
#include "motorController.h"
#include "bms.h"
#include "wheelSpeeds.h"

/*
typedef enum { CHECK_tpsOutOfRange    , CHECK_bpsOutOfRange
             , CHECK_tpsOpenOrShort   , CHECK_bpsOpenOrShort
             , CHECK_tpsNotCalibrated , CHECK_bpsNotCalibrated
             , CHECK_tpsOutOfSync     , CHECK_tpsbpsImplausible
             } SafetyCheck;
*/

/*****************************************************************************
 * SafetyChecker object
 ******************************************************************************
 * ToDo: change to ubyte1[8] (64 flags possible)
 * 1 = fault
 * 0 = no fault
 ****************************************************************************/

typedef struct _SafetyChecker
{
    // Problems that require motor torque to be disabled
    ubyte4 faults;
    ubyte2 warnings;
    ubyte2 notices;
    ubyte1 maxAmpsCharge;
    ubyte1 maxAmpsDischarge;

    bool softBSPD_bpsHigh;
    bool softBSPD_kwHigh;
    bool softBSPD_fault;

    bool bypass;
    ubyte4 timestamp_bypassSafetyChecks;
    ubyte4 bypassSafetyChecksTimeout_us;
} SafetyChecker;

void SafetyChecker_new(SafetyChecker *me, ubyte2 maxChargeAmps, ubyte2 maxDischargeAmps);
void SafetyChecker_update(SafetyChecker *me);
void SafetyChecker_reduceTorque(SafetyChecker *me);
ubyte1 SafetyChecker_getsoftbspd(SafetyChecker *me);
void set_flags(ubyte4 *fault, ubyte4 flag, bool condition);
void checkBatteryPackTemp(BatteryManagementSystem *bms);
ubyte2 checkPowerDraw(BatteryManagementSystem *bms);

#endif //  _SAFETY_H