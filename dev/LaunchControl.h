
#ifndef _LAUNCHCONTROL_H
#define _LAUNCHCONTROL_H

#include "IO_Driver.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"
#include "PID.h"
#include "drs.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
typedef struct _LaunchControl {
    PID *pidTorque;
    sbyte2 lcTorqueCommand;
    sbyte2 initialTorque;

    float4 slipRatio;
    sbyte2 slipRatioThreeDigits;

    ubyte4 safteyTimer;
    ubyte1 lcReady;
    ubyte1 lcActive;
    ubyte1 initialCurve;
    ubyte1 overTorque;
    ubyte1 flags;


    PID *pidSpeed;
    sbyte2 lcSpeedCommand;

    ubyte1 constantSpeedTestOverride; // flag for speed mode override
    sbyte2 overrideTestSpeedCommand;

    ubyte1 buttonDebug;
} LaunchControl;

LaunchControl *LaunchControl_new();
#ifdef LAUNCHCONTROL_ENABLE
void LaunchControl_calculateSlipRatio(LaunchControl *lc, MotorController *mcm, WheelSpeeds *wss);
void LaunchControl_calculateTorqueCommand(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs);
void LaunchControl_checkState(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs);
ubyte1 LaunchControl_getStatus(LaunchControl *lc);
sbyte2 LaunchControl_getTorqueCommand(LaunchControl *lc);
void LaunchControl_initialTorqueCurve(LaunchControl* me, MotorController* mcm);
void LaunchControl_initialRPMCurve(LaunchControl* me, MotorController* mcm);
float LaunchControl_getSlipRatio(LaunchControl *lc);
sbyte2 LaunchControl_getSlipRatioThreeDigits(LaunchControl *lc);
ubyte1 LaunchControl_getButtonDebug(LaunchControl *lc);

#endif //_LAUNCHCONTROL_H

#endif