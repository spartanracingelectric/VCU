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
    PID *pid;

    float4 slipRatio;
    sbyte2 slipRatioThreeDigits;
    bool lcReady;
    bool lcActive; // Just for CAN to showcase when enabled
    ubyte1 buttonDebug;
    sbyte2 lcTorqueCommand;
    bool constantSpeedTest; // flag for speed mode override
    sbyte2 speedCommand;
} LaunchControl;

LaunchControl *LaunchControl_new();
void LaunchControl_calculateSlipRatio(LaunchControl *lc, WheelSpeeds *wss);
void LaunchControl_calculateTorqueCommand(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs);
void LaunchControl_checkState(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs);
bool LaunchControl_getStatus(LaunchControl *lc);
sbyte2 LaunchControl_getTorqueCommand(LaunchControl *lc);
float LaunchControl_getSlipRatio(LaunchControl *lc);
sbyte2 LaunchControl_getSlipRatioThreeDigits(LaunchControl *lc);
ubyte1 LaunchControl_getButtonDebug(LaunchControl *lc);

#endif