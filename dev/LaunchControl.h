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
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

/*
typedef struct _PIDController {
    float kp;         // Proportional gain
    float ki;         // Integral gain
    float kd;         // Derivative gain
    float errorSum;   // Running sum of errors for the integral term
    float lastError;  // Previous error for the derivative term
} PIDController;
*/
typedef struct _LaunchControl {
    PID *pid;



    sbyte2 slipRatioThreeDecimals;
    bool lcReady;
    bool lcActive; // Just for CAN to showcase when enabled
    ubyte1 buttonDebug;
    sbyte2 lcTorqueCommand;
    sbyte2 lcCommandedTorque;

} LaunchControl;

LaunchControl *LaunchControl_new();
void LaunchControl_calculateSlipRatio(LaunchControl *me, WheelSpeeds *wss);
void LaunchControl_calculateTorqueCommand(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm);
void LaunchControl_checkState(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm);
bool LaunchControl_getStatus(LaunchControl *lc);
sbyte2 LaunchControl_getCalculatedTorque(LaunchControl *lc);
ubyte1 LaunchControl_getButtonDebug(LaunchControl *lc);

#endif