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

typedef struct _PIDController {
    float4 kp;         // Proportional gain
    float4 ki;         // Integral gain
    float4 kd;         // Derivative gain
    float4 errorSum;   // Running sum of errors for the integral term
    float4 lastError;  // Previous error for the derivative term
} PIDController;

typedef struct _LaunchControl {
    float4 slipRatio;
    ubyte2 lcTorque;
    bool LCReady;
    bool LCStatus; // Just for CAN to showcase when enabled
    PIDController *pidController;
    ubyte1 potLC;
} LaunchControl;

LaunchControl *LaunchControl_new(ubyte1 potLC);
void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *lc);
void launchControlTorqueCalculation(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm);
bool getLaunchControlStatus(LaunchControl *lc);
sbyte2 getCalculatedTorque();

#endif