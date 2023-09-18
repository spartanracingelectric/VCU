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

typedef struct _LaunchControl {
    float slipRatio;
    ubyte2 lcTorque;
    bool LCReady;
    bool LCStatus; // Just for CAN to showcase when enabled
    ubyte1 potLC;
} LaunchControl;

LaunchControl *LaunchControl_new(ubyte1 potLC);
void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *lc);
void launchControlTorqueCalculation(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm);
bool getLaunchControlStatus(LaunchControl *lc);
sbyte2 getCalculatedTorque();

#endif