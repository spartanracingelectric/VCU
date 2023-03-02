#ifndef _LAUNCHCONTROL_H
#define _LAUNCHCONTROL_H

#include "IO_Driver.h"

#include "motorController.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"

typedef struct _LaunchControl LaunchControl;

LaunchControl *LaunchControl_new(void);
void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *lc);
void launchControlTorqueCalculation(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm0);
bool getLaunchControlStatus(LaunchControl *lc);

#endif