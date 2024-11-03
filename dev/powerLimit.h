#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H
#include "IO_Driver.h" 
#include "motorController.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"

typedef struct _PowerLimit {
    bool PLStatus;
    ubyte2 value;
}PowerLimit;

PowerLimit* PL_new();
void testing(PowerLimit *me);
void powerLimitTorqueCalculation(PowerLimit *me,  MotorController* mcm);
#endif