#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include "IO_Driver.h"
#include "IO_CAN.h"

#define CYCLE_TIME 0.01f

typedef struct _PowerLimit
{
    float setPoint;
    float kp;
    float ki;
    float integral;
    float prevError;


} PowerLimit;

PowerLimit *powerLimitNew(ubyte2 limit, float p, float i);
float control(PowerLimit *me, float power);
// float PL_getKp(PowerLimit *me);
// float PL_getKi(PowerLimit *me);

#endif