#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_DIO.h"

#include "powerLimit.h"

PowerLimit *powerLimitNew(ubyte2 limit, float p, float i) {
    PowerLimit *me = (PowerLimit *)malloc(sizeof(struct _PowerLimit));
    me->setPoint = (float)limit; //kw
    me->prevError = 0.0f;
    me->integral = 0.0f;

    me->kp = p;
    me->ki = i;
    

    return me;
}

float control(PowerLimit *me, float power) {
    float error = me->setPoint - power;

    float proportional = me->kp * error;

    me->integral+=(error * CYCLE_TIME);
    float integral = me->ki * me->integral;

    return proportional + integral;
}

// float PL_getKp(PowerLimit *me) {
//     return me->kp;
// }

// float PL_getKi(PowerLimit *me) {
//     return me->ki;
// }