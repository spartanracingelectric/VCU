#ifndef _PID_H
#define _PID_H

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
typedef struct _PID {
    ubyte2 Kp;               // Proportional gain
    ubyte2 Ki;               // Integral     gain
    ubyte2 Kd;               // Derivative   gain
    ubyte2 setpoint;         // Target value
    sbyte2 previousError;
    sbyte2 totalError;
    float dt;               // Time interval between PID updates in seconds (VCU tick speed)
} PID;

PID* PID_new(ubyte2 Kp, ubyte2 Ki, ubyte2 Kd, ubyte2 setpoint);
void  PID_setpointUpdate(PID *pid, ubyte2 setpoint);
void  PID_dtUpdate(PID *pid, float dt);
sbyte2 PID_compute(PID *pid, ubyte2 sensorValue);
void  PID_setGain(PID *pid, ubyte2 Kp, ubyte2 Ki, ubyte2 Kd);

#endif