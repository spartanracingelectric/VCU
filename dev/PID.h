/*****************************************************************************
 * pid.h - Proportional-Integral-Derivative (PID) controller
 * Initial Author: Harleen Sandhu / Mehul Williams
 * Additional Author: Shaun Gilmore
 ******************************************************************************
 * General purpose PID controller, initially designed for Torque Vectoring.
 ****************************************************************************/

#ifndef _PID_H
#define _PID_H

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

// Define a structure for the PID controller
typedef struct _PID {
    float4 Kp;               // Proportional gain
    float4 Ki;               // Integral     gain
    float4 Kd;               // Derivative   gain
    float4 setpoint;         // Target       value
    float4 previousError;
    float4 totalError;
    float4 dt;               // Time interval between PID updates in seconds (VCU tick speed)
} PID;

PID*  PID_new(float Kp, float Ki, float Kd, float setpoint);
void PID_resetpidOffset(PID* pid, float4 error);
void  PID_setpointUpdate(PID *pid, float setpoint);
float PID_compute(PID *pid, float sensorValue);

#endif //_PID_H