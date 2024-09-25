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
    float Kp;               // Proportional gain
    float Ki;               // Integral     gain
    float Kd;               // Derivative   gain
    float setpoint;         // Target value
    float previousError;
    float totalError;
    float dt;               // Time interval between PID updates in seconds (VCU tick speed)
} PID;

PID*  PID_new(float Kp, float Ki, float Kd, float setpoint);
void PID_resetpidOffset(PID* pid, float4 error);
void  PID_setpointUpdate(PID *pid, float setpoint);
void  PID_setGain(PID *pid, float Kp, float Ki, float Kd);
void  PID_dtUpdate(PID *pid, float dt);
float PID_computeOffset(PID *pid, float sensorValue);
float PID_efficiencycheck(PID *pid, float commandedTQ, float motorRPM, float idealTQ);
#endif //_PID_H