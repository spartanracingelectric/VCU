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
void  PID_updateSetpoint(PID *pid, float setpoint);
float PID_computeOffset(PID *pid, float sensorValue);

#endif //_PID_H