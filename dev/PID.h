/*****************************************************************************
 * pid.h - Proportional-Integral-Derivative (PID) controller
 * Initial Author: Harleen Sandhu / Mehul Williams
 ******************************************************************************
 * General purpose PID controller, initially designed for Torque Vectoring.
 ****************************************************************************/

#ifndef _PID_H
#define _PID_H

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file


// Define a structure for the PID controller
typedef struct _PID {
    float4 kp; // Proportional gain
    float4 ki; // Integral gain
    float4 kd; // Derivative gain
    float4 setpoint; //Reference,Target value
    float4 prev_error; // Previous error
    float4 total_error; // total error 
    float4 dt;//basically the time interval of each sensor value this is in a 
    // dt will be a seperate param in method 
} PID;

PID* PID_new(float4 kp, float4 ki, float4 kd, float4 setpoint);
void PID_setpointUpdate(PID *pid, float4 setpoint);
void PID_dtUpdate(PID *pid, float4 new_dt);
float4 PID_compute(PID *pid, float4 sensorVal);

#endif //_PID_H