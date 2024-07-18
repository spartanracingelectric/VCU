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
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float setpoint; //Reference,Target value
    float prev_error; // Previous error
    float total_error; // total error 
    float dt;//basically the time interval of each sensor value this is in a 
    // dt will be a seperate param in method 
} PID;

PID* PID_new(float kp, float ki, float kd, float setpoint);
void PID_setpointUpdate(PID *pid, float setpoint);
void PID_setgain(PID *pid, float kp, float ki, float kd);
void PID_dtUpdate(PID *pid, float new_dt);
float PID_compute(PID *pid, float sensorVal);

#endif //_PID_H