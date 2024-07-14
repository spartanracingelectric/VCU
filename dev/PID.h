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
    ubyte1 kp; // Proportional gain
    ubyte1 ki; // Integral gain
    ubyte1 kd; // Derivative gain
    ubyte2 setpoint; //Reference,Target value
    ubyte2 prev_error; // Previous error
    ubyte2 total_error; // total error 
    ubyte4 dt;//basically the time interval of each sensor value this is in a 
    // dt will be a seperate param in method 
} PID;

PID* PID_new(ubyte2 kp, ubyte2 ki, ubyte2 kd, ubyte2 setpoint);
void PID_setpointUpdate(PID *pid, ubyte2 setpoint);
void PID_dtUpdate(PID *pid, ubyte4 new_dt);
ubyte2 PID_compute(PID *pid, ubyte2 sensorVal);

#endif //_PID_H