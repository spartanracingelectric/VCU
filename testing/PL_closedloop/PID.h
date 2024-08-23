/*****************************************************************************
 * pid.h - Proportional-Integral-Derivative (PID) controller
 * Initial Author: Harleen Sandhu / Mehul Williams
 * Additional Author: Shaun Gilmore
 ******************************************************************************
 * General purpose PID controller, initially designed for Torque Vectoring.
 ****************************************************************************/

#ifndef _PID_H
#define _PID_H

// Define a structure for the PID controller
typedef struct _PID {
    int Kp;               // Proportional gain
    int Ki;               // Integral     gain
    int Kd;               // Derivative   gain
    int setpoint;         // Target       value
    int previousError;
    int totalError;
    int dH;               // Time interval between PID updates in seconds (VCU tick speed)
    int output;
}PID;

/* Kp, Ki, & Kd are in deci- units, meaning PID_new(10,0,0,500) gives a Kp of 1.0 and a setpoint of 500 */
PID* PID_new(int Kp, int Ki, int Kd, int setpoint);
void PID_setTotalError(PID* pid, int totalError);
void PID_updateSetpoint(PID *pid, int setpoint);
void PID_updateGainValues(PID* pid, int Kp, int Ki, int Kd);
int PID_computeOutput(PID *pid, int sensorValue);

/** GETTER FUNCTIONS **/

int PID_getKp(PID *pid);
int PID_getKi(PID *pid);
int PID_getKd(PID *pid);
int PID_getSetpoint(PID *pid);
int PID_getTotalError(PID* pid);
int PID_getOutput(PID* pid);

#endif //_PID_H