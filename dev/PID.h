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
    ubyte2 Kp;               // Proportional gain
    ubyte2 Ki;               // Integral     gain
    ubyte2 Kd;               // Derivative   gain
    ubyte2 setpoint;         // Target       value
    sbyte2 previousError;
    sbyte2 totalError;
    ubyte4 dH;               // Time interval between PID updates in seconds (VCU tick speed)
    sbyte2 output;
    sbyte2 proportional;
    sbyte2 integral;
    sbyte2 derivative;
    ubyte2 saturationValue;
    bool antiWindupFlag;
}PID;

PID* PID_new(ubyte2 Kp, ubyte2 Ki, ubyte2 Kd, ubyte2 saturationValue);

/** SETTER FUNCTIONS  **/

void PID_setTotalError(PID* pid, sbyte2 totalError);
void PID_setSaturationPoint(PID *pid, ubyte2 saturationValue);
void PID_updateSetpoint(PID *pid, ubyte2 setpoint);
void PID_updateGainValues(PID* pid, ubyte2 Kp, ubyte2 Ki, ubyte2 Kd);

/** COMPUTATIONS **/

void PID_computeOutput(PID *pid, sbyte2 sensorValue);

/** GETTER FUNCTIONS **/

ubyte2 PID_getKp(PID *pid);
ubyte2 PID_getKi(PID *pid);
ubyte2 PID_getKd(PID *pid);
ubyte2 PID_getSetpoint(PID *pid);
sbyte2 PID_getPreviousError(PID *pid);
sbyte2 PID_getTotalError(PID* pid);
sbyte2 PID_getOutput(PID *pid);
sbyte2 PID_getProportional(PID *pid);
sbyte2 PID_getIntegral(PID *pid);
sbyte2 PID_getDerivative(PID *pid);
sbyte2 PID_getSaturationValue(PID *pid);
bool   PID_getAntiWindupFlag(PID *pid);
#endif //_PID_H