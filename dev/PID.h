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
    sbyte1 Kp;               // Proportional gain
    sbyte1 Ki;               // Integral     gain
    sbyte1 Kd;               // Derivative   gain
    sbyte2 setpoint;         // Target       value
    sbyte2 previousError;
    sbyte4 totalError;
    sbyte2 dH;               // Time interval between PID updates in seconds (VCU tick speed)
    sbyte2 output;
    sbyte2 proportional;
    sbyte2 integral;
    sbyte2 derivative;
    sbyte2 saturationValue;
    bool antiWindupFlag;
}PID;

/**
 * Kp, Ki, & Kd are in deci- units, meaning PID_new(10,0,0,231) gives a Kp of 1.0 and a saturation value of 231. 
 * Setting the saturation value to 0 uncaps the PID, and it will never exhibit clamping behavior.
 * If using the PID with deci-newton meters, the maximum safe Kp value is 141 aka 14.1, in the event of a 
 * maximized currenterror (pid->setpoint - sensorValue = 2310)
 * */
PID* PID_new(sbyte1 Kp, sbyte1 Ki, sbyte1 Kd, sbyte2 saturationValue);

/** SETTER FUNCTIONS  **/

void PID_setTotalError(PID* pid, sbyte2 totalError);
void PID_setSaturationPoint(PID *pid, sbyte2 saturationValue);
void PID_updateSetpoint(PID *pid, sbyte2 setpoint);
void PID_updateGainValues(PID* pid, sbyte1 Kp, sbyte1 Ki, sbyte1 Kd);

/** COMPUTATIONS **/

void PID_computeOutput(PID *pid, sbyte2 sensorValue, sbyte2 scaleFactor);

/** GETTER FUNCTIONS **/

sbyte1 PID_getKp(PID *pid);
sbyte1 PID_getKi(PID *pid);
sbyte1 PID_getKd(PID *pid);
sbyte2 PID_getSetpoint(PID *pid);
sbyte2 PID_getPreviousError(PID *pid);
sbyte4 PID_getTotalError(PID* pid);
sbyte2 PID_getOutput(PID *pid);
sbyte2 PID_getProportional(PID *pid);
sbyte2 PID_getIntegral(PID *pid);
sbyte2 PID_getDerivative(PID *pid);
sbyte2 PID_getSaturationValue(PID *pid);
bool   PID_getAntiWindupFlag(PID *pid);
#endif //_PID_H