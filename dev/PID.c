/*****************************************************************************
 * pid.c - Proportional-Integral-Derivative (PID) controller
 * Initial Author: Harleen Sandhu / Mehul Williams
 * Additional Author: Shaun Gilmore
 ******************************************************************************
 * General purpose PID controller, initially designed for Torque Vectoring.
 * 
 * The PID controller works by using three terms to calculate an output value that is used to control a system. The three terms are:
 * Proportional: This term is proportional to the error between the target and current values. It is multiplied by a constant gain value (Kp) that determines how much the controller responds to changes in the error.
 * Integral: This term is proportional to the running sum of errors over time. It is multiplied by a constant gain value (Ki) that determines how much the controller responds to steady-state errors.
 * Derivative: This term is proportional to the rate of change of the error. It is multiplied by a constant gain value (Kd) that determines how much the controller responds to changes in the rate of change of the error.
 * By adjusting the values of the three gain constants (Kp, Ki, and Kd), the controller can be tuned to respond differently to changes in the error, steady-state errors, and changes in the rate of change of the error.
 * Generally, higher values of Kp will lead to faster response to changes in the error, while higher values of Ki will lead to faster response to steady-state errors, and higher values of Kd will lead to faster response to changes in the rate of change of the error.
 * Conversion between SlipR and Torque -> Kp
 * Proportional test first with other output 0, get midway with target and then tune other items. There are many factors of noise.
 * Kp will give you the difference between 0.1 current vs 0.2 target -> if you want to apply 50nm if your error is 0.1 then you need 500 for Kp to get target
 ****************************************************************************/
#include "PID.h"

PID* PID_new(ubyte2 Kp, ubyte2 Ki, ubyte2 Kd, ubyte2 saturationValue) {
    PID* pid = (PID*)malloc(sizeof(PID));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint      = 0; 
    pid->previousError = 0;
    pid->totalError    = 0;
    pid->dH            = 100; // 100 Hz aka 10 ms cycle time. view as inverese of 0.01 seconds, being done to avoid fpu usage
    pid->output        = 0;
    pid->proportional  = 0;
    pid->integral      = 0;
    pid->derivative    = 0;
    pid->saturationValue = saturationValue;
    pid->antiWindupFlag = FALSE;
    return pid;
}
/** SETTER FUNCTIONS  **/

void PID_updateGainValues(PID* pid, ubyte2 Kp, ubyte2 Ki, ubyte2 Kd){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_setTotalError(PID* pid, sbyte2 error){
    pid->totalError = error;
}

void PID_setSaturationPoint(PID *pid, ubyte2 saturationValue){
    pid->saturationValue = saturationValue;
}

void PID_updateSetpoint(PID *pid, ubyte2 setpoint) {
    if(pid->saturationValue > setpoint)
        pid->setpoint = setpoint;
    else
        pid->setpoint = pid->saturationValue;
        
    //this if statement exists for any uncapped pid that has no saturation point.
    if(pid->saturationValue == 0)
        pid->setpoint = setpoint;
}

/** COMPUTATIONS **/

void PID_computeOutput(PID *pid, sbyte2 sensorValue) {
    
    sbyte2 currentError = pid->setpoint - sensorValue;
    sbyte2 proportional = pid->Kp * currentError;
    sbyte2 integral     = pid->Ki * (pid->totalError + currentError) * pid->dH;
    sbyte2 derivative   = pid->Kd * (currentError - pid->previousError) / pid->dH;
    sbyte2 output       = proportional + integral + derivative;
    pid->previousError  = currentError;
    pid->totalError    += currentError * pid->dH;
    pid->output         = output;
}

/** GETTER FUNCTIONS **/

ubyte2 PID_getKp(PID *pid){
    return pid->Kp;
}

ubyte2 PID_getKi(PID *pid){
    return pid->Ki;
}

ubyte2 PID_getKd(PID *pid){
    return pid->Kd;
}

ubyte2 PID_getSetpoint(PID *pid){
    return pid->setpoint;
}

sbyte2 PID_getPreviousError(PID *pid){
    return pid->previousError;
}

sbyte2 PID_getTotalError(PID* pid){
    return pid->totalError;
}

sbyte2 PID_getOutput(PID *pid){
    return pid->output;
}

sbyte2 PID_getProportional(PID *pid){
    return pid->proportional;
}

sbyte2 PID_getIntegral(PID *pid){
    return pid->integral;
}

sbyte2 PID_getDerivative(PID *pid){
    return pid->derivative;
}

sbyte2 PID_getSaturationValue(PID *pid){
    return pid->saturationValue;
}

bool PID_getAntiWindupFlag(PID *pid){
    return pid->antiWindupFlag;
}
