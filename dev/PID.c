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
#include <stdlib.h>
#include "PID.h"

PID* PID_new(sbyte1 Kp, sbyte1 Ki, sbyte1 Kd, sbyte2 setpoint) {
    // for some reason the kp ki kd values are not updated correctly so we reinit them 
    // Kp Ki & Kd are in deci units, aka a Kp of 12 is actually 1.2.
    PID* pid = (PID*)malloc(sizeof(PID));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint      = setpoint; 
    pid->previousError = 0;
    pid->totalError    = 0;
    pid->dH            = 100; // 100 Hz aka 10 ms cycle time
    pid->output        = 0;
    pid->currentError = 0;
    pid->proportional = 0;
    pid->integral     = 0;
    pid->derivative   = 0;
    pid->saturationValue = 0;
    pid->antiWindupFlag = FALSE;
    return pid;
}

/** SETTER FUNCTIONS  **/

void PID_updateGainValues(PID* pid, sbyte1 Kp, sbyte1 Ki, sbyte1 Kd){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_setTotalError(PID* pid, sbyte2 error){
    pid->totalError = error;
}

void PID_setSaturationValue(PID *pid, sbyte2 saturationValue){
    pid->saturationValue = saturationValue;
}

void PID_updateSetpoint(PID *pid, sbyte2 setpoint) {
    if(pid->saturationValue > 0){
        if(pid->saturationValue > setpoint)
            pid->setpoint = setpoint;
        else
            pid->setpoint = pid->saturationValue;
    }
    else //this else statement exists for any uncapped pid that has no saturation point.
        pid->setpoint = setpoint;
}

/** COMPUTATIONS **/

sbyte2 PID_computeOutput(PID *pid, sbyte2 sensorValue) {
    pid->currentError = pid->setpoint - sensorValue;
    pid->proportional = (pid->Kp * pid->currentError) / 10; //proportional
    pid->integral   = (pid->Ki * (pid->totalError + pid->currentError) / pid->dH) / 10; //integral
    pid->derivative = (pid->Kd * (pid->currentError - pid->previousError) * pid->dH) / 10; //derivative
    pid->previousError = pid->currentError;
    pid->totalError   += pid->currentError;

    pid->output = pid->proportional;
    //Check to see if motor is saturated at max torque request already
    if(pid->saturationValue <= sensorValue)
    {
        pid->antiWindupFlag = FALSE;
        pid->output += pid->integral;
        pid->output += pid->derivative;
    }
    else
        pid->antiWindupFlag = TRUE;

    return pid->output;
}

/** GETTER FUNCTIONS **/

sbyte1 PID_getKp(PID *pid){
    return pid->Kp;
}

sbyte1 PID_getKi(PID *pid){
    return pid->Ki;
}

sbyte1 PID_getKd(PID *pid){
    return pid->Kd;
}

sbyte2 PID_getSetpoint(PID *pid){
    return pid->setpoint;
}

sbyte2 PID_getTotalError(PID* pid){
    return pid->totalError;
}

sbyte2 PID_getOutput(PID *pid){
    return pid->output;
}

sbyte2 PID_getSaturationValue(PID *pid){
    return pid->saturationValue;
}

bool PID_getAntiWindupFlag(PID *pid){
    return pid->antiWindupFlag;
}