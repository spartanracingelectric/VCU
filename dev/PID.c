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

PID* PID_new(float4 Kp, float4 Ki, float4 Kd, float4 setpoint) {
    // for some reason the kp ki kd values are not updated correctly so we reinit them 
    PID* pid = (PID*)malloc(sizeof(PID));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint      = setpoint; 
    pid->previousError = 0.0;
    pid->totalError    = 0.0;
    pid->dt            = 0.01;
    return pid;

}
void PID_setTotalError(PID* pid, float4 error){
    pid->totalError = error;
}
void PID_updateSetpoint(PID *pid, float4 setpoint) {
    pid->setpoint = setpoint; 
}
void PID_setGain(PID *pid, float4 Kp, float4 Ki, float4 Kd){
    pid-> Kp = Kp;
    pid-> Ki = Ki;
    pid-> Kd = Kd;
}
sbyte2 PID_computeOffset(PID *pid, float4 sensorValue) {
    float4 currentError =  pid->setpoint - sensorValue;
    float4 proportional =  pid->Kp * currentError; //proportional
    float4 integral     =  pid->Ki * (pid->totalError + currentError) * pid->dt; //integral
    float4 derivative   =  pid->Kd * (currentError - pid->previousError) / pid->dt; //derivative
    pid->previousError  = currentError;
    pid->totalError    += currentError;
    return (sbyte2)(proportional + integral + derivative);
}