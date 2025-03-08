/*****************************************************************************
 * pid.c - Proportional-Integral-Derivative (PID) controller
 * Initial Author: Harleen Sandhu / Mehul Williams
 ******************************************************************************
 * General purpose PID controller, initially designed for Torque Vectoring.
 * 
 * The PID controller works by using three terms to calculate an output value that is used to control a system. The three terms are:
 * Proportional: This term is proportional to the error between the target and current values. It is multiplied by a constant gain value (kp) that determines how much the controller responds to changes in the error.
 * Integral: This term is proportional to the running sum of errors over time. It is multiplied by a constant gain value (ki) that determines how much the controller responds to steady-state errors.
 * Derivative: This term is proportional to the rate of change of the error. It is multiplied by a constant gain value (kd) that determines how much the controller responds to changes in the rate of change of the error.
 * By adjusting the values of the three gain constants (kp, ki, and kd), the controller can be tuned to respond differently to changes in the error, steady-state errors, and changes in the rate of change of the error.
 * Generally, higher values of kp will lead to faster response to changes in the error, while higher values of ki will lead to faster response to steady-state errors, and higher values of kd will lead to faster response to changes in the rate of change of the error.
 * Conversion between SlipR and Torque -> kp
 * Proportional test first with other output 0, get midway with target and then tune other items. There are many factors of noise.
 * Kp will give you the difference between 0.1 current vs 0.2 target -> if you want to apply 50nm if your error is 0.1 then you need 500 for kp to get target
 ****************************************************************************/

#include <stdlib.h>
#include "pid.h"


// VCU will run this once, outside of the while loop 
PID* PID_new(float kp, float ki, float kd, float setpoint) {
    PID* me = (PID*)malloc(sizeof(PID));
    me->kp = kp;
    me->ki = ki;
    me->kd = kd;
    me->setpoint = setpoint; 
    me->prev_error=0.0;
    me->total_error=0.0;
    me->dt=0.01; // tick rate of vcu

    return me;
}


// Within the while loop in VCU 
void PID_setpointUpdate(PID *pid, float setpoint) {
    pid->setpoint = setpoint; 
}


void PID_dtUpdate(PID *pid, float new_dt) {
    pid->dt = new_dt;
}
void PID_setgain(PID *pid, float kp, float ki, float kd){
    pid-> kp = kp;
    pid-> ki = ki;
    pid-> kd = kd;
}

//sensorVal for yaw PID is from IMU
float PID_compute(PID *pid, float sensorVal) {
   //pid->kp = 1.0; // this is here severely for checking can messages. 
    //pid->ki = 1.0;
    float setpoint = pid->setpoint;
    float error = (float)(setpoint - sensorVal); 
    float proportional = (float)(pid->kp*error); 
    float integral = (float)(pid->ki * (pid->total_error + error)* pid->dt);
    float derivative = (float)(pid->kd * (error - pid->prev_error)/ pid->dt);
    float output = proportional + integral + derivative;
    pid->prev_error = error;
    pid->total_error += error* pid->dt; 
    return (output>0.0) ? 0.0 : output;
}


