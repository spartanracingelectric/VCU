#include "PID.h"


PID* PID_new(ubyte2 Kp, ubyte2 Ki, ubyte2 Kd, ubyte2 setpoint){
 PID* pid = (PID*)malloc(sizeof(PID));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint; 
    pid->previousError = 0;
    pid->totalError    = 0;
    pid->dt           = 0.01;
    return pid;
}
void  PID_setpointUpdate(PID *pid, ubyte2 setpoint){
     pid->setpoint = setpoint; 
}
void  PID_dtUpdate(PID *pid, float dt){
    pid->dt  = dt;
}
sbyte2 PID_compute(PID *pid, ubyte2 sensorValue){
    sbyte2 currentError = (sbyte2)(pid->setpoint - sensorValue);
    sbyte2 proportional = (sbyte2)(pid->Kp * currentError);
    sbyte2 integral     = (sbyte2)(pid->Ki * (pid->totalError + currentError) * pid->dt);
    sbyte2 derivative   = (sbyte2)(pid->Kd * (currentError - pid->previousError) / pid->dt);
    pid->previousError = currentError;
    pid->totalError   += currentError;
    return proportional + integral + derivative;
}
void  PID_setGain(PID *pid, ubyte2 Kp, ubyte2 Ki, ubyte2 Kd){
    pid-> Kp = Kp;
    pid-> Ki = Ki;
    pid-> Kd = Kd;
}