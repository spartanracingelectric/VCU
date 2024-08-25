#include <stdlib.h>
//#include "pid.c"

int main(){

//PID *plPID = PID_new(1.0,0.0,0.0,0.0);
float powerLimit = 50000.0;
float driverTQ = 231.0;
float RPM = 3090.0;
float setpointTQ = 0.0;
float totalError = 0.0;
float previousError = 0.0;
float offsetTQ = 0.0;
float MCMpowerCalced = 0.0;
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.0;
float NewMCMpowerCalced = 0.0;
float PowerOffset = 0.0;
for(int i =0; i < 10; ++i){

setpointTQ = powerLimit *9.549/RPM;
//printf("\nsetpointTQ = %f", setpointTQ);

//plPID PID_compute(PID *pid, float sensorValue);
float currentError = (float)(setpointTQ - driverTQ);
totalError   += currentError;
float proportional = (float)(Kp * currentError);
//printf("\nproportional = %f", proportional);

float integral     = (float)(Ki * totalError * 0.01);
//printf("\nintegral = %f", integral);

float derivative   = (float)(Kd * (currentError - previousError) / 0.01);
//printf("\nderivative = %f", derivative);

previousError = currentError;

offsetTQ = proportional + integral + derivative;
//printf("\nPID output = %f", offsetTQ);

driverTQ += offsetTQ;
//printf("\nNon adjusted DRIVERSTQ = %f", driverTQ);
if (driverTQ > 231)
    driverTQ = 231;
//printf("\nAdjusted DRIVERSTQ = %f", driverTQ);

//print

NewMCMpowerCalced = driverTQ * RPM / 9.549;
PowerOffset = MCMpowerCalced - NewMCMpowerCalced;
MCMpowerCalced = NewMCMpowerCalced;
printf("\nPowerOffset = %f", PowerOffset);
//printf("\nMCMpowerCalced = %f",MCMpowerCalced);
RPM = RPM + (offsetTQ*20);
//printf("\nNew RPM = %f", RPM);
}
}