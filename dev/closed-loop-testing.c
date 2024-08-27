#include <stdlib.h>
//#include "pid.c"

int main(){

//PID *plPID = PID_new(1.0,0.0,0.0,0.0);
float powerLimit = 50000.0;
float driverTQ = 231.0;
float RPM = 300.0;
float setpointTQ = 0.0;
float totalError = 0.0;
float previousError = 0.0;
float offsetTQ = 0.0;
float MCMpowerCalced = 0.0;
// 
//
float ratioTQ = 1.0;
float previousTQ = driverTQ;
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.0;
float NewTheoryRPM = 0.0;
float NewMCMpowerCalced = 0.0;
float PowerOffset = 0.0;
for(int i =0; i < 100; ++i){

setpointTQ = powerLimit *9.549/RPM;
//printf("\nsetpointTQ = %f", setpointTQ);

//plPID PID_compute(PID *pid, float sensorValue);
float currentError = (float)(setpointTQ - driverTQ);
float proportional = (float)(Kp * currentError);
//printf("\nproportional = %f", proportional);

float integral = integral + (float)(Ki * (totalError + (currentError * 0.01)));
//printf("\nintegral = %f", integral);

float derivative   = (float)(Kd * (currentError - previousError) / 0.01);
//printf("\nderivative = %f", derivative);

previousError = currentError;

offsetTQ = proportional + integral + derivative;

totalError   += currentError*0.01;

//printf("\nPID output = %f", offsetTQ);
previousTQ = driverTQ;
driverTQ += offsetTQ;
//printf("\nNon adjusted DRIVERSTQ = %f", driverTQ);
if (driverTQ > 231)
    driverTQ = 231;
if (driverTQ <0)
    driverTQ = 0;
//printf("\nAdjusted DRIVERSTQ = %f", driverTQ);

NewMCMpowerCalced = driverTQ * RPM / 9.549;
PowerOffset = MCMpowerCalced - NewMCMpowerCalced;
MCMpowerCalced = NewMCMpowerCalced;
RPM += driverTQ/231*90;
//printf("\nPowerOffset = %f", PowerOffset);
if(MCMpowerCalced > 45000.0 && RPM < 5000)
{
printf("\nIteration:,%d",i);
printf("\t\tMCMpowerCalced = %f",MCMpowerCalced);
printf("\t\tNew RPM = %f", RPM);

}
/**
NewTheoryRPM = RPM + (offsetTQ*20);
if(abs(NewTheoryRPM-RPM)>20) // Based on theory from datasheet & josie.
{
    if (NewTheoryRPM > RPM)
    RPM +=20;
    if(NewTheoryRPM < RPM)
    RPM -=20;
}
else
{
RPM = NewTheoryRPM;
}
//printf("\nNew RPM = %f", RPM);
**/

}
}