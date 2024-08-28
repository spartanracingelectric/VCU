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
float previousTQ = driverTQ;
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.0;
for(int i =0; i < 100; ++i){
setpointTQ = powerLimit *9.549/RPM;
float currentError = (float)(setpointTQ - driverTQ);
float proportional = (float)(Kp * currentError);
float integral     = (float)(Ki * (totalError + (currentError * 0.01)));
float derivative   = (float)(Kd * (currentError - previousError) / 0.01);

previousError = currentError;
totalError   += currentError*0.01;
offsetTQ = proportional + integral + derivative;
previousTQ = driverTQ;
driverTQ += offsetTQ;
if (driverTQ > 231)
    driverTQ = 231;
if (driverTQ <0)
    driverTQ = 0;

MCMpowerCalced = driverTQ * RPM / 9.549;
//unsustantiated guesswork for max RPM delta as it relates to current RPM, as the ability to change RPM decreases as RPM increases;
float deltaRPM = 90 - (RPM-3000)/6000*90*0.6 - (RPM-4000)/6000*90*0.3 -(RPM-5000)/6000*90*0.1;
if (deltaRPM > 90.0)
    deltaRPM = 90.0;
if (deltaRPM < 0.0)
    deltaRPM = 0.0;
RPM += driverTQ/231*deltaRPM;
if (RPM > 6000.0)
    RPM = 6000.0;
//if(MCMpowerCalced > 45000.0 && RPM < 5000)
//{
    printf("\nIteration:%d",i);
    printf("\t\tMCMpowerCalced = %f",MCMpowerCalced);
    printf("\t\tNew driverTQ = %f", driverTQ);
    printf("\t\tNew RPM = %f", RPM);
//}
}
}