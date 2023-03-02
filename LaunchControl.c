#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"

#include "LaunchControl.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"

extern Sensor Sensor_LCButton;

/* Start of Launch Control */

LaunchControl *LaunchControl_new(){
    LaunchControl* me = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    me->slipRatio = 0;
    me->lcTorque = 0;
    me->LCReady = FALSE;
    me->LCStatus = FALSE; 

    return me;
}

void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *me){
    me->slipRatio = (WheelSpeeds_getSlowestFront(wss) / (WheelSpeeds_getFastestRear(wss))) - 1;
}

void launchControlTorqueCalculation(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm){

    float bps0percent;
    BrakePressureSensor_getIndividualSensorPercent(bps, 0, &bps0percent);

    float tpsPercentage;
    TorqueEncoder_getOutputPercent(tps, &tpsPercentage);

    sbyte2 mcm_Torque_max = (MCM_commands_getTorqueLimit(mcm) / 10.0);

    sbyte2 groundSpeed = MCM_getGroundSpeedKPH(mcm);
    sbyte2 steeringAngle = steering_degrees();
    
     if(Sensor_LCButton.sensorValue == TRUE && groundSpeed < 5 && tpsPercentage > 0.95 && bps0percent < 0.05 && steeringAngle > -5 && steeringAngle < 5) {
        me->LCReady = TRUE;
     }
     
     if(me->LCReady == TRUE && Sensor_LCButton.sensorValue == TRUE){
        me->lcTorque = 0; // On the motorcontroller side, this torque should stay this way regardless of the values by the pedals while LC is ready
     }

     if(me->LCReady == TRUE && Sensor_LCButton.sensorValue == FALSE){
        me->LCStatus = TRUE;
        me->lcTorque = mcm_Torque_max - 150; // Example Value Set : Configure for later : Integrate PID
     }

     if(bps0percent > 0.05 || tpsPercentage < 0.80 || steeringAngle > 35 || steeringAngle < -35){
        me->LCStatus = FALSE;
        me->LCReady = FALSE;
     }

    MCM_update_LaunchControl_State(mcm, me->LCStatus);
    MCM_update_LaunchControl_TorqueLimit(mcm, me->lcTorque);
}

bool getLaunchControlStatus(LaunchControl *me){
    return me->LCStatus;
}


