#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"

#include "motorController.h"
#include "LaunchControl.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"

extern Sensor Sensor_LCButton;

/* Start of Launch Control */

struct _LaunchControl {
    float slipRatio;
    sbyte2 lcTorque;
    bool LCReady;
    bool LCStatus; // Just for CAN to showcase when enabled
};

LaunchControl *LaunchControl_new(void){
    LaunchControl* lc = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    lc->slipRatio = 0;
    lc->lcTorque = -1;
    lc->LCReady = FALSE;
    lc->LCStatus = FALSE; 

    return lc;
}

void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *lc){
    lc->slipRatio = (WheelSpeeds_getSlowestFront(wss) / (WheelSpeeds_getFastestRear(wss))) - 1;
}

void launchControlTorqueCalculation(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm0){
    float tps0percent;
    TorqueEncoder_getIndividualSensorPercent(tps, 0, &tps0percent);

    float bps0percent;
    BrakePressureSensor_getIndividualSensorPercent(bps, 0, &bps0percent);

    sbyte2 groundSpeed = MCM_getGroundSpeedKPH(mcm0);
    sbyte2 steeringAngle = steering_degrees();
    
     if(Sensor_LCButton.sensorValue == TRUE && groundSpeed < 5 && tps0percent > 0.95 && bps0percent < 5 && steeringAngle > -5 && steeringAngle < 5) {
        lc->LCReady = TRUE;
     }
     
     if(lc->LCReady == TRUE && Sensor_LCButton.sensorValue == TRUE){
        lc->lcTorque = 0; // On the motorcontroller side, this torque should stay this way regardless of the values by the pedals while LC is ready
     }

     if(lc->LCReady == TRUE && Sensor_LCButton.sensorValue == FALSE){
        lc->LCStatus = TRUE;
        lc->lcTorque = 180; // Example Value Set : Configure for later : Integrate PID
     }

     if(bps0percent > 5 || tps0percent < 0.80 || steeringAngle > 35 || steeringAngle < -35){
        lc->LCStatus = FALSE;
        lc->LCReady = FALSE;
     }
}

bool getLaunchControlStatus(LaunchControl *lc){
    return lc->LCStatus;
}


