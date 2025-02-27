#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "launchControl.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"
#include "sensorCalculations.h"
#include "drs.h"
#include "PID.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

extern Sensor Sensor_LCButton;
extern Sensor Sensor_DRSKnob;

LaunchControl *LaunchControl_new(){
    LaunchControl* me = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    // malloc returns NULL if it fails to allocate memory
    if (me == NULL)
        return NULL;
    me->pidTorque = PID_new(200, 0, 0, 0); //No saturation point to see what the behavior of the PID is, will need a saturation value somewhere to prevent wind-up of the pid in the future
    me->pidSpeed = PID_new(200, 0, 0, 0); //No saturation point to see what the behavior of the PID is, will need a saturation value somewhere to prevent wind-up of the pid in the future
    PID_updateSetpoint(me->pidTorque, 20); // Having a statically coded slip ratio may not be the best. this requires knowing that this is both a) the best slip ratio for the track, and b) that our fronts are not in any way slipping / entirely truthful regarding the groundspeed of the car. Using accel as a target is perhaps better, but needs to be better understood.
    me->slipRatio = 0;
    me->lcTorqueCommand = NULL;
    me->lcSpeedCommand = NULL;
    me->lcReady = FALSE;
    me->lcActive = FALSE;
    me->buttonDebug = 0;

    /** Variables for constantSpeedTestOverride Function. 
     * Enabling this mode disabled Launch Control & 
     * changes button function to act as a cruise control targeting a specified speed */
    me->constantSpeedTestOverride = FALSE;
    me->overrideTestSpeedCommand = 3000; // CONSTANT SPEED TARGET
    return me;
}


void LaunchControl_calculateSlipRatio(LaunchControl *me, WheelSpeeds *wss){
    me->slipRatio = ( WheelSpeeds_getSlowestFront(wss) / WheelSpeeds_getFastestRear(wss) ) - 1;
    // me->slipRatio = ( WheelSpeeds_getSlowestFrontRPM(wss) / MCM_getMotorRPM(mcm) ) - 1;
    if (me->slipRatio >= 1.0) { // the >= is preferred over the > symbol because floats are checked left-to-right instead of right to left, and therefore this should hopefully speed up this check.
        me->slipRatio = 1.0;
    }
    if (me->slipRatio <= -1.0) {
        me->slipRatio = -1.0;
    }
}

void LaunchControl_calculateTorqueCommand(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    if(me->lcActive){
        me->slipRatioThreeDigits = (sbyte2) (me->slipRatio * 100);
        PID_computeOutput(me->pidTorque, me->slipRatioThreeDigits);
        me->lcTorqueCommand = MCM_getCommandedTorque(mcm) + PID_getOutput(me->pidTorque); // adds the ajusted value from the pid to the torqueval}

        if(MCM_getGroundSpeedKPH(mcm) < 3){
            me->lcTorqueCommand = 20;
        }
        // Tune
        if(MCM_getGroundSpeedKPH(mcm) > 30){
            DRS_open(drs);
        }
        // Update launch control torque command in mcm struct
        MCM_update_LC_torqueCommand(mcm, me->lcTorqueCommand * 10); // Move the mul by 10 to within MCM struct at some point
    }
}

void LaunchControl_calculateSpeedCommand(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    if(me->lcActive && !me->constantSpeedTestOverride){
        me->slipRatioThreeDigits = (sbyte2) (me->slipRatio * 100);
        PID_computeOutput(me->pidSpeed, me->slipRatioThreeDigits);
        me->lcSpeedCommand = PID_getOutput(me->pidSpeed); // adds the ajusted value from the pid to the torqueval}

        if(MCM_getGroundSpeedKPH(mcm) < 3){
            me->lcSpeedCommand = 20; //Timer-based function insert here
        }
        // Tune
        if(MCM_getGroundSpeedKPH(mcm) > 30){
            DRS_open(drs);
        }
        // Update launch control torque command in mcm struct
        MCM_update_LC_speedCommand(mcm, me->lcSpeedCommand);
    }
    // constantSpeedTestOverride
    else if(me->constantSpeedTestOverride && Sensor_LCButton.sensorValue == FALSE){
        MCM_update_LC_speedCommand(mcm, me->overrideTestSpeedCommand);
        MCM_update_LC_activeStatus(mcm, TRUE); //We fake the "active" status to enable speed mode in the mcm
    }
}

void LaunchControl_checkState(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    sbyte2 speedKph         = MCM_getGroundSpeedKPH(mcm);
    sbyte2 steeringAngle    = steering_degrees();

    /* LC STATUS CONDITIONS *//*
     * lcReady = FALSE && lcActive = FALSE -> NOTHING HAPPENS
     * lcReady = TRUE  && lcActive = FALSE -> We are in the prep stage for lc, and all entry conditions for being in prep stage have and continue to be monitored
     * lcReady = FALSE && lcActive = TRUE  -> We have left the prep stage by pressing the lc button on the steering wheel, stay in until exit conditions are met
     * AT ALL TIMES, EXIT CONDITIONS ARE CHECKED FOR BOTH STATES
    */

    // SENSOR_LCBUTTON values are reversed: FALSE = TRUE and TRUE = FALSE, due to the VCU internal Pull-Up for the button and the button's Pull-Down on Vehicle
    if(Sensor_LCButton.sensorValue == TRUE && speedKph < 5) {
        me->lcReady = TRUE;
    }

    else if(me->lcReady == TRUE && Sensor_LCButton.sensorValue == FALSE){
        PID_setTotalError(me->pidTorque, 170); // Error should be set here, so for every launch we reset our error to this value (check if this is the best value)
        me->lcTorqueCommand = 0; // On the motorcontroller side, this torque should stay this way regardless of the values by the pedals while LC is ready
        me->lcActive = TRUE;
        me->lcReady = FALSE;
        DRS_close(drs);
    }

    else if(bps->percent > .35 || steeringAngle > 35 || steeringAngle < -35){
        me->lcReady = FALSE;
    }

    if(tps->travelPercent < 0.90 || bps->percent > 0.05){
        me->lcActive = FALSE;
        me->lcTorqueCommand = NULL;
    }
    
    //MCM struct only cares about lcActive, so we inform it here
    MCM_update_LC_activeStatus(mcm, me->lcActive);
}

bool LaunchControl_getStatus(LaunchControl *me){ return me->lcActive; }

sbyte2 LaunchControl_getTorqueCommand(LaunchControl *me){ return me->lcTorqueCommand; }

float LaunchControl_getSlipRatio(LaunchControl *me){ return me->slipRatio; }

sbyte2 LaunchControl_getSlipRatioThreeDigits(LaunchControl *me){ return me->slipRatioThreeDigits; }

ubyte1 LaunchControl_getButtonDebug(LaunchControl *me) { return me->buttonDebug; }