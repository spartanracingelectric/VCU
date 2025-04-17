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
    me->pidTorque = PID_new(20, 0, 0, 0); //No saturation point to see what the behavior of the PID is, will need a saturation value somewhere to prevent wind-up of the pid in the future
    me->pidSpeed = PID_new(200, 0, 0, 0); //No saturation point to see what the behavior of the PID is, will need a saturation value somewhere to prevent wind-up of the pid in the future
    PID_updateSetpoint(me->pidTorque, 200); // Having a statically coded slip ratio may not be the best. this requires knowing that this is both a) the best slip ratio for the track, and b) that our fronts are not in any way slipping / entirely truthful regarding the groundspeed of the car. Using accel as a target is perhaps better, but needs to be better understood.
    me->slipRatio = 0;
    me->lcTorqueCommand = NULL;
    me->lcSpeedCommand = NULL;
    me->lcReady = FALSE;
    me->lcActive = FALSE;
    me->buttonDebug = 0; // This exists as a holdover piece of code to what I presume is debugging which button was which on the steering wheel. should remove / place elsewhere
    me->safteyTimer = 0;

    me->initialTorque = 100;

    /** Variables for constantSpeedTestOverride Function. 
     * Enabling this mode disabled Launch Control & 
     * changes button function to act as a cruise control targeting a specified speed */
    me->constantSpeedTestOverride = FALSE;
    me->overrideTestSpeedCommand = 3000; // CONSTANT SPEED TARGET
    return me;
}

void LaunchControl_calculateSlipRatio(LaunchControl *me, MotorController *mcm, WheelSpeeds *wss){
    if (WheelSpeeds_getWheelSpeed(me,FL) == 0){
        me->slipRatio = ( WheelSpeeds_getWheelSpeed(me,FR) / WheelSpeeds_getFastestRear(wss) ) - 1;
    }
    else if (WheelSpeeds_getWheelSpeed(me,FR) == 0){
        me->slipRatio = ( WheelSpeeds_getWheelSpeed(me,FL) / WheelSpeeds_getFastestRear(wss) ) - 1;
    }
    else{
        me->slipRatio = ( WheelSpeeds_getSlowestFront(wss) / WheelSpeeds_getFastestRear(wss) ) - 1;
        // me->slipRatio = ( WheelSpeeds_getSlowestFrontRPM(wss) / MCM_getMotorRPM(mcm) ) - 1;
    }
    
    if (me->slipRatio >= 1.0) { me->slipRatio = 1.0; }

    else if (me->slipRatio <= -1.0) { me->slipRatio = -1.0; }
}

void LaunchControl_calculateTorqueCommand(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    if(me->lcActive){
        if( MCM_getGroundSpeedKPH(mcm) < 3 ){ LaunchControl_initialTorqueCurve(me, mcm); }

        else{
            me->slipRatioThreeDigits = (sbyte2) (me->slipRatio * 100);
            PID_computeOutput(me->pidTorque, me->slipRatioThreeDigits);
            me->lcTorqueCommand = (sbyte2)MCM_getCommandedTorque(mcm) + PID_getOutput(me->pidTorque); // adds the adjusted value from the pid to the torqueval
        }
        // Tune 
        if( MCM_getGroundSpeedKPH(mcm) == 30 ){ DRS_open(drs); }
        // Update launch control torque command in mcm struct
        MCM_update_LC_torqueCommand(mcm, me->lcTorqueCommand * 10); // Move the mul by 10 to within MCM struct at some point
    }
}

void LaunchControl_calculateSpeedCommand(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    if(me->lcActive && !me->constantSpeedTestOverride){
        if(MCM_getGroundSpeedKPH(mcm) < 3){
            LaunchControl_initialRPMCurve(me,mcm);
        }
        else{
            me->slipRatioThreeDigits = (sbyte2) (me->slipRatio * 100);
            PID_computeOutput(me->pidSpeed, me->slipRatioThreeDigits);
            me->lcSpeedCommand = MCM_getMotorRPM(mcm)+ PID_getOutput(me->pidSpeed);
        }
        // Tune
        if(MCM_getGroundSpeedKPH(mcm) == 30 ){ DRS_open(drs); }
        // Update launch control speed command in mcm struct
        MCM_update_LC_speedCommand(mcm, me->lcSpeedCommand);
    }
    // constantSpeedTestOverride
    else if(me->constantSpeedTestOverride && Sensor_LCButton.sensorValue == TRUE){
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
     * lcReady = FALSE && lcActive = TRUE  -> We have left the prep stage by releasing the lc button on the steering wheel, stay in Launch until exit conditions are met
     * AT ALL TIMES, EXIT CONDITIONS ARE CHECKED FOR BOTH STATES
    */

    /**
     * Launch Control Pre-Staging Operations:
     * If the car is near 0 kph (in case of wss float issues) & the Launch Button is pressed,
     * we initialise a 3 second timer to confirm a valid Launch attempt
     * Once this timer reaches maturity, we are now in "ready" state
     * The driver can now fully press TPS/APPS without moving car
     * 
     * Upon button release, we are now in "active" state and will proceed with our launch as intended -> car go eeeeeeeee (e-motor sounds)
     * 
     * At any time, an exit condition can be triggered to reset this staging operation and cancel our launch attempt
     */
    if(Sensor_LCButton.sensorValue == TRUE && speedKph < 1) {
        if (me->safteyTimer == 0){
            IO_RTC_StartTime(&me->safteyTimer);
            DRS_open(drs);
        }
        else if (IO_RTC_GetTimeUS(me->safteyTimer) >= 3000000) {
            me->lcReady = TRUE;
            DRS_close(drs);
            me->safteyTimer = 0; // We don't need to track the timer anymore
        }
    }

    else if(me->lcReady == TRUE && Sensor_LCButton.sensorValue == FALSE){
        PID_setTotalError(me->pidTorque, 170); // Error should be set here, so for every launch we reset our error to this value (check if this is the best value)
        me->lcActive = TRUE;
        me->lcReady = FALSE;
    }

    if(tps->travelPercent < 0.90 || bps->percent > 0.05 || steeringAngle > 35 || steeringAngle < -35){
        me->lcActive = FALSE;
        me->lcTorqueCommand = NULL;
        me->safteyTimer = 0;
    }
    
    MCM_update_LC_activeStatus(mcm, me->lcActive);
    MCM_update_LC_readyStatus(mcm, me->lcReady);
}

void LaunchControl_initialTorqueCurve(LaunchControl* me, MotorController* mcm){
    me->lcTorqueCommand = me->initialTorque + ( MCM_getMotorRPM(mcm) / 2 ); // Tunable Values will be the inital Torque Request @ 0 and the scalar factor
}

void LaunchControl_initialRPMCurve(LaunchControl* me, MotorController* mcm){
    me->lcSpeedCommand = 100 + ( MCM_getMotorRPM(mcm) * 30 ); // Tunable Values will be the inital Speed Request @ 0 and the scalar factor
}

bool LaunchControl_getStatus(LaunchControl *me){ return (me->lcReady << 1 || me->lcActive); }

sbyte2 LaunchControl_getTorqueCommand(LaunchControl *me){ return me->lcTorqueCommand; }

float LaunchControl_getSlipRatio(LaunchControl *me){ return me->slipRatio; }

sbyte2 LaunchControl_getSlipRatioThreeDigits(LaunchControl *me){ return me->slipRatioThreeDigits; }

ubyte1 LaunchControl_getButtonDebug(LaunchControl *me) { return me->buttonDebug; }