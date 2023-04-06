#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_DIO.h"

#include "drs.h"
#include "sensors.h"
#include "wheelSpeeds.h"
#include "brakePressureSensor.h"
#include "torqueEncoder.h"
#include "motorController.h"
#include "sensorCalculations.h"

extern Sensor Sensor_DRSButton; 
extern Sensor Sensor_DRSKnob;

DRS *DRS_new() 
{
    DRS *me = (DRS *)malloc(sizeof(struct _DRS));

    //flags
    me->AutoDRSActive = FALSE;
    me->currentDRSMode = MANUAL; 
    me->openSolenoidState = FALSE;
    me->closeSolenoidState = FALSE;

    //physical inputs
    me->driverDRSButton = FALSE;
    me->drsRotarySwitch = 2.5; //using an arbitrary, single value for development

    return me;
}

//----------------------------------------------------------------------
// Drag reduction system calculations.
// First, check the mode indicated by the driver rotary switch
//      Mode 0 - Always off
//      Mode 1 - Always on
//      Mode 2 - Driver controlled
//      Mode 3 - Auto (sensor controlled)
// Second, set AutoDRSActive flag
// **DOES NOT** physically activate the wing
//
// Auto Params:
//    Enable: 
//      Throttle Position > 90%
//      Wheel Speed > 35 mph
//      Steering Angle < 5% off 0 degrees
//      
//    Disable:
//      Brake Pressure 
//----------------------------------------------------------------------


void update_DRS_mode(DRS *me, MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps) {
    //me->opening -= 1;

    // .sensorvalue true/false are switched to account for Pull Up
    if(Sensor_DRSButton.sensorValue) {
        me->driverDRSButton = FALSE;
    } else {
        me->driverDRSButton = TRUE;
    }

    // Still need to implement rotary switch input
    //me->currentDRSMode = AUTO;
    update_knob(me); // "knob"
    switch(me->currentDRSMode)
    {
        // STAY_CLOSED and STAY_OPEN two needs to be changed to only
        // update every 2 seconds, or maybe add another condition?
        case STAY_CLOSED:
            DRS_close(me);
            break;
        case STAY_OPEN:
            DRS_open(me);           
            break;
        case MANUAL:
            if(me->driverDRSButton) {
                DRS_open(me);
            } else {
                DRS_close(me);
            }
            break;
        case AUTO:
            updateAuto(me, mcm, tps, bps);
            toggleDRS(me);
            break;
        default:
            // ;)
            break;
    }
    
}

void updateAuto(DRS *me, MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps) {
    sbyte2 vehicle_speed_mph = 0.62 * MCM_getGroundSpeedKPH(mcm); // >30mph
    sbyte2 curr_steer_angle = steering_degrees(); // < +-15 deg
    float4 brake_travel = bps->percent; // > 50%
    float4 throttle_travel = tps->travelPercent; // > 90%
    
    if (vehicle_speed_mph > 30 && throttle_travel > .9 && curr_steer_angle > -20 && curr_steer_angle < 20 && brake_travel < .15) {
            me->AutoDRSActive = TRUE;
        }
    else {
        me->AutoDRSActive = FALSE;
    }
}

void toggleDRS(DRS *me) {
    if(!me->AutoDRSActive) {
        DRS_open(me);
    }
    else { 
        DRS_close(me);
    }
}

//----------------------------------------------------------------------
// Enable the DRS system by actuating the wing
//----------------------------------------------------------------------
void DRS_open(DRS *me) {
    if (! me->openSolenoidState) {
        IO_DO_Set(IO_DO_06, TRUE);
        me->openSolenoidState = TRUE;
        IO_DO_Set(IO_DO_07, FALSE);
        me->closeSolenoidState = FALSE;
    }
}

//----------------------------------------------------------------------
// Disable the DRS system by retracting the wing
//----------------------------------------------------------------------
void DRS_close(DRS *me) {
    if (me->openSolenoidState) {
        IO_DO_Set(IO_DO_06, FALSE);
        me->openSolenoidState = FALSE;
        IO_DO_Set(IO_DO_07, TRUE);
        me->closeSolenoidState = TRUE;
    }
}

//based on guessed values about rotary input value range 
void update_knob(DRS *me) {
    int val = DRS_knob_value();
        if (val < 1000)
        {    me->currentDRSMode = STAY_CLOSED;}
        else if (val < 6000)
        {    me->currentDRSMode = MANUAL;}
        else if (val < 11000)
        {    me->currentDRSMode = AUTO;}
        else if (val < 16000)
        {    me->currentDRSMode = STAY_OPEN;}
        else if (val > 21000)
        {    me->currentDRSMode = STAY_CLOSED;}
}



