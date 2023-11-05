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

extern Button Sensor_DRSButton; 
extern Sensor Sensor_DRSKnob;

DRS *DRS_new() 
{
    DRS *me = (DRS *)malloc(sizeof(struct _DRS));

    //flags
    me->AutoDRSActive = TRUE;
    me->currentDRSMode = MANUAL; 
    me->drsFlap = 0;

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
//      Throttle Position > 80%
//      Wheel Speed > 30 mph
//      Steering Angle < 5% off 0 degrees
//      
//    Disable:
//      Brake Pressure 
//----------------------------------------------------------------------

void DRS_update(DRS *me, MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps, ubyte1 pot_DRS_LC, bool lc_status) {
    if (lc_status == TRUE) {
        me->currentDRSMode = STAY_OPEN;
    } else if(pot_DRS_LC == 1) {
        me->currentDRSMode = MANUAL;
    } else {
        //update_knob(me); Change to when we have a working rotary
        me->currentDRSMode = MANUAL;
    }

    switch(me->currentDRSMode)
        {
            case STAY_CLOSED:
                DRS_close(me);
                break;
            case STAY_OPEN:
                DRS_open(me);           
                break;
            case MANUAL:
                if(Sensor_DRSButton.sensorValue == TRUE) {
                    me->buttonPressed = TRUE;
                    DRS_open(me);
                } else {
                    me->buttonPressed = FALSE;
                    DRS_close(me);
                }
                break;
            case AUTO:
                runAuto(me, mcm, tps, bps);
                break;
            default:
                break;
        }
}

void runAuto(DRS *me, MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps) {
    sbyte2 vehicle_speed_mph = 0.62 * MCM_getGroundSpeedKPH(mcm); // >30mph
    sbyte2 curr_steer_angle = steering_degrees(); // < +-15 deg
    float4 brake_travel = bps->percent; // > 50%
    float4 throttle_travel = tps->travelPercent; // > 90%

    if (vehicle_speed_mph > 5 && throttle_travel > .75 && curr_steer_angle > -15 && curr_steer_angle < 15 && brake_travel < .10) {
        DRS_open(me);
    } else {
        DRS_close(me);
    }
}

void DRS_open(DRS *me) {
    IO_DO_Set(IO_DO_06, TRUE);
    IO_DO_Set(IO_DO_07, FALSE);
    me->drsFlap = 1;
}
void DRS_close(DRS *me) {
    IO_DO_Set(IO_DO_06, FALSE);
    IO_DO_Set(IO_DO_07, TRUE);
    me->drsFlap = 0;
}

//Change to future regarding rotary voltage values
void update_knob(DRS *me) {
        if (Sensor_DRSKnob.sensorValue == 0)
        {    me->currentDRSMode = STAY_CLOSED;}
        else if (Sensor_DRSKnob.sensorValue <= 1.1)
        {    me->currentDRSMode = MANUAL;}
        else if (Sensor_DRSKnob.sensorValue <= 2.2)
        {    me->currentDRSMode = AUTO;}
        else if (Sensor_DRSKnob.sensorValue <= 3.3)
        {    me->currentDRSMode = STAY_OPEN;}
        else if (Sensor_DRSKnob.sensorValue > 3.3)
        {    me->currentDRSMode = STAY_CLOSED;}
}
