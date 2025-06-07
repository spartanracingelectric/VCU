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



void DRS_update(DRS *me, MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps) {
    // get the drs mode from sensor
    me->currentDRSMode = getDRSMode(&Sensor_DRSKnob);
    
    sbyte2 steeringAngle = steering_degrees();
    float4 bpsPercent = bps->percent;
    float4 appsPercent = tps->travelPercent;
    sbyte2 groundspeedMPH = 0.62 * MCM_getGroundSpeedKPH(mcm);
    
    if(Sensor_DRSButton.sensorValue) { 
        me->buttonPressed = TRUE;
    } else {
        me->buttonPressed = FALSE;
    }

    switch(me->currentDRSMode) {
        case DRS_MODE_STAY_CLOSED:
            DRS_close(me);
            break;
            
        case DRS_MODE_STAY_OPEN:
            DRS_open(me);
            break;
            
        case DRS_MODE_MANUAL:
            if(me->buttonPressed) {
                DRS_open(me);
            } else {
                DRS_close(me);
            }
            break;
            
        case DRS_MODE_ASSISTIVE:
            /* 
            1. To open, check if button is pressed & DRS flap closed (should be false) & Have waited at least 5 cycles (50ms)
            2. Restar timer & Open DRS
            3. To close, check if button is pressed & DRS engaged (should be TRUE) & Have waited at least 5 cycles (50ms)
            4. Restart timer & Close DRS
            5. EXIT CONDITONS: (Brake pressure is > 20% or steering angle is +/- 15° ) */

            // conditions are listed in prioritised order, check flap state first (put ourselves in the relevant if statement), then button press, then timer
            if(!me->drsFlapOpen && me->buttonPressed && IO_RTC_GetTimeUS(&me->drsSafetyTimer) >= 45000) {
                IO_RTC_StartTime(&me->drsSafetyTimer); //restart timer
                DRS_open(me);
            }
            // conditions are listed in prioritised order
            else if(me->drsFlapOpen && me->buttonPressed && IO_RTC_GetTimeUS(&me->drsSafetyTimer) >= 10000) {
                IO_RTC_StartTime(&me->drsSafetyTimer); //restart timer
                DRS_close(me);
            }

            if(me->drsFlapOpen && (bpsPercent > .20 || steeringAngle > 15 || steeringAngle < -15)){ //check if [ bps > 20% or steering angle > +/- 15deg ] and drs is open 
                // IO_RTC_StartTime(&me->drsSafteyTimer); //restart timer? or allow immediate correcting in edge cases...
                DRS_close(me);
            }
            break;

        case DRS_MODE_AUTO:

            // Unknown for now if physical components can be damaged when requesting flap open  when already opened, hence the nested if
            if (groundspeedMPH > 5 && appsPercent > .75 && steeringAngle > -15 && steeringAngle < 15 && bpsPercent < .10) {
                me->AutoDRSActive = TRUE;
                if(!me->drsFlapOpen){ DRS_open(me); }
            }
            // Unknown for now if physical components can be damaged when requesting flap close  when already closed, hence the nested if
            else {
                me->AutoDRSActive = FALSE;
                if(me->drsFlapOpen) { DRS_close(me); }
            }
            // Use & uncomment instead if no harm to components
            /*
            else {
                DRS_close(me);
            } */
            break;
        default:
            break;
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

