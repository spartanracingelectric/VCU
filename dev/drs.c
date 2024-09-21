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


void DRS_update(DRS *me, MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps /*add watch dog parameter & steering angle
*/) {

    // permanantly in pot_DRS_LC == 0 (! retired functionality of pot_DRS_LC)
    // if(pot_DRS_LC == 1) {
    //     me->currentDRSMode = AUTO;
    // } else {
    //     //update_knob(me); Change to when we have a working rotary
    //     me->currentDRSMode = MANUAL;
    // }
    sbyte2 curr_steer_angle = steering_degrees(); // < +-15 deg
    float4 brake_travel = bps->percent; // > 20%
    me->currentDRSMode = MANUAL; 

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
                } 
                else {
                    me->buttonPressed = FALSE;
                    DRS_close(me);
                }
                break;
            case ASSISTIVE:
            /* 
            1. Check if button is pressed & DRS engaged (should be false)
            2. Open DRS & Log time the button is pressed
            3. Wait at least 5 cycle (50ms) to check if button pressed again

            4. To close check if button is pressed & DRS engaged (should be true)
            5. Close DRS & Log time the button is pressed
            6. Wait at least 5 cycle (50ms) to check if button pressed again

            7. ALWAYS BEING CHECKED: Exit conditions (Brake pressure is ??% or streering angle is 15% to right or left) then close DRS
            */
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

void DRS_Assistive(DRS *me){
    ubyte4 timestamp_startTime = 0;
    ubyte4 timestamp_EcoButton = 0;

    SerialManager *serialMan = SerialManager_new();
    IO_RTC_StartTime(&timestamp_startTime);

    // while(1) //looped?
    // {
    if(Sensor_DRSButton.sensorValue == TRUE)
    {
        if (timestamp_EcoButton == 0)
        {
            SerialManager_send(serialMan, "Eco button detected\n");
            IO_RTC_StartTime(&timestamp_EcoButton);
<<<<<<< HEAD

            //if drsSafety == 1 & 5 cycles has passed from log time
           //set drsSafety == 0

            if(Sensor_DRSButton.sensorValue == true &&  me->drsFlap == 0){ //check if button is pressed && drs is inactive && if drsSafety == 0
                DRS_open(me); //open drs
                //log time, set boolean value drsSafety to 1       
            }

            if(Sensor_DRSButton.sensorValue == true && me->drsFlap == 1){ //check if button is pressed %% drs is active && if drsSafety == 0
                DRS_close(me); ///close drs
                //log time, set boolean value drsSafety to 1
            }


            if(brake_travel < .20 || curr_steer_angle > -15 || curr_steer_angle < 15 && me->drsFlap == 1){ //check if bps < 20% or steering angle +/- 15deg and drs is open 
                drs_close(me);
            } 
=======
        }
        else if (IO_RTC_GetTimeUS(timestamp_EcoButton) >= 100000) // pressed longer than 0.1 sec
        {
            // SerialManager_send(serialMan, "Eco button held 3s - starting calibrations\n"); // i dont think we need this
            // code here
>>>>>>> parent of 307c4f1 (carlie chris code combined)
            me->drsFlap = 0; 
            timestamp_EcoButton = 0; //timer rest
        }
        else if (IO_RTC_GetTimeUS(timestamp_EcoButton) >= 500000) //  wait 0.5 sec to check again
        {
            // SerialManager_send(serialMan, "Eco button held 3s - starting calibrations\n"); // i dont think we need this
            timestamp_EcoButton = 0;
        

        }
    }
    // }
}

//Change to future regarding rotary voltage values
void update_knob(DRS *me) {
        if (Sensor_DRSKnob.sensorValue == 0)
        {    me->currentDRSMode = STAY_CLOSED;}
        else if (Sensor_DRSKnob.sensorValue <= 1.1)
        {    me->currentDRSMode = MANUAL;}
        else if (Sensor_DRSKnob.sensorValue <= 2.2)
        {    me->currentDRSMode = ASSISTIVE;}
        else if (Sensor_DRSKnob.sensorValue <= 3.3)
        {    me->currentDRSMode = STAY_OPEN;}
        else if (Sensor_DRSKnob.sensorValue > 3.3)
        {    me->currentDRSMode = STAY_CLOSED;}
}
