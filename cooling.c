#include <stdlib.h>
#include "IO_Driver.h"
//#include "IO_DIO.h"
//#include "IO_PWM.h"

#include "serial.h"
#include "sensors.h"
#include "cooling.h"
#include "motorController.h"
#include "mathFunctions.h"
#include "bms.h"

extern Sensor Sensor_HVILTerminationSense;

//All temperatures in C
CoolingSystem *CoolingSystem_new(SerialManager *serialMan)
{
    CoolingSystem *me = (CoolingSystem *)malloc(sizeof(struct _CoolingSystem));
    SerialManager *sm = serialMan;

    //-------------------------------------------------------------------
    // Cooling System Configuration
    //-------------------------------------------------------------------

    // Water pump PWM control (for motor and controller)
    // Note: the water pump needs to receive a PWM singal within n seconds of being turned on
    me->waterPumpMinPercent = 0.2;
    me->waterPumpLow = 25; //Start ramping beyond min at this temp
    me->waterPumpHigh = 40;
    me->waterPumpPercent = 0.2;

    // Power pack fan relay  (for 2021 3ator)
    me->motorFanLow = 38;     //Turn off BELOW this point
    me->motorFanHigh = 43;    //Turn on at this temperature
    me->motorFanState = TRUE; //float4 motorFanPercent;

    me->radFanMinPercent = 0;
    me->radFanLow = 25;
    me->radFanHigh = 40;
    me->radFanPercent = 0;

    // Battery fans (Unused in 2021)
    me->batteryFanLow = 38;     //Turn off BELOW this point
    me->batteryFanHigh = 43;    //Turn on at this temperature
    me->batteryFanState = TRUE; //float4 batteryFanPercent;

    return me;
}

//-------------------------------------------------------------------
// Cooling system calculations - turns fans PWM, sends water pump DO control signal
//Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_calculations(CoolingSystem *me, sbyte2 motorControllerTemp, sbyte2 motorTemp, sbyte1 batteryTemp, Sensor *HVILTermSense)
{
    //Water pump ------------------ ALWAYS ON
    if(HVILTermSense->sensorValue == TRUE){
        me->waterPumpPercent = 1;
    } else if (HVILTermSense->sensorValue == FALSE && (motorControllerTemp >= 50.0 || motorTemp >= 50.0)){
        me->waterPumpPercent = 1;
    } else {
        me->waterPumpPercent = 1;
    }
    //On the car- if pumps dont turn on with HV reverse TRUE/FALSE, if pumps dont turn on correctly with HV low then change waterPumpPercent 0/1

    if (motorControllerTemp >= me->radFanHigh || motorTemp >= me->radFanHigh)
    {
        me->radFanPercent = 1.0;  //0.9
    }
    else if (motorControllerTemp < me->radFanLow && motorTemp < me->radFanLow)
    {
        me->radFanPercent = 0.3; //0.2
    }
    else
    {
        //me->radFanPercent = .2 + .7 * getPercent(max(motorControllerTemp, motorTemp), me->radFanLow, me->radFanHigh, TRUE);
        me->radFanPercent = 0.3;
    }

    /*
    //Motor fan / rad fan
    if (me->motorFanState == FALSE)
    {
        if ((motorControllerTemp >= me->motorFanHigh) || (motorTemp >= me->motorFanHigh))
        {
            me->motorFanState = TRUE;
            SerialManager_send(me->sm, "Turning motor fans on.\n");
        }
    }
    else //motor fan is on
    {
        if ((motorControllerTemp < me->motorFanLow) && (motorTemp < me->motorFanLow))
        // Shouldn't this be an || instead of an &&
        {
            me->motorFanState = FALSE;
            SerialManager_send(me->sm, "Turning motor fans off.\n");
        }
    }

    //Battery fans
    if (me->batteryFanState == TRUE)
    {
        if (batteryTemp < me->batteryFanLow)
        {
            me->batteryFanState = FALSE;
            SerialManager_send(me->sm, "Turning battery fans off.\n");
        }
    }
    else //fans are off
    {
        if (batteryTemp >= me->batteryFanHigh)
        {
            me->batteryFanState = TRUE;
            SerialManager_send(me->sm, "Turning battery fans on.\n");
        }
    }
    */
}

//-------------------------------------------------------------------
// Cooling system control - turns fans on/off, sends water pump PWM control signal
//Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_enactCooling(CoolingSystem *me)
{
    //Send PWM control signal to water pump
    Light_set(Cooling_waterPump, me->waterPumpPercent);
    Light_set(Cooling_RadFans, me->radFanPercent);

    // Issue #110 https://github.com/spartanracingelectric/VCU/issues/110
    // Relay wiring seems to be backwards for 2021 car: Fans are on while everything is cool,
    // and they turn OFF when systems get hot.  This boolean flips the software logic, but the
    // wiring needs to be fixed and this software hack needs to be removed in the future.
    /*
    bool wiringIsWrong = TRUE;
    
    if (wiringIsWrong)
    {
        Light_set(Cooling_Fans, me->motorFanState == TRUE ? 0 : 1);
        Light_set(Cooling_batteryFans, me->batteryFanState == TRUE ? 0 : 1);
    }
    else
    {
        Light_set(Cooling_motorFans, me->motorFanState == TRUE ? 1 : 0);
        Light_set(Cooling_batteryFans, me->batteryFanState == TRUE ? 1 : 0);
    }
    */
}