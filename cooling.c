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

    // Battery fans (Unused in 2021)
    me->batteryFanLow = 38;     //Turn off BELOW this point
    me->batteryFanHigh = 43;    //Turn on at this temperature
    me->batteryFanState = TRUE; //float4 batteryFanPercent;

    return me;
}

//-------------------------------------------------------------------
// Cooling system calculations - turns fans on/off, sends water pump PWM control signal
//Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_calculations(CoolingSystem *me, sbyte2 motorControllerTemp, sbyte2 motorTemp, sbyte1 batteryTemp)
{
    //Water pump ------------------
    //Water pump PWM protocol unknown
    if (motorControllerTemp >= me->waterPumpHigh || motorTemp >= me->waterPumpHigh)
    {
        me->waterPumpPercent = .9;
    }
    else if (motorControllerTemp < me->waterPumpLow && motorTemp < me->waterPumpLow)
    {
        me->waterPumpPercent = .2;
    }
    else
    {
        me->waterPumpPercent = .2 + .7 * getPercent(max(motorControllerTemp, motorTemp), me->waterPumpLow, me->waterPumpHigh, TRUE);
    }

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
}

//-------------------------------------------------------------------
// Cooling system control - turns fans on/off, sends water pump PWM control signal
//Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_enactCooling(CoolingSystem *me)
{
    //Send PWM control signal to water pump
    Light_set(Cooling_waterPump, me->waterPumpPercent);

    // Issue #110 https://github.com/spartanracingelectric/VCU/issues/110
    // Relay wiring seems to be backwards for 2021 car: Fans are on while everything is cool,
    // and they turn OFF when systems get hot.  This boolean flips the software logic, but the
    // wiring needs to be fixed and this software hack needs to be removed in the future.
    bool wiringIsWrong = TRUE;
    
    if (wiringIsWrong)
    {
        Light_set(Cooling_motorFans, me->motorFanState == TRUE ? 0 : 1);
        Light_set(Cooling_batteryFans, me->batteryFanState == TRUE ? 0 : 1);
    }
    else
    {
        Light_set(Cooling_motorFans, me->motorFanState == TRUE ? 1 : 0);
        Light_set(Cooling_batteryFans, me->batteryFanState == TRUE ? 1 : 0);
    }
}
