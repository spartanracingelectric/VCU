#include "IO_Driver.h"
#include <stdlib.h>
// #include "IO_DIO.h"
// #include "IO_PWM.h"

#include "bms.h"
#include "cooling.h"
#include "mathFunctions.h"
#include "motorController.h"
#include "sensors.h"
#include "serial.h"

extern Sensor Sensor_HVILTerminationSense;

// All temperatures in C
CoolingSystem *CoolingSystem_new(SerialManager *serialMan)
{
    CoolingSystem *me = (CoolingSystem *) malloc(sizeof(struct _CoolingSystem));
    SerialManager *sm = serialMan;

    //-------------------------------------------------------------------
    // Cooling System Configuration
    //-------------------------------------------------------------------

    // Water pump PWM control (for motor and controller)
    // Note: the water pump needs to receive a PWM singal within n seconds of being turned on
    me->waterPumpMinPercent = 0.2;
    me->waterPumpLow        = 25; // Start ramping beyond min at this temp
    me->waterPumpHigh       = 40;
    me->waterPumpPercent    = 0.2;

    // Power pack fan relay  (for 2021 3ator)
    me->motorFanLow   = 38;   // Turn off BELOW this point
    me->motorFanHigh  = 43;   // Turn on at this temperature
    me->motorFanState = TRUE; // float4 motorFanPercent;

    me->radFanMinPercent = 0;
    me->radFanLow        = 25;
    me->radFanHigh       = 40;
    me->radFanPercent    = 0;

    // Battery fans (Unused in 2021)
    me->batteryFanLow   = 38;   // Turn off BELOW this point
    me->batteryFanHigh  = 43;   // Turn on at this temperature
    me->batteryFanState = TRUE; // float4 batteryFanPercent;

    return me;
}

//-------------------------------------------------------------------
// Cooling system calculations - turns fans PWM, sends water pump DO control signal
// Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------

void CoolingSystem_calculationsPump(CoolingSystem *me, sbyte2 motorControllerTemp, sbyte2 motorTemp, sbyte1 batteryTemp,
                                    Sensor *HVILTermSense)
{
    // Water pump ------------------ ALWAYS ON
    if (HVILTermSense->sensorValue == TRUE)
    {
        me->waterPumpPercent = 1;
    }
    else if (HVILTermSense->sensorValue == FALSE && (motorControllerTemp >= 50.0 || motorTemp >= 50.0))
    {
        me->waterPumpPercent = 1;
    }
    else
    {
        me->waterPumpPercent = 1;
    }
}

void CoolingSystem_calculationsFans(CoolingSystem *me, sbyte2 motorControllerTemp, sbyte2 motorTemp, sbyte1 batteryTemp,
                                    Sensor *HVILTermSense)
{
    if (motorControllerTemp >= me->radFanHigh || motorTemp >= me->radFanHigh)
    {
        me->radFanPercent = 1.0; // 0.9
    }
    else if (motorControllerTemp < me->radFanLow && motorTemp < me->radFanLow)
    {
        me->radFanPercent = 1.0; // 0.2
    }
    else
    {
        // me->radFanPercent = .2 + .7 * getPercent(max(motorControllerTemp, motorTemp), me->radFanLow, me->radFanHigh,
        // TRUE);
        me->radFanPercent = 1.0;
    }
}

//-------------------------------------------------------------------
// Cooling system control - turns fans on/off, sends water pump PWM control signal
// Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
// void CoolingSystem_enactCooling(CoolingSystem *me)
// {
//     //Send PWM control signal to water pump
//     Light_set(Cooling_waterPump, me->waterPumpPercent);
//     Light_set(Cooling_RadFans, me->radFanPercent);

//     // Issue #110 https://github.com/spartanracingelectric/VCU/issues/110
//     // Relay wiring seems to be backwards for 2021 car: Fans are on while everything is cool,
//     // and they turn OFF when systems get hot.  This boolean flips the software logic, but the
//     // wiring needs to be fixed and this software hack needs to be removed in the future.
//     /*
//     bool wiringIsWrong = TRUE;

//     if (wiringIsWrong)
//     {
//         Light_set(Cooling_Fans, me->motorFanState == TRUE ? 0 : 1);
//         Light_set(Cooling_batteryFans, me->batteryFanState == TRUE ? 0 : 1);
//     }
//     else
//     {
//         Light_set(Cooling_motorFans, me->motorFanState == TRUE ? 1 : 0);
//         Light_set(Cooling_batteryFans, me->batteryFanState == TRUE ? 1 : 0);
//     }
//     */
// }

void CoolingSystem_enactCoolingPump(CoolingSystem *me) { Light_set(Cooling_waterPump, me->waterPumpPercent); }

void CoolingSystem_enactCoolingFans(CoolingSystem *me) { Light_set(Cooling_RadFans, me->radFanPercent); }
