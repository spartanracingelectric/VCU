#include <stdlib.h>
#include "IO_Driver.h"

#include "sensors.h"
#include "cooling.h"
#include "motorController.h"
#include "mathFunctions.h"
#include "bms.h"

extern Button Sensor_HVILTerminationSense;
extern DigitalOutput Water_Pump;
extern PWMOutput Rad_Fans;

// All temperatures in C
void CoolingSystem_new(CoolingSystem *me)
{
    //-------------------------------------------------------------------
    // Cooling System Configuration
    //-------------------------------------------------------------------

    // Water pump PWM control (for motor and controller)
    // Note: the water pump needs to receive a PWM signal within n seconds of being turned on
    me->waterPumpMinPercent = 0.2;
    me->waterPumpLow = 25; // Start ramping beyond min at this temp
    me->waterPumpHigh = 40;
    me->waterPumpPercent = 0.2;

    // Power pack fan relay  (for 2021 radiator)
    me->motorFanLow = 38;     // Turn off BELOW this point
    me->motorFanHigh = 43;    // Turn on at this temperature
    me->motorFanState = TRUE; // float4 motorFanPercent;

    me->radFanMinPercent = 0;
    me->radFanLow = 25;
    me->radFanHigh = 40;
    me->radFanPercent = 0;

    // Battery fans (Unused in 2021)
    me->batteryFanLow = 38;     // Turn off BELOW this point
    me->batteryFanHigh = 43;    // Turn on at this temperature
    me->batteryFanState = TRUE; // float4 batteryFanPercent;
}

//-------------------------------------------------------------------
// Cooling system calculations - turns fans PWM, sends water pump DO control signal
// Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_calculations(CoolingSystem *me, sbyte2 motorControllerTemp, sbyte2 motorTemp, sbyte1 batteryTemp, Sensor *HVILTermSense)
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
    // On the car- if pumps dont turn on with HV reverse TRUE/FALSE, if pumps dont turn on correctly with HV low then change waterPumpPercent 0/1

    if (motorControllerTemp >= me->radFanHigh || motorTemp >= me->radFanHigh)
    {
        me->radFanPercent = 1.0; // 0.9
    }
    else if (motorControllerTemp < me->radFanLow && motorTemp < me->radFanLow)
    {
        me->radFanPercent = 0.3; // 0.2
    }
    else
    {
        // me->radFanPercent = .2 + .7 * getPercent(max(motorControllerTemp, motorTemp), me->radFanLow, me->radFanHigh, TRUE);
        me->radFanPercent = 0.3;
    }
}

//-------------------------------------------------------------------
// Cooling system control - turns fans on/off, sends water pump PWM control signal
// Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_enactCooling(CoolingSystem *me)
{
    // Send PWM control signal to water pump
    DigitalOutput_set(&Water_Pump, me->waterPumpPercent);
    PWMOutput_set(&Rad_Fans, me->radFanPercent);
}