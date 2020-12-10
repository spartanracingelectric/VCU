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

    //Cooling systems:
    //Water pump (motor, controller) - PWM
    me->waterPumpMinPercent = 0.2;
    me->waterPumpLow = 25; //Start ramping beyond min at this temp
    me->waterPumpHigh = 40;
    me->waterPumpPercent = .2;

    //PP fans (motor, radiator) - Relay
    //Motor fan + radiator on same circuit
    me->motorFanLow = 30;     //Turn off BELOW tuhis point
    me->motorFanHigh = 32;    //Turn on at this temperature
    me->motorFanState = TRUE; //float4 motorFanPercent;

    //Radiator fan
    //sbyte1 radiatorFanLo = 30; //Turn off BELOW tuhis point
    //sbyte1 radiatorFanHigh = 32; //Turn on at this temperature
    //bool motorFanState;

    //Battery fans (batteries) - Relay
    me->batteryFanLow = 32;     //Turn off BELOW tuhis point
    me->batteryFanHigh = 35;    //Turn on at this temperature
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

    //ubyte1* tempMsg[25];
    //sprintf(tempMsg, "Motor temp: %d\n", motorTemp);
    //SerialManager_send(me->sm, tempMsg);

    // START - RABEEL's CODE
    /*******************************
    sprintf(tempMsg, "MCM temp %d >= %d ? ", motorControllerTemp, me->motorFanHigh);
    SerialManager_send(me->sm, tempMsg);
    if ((motorControllerTemp >= me->motorFanHigh)) // || (motorTemp >= me->motorFanHigh))
    {
        SerialManager_send(me->sm, "Yes.\n");
        me->motorFanState = TRUE;
    }
    else
    {
        SerialManager_send(me->sm, "No.\n");
        me->motorFanState = FALSE;
    }

    sprintf(tempMsg, "Batt temp %d > %d ? ", batteryTemp, me->batteryFanHigh);
    SerialManager_send(me->sm, tempMsg);
    if ((batteryTemp >= me->batteryFanHigh))
    {
        SerialManager_send(me->sm, "Yes.\n");
        me->batteryFanState = TRUE;
    }
    else
    {
        SerialManager_send(me->sm, "No.\n");
        me->batteryFanState = FALSE; 
    }

    // END - RABEEL's CODE
    *******************************/

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

//    // Turn on FANS
//    //IO_DO_Init(IO_DO_06);
//    IO_DO_Set(IO_DO_06, TRUE); //pin 142 - sending 12V
//    IO_PWM_SetDuty(IO_PWM_05, 0, NULL);  //Water pump
//}
//else
//{
//    IO_DO_Set(IO_DO_06, FALSE);
//}

//-------------------------------------------------------------------
// Cooling system control - turns fans on/off, sends water pump PWM control signal
//Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_enactCooling(CoolingSystem *me)
{
    //Send PWM control signal to water pump
    Light_set(Cooling_waterPump, me->waterPumpPercent);
    Light_set(Cooling_motorFans, me->motorFanState == TRUE ? 1 : 0);
    Light_set(Cooling_batteryFans, me->batteryFanState == TRUE ? 1 : 0);
}