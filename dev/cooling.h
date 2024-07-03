#ifndef _COOLING_H
#define _COOLING_H

#include "IO_Driver.h"

typedef struct _CoolingSystem
{
    SerialManager *sm;

    //Cooling systems:
    //Water pump (motor, controller) - PWM
    float4 waterPumpMinPercent;
    sbyte1 waterPumpLow; //Start ramping beyond min at this temp
    sbyte1 waterPumpHigh;
    float4 waterPumpPercent;

    //PP fans (motor, radiator) - Relay
    //Motor fan + radiator on same circuit
    sbyte1 motorFanLow;  //Turn off BELOW tuhis point
    sbyte1 motorFanHigh; // Turn on at this temperature
    bool motorFanState;
    //float4 motorFanPercent;

    float4 radFanMinPercent;
    sbyte1 radFanLow; //Start ramping beyond min at this temp
    sbyte1 radFanHigh;
    float4 radFanPercent;

    //Battery fans (batteries) - Relay
    sbyte1 batteryFanLow;  //Turn off BELOW tuhis point
    sbyte1 batteryFanHigh; // Turn on at this temperature
    bool batteryFanState;
    //float4 batteryFanPercent;
    
} CoolingSystem;

CoolingSystem *CoolingSystem_new(SerialManager *sm);
// void CoolingSystem_calculations(CoolingSystem *me, sbyte2 motorControllerTemp, sbyte2 motorTemp, sbyte1 batteryTemp, Sensor *HVILTermSense);
void CoolingSystem_calculationsPump(CoolingSystem *me, sbyte2 motorControllerTemp, sbyte2 motorTemp, sbyte1 batteryTemp, Sensor *HVILTermSense);
void CoolingSystem_calculationsFan(CoolingSystem *me, sbyte2 motorControllerTemp, sbyte2 motorTemp, sbyte1 batteryTemp, Sensor *HVILTermSense);
// void CoolingSystem_enactCooling(CoolingSystem *me);
void CoolingSystem_enactCoolingPump(CoolingSystem *me);
void CoolingSystem_enactCoolingFans(CoolingSystem *me);

#endif