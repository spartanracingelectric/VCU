#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "derating.h"
#include "bms.h" //cell temps
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h" //torque request
#include "brakePressureSensor.h" //brakes
#include "motorController.h" //torque info, feedback 
#include "sensorCalculations.h"
//extern Sensor Sensor_PushToPass //Overriding Derating

/* Torque Limit
Endurance Limit: 170Nm
    170Nm, stays below 80kW
1. Derating only activates for limp mode
    - 55C, 125Nm
    1.Reading Data
        - Use pre-made functions from other files (BMS & MCM)
        - Refer to bms.c and motorController.c for get_functions
2. Sofrware integration


Step 1. Reading Data
Step 2. Logic stuff to alter "max" torque 

*/

Derating *Derating_new(){
    Derating* me = (Derating*)malloc(sizeof(Derating));

    me->Derating_status = OFF;
    me->Derating_cellTempLim = 55; //Degree C, Highest cell temp before limp mode is activated 
    me->Derating_socLim = 0.15; //%, Lowest SOC before limp mode is acctivated 
    me->Derating_torqueLim = 1250; //Nm, The new max torque for limp mode
    me->Derating_powerLim = 0; //kW, The new power limit for limp mode

    me->Derating_originalMaxTorque = 0; // Original max torque before derating is activated (updated to value in method)

    return me;
}

void DeratingLimpMode(Derating* me, MotorController* mcm, BatteryManagementSystem* bms){ //Car will decrease torque (power once pl works) if (cells passes a certain temp || SOC passes a certain charge)
    // sbyte2 mcm_torqueMax = (MCM_commands_getTorqueLimit(mcm) / 10.0); //Max torque set on mcm side
    // sbyte2 pl_powerMax = PL_getPowerLimit(pl); //idk the actual get command ideally look smth like that
    if((BMS_getHighestCellTemp_degC(bms) > me->Derating_cellTempLim || BMS_getLowestCellVoltage_mV(bms) < me->Derating_socLim) && me->Derating_status == OFF){
        me->Derating_status = ACTIVE;
        //Send messages over CAN
        me->Derating_originalMaxTorque = MCM_getMaxTorqueDNm(mcm);
        MCM_setMaxTorqueDNm(mcm, me->Derating_torqueLim); 
    }
    //2310 dN should be original torqueMax
    if ((BMS_getHighestCellTemp_degC(bms) <= me->Derating_cellTempLim && BMS_getLowestCellVoltage_mV(bms) >= me->Derating_socLim) && me->Derating_status == ACTIVE)
    {
        me->Derating_status = OFF;
        MCM_setMaxTorqueDNm(mcm, me->Derating_originalMaxTorque);
    }
    
    
    //Testing use LED to see derating status

    //If Push to pass sensor is TRUE -> Status = PUSHTOPASS -> car return to normal behavior
    // if (Sensor_PushToPass.sensorValue == TRUE)
    // {
    //     me->Derating_status = PUSHTOPASS;
    //     MCM_commands_setTorqueLimit(mcm, VCU_MCM_MAXTORQUE);
    // }
}

ubyte2 getDeratingStatus(Derating* me){
    return me->Derating_status;
}
