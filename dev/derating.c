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

    me->Derating_status = FALSE;
    me->Derating_cellTempLim = 55; //Degree C, Highest cell temp before limp mode is activated 
    me->Derating_socLim = 0; //%, Lowest SOC before limp mode is acctivated 
    me->Derating_torqueLim = 125; //Nm, The new max torque for limp mode
    me->Derating_powerLim = 0; //kW, The new power limit for limp mode

    return me;
}

void DeratingLimpMode(Derating* me, MotorController* mcm, BatteryManagementSystem* bms){ //Car will decrease torque (power once pl works) if (cells passes a certain temp || SOC passes a certain charge)
    sbyte2 mcm_torqueMax = (MCM_commands_getTorqueLimit(mcm) / 10.0); //Max torque set on mcm side
    // sbyte2 pl_powerMax = PL_getPowerLimit(pl); //idk the actual get command ideally look smth like that
    
}

bool getDeratingStatus(Derating* me){
    return me->Derating_status;
}
