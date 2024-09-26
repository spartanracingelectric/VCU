
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"

#ifndef POWERLIMITCONSTANTS
#define POWERLIMITCONSTANTS
 
//#define VOLTAGE_MIN      (ubyte2) 280
//#define VOLTAGE_MAX      (ubyte2) 405
//#define RPM_MIN          (ubyte2) 2000 // Data analysis says 2340 rpm min @ 70kW, on oct7-8 launch for sr-14
//#define RPM_MAX          (ubyte2) 6000
//#define NUM_V            (ubyte1) 25
//#define NUM_S            (ubyte1) 25
#define VOLTAGE_STEP     (ubyte2) 5        //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1);
#define RPM_STEP         (ubyte2) 160      //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); 

#define PI               (float4) 3.14159
#define KWH_LIMIT        (float4) 80000.0  // watts
#define PL_INIT          (float4) 55000.0  // 5kwh buffer to init PL before PL limit is hit
#define UNIT_CONVERSTION (float4) 95.49    // 9.549 *10.0 to convert to deci-newtonmeters

#endif


#define POWERLIMIT_METHOD   1 // STATES: 1-3 are for the 3 different PL methods currently in place
#define CAN_VERBOSE         0 // To be implemented later, but idea is want to check if can manager and here to see if we should be setting & transmitting certain values over can for debugging

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    if(POWERLIMIT_METHOD == 3)
    {
        me->hashtable = HashTable_new();
        PowerLimit_populateHashTable(me->hashtable); 
    }

    me->plStatus = FALSE;   
    me->power = 0.0; 
    me->rpm = 0.0; 

    me->pidOffset = 0.0; 
    me->plfinaltq = 0.0; 
    me->pidsetpoint = 0.0; 
    me->pidactual = 0.0; 
    me->LUTtq = 0.0;
     
    return me;
}

void PL_calculateTorqueOffset(MotorController* mcm, PowerLimit* me, PID* plPID){
    me->watts = (float)MCM_getPower(mcm);
    if(me->watts > PL_INIT) {
        me->plState          = TRUE;
        int maxTQ            = MCM_getTorqueMax(mcm);
        sbyte2 commandedTQ   = MCM_commands_PL_getTorque(me);
        me->offset           = PID_computeOffset(plPID, me->watts);
        ubyte2 torqueCommand = (ubyte2)commandedTQ * (1 + ((ubyte2)(me->offset / me->watts))); //offsetTQ is the complete torque request
        MCM_update_PL_torqueCommand(mcm, torqueCommand);
    }
    else { me->plState    = FALSE; }
    MCM_update_PL_state(mcm, me->plState);
}