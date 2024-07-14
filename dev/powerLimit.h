
#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "math.h"

// Define a structure for the PID controller
typedef struct _PowerLimit {
    PID *pid; 
    HashTable* hashtable;
    sbyte2 powerLimittq; 
    bool PLstatus;
    sbyte2 error; 

} PowerLimit;


void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws);
void populatePLHashTable(HashTable* table);

ubyte4 getTorque(PowerLimit* pl, HashTable* torque_hashtable, float4 voltage, sbyte4 rpm);
PowerLimit* PL_new(); 

#endif //_PID_H