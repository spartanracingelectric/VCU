
#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "bms.h"
#include "wheelSpeeds.h"

#include "torqueEncoder.h"

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
PowerLimit* PL_new(); 

#endif //_PID_H