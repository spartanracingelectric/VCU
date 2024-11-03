#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H
#include "IO_Driver.h" 
#include "motorController.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "PID.h"
#include "hashTable.h"

typedef struct _PowerLimit {
    HashTable* hashtable;
    ubyte2 value;
    ubyte2 LUTval;
    bool PLStatus;
    ubyte2 setpoint;
    ubyte2 actual;
    ubyte2 pltorque;
    sbyte2 piderror;


}PowerLimit;

PowerLimit* PL_new();
void testing(PowerLimit *me);
void POWERLIMIT_calculateTorqueCommand(PowerLimit *me,  MotorController* mcm, PID* plPID);
ubyte2 POWERLIMIT_calculateTorqueFromLUT(PowerLimit* me, HashTable* torqueHashTable, sbyte4 voltage, sbyte4 rpm);
void POWERLIMIT_populateHashTable(HashTable* table);

#endif