
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

#define KWH_LIMIT (float4) 50000.0 // watts
#define KWH_THRESHOLD (float4) 50000.0 // watts


// Define a structure for the PID controller
typedef struct _PowerLimit {
    bool plState;
    float watts;
    float offset;
} PowerLimit;

void PL_calculateTorqueOffset(MotorController* mcm, PowerLimit* me, PID* plPID);
PowerLimit* PL_new(); 

#endif //_PID_H