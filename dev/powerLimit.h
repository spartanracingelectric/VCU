
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

#define KWH_LIMIT 10.0

// Define a structure for the PID controller
typedef struct _PowerLimit {
    PID *pid; 
    HashTable* hashtable;
    bool PLstatus;

//-------------CAN IN ORDER: 511: MCM Values For Power Limit-----------------------------------------------------

      float4 mcm_voltage; 
    float4 mcm_current; 
   float4 power;
   float4 wheelspeed;


//-------------CAN IN ORDER: 512: Power Limit-----------------------------------------------------

    float4 LUT_val;
    float4 error; 
    float4 estimatedtq; // in dNm
    float4 setpointtq;// in dNm

} PowerLimit;


void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid);
void populatePLHashTable(HashTable* table);

float4 getTorque(PowerLimit* me, HashTable* torque_hashtable, float4 voltage, float4 rpm);
PowerLimit* PL_new(); 

#endif //_PID_H