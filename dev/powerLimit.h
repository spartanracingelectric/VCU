
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

#define KWH_LIMIT 80000.0 // watts

// Define a structure for the PID controller
typedef struct _PowerLimit {
    PID *pid; 
    HashTable* hashtable;
    bool PLstatus;

//-------------CAN IN ORDER: 511: MCM Values For Power Limit-----------------------------------------------------

      float mcm_voltage; 
    float mcm_current; 
   float power;
   float wheelspeed;


//-------------CAN IN ORDER: 512: Power Limit-----------------------------------------------------

    float LUT_val;
    float error; 
    float estimatedtq; // in dNm
    float setpointtq;// in dNm

} PowerLimit;


void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid);
void populatePLHashTable(HashTable* table);

float getTorque(PowerLimit* me, HashTable* torque_hashtable, float voltage, float rpm);
PowerLimit* PL_new(); 

#endif //_PID_H