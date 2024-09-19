
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
    bool plStatus;

//-------------CAN IN ORDER: 511: MCM Values For Power Limit-----------------------------------------------------

   // float mcm_current; 
    float power;
    float rpm;
    float LUTtq;
//-------------CAN IN ORDER: 512: Power Limit-----------------------------------------------------
// we need up update can.c/  dbc for all these 
    float pidOffset;
    float plfinaltq; 
    float pidsetpoint; // in dNm
    float pidactual;// in dNm

} PowerLimit;

void PL_calculateTorqueOffset(MotorController* mcm, PowerLimit* me, PID* plPID);
PowerLimit* PL_new(); 

#endif //_PID_H