
#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include "PID.h"
#include "hashTable.h"
#include "mathFunctions.h"
#include <stdlib.h>
#include "stdbool.h"

#define KWH_LIMIT 20

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


PowerLimit* PL_new(); 

#endif //_PID_H