/*****************************************************************************
 * powerLimit.h - Power Limiting using a PID controller & LUT to simplify calculations
 * Initial Author(s): Shaun Gilmore / Harleen Sandhu
 ******************************************************************************
 * Power Limiting code with a flexible Power Target & Initialization Limit
 ****************************************************************************/
#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include <stdlib.h>
#include "stdbool.h"
#include "PID.h"

#include "math.h"

// Define a structure for the PID controller
typedef struct _PowerLimit {
    PID *pid;
    HashTable* hashtable;

//-------------CAN IN ORDER: 511: Power Limit Overview-----------------------------------------------------

    bool plStatus;
    int pidOutput;
    int plTorqueCommand;
    int plInitializationThreshold;
    int plTargetPower;
    int plMode;
    
    // me->pid->Kd; int

//-------------CAN IN ORDER: 512: Power Limit LUT Parameters-----------------------------------------------------

    int vFloorRFloor;
    int vFloorRCeiling;
    int vCeilingRFloor;
    int vCeilingRCeiling;


//-------------CAN IN ORDER: 513: Power Limit PID Outputs-----------------------------------------------------

    // me->pid->setpoint; int
    // me->pid->totalError; sbyte4
    // me->pid->Kp; int
    // me->pid->Ki; int

    /* WANT TO ADD */
    // me->pid->proportional;
    // me->pid->integral;
    // me->pid->derivative;

} PowerLimit;

PowerLimit* PL_new(); 

#endif //_POWERLIMIT_H