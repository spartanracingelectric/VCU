/*****************************************************************************
 * powerLimit.h - Power Limiting using a PID controller & LUT to simplify calculations
 * Initial Author(s): Shaun Gilmore / Harleen Sandhu
 ******************************************************************************
 * Power Limiting code with a flexible Power Target & Initialization Limit
 ****************************************************************************/
#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
//#include "hashTable.h"
#include "math.h"
#include "sensors.h"


// Define a structure for the PID controller
typedef struct _PowerLimit {
    PID *pid;
    // HashTable* hashtable;

//-------------CAN IN ORDER: 511: Power Limit Overview-----------------------------------------------------

    bool   plStatus;
    ubyte1 plMode;
    ubyte1 plTargetPower;
    ubyte1 plKwLimit;
    ubyte1 plInitializationThreshold;
    sbyte2 plTorqueCommand;
    ubyte1 clampingMethod;
    //me->pid->pidOutput;   sbyte2

//-------------CAN IN ORDER: 512: Power Limit PID Output Details-----------------------------------------------------

    // me->pid->proportional;   sbyte2
    // me->pid->integral;       sbyte2
    // me->pid->derivative;     sbyte2
    // me->pid->antiWindupFlag; bool

    // For Testing Purposes
    // POWERLIMIT_getStatusCodeBlock(pl);


//-------------CAN IN ORDER: 513: Power Limit LUT Parameters-----------------------------------------------------

    ubyte1 vFloorRFloor;
    ubyte1 vFloorRCeiling;
    ubyte1 vCeilingRFloor;
    ubyte1 vCeilingRCeiling;

//-------------CAN IN ORDER: 514: Power Limit PID Information-----------------------------------------------------

    // me->pid->setpoint;   sbyte2
    // me->pid->totalError; sbyte4
    // me->pid->Kp;         ubyte1
    // me->pid->Ki;         ubyte1

    //Odd Man Out
    // me->pid->Kd;         ubyte1

// Unassigned in CAN
    bool plAlwaysOn;
    ubyte1 plThresholdDiscrepancy;

} PowerLimit;

PowerLimit* POWERLIMIT_new(); 

/** SETTER FUNCTIONS  **/
void POWERLIMIT_setLimpModeOverride(PowerLimit* me);
void PowerLimit_InitializeParameters(PowerLimit* me);

/** COMPUTATIONS **/
void PowerLimit_calculateCommand(PowerLimit *me, MotorController *mcm, TorqueEncoder *tps);
void POWERLIMIT_calculateTorqueCommandTorqueEquation(PowerLimit *me, MotorController *mcm);
void POWERLIMIT_calculateTorqueCommandPowerPID(PowerLimit *me, MotorController *mcm);
void POWERLIMIT_updatePIDController(PowerLimit* me, sbyte2 pidSetpoint, sbyte2 commandedTorque, ubyte1 clampingMethod);
/** GETTER FUNCTIONS **/

ubyte1 POWERLIMIT_getClampingMethod(PowerLimit* me);
bool   POWERLIMIT_getStatus(PowerLimit* me);
ubyte1 POWERLIMIT_getMode(PowerLimit* me);
sbyte2 POWERLIMIT_getTorqueCommand(PowerLimit* me);
ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me);
ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me);
//Returns 0xFF if an invalid corner is given
ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner);

#endif //_POWERLIMIT_H
