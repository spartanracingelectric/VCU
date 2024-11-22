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

// Define a structure for the PID controller
typedef struct _PowerLimit {
    // PID *pid;
    // HashTable* hashtable;

//-------------CAN IN ORDER: 511: Power Limit Overview-----------------------------------------------------

    bool plStatus;
    sbyte2 pidOutput;
    sbyte2 plTorqueCommand;
    ubyte1 plInitializationThreshold;
    ubyte1 plTargetPower;
    ubyte1 plMode;
    
    // me->pid->Kd; ubyte1

//-------------CAN IN ORDER: 512: Power Limit LUT Parameters-----------------------------------------------------

    ubyte1 vFloorRFloor;
    ubyte1 vFloorRCeiling;
    ubyte1 vCeilingRFloor;
    ubyte1 vCeilingRCeiling;


//-------------CAN IN ORDER: 513: Power Limit PID Outputs-----------------------------------------------------

    // me->pid->setpoint; sbyte2
    // me->pid->totalError; sbyte4
    // me->pid->Kp; ubyte1
    // me->pid->Ki; ubyte1

    /* WANT TO ADD */
    // me->pid->proportional;
    // me->pid->integral;
    // me->pid->derivative;

} PowerLimit;

void POWERLIMIT_setModeParameters(PowerLimit* me);
void POWERLIMIT_setLimpModeOverride(PowerLimit* me);
void POWERLIMIT_calculateTorqueCommand(MotorController* mcm, PowerLimit* me, PID* plPID);
//void POWERLIMIT_populateHashTable(HashTable* table, ubyte1 mode);
//ubyte2 POWERLIMIT_calculateTorqueFromLUT(PowerLimit* me, HashTable* torqueHashtable, sbyte4 noLoadVoltage, sbyte4 rpm);
ubyte1 POWERLIMIT_getArray(ubyte4 noLoadVoltage, ubyte4 rpm);
ubyte2 POWERLIMIT_calculateTorqueFromLUT(PowerLimit* me, sbyte4 noLoadVoltage, sbyte4 rpm);
PowerLimit* POWERLIMIT_new(); 

/** GETTER FUNCTIONS **/

bool POWERLIMIT_getStatus(PowerLimit* me);
ubyte1 POWERLIMIT_getMode(PowerLimit* me);
sbyte2 POWERLIMIT_getTorqueCommand(PowerLimit* me);
ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me);
ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me);
//Returns 0xFF if an invalid corner is given
ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner);
sbyte2 POWERLIMIT_getPIDOutput(PowerLimit* me);

#endif //_POWERLIMIT_H