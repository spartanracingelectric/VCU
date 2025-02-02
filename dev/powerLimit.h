/*****************************************************************************
 * powerLimit.h - Power Limiting using a PID controller & LUT
 ****************************************************************************/
#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include "pid.h"

/**
 * Forward-declare the MotorController type so that we do not need
 * a separate header. We'll define its details in the test file.
 */
typedef struct _MotorController MotorController;

typedef enum _PLMode {
    TQ_ONLY_MODE = 1,
    PID_ONLY_MODE = 2,
    LUT_ONLY_MODE = 3,
    TQ_LUT_MODE = 4
} PLMode;

/**
 * The PowerLimit object.
 */
typedef struct _PowerLimit {
    PID *pid;

    // CAN in order (511): Power Limit Overview
    bool   plStatus;
    ubyte1 plMode;
    ubyte1 plTargetPower;
    ubyte1 plInitializationThreshold;
    sbyte2 plTorqueCommand; // The final torque command (in deci-Nm for your system).
    

    // CAN in order (513): Power Limit LUT 
    ubyte1 vFloorRFloor;
    ubyte1 vFloorRCeiling;
    ubyte1 vCeilingRFloor;
    ubyte1 vCeilingRCeiling;

    // Unassigned in CAN
    bool fieldWeakening; 
    bool plExit; // True = PL exits on exit conditions; False = PL stays on once PL turned on;
    bool plOn; 
} PowerLimit;

/** Constructor **/
PowerLimit* POWERLIMIT_new();

/** SETTER FUNCTIONS  **/
/** Set limp mode if needed (placeholder example) **/
void POWERLIMIT_setLimpModeOverride(PowerLimit* me);

/**
 * @brief High-level entry point that decides which method to call 
 *        based on plMode (1: Tq eqn, 2: Power PID, 3: LUT, 4: combination).
 */
void PowerLimit_calculateCommand(PowerLimit *me, MotorController *mcm, 
                                 bool fieldWeakening);

/**
 * @brief LUT-based method
 */
void POWERLIMIT_calculateTorqueCommand(PowerLimit *me, MotorController *mcm);

/**
 * @brief Tq = kW -> mechanical eqn method
 */
void POWERLIMIT_calculateTorqueCommandTorqueEquation(PowerLimit *me, MotorController *mcm);

/**
 * @brief Entirely power-based PID
 */
void POWERLIMIT_calculateTorqueCommandPowerPID(PowerLimit *me, MotorController *mcm);

/**
 * @brief Retrieve torque from LUT given noLoadVoltage and rpm
 */
sbyte4 POWERLIMIT_retrieveTorqueFromLUT(PowerLimit* me, sbyte4 noLoadVoltage, sbyte4 rpm);

/**
 * @brief The actual lookup in the array
 */
ubyte1 POWERLIMIT_getTorqueFromArray(ubyte4 noLoadVoltage, ubyte4 rpm);

// ---------- Basic Getters -------------
ubyte1 POWERLIMIT_getStatusCodeBlock(PowerLimit* me);
bool   POWERLIMIT_getStatus(PowerLimit* me);
ubyte1 POWERLIMIT_getMode(PowerLimit* me);
sbyte4 POWERLIMIT_getTorqueCommand(PowerLimit* me);
ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me);
ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me);
ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner);

#endif //_POWERLIMIT_H

