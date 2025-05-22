/*****************************************************************************
 * powerLimit.c - Power Limiting using a PID controller & LUT to simplify calculations
 * Initial Author(s): Harleen Sandhu/Shaun Gilmore 
 ******************************************************************************
 * Power Limiting code with a flexible Power Target & Initialization Limit
 * Goal: Find a way to limit power under a certain KWH limit (80kwh) while maximizing torque
 * Methods: Currently we are using three methods that are highlighted here:
 *  POWERLIMIT_calculateTorqueCommand: Algorithm is based on using a combination of LUT and the torque equation method
 *  POWERLIMIT_calculateTorqueCommandTorqueEquation: Algorithm is based on a mechanical conversion of power to torque
 *  POWERLIMIT_calculateTorqueCommandPowerPID: Algorithm uses power as a parameter inside the PID the percentage difference of the power is then used to offset torque.  
 * 
 ****************************************************************************/
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
//#include "hashTable.h"
#include "powerLimit.h"
#include "mathFunctions.h"

#ifndef POWERLIMITCONSTANTS
#define POWERLIMITCONSTANTS

#define VOLTAGE_STEP     5        //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1);
#define RPM_STEP         160      //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1);

#endif

// #define ELIMINATE_CAN_MESSAGES

PowerLimit* POWERLIMIT_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->pid = PID_new(1, 0, 0, 231,1); // last value tells you gain value factor
    me->plMode = 1;    // each number corresponds to a different method
    //1.TQ equation only
    //2.PowerPID only 
    //3.LUT only 
    //4. Both TQ equation and LUT together-(Final Algorithm)
    me->plStatus = FALSE;
    me->plTorqueCommand = 0; 
    me->plTargetPower = 20;// HERE IS WHERE YOU CHANGE POWERLIMIT
    me->plThresholdDiscrepancy = 5;
    me->plInitializationThreshold = 0;
    me->clampingMethod = 1;
    me->plAlwaysOn = TRUE;
    //LUT Corners
    me->vFloorRFloor = 0;
    
    me->vFloorRCeiling = 0;
    me->vCeilingRFloor = 0;
    me->vCeilingRCeiling = 0;

    return me;
}

void PowerLimit_setPLInitializationThreshold(PowerLimit* me){
    me->plInitializationThreshold = me->plTargetPower - me->plThresholdDiscrepancy;
}


void POWERLIMIT_setLimpModeOverride(PowerLimit* me){
    /*
    if(button press)
        me->plMode = 5;
        me->plTargetPower = 20;
            me->plInitializationThreshold = 0;

    */
}

/** COMPUTATIONS **/

void PowerLimit_calculateCommand(PowerLimit *me, MotorController *mcm, TorqueEncoder *tps){
    

    // if (!me->plStatus) 
    // {
    //     if ((MCM_getPower(mcm) / 1000) > me->plInitializationThreshold) {
    //         me->plStatus = TRUE;
    //         me->clampingMethod = 3;
    //     }
    // }

    PowerLimit_setPLInitializationThreshold(me);
    if (MCM_commands_getAppsTorque(mcm) == 0) { // hardcoded value because it doesn't take into account the curve which is deactivated right now
        me->plStatus = FALSE;
    } else {
        if ((MCM_getPower(mcm) / 1000) > me->plInitializationThreshold) {
            me->plStatus = TRUE;
        }
        if (!me->plAlwaysOn && ((MCM_getPower(mcm) / 1000) < me->plInitializationThreshold)) {
            me->plStatus = FALSE;
        }
    }


  
//1.TQ equation only
//2.PowerPID only 
//3.LUT only 
//4. Both TQ equation and LUT together-(Final Algorithm)
if (me->plStatus){
    if(me->plMode==1){
        POWERLIMIT_calculateTorqueCommandTorqueEquation(me, mcm);
      }
    //   else if (me->plMode==2){
    //     POWERLIMIT_calculateTorqueCommandPowerPID(me, mcm);
    //   }
    //    else if (me->plMode==3){ // write the saftey checks for these make sure that if the lut is out of range it uses tq equation
    //     POWERLIMIT_calculateLUTCommand(me, mcm);
    //   }
    //    else if (me->plMode==4){
    //     POWERLIMIT_calculateTorqueCommandTQAndLUT(me, mcm, fieldWeakening);
    //   }
}
else{
    MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
    MCM_set_PL_updateStatus(mcm, me->plStatus);
}
}

void POWERLIMIT_calculateTorqueCommandTorqueEquation(PowerLimit *me, MotorController *mcm){
    //doing this should be illegal, but since pl mode is also going to be used for the equation version for right now, i feel fine about it. 2 for second pl method, 1 representing the pwoer target
    me->plMode = 1;
    PID_setSaturationPoint(me->pid, 8000);

    /* Sensor inputs */
    sbyte4 motorRPM   = MCM_getMotorRPM(mcm);

    sbyte2 pidSetpoint = (me->plTargetPower - (sbyte1)(2)) * (9549.0/motorRPM); //DONT FUCKING TOUCH THIS LINE, please

    sbyte2 commandedTorque = (sbyte2)MCM_getCommandedTorque(mcm);

    POWERLIMIT_updatePIDController(me, pidSetpoint, commandedTorque, me->clampingMethod);

    me->plTorqueCommand = (commandedTorque + PID_getOutput(me->pid) ) * 10; //deciNewton-meters
    MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
    MCM_set_PL_updateStatus(mcm, me->plStatus);
}

void POWERLIMIT_updatePIDController(PowerLimit* me, sbyte2 pidSetpoint, sbyte2 sensorValue, ubyte1 clampingMethod) {
        sbyte2 currentError = PID_getSetpoint(me->pid) - sensorValue;
        switch (me->clampingMethod) {
            case 1: 
                if (currentError > 0) {
                    PID_setSaturationPoint(me->pid, 231); 
                }
            case 2: 
                if (currentError > 0) {
                    PID_setSaturationPoint(me->pid, sensorValue); 
                }
            case 3: 
                if (currentError > 0) {
                    me->pid->totalError -= PID_getPreviousError(me->pid); 
                }
            case 4: 
                if (currentError > 0) {
                    pidSetpoint = sensorValue; 
                }
        } 

        PID_updateSetpoint(me->pid, pidSetpoint);
        PID_computeOutput(me->pid, sensorValue);
}


ubyte1 POWERLIMIT_getClampingMethod(PowerLimit* me){
    return me->clampingMethod;
}

bool POWERLIMIT_getStatus(PowerLimit* me){
    return me->plStatus;
}

ubyte1 POWERLIMIT_getMode(PowerLimit* me){
    return me->plMode;
}

sbyte2 POWERLIMIT_getTorqueCommand(PowerLimit* me){
    return me->plTorqueCommand;
}

ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me){
    return me->plTargetPower;
}

ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me){
    return me->plInitializationThreshold;
}

