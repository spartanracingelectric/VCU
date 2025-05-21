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

PowerLimit* POWERLIMIT_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    // malloc returns NULL if it fails to allocate memory
    if (me == NULL)
        return NULL;

    me->pid = PID_new(400, 2, 0, 231, 100);
    PID_updateSettings(me->pid, frequency, 3);
    me->plMode = 0;

    me->cycle = FALSE;
    me->plTorqueCommand = 0; 
    me->plTargetPower = 80;
    me->plInitializationThreshold = me->plTargetPower - 20;
    return me;
}
#ifdef POWERLIMIT_ENABLE
/** SETTER FUNCTIONS  **/

void POWERLIMIT_setModeParameters(PowerLimit* me){

    /* SPACE FOR DASH ROTARY & BUTTON INTERACTIONS */

    /* The below code will be the proper way of interacting with the rotary dial once the button is made. For now, it remains commented out */
    /*
    get the button position and transcribe it to a mode setting

    me->plTargetPower = (9 - me->plMode) * 10;
    me->plInitializationThreshold = me->plTargetPower - 15;
    POWERLIMIT_setLimpModeOverride(PowerLimit* me);

    */

    /* Determine Power Limiting Power Target */
    me->plMode = 9 - (me->plTargetPower / 10); // 9 - 80/10 = 9 - 8 = 1; 9 - 70/10 = 9 - 7 = 2; etc...
    if(me->plTargetPower == 20)
        me->plMode = 5;

    me->plInitializationThreshold = me->plTargetPower - 20;
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

void POWERLIMIT_calculateTorqueCommand(PowerLimit *me, MotorController *mcm){
    
    //if(rotary_button_input != plMode)
    //POWERLIMIT_setModeParameters(me);
    if( (MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        sbyte2 pidSetpoint = (sbyte4)(me->plTargetPower * 9549 / MCM_getMotorRPM(mcm));

        PID_updateSettings(me->pid, setpoint, pidSetpoint);
        PID_computeOutput(me->pid, MCM_getCommandedTorque(mcm));

        me->plTorqueCommand = MCM_getCommandedTorque(mcm) + me->pid->output;
        if(me->plTorqueCommand > 231){
            me->pid->totalError -= (me->plTorqueCommand - 231) * me->pid->previousError / me->pid->output;
            me->plTorqueCommand = 231; // Need to integrate this into PID.c/.h or redesign the whole system
        }
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand * 10);
        MCM_set_PL_updateStatus(mcm, TRUE);
    }
    else {
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, FALSE);
    }
}


/** GETTER FUNCTIONS **/

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

#endif