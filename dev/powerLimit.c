/*****************************************************************************
 * powerLimit.c - Power Limiting using a PID controller & LUT to simplify calculations
 * Initial Author(s): Shaun Gilmore / Harleen Sandhu
 ******************************************************************************
 * Power Limiting code with a flexible Power Target & Initialization Limit
 * 
 * DESCRIPTION COMING SOON
 * 
 ****************************************************************************/
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "powerLimit.h"
#include "mathFunctions.h"

#ifndef POWERLIMITCONSTANTS
#define POWERLIMITCONSTANTS

#define VOLTAGE_STEP     5        //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1);
#define RPM_STEP         160      //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1);

#endif


PowerLimit* POWERLIMIT_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    // me->pid = PID_new(20, 0, 0, 0);

    me->plStatus = FALSE;
    me->pidOutput = 0; 
    me->plTorqueCommand = 0; 
    me->plTargetPower = 80;
    me->plInitializationThreshold = 0;

    me->vFloorRFloor = 0;
    me->vFloorRCeiling = 0;
    me->vCeilingRFloor = 0;
    me->vCeilingRCeiling = 0;


    return me;
}

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

    me->plInitializationThreshold = me->plTargetPower - 15;
}

void POWERLIMIT_setLimpModeOverride(PowerLimit* me){
    /*
    if(button press)
        me->plMode = 5;
        me->plTargetPower = 20;
            me->plInitializationThreshold = 0;

    */
}

void POWERLIMIT_calculateTorqueCommand(MotorController* mcm, PowerLimit* me, PID* plPID){
    
    //if(rotary_button_input != plMode)
    POWERLIMIT_setModeParameters(me);

    if( (MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){
        me->plStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model is 0.027 ohms
        sbyte4 noLoadVoltage = (mcmCurrent * 27 / 1000 ) + mcmVoltage; // 27 / 100 (0.027) is the estimated IR. Should attempt to revalidate on with new powerpack.
        //sbyte4 pidSetpoint = (sbyte4)POWERLIMIT_calculateTorqueFromLUT(me, &me->hashtable[me->plMode], noLoadVoltage, motorRPM);
        sbyte2 pidSetpoint = (sbyte2)POWERLIMIT_getArray(noLoadVoltage, motorRPM);

        // If the LUT gives a bad value this is our catch all
        if(pidSetpoint == -1){
            pidSetpoint = (me->plTargetPower *  9549 / MCM_getMotorRPM(mcm)) / 100; 
        }
        if(pidSetpoint > 231)
        {
            pidSetpoint = 231;
        }
        ubyte2 commandedTorque = MCM_getCommandedTorque(mcm);

        PID_updateSetpoint(plPID, pidSetpoint);
        sbyte2 pidOutput =  PID_computeOutput(plPID, commandedTorque);
        sbyte2 torqueRequest = ((sbyte2)commandedTorque) + pidOutput;
        torqueRequest = torqueRequest *10;
        me->pidOutput = pidOutput;
        me->plTorqueCommand = torqueRequest;
        MCM_update_PL_setTorqueCommand(mcm, torqueRequest);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, 0);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
}

ubyte2 POWERLIMIT_calculateTorqueFromLUT(PowerLimit* me, sbyte4 voltage, sbyte4 rpm){    // Find the floor and ceiling values for voltage and rpm
    
    // LUT Lower Bounds
    ubyte4 VOLTAGE_MIN      = 280;
    ubyte4 RPM_MIN          = 2000;
    
    // Calculating hashtable keys
    ubyte4 rpmInput         = (ubyte4)rpm - RPM_MIN;
    ubyte4 voltageInput     = (ubyte4)voltage - VOLTAGE_MIN;
    ubyte4 voltageFloor     = ubyte4_lowerStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 voltageCeiling   = ubyte4_upperStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 rpmFloor         = ubyte4_lowerStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    ubyte4 rpmCeiling       = ubyte4_upperStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    
    // Calculating these now to speed up interpolation later in method
    ubyte4 voltageLowerDiff = voltage - voltageFloor;
    ubyte4 voltageUpperDiff = voltageCeiling - voltage;
    ubyte4 rpmLowerDiff     = rpm - rpmFloor;
    ubyte4 rpmUpperDiff     = rpmCeiling - rpm;

    // Retrieve torque values from the hash table for the four corners
    me->vFloorRFloor     = POWERLIMIT_getArray(voltageFloor, rpmFloor);
    me->vFloorRCeiling   = POWERLIMIT_getArray(voltageFloor, rpmCeiling);
    me->vCeilingRFloor   = POWERLIMIT_getArray(voltageCeiling, rpmFloor);
    me->vCeilingRCeiling = POWERLIMIT_getArray(voltageCeiling, rpmCeiling);

    // Calculate interpolation values
    ubyte4 stepDivider          = (ubyte4)VOLTAGE_STEP          * RPM_STEP;
    ubyte4 torqueFloorFloor     = (ubyte4)me->vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    ubyte4 torqueFloorCeiling   = (ubyte4)me->vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    ubyte4 torqueCeilingFloor   = (ubyte4)me->vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    ubyte4 torqueCeilingCeiling = (ubyte4)me->vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;

    // Final TQ from LUT
    return (ubyte2)((torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider);
}

ubyte1 POWERLIMIT_getArray(ubyte4 noLoadVoltage, ubyte4 rpm)
{
    ubyte2 VOLTAGE_MIN = 280;
    ubyte2 VOLTAGE_MAX = 405;
    ubyte2 RPM_MIN = 2000; // Data analysis says 2340 rpm min @ 70kW, on oct7-8 launch for sr-14
    ubyte2 RPM_MAX = 6000;
    const ubyte1 NUM_V = 26;
    const ubyte1 NUM_S = 26;

    const ubyte1 POWER_LIM_LUT_80[26][26] = {
        {231, 231, 199, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {222, 229, 180, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {205, 214, 161, 228, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {187, 198, 146, 214, 221, 227, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {172, 180, 130, 198, 205, 214, 221, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {157, 166, 117, 182, 189, 198, 205, 213, 218, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {144, 152, 103, 168, 175, 183, 190, 199, 205, 213, 218, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {131, 138, 90, 154, 161, 170, 177, 184, 193, 199, 207, 221, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {118, 126, 77, 141, 150, 157, 164, 171, 179, 185, 192, 208, 221, 226, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227},
        {106, 114, 63, 131, 138, 145, 153, 159, 167, 173, 180, 198, 209, 216, 216, 217, 217, 217, 217, 216, 217, 217, 217, 217, 217, 217},
        {95, 104, 48, 120, 127, 133, 140, 148, 155, 162, 169, 185, 199, 205, 207, 207, 207, 208, 208, 208, 208, 208, 208, 208, 208, 208},
        {84, 93, 32, 109, 116, 123, 131, 137, 144, 151, 157, 174, 186, 197, 198, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199},
        {73, 81, 0, 98, 106, 113, 121, 127, 134, 139, 147, 164, 176, 186, 190, 191, 191, 191, 191, 191, 191, 192, 191, 191, 192, 192},
        {61, 71, 0, 88, 95, 103, 110, 117, 124, 131, 138, 153, 166, 177, 182, 183, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184},
        {48, 59, 0, 77, 86, 94, 101, 108, 115, 120, 128, 144, 157, 166, 175, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177},
        {34, 47, 0, 67, 76, 84, 91, 99, 105, 111, 119, 135, 147, 157, 170, 170, 170, 171, 171, 171, 171, 171, 171, 171, 171, 171},
        {12, 33, 0, 56, 66, 74, 82, 89, 96, 103, 110, 126, 138, 150, 159, 164, 164, 165, 165, 165, 165, 165, 165, 165, 165, 165},
        {0, 10, 0, 44, 55, 64, 72, 80, 88, 94, 102, 117, 129, 139, 150, 156, 159, 159, 159, 159, 159, 160, 160, 160, 160, 160},
        {0, 0, 0, 30, 43, 54, 63, 71, 78, 86, 93, 109, 120, 132, 141, 151, 153, 154, 154, 154, 154, 154, 154, 154, 155, 155},
        {0, 0, 0, 6, 29, 42, 52, 61, 69, 77, 85, 101, 113, 125, 134, 142, 148, 149, 149, 150, 149, 150, 150, 150, 150, 150},
        {0, 0, 0, 0, 4, 28, 41, 51, 60, 68, 76, 93, 105, 117, 126, 135, 142, 144, 145, 145, 145, 145, 145, 145, 145, 142},
        {0, 0, 0, 0, 0, 0, 27, 40, 50, 59, 68, 85, 98, 109, 119, 127, 136, 139, 140, 140, 141, 141, 141, 141, 141, 141},
        {0, 0, 0, 0, 0, 0, 0, 26, 39, 49, 59, 77, 90, 101, 111, 120, 128, 134, 136, 136, 137, 137, 137, 137, 137, 137},
        {0, 0, 0, 0, 0, 0, 0, 0, 25, 38, 49, 68, 83, 94, 104, 113, 121, 129, 132, 132, 133, 133, 133, 133, 133, 133},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 38, 60, 75, 87, 97, 106, 115, 121, 127, 129, 129, 129, 129, 129, 129, 129},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 51, 67, 80, 90, 99, 107, 115, 122, 125, 126, 126, 126, 126, 126, 126}};

    ubyte2 column = (ubyte2) (noLoadVoltage - VOLTAGE_MIN) / VOLTAGE_STEP;
    ubyte2 row    = (ubyte2) (rpm - RPM_MIN) / RPM_STEP;
    ubyte1 value = POWER_LIM_LUT_80[row][column];
    return value;
}

/** GETTER FUNCTIONS **/

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

ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner){
    // corner cases:
    // 1 - lowerX lowerY
    // 2 - lowerX lowerY
    // 3 - higherX lowerY
    // 4 - higherX higherY
    switch(corner){
        case 1:
        return me->vFloorRFloor;

        case 2:
        return me->vFloorRCeiling;
        
        case 3:
        return me->vCeilingRFloor;
        
        case 4:
        return me->vCeilingRCeiling;
        
        default:
        return 0xFF;
    }
}

sbyte2 POWERLIMIT_getPIDOutput(PowerLimit* me){
    return me->pidOutput;
}
