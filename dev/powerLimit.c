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
    me->pid = PID_new(10, 0, 0, 231);
    me->plMode = 1;    // each number corresponds to a different method
    //1.TQ equation only
    //2.PowerPID only 
    //3.LUT only 
    //4. Both TQ equation and LUT together-(Final Algorithm)
    me->plStatus = FALSE;
    me->plTorqueCommand = 0; 
    me->plTargetPower = 55;// HERE IS WHERE YOU CHANGE POWERLIMIT
    me->plKwLimit = 80;
    me->plInitializationThreshold = me->plTargetPower - 15;
    me->clampingMethod = 1;

    //LUT Corners
    me->vFloorRFloor = 0;
    me->vFloorRCeiling = 0;
    me->vCeilingRFloor = 0;
    me->vCeilingRCeiling = 0;

    return me;
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

void PowerLimit_calculateCommand(PowerLimit *me, MotorController *mcm){
    me->plInitializationThreshold = me->plTargetPower - 15;

    if ((MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){
        if (!me->plStatus) {
            me->plStatus = TRUE;
            me->clampingMethod = 3;
        }
    }

  bool fieldWeakening = MCM_getFieldWeakening(mcm);
  
//1.TQ equation only
//2.PowerPID only 
//3.LUT only 
//4. Both TQ equation and LUT together-(Final Algorithm)

  if(me->plMode==1){
    POWERLIMIT_calculateTorqueCommandTorqueEquation(me, mcm);
  }
  else if (me->plMode==2){
    POWERLIMIT_calculateTorqueCommandPowerPID(me, mcm);
  }
   else if (me->plMode==3){ // write the saftey checks for these make sure that if the lut is out of range it uses tq equation
    POWERLIMIT_calculateLUTCommand(me, mcm);
  }
   else if (me->plMode==4){
    POWERLIMIT_calculateTorqueCommandTQAndLUT(me, mcm, fieldWeakening);
  }
}

void POWERLIMIT_calculateLUTCommand(PowerLimit *me, MotorController *mcm){
    
    //if(rotary_button_input != plMode)
    if( (MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){
        me->plStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model is 0.027 ohms
        sbyte4 noLoadVoltage = (mcmCurrent * 27 / 1000 ) + mcmVoltage; // 27 / 100 (0.027) is the estimated IR. Should attempt to revalidate on with new powerpack.
        //sbyte4 pidSetpoint = (sbyte4)POWERLIMIT_retrieveTorqueFromLUT(me, &me->hashtable[me->plMode], noLoadVoltage, motorRPM);
        //sbyte2 pidSetpoint = (sbyte2)POWERLIMIT_retrieveTorqueFromLUT(me, me->hashtable, noLoadVoltage, motorRPM);
        
        //issue here
        sbyte2 pidSetpoint = POWERLIMIT_retrieveTorqueFromLUT(me, noLoadVoltage, motorRPM);

        //TQ equation. uncomment to run this instead

        //pidSetpoint = (sbyte2)(me->plTargetPower * 9549 / MCM_getMotorRPM(mcm));

        // If the LUT gives a bad value this is our catch all
        if(pidSetpoint < 0 | pidSetpoint > 231){
            pidSetpoint = (sbyte2)(me->plTargetPower * 9549 / MCM_getMotorRPM(mcm)); 
        }

        sbyte2 commandedTorque = (sbyte2)MCM_getCommandedTorque(mcm);

        POWERLIMIT_updatePIDController(me, pidSetpoint, commandedTorque, me->clampingMethod);

        me->plTorqueCommand = ( commandedTorque + PID_getOutput(me->pid) ) * 10; //deciNewton-meters
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    /* FIX THIS 
    if(POWERLIMIT_getMode(me) >= 20 && POWERLIMIT_getMode(me) < 30){
        POWERLIMIT_calculateTorqueCommandTorqueEquation(me, mcm);
    }

    if(POWERLIMIT_getMode(me) >= 30 && POWERLIMIT_getMode(me) < 40){
        POWERLIMIT_calculateTorqueCommandPowerPID(me, mcm);
    }
    */
}

sbyte2 POWERLIMIT_retrieveTorqueFromLUT(PowerLimit *me, sbyte4 voltage, sbyte4 rpm){    // Find the floor and ceiling values for voltage and rpm
    
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
    me->vFloorRFloor     = POWERLIMIT_getTorqueFromArray(voltageFloor, rpmFloor);
    me->vFloorRCeiling   = POWERLIMIT_getTorqueFromArray(voltageFloor, rpmCeiling);
    me->vCeilingRFloor   = POWERLIMIT_getTorqueFromArray(voltageCeiling, rpmFloor);
    me->vCeilingRCeiling = POWERLIMIT_getTorqueFromArray(voltageCeiling, rpmCeiling);

    // If voltageFloor == voltageCeiling then voltageLowerDiff == voltageUpperDiff == 0, which means we get a multiply by 0 error.
    // We want a single interpolation bypass for any of these scenarios

    if(voltageLowerDiff == 0){
        return (sbyte2) ((ubyte4) me->vFloorRFloor + (rpmLowerDiff) * (me->vFloorRCeiling - me->vFloorRFloor) / RPM_STEP);
    }

    if(rpmLowerDiff == 0){
        return (sbyte2) ((ubyte4) me->vFloorRFloor + (voltageLowerDiff) * (me->vCeilingRFloor - me->vFloorRFloor) / VOLTAGE_STEP);
    }

    // Calculate interpolation values
    ubyte4 stepDivider          = (ubyte4)VOLTAGE_STEP          * RPM_STEP;
    ubyte4 torqueFloorFloor     = (ubyte4)me->vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    ubyte4 torqueFloorCeiling   = (ubyte4)me->vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    ubyte4 torqueCeilingFloor   = (ubyte4)me->vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    ubyte4 torqueCeilingCeiling = (ubyte4)me->vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;

    // Final TQ from LUT
    return (sbyte2)((torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider);
}

void POWERLIMIT_calculateTorqueCommandTorqueEquation(PowerLimit *me, MotorController *mcm){
    //doing this should be illegal, but since pl mode is also going to be used for the equation version for right now, i feel fine about it. 2 for second pl method, 1 representing the pwoer target
    me->plMode = 1;
    PID_setSaturationPoint(me->pid, 8000);
    if( (MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){
        me->plStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);

        sbyte2 pidSetpoint = (sbyte2)((sbyte4)me->plTargetPower * 9549 / MCM_getMotorRPM(mcm));

        sbyte2 commandedTorque = (sbyte2)MCM_getCommandedTorque(mcm);

        POWERLIMIT_updatePIDController(me, pidSetpoint, commandedTorque, me->clampingMethod);

        me->plTorqueCommand = ( commandedTorque + PID_getOutput(me->pid) ) * 10; //deciNewton-meters
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
}

void POWERLIMIT_calculateTorqueCommandPowerPID(PowerLimit *me, MotorController *mcm){
        //doing this should be illegal, but since pl mode is also going to be used for the equation version for right now, i feel fine about it. 3 for third pl method, 1 representing the pwoer target
    PID_setSaturationPoint(me->pid, 8000);
    me->plMode = 2;
    if( (MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){
        me->plStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        sbyte4 pidTargetValue = (sbyte4)(POWERLIMIT_getTargetPower(me) * 1000); // W
        sbyte4 pidCurrentValue = (sbyte4)(MCM_getPower(mcm) / 10); // W

        sbyte2 commandedTorque = (sbyte2)MCM_getCommandedTorque(mcm); // Nm

        POWERLIMIT_updatePIDController(me, pidTargetValue, pidCurrentValue, me->clampingMethod);


        sbyte4 pidOutput = PID_getOutput(me->pid);
        if (motorRPM == 0){
            motorRPM = 1; //avoid division by 0
        }
        sbyte4 PLTQ = (pidOutput + pidCurrentValue) / (motorRPM * 9.549);

        me->plTorqueCommand = PLTQ * 10; //convert to deci-Nm
        if (me->plTorqueCommand > 2310)
            me->plTorqueCommand = 2310;
            
            
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
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


ubyte1 POWERLIMIT_getStatusCodeBlock(PowerLimit* me){
    return me->plMode;
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

void POWERLIMIT_calculateTorqueCommandTQAndLUT(PowerLimit *me, MotorController *mcm, bool fieldWeakening){

    if (fieldWeakening)
        POWERLIMIT_calculateLUTCommand(me, mcm);
    else
        POWERLIMIT_calculateTorqueCommandTorqueEquation(me, mcm);
}


ubyte1 POWERLIMIT_getTorqueFromArray(ubyte4 noLoadVoltage, ubyte4 rpm)
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
