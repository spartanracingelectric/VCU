#include "IO_Driver.h" 
#include "motorController.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "PID.h"
#include "hashTable.h"
PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->hashtable = HashTable_new();
    POWERLIMIT_populateHashTable(me->hashtable);
    me->value=0;

    me->PLStatus = FALSE;
    me->LUTval=0;
    me->setpoint=0;
    me->actual= 0;
    me->pltorque= 0;
    me->piderror= 0;
   
    return me;
    }

void testing(PowerLimit *me){
    me->PLStatus = TRUE;
    me->value = 1000;
}

void POWERLIMIT_calculateTorqueCommand(PowerLimit* me, MotorController* mcm, PID* plPID){
    
    if(MCM_getPower(mcm) > 55000){
        me->PLStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model is 0.027 ohms
        sbyte4 noLoadVoltage = (mcmCurrent * 27 / 1000 ) + mcmVoltage; // 27 / 100 (0.027) is the estimated IR. Should attempt to revalidate on with new powerpack.
        //sbyte4 pidSetpoint = (sbyte4)POWERLIMIT_calculateTorqueFromLUT(me, &me->hashtable[me->plMode], noLoadVoltage, motorRPM);
        sbyte2 pidSetpoint = (sbyte2)POWERLIMIT_calculateTorqueFromLUT(me, me->hashtable, noLoadVoltage, motorRPM);

        ubyte2 commandedTorque = MCM_getCommandedTorque(mcm);

        PID_updateSetpoint(plPID, pidSetpoint);
        sbyte2 pidOutput =  PID_computeOutput(plPID, commandedTorque);
        sbyte2 torqueRequest = ((sbyte2)commandedTorque) + pidOutput;
        torqueRequest = torqueRequest *10;
        MCM_update_PL_setTorqueCommand(mcm, torqueRequest);
        MCM_set_PL_updateStatus(mcm, me->PLStatus);
    }
    else {
        me->PLStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, 0);
        MCM_set_PL_updateStatus(mcm, me->PLStatus);
    }
}

ubyte2 POWERLIMIT_calculateTorqueFromLUT(PowerLimit* me, HashTable* torqueHashTable, sbyte4 voltage, sbyte4 rpm){    // Find the floor and ceiling values for voltage and rpm
    
    // LUT Lower Bounds
    ubyte4 VOLTAGE_MIN      = 280;
    ubyte4 RPM_MIN          = 2000;
    
    // Calculating hashtable keys
    ubyte4 rpmInput         = ((ubyte4)rpm) - RPM_MIN;
    ubyte4 voltageInput     = ((ubyte4)voltage) - VOLTAGE_MIN;
    ubyte4 voltageFloor     = ubyte4_lowerStepInterval(voltageInput,(ubyte4)5) + VOLTAGE_MIN;
    ubyte4 voltageCeiling   = ubyte4_upperStepInterval(voltageInput, (ubyte4)5) + VOLTAGE_MIN;
    ubyte4 rpmFloor         = ubyte4_lowerStepInterval(rpmInput, (ubyte4)160) + RPM_MIN;
    ubyte4 rpmCeiling       = ubyte4_upperStepInterval(rpmInput, (ubyte4)160) + RPM_MIN;
    
    // Calculating these now to speed up interpolation later in method
    ubyte4 voltageLowerDiff = voltage - voltageFloor;
    ubyte4 voltageUpperDiff = voltageCeiling - voltage;
    ubyte4 rpmLowerDiff     = rpm - rpmFloor;
    ubyte4 rpmUpperDiff     = rpmCeiling - rpm;

    // Retrieve torque values from the hash table for the four corners
    ubyte4 vFloorRFloor     = (ubyte4)HashTable_getValue(torqueHashTable, voltageFloor, rpmFloor);
    ubyte4 vFloorRCeiling   = (ubyte4)HashTable_getValue(torqueHashTable, voltageFloor, rpmCeiling);
    ubyte4 vCeilingRFloor   = (ubyte4)HashTable_getValue(torqueHashTable, voltageCeiling, rpmFloor);
    ubyte4 vCeilingRCeiling = (ubyte4)HashTable_getValue(torqueHashTable, voltageCeiling, rpmCeiling);

    // Calculate interpolation values
    ubyte4 stepDivider          = (ubyte4)(5          * 160);
    ubyte4 torqueFloorFloor     = (ubyte4)vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    ubyte4 torqueFloorCeiling   = (ubyte4)vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    ubyte4 torqueCeilingFloor   = (ubyte4)vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    ubyte4 torqueCeilingCeiling = (ubyte4)vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;

    // Final TQ from LUT
    return (ubyte2)((torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider);
}

void POWERLIMIT_populateHashTable(HashTable* table)
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

    for(ubyte1 row = 0; row < NUM_S; ++row) {
        for(ubyte1 column = 0; column < NUM_V; ++column) {
            ubyte2 voltage = VOLTAGE_MIN + column * 5;
            ubyte2 rpm   = RPM_MIN + row * 160;
            ubyte1 value = POWER_LIM_LUT_80[row][column];
            HashTable_insertPair(table, voltage, rpm, value);
        }
    }

}