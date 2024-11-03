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

        PID_setpointUpdate(plPID, pidSetpoint);
        sbyte2 pidOutput =  PID_compute(plPID, commandedTorque);
        sbyte2 torqueRequest = ((sbyte2)commandedTorque) + pidOutput;
        torqueRequest = torqueRequest *10;
        MCM_update_PL_TorqueLimit(mcm, torqueRequest);
        MCM_update_PL_State(mcm, me->PLStatus);

    }
    else {
        me->PLStatus = FALSE;
        MCM_update_PL_TorqueLimit(mcm, 0);
        MCM_update_PL_State(mcm, me->PLStatus);
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
        {231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {208, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {191, 221, 223, 223, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224},
        {175, 206, 210, 210, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211, 211},
        {160, 192, 198, 199, 199, 199, 199, 199, 199, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200},
        {148, 178, 188, 188, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189},
        {133, 164, 177, 179, 179, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180},
        {122, 151, 168, 170, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171},
        {110, 139, 159, 162, 163, 163, 163, 163, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164},
        {99, 129, 146, 155, 156, 156, 156, 156, 156, 157, 157, 156, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157},
        {88, 118, 137, 148, 149, 149, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150},
        {76, 108, 126, 140, 143, 143, 143, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144, 144},
        {65, 98, 117, 130, 137, 138, 138, 138, 138, 138, 138, 139, 139, 139, 139, 139, 139, 139, 139, 139, 139, 139, 139, 139, 139, 139},
        {54, 88, 107, 120, 132, 133, 133, 133, 133, 133, 133, 133, 133, 133, 133, 134, 134, 134, 134, 134, 134, 134, 134, 134, 134, 134},
        {40, 78, 98, 113, 125, 127, 128, 128, 128, 128, 128, 129, 129, 129, 129, 129, 129, 129, 129, 129, 129, 129, 129, 129, 129, 129},
        {23, 68, 88, 103, 116, 123, 123, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 125, 125, 125, 125},
        {0, 58, 79, 95, 107, 118, 119, 119, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120},
        {0, 47, 70, 86, 99, 109, 115, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116},
        {0, 34, 60, 77, 90, 101, 111, 112, 112, 112, 112, 112, 112, 112, 112, 113, 112, 113, 113, 113, 113, 113, 113, 113, 113, 113},
        {0, 15, 50, 68, 82, 94, 103, 108, 108, 109, 109, 109, 109, 109, 109, 109, 109, 109, 109, 109, 109, 109, 110, 109, 109, 109},
        {0, 0, 38, 59, 74, 86, 96, 104, 105, 105, 105, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106},
        {0, 0, 24, 49, 65, 78, 88, 97, 102, 102, 102, 102, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103},
        {0, 0, 0, 38, 56, 69, 80, 90, 98, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
        {0, 0, 0, 24, 47, 61, 73, 82, 91, 96, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 95, 98},
        {0, 0, 0, 0, 36, 52, 65, 75, 84, 93, 94, 94, 94, 94, 94, 94, 94, 94, 95, 95, 94, 95, 95, 95, 95, 95}};

    for(ubyte1 row = 0; row < NUM_S; ++row) {
        for(ubyte1 column = 0; column < NUM_V; ++column) {
            ubyte2 voltage = VOLTAGE_MIN + column * 5;
            ubyte2 rpm   = RPM_MIN + row * 160;
            ubyte1 value = POWER_LIM_LUT_80[row][column];
            HashTable_insertPair(table, voltage, rpm, value);
        }
    }

}