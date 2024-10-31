#include "hashTable.h"
#include "powerLimit.h"
#include "mathFunctions.h"
#define VOLTAGE_STEP     (int) 5        //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1);
#define RPM_STEP         (int) 160      //int rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); 

void PL_populateHashTable(HashTable* table)

{
    /*
    voltage is x axis
    rpm is y axis 
    values are torque in Nm
    */
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------    
    const int lookupTable[26][26] = {
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
    const int VOLTAGE_MIN = 280;
    const int VOLTAGE_MAX = 405;
    const int RPM_MIN = 2000;
    const int RPM_MAX = 6000;
    const int NUM_V = 26;
    const int NUM_S = 26;
    for(int row = 0; row < NUM_S; ++row) {
        for(int column = 0; column < NUM_V; ++column) {
            int voltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            int rpm   = RPM_MIN + row * RPM_STEP;
            int value = lookupTable[(int)row][(int)column];
            HashTable_insertPair(table, voltage, rpm, value);
        }
    }
}
int PL_getTorqueFromLUT(HashTable* torqueHashTable, int voltage, int rpm){
    int voltageFloor      = floorToNearest5(voltage);
    int voltageCeiling    = ceilToNearest5(voltage);
    int rpmFloor          = floorToNearest160(rpm);
    int rpmCeiling        = ceilToNearest160(rpm);
    
    // Calculating these now to speed up interpolation later in method

    // Retrieve torque values from the hash table for the four corners
    int vFloorRFloor      = HashTable_getValue(torqueHashTable, voltageFloor, rpmFloor);
    int vFloorRCeiling    = HashTable_getValue(torqueHashTable, voltageFloor, rpmCeiling);
    int vCeilingRFloor    = HashTable_getValue(torqueHashTable, voltageCeiling, rpmFloor);
    int vCeilingRCeiling  = HashTable_getValue(torqueHashTable, voltageCeiling, rpmCeiling);

    // Early escape in case values are the same. May want to make more complex for scenarios such as 2 of the values are the same.
    if(vFloorRFloor == vFloorRCeiling && vCeilingRFloor == vCeilingRCeiling)
    {
        return vFloorRFloor;
    }

    
    int horizontal_Interp = (((vCeilingRFloor - vFloorRFloor) / 5.0) + ((vCeilingRCeiling - vFloorRCeiling) / 5.0)) / 2.0;
    int vertical_Interp = (((vFloorRCeiling - vFloorRFloor) / 160.0) + ((vCeilingRCeiling - vCeilingRFloor) / 160.0)) / 2.0;
    // Calculate interpolation values
   int gainValueHoriz = voltage % 5;
    int gainValueVertical = rpm % 160;

    // Final TQ from LUT
    int TQ =  (gainValueHoriz * horizontal_Interp) + (gainValueVertical * vertical_Interp) + vFloorRFloor;
    
    /*
    float4 horizontalInterpolation = (((vCeilingRFloor - vFloorRFloor) / VOLTAGE_STEP) + ((vCeilingRCeiling - vFloorRCeiling) / VOLTAGE_STEP)) / 2.0;
    float4 verticalInterpolation   = (((vFloorRCeiling - vFloorRFloor) / RPM_STEP) + ((vCeilingRCeiling - vCeilingRFloor) / RPM_STEP)) / 2.0;

    // Calculate gains
    float4 gainValueHorizontal = (float4)fmod(voltage, VOLTAGE_STEP);
    float4 gainValueVertical   = (float4)fmod(rpm, RPM_STEP);

    // Combine interpolated values
    me->lutTorque = (gainValueHorizontal * horizontalInterpolation) + (gainValueVertical * verticalInterpolation) + vFloorRFloor;
    */
    return TQ;  // Adjust gain if necessary

}  // Find the floor and ceiling values for voltage and rpm)

int PL_getTorqueFromLUT2(HashTable* torqueHashTable, int voltage, int rpm){
    int voltageFloor      = int_lowerStepInterval(voltage, VOLTAGE_STEP);
    int voltageCeiling    = (int)int_upperStepInterval(voltage, VOLTAGE_STEP);
    int rpmFloor          = (int)int_lowerStepInterval(rpm, RPM_STEP);
    int rpmCeiling        = (int)int_upperStepInterval(rpm, RPM_STEP);
    
    // Calculating these now to speed up interpolation later in method
    int voltageLowerDiff  = voltage - voltageFloor;
    int voltageUpperDiff  = voltageCeiling - voltage;
    int rpmLowerDiff      = rpm - rpmFloor;
    int rpmUpperDiff      = rpmCeiling - rpm;

    // Retrieve torque values from the hash table for the four corners
    int vFloorRFloor      = HashTable_getValue(torqueHashTable, voltageFloor, rpmFloor);
    int vFloorRCeiling    = HashTable_getValue(torqueHashTable, voltageFloor, rpmCeiling);
    int vCeilingRFloor    = HashTable_getValue(torqueHashTable, voltageCeiling, rpmFloor);
    int vCeilingRCeiling  = HashTable_getValue(torqueHashTable, voltageCeiling, rpmCeiling);

    // Early escape in case values are the same. May want to make more complex for scenarios such as 2 of the values are the same.
    if(vFloorRFloor == vFloorRCeiling && vCeilingRFloor == vCeilingRCeiling)
    {
        return vFloorRFloor;
    }

    // Calculate interpolation values
    int stepDivider          = VOLTAGE_STEP      * RPM_STEP;
    int torqueFloorFloor     = vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    int torqueFloorCeiling   = vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    int torqueCeilingFloor   = vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    int torqueCeilingCeiling = vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;

    // Final TQ from LUT
    int TQ = (torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider; 
    
    /*
    float4 horizontalInterpolation = (((vCeilingRFloor - vFloorRFloor) / VOLTAGE_STEP) + ((vCeilingRCeiling - vFloorRCeiling) / VOLTAGE_STEP)) / 2.0;
    float4 verticalInterpolation   = (((vFloorRCeiling - vFloorRFloor) / RPM_STEP) + ((vCeilingRCeiling - vCeilingRFloor) / RPM_STEP)) / 2.0;

    // Calculate gains
    float4 gainValueHorizontal = (float4)fmod(voltage, VOLTAGE_STEP);
    float4 gainValueVertical   = (float4)fmod(rpm, RPM_STEP);

    // Combine interpolated values
    me->lutTorque = (gainValueHorizontal * horizontalInterpolation) + (gainValueVertical * verticalInterpolation) + vFloorRFloor;
    */

    return TQ; 
}
