
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"

#ifndef POWERLIMITCONSTANTS
#define POWERLIMITCONSTANTS
 
//#define VOLTAGE_MIN      (ubyte2) 280
//#define VOLTAGE_MAX      (ubyte2) 405
//#define RPM_MIN          (ubyte2) 2000 // Data analysis says 2340 rpm min @ 70kW, on oct7-8 launch for sr-14
//#define RPM_MAX          (ubyte2) 6000
//#define NUM_V            (ubyte1) 26
//#define NUM_S            (ubyte1) 26
#define VOLTAGE_STEP     (ubyte2) 5        //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1);
#define RPM_STEP         (ubyte2) 160      //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); 

#define PI               (float4) 3.14159
#define KWH_LIMIT        (float4) 80000.0  // watts
#define POWERLIMIT_INIT  (sbyte4) 55000  // 5kwh buffer to init PL before PL limit is hit
#define UNIT_CONVERSTION (float4) 95.49    // 9.549 *10.0 to convert to deci-newtonmeters

#endif

#define POWERLIMIT_METHOD
#define CAN_VERBOSE         0 // To be implemented later, but idea is want to check if can manager and here to see if we should be setting & transmitting certain values over can for debugging

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
<<<<<<< HEAD
    if(POWERLIMIT_METHOD == 3)
    {
        me->hashtable = HashTable_new();
        PowerLimit_populateHashTable(me->hashtable); 
    }
=======
>>>>>>> 3198690 (Finally a working ifdef solution. Also, changed some things in Power PID to be casted as float4 since the integer math would just produce 0, instead of the intended values.)

    me->hashtable = HashTable_new();
    PL_populateHashTable(me->hashtable); 

    me->plState = FALSE;

    me->power = 0.0; 
    me->rpm = 0.0; 

<<<<<<< HEAD
    me->pidOffset = 0.0; 
    me->plfinaltq = 0.0; 
    me->pidsetpoint = 0.0; 
    me->pidactual = 0.0; 
    me->LUTtq = 0.0;
=======
    me->pidOffset = 0; 
    me->plTorqueCommand = 0; 
    me->pidSetpoint = 0.0; 
    me->pidActual = 0.0;
    me->lutTorque = 0;
>>>>>>> 3198690 (Finally a working ifdef solution. Also, changed some things in Power PID to be casted as float4 since the integer math would just produce 0, instead of the intended values.)
     
    return me;
}
// set to NOTDEFINED to invalidate code, to use change to POWERLIMIT_METHOD
#ifdef POWERLIMIT_METHOD
/** TQ CALCULATIONS **/ 
void PL_calculateTorqueCommand(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
    
    
    // Getting APPS OUTPUT
    ubyte2 torqueMax = MCM_getTorqueMax(mcm);
    float4 appsPercent;
    TorqueEncoder_getOutputPercent(tps, &appsPercent);
    float4 appsTorque = appsPercent * torqueMax;
    
    //other variables

    float4 gain = 9.549; 
    float4 decitq = 10.0; 
    sbyte4 watts = MCM_getPower(mcm); // divide by 1000 to get watts --> kilowatts 
    float4 rpm = (float4)MCM_getMotorRPM(mcm);

    
    if(watts > POWERLIMIT_INIT){
        me-> plState = TRUE;
        // still need to make/ update all the struct parameters aka values for can validation 
        float4 pidSetpoint = KWH_LIMIT *  gain / rpm * decitq; 
        float4 pidActual = ((float4)watts) * gain / rpm *decitq;
        PID_updateSetpoint(pid,pidSetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
        sbyte2 pidOffset =  PID_computeOffset(pid, pidActual);
        sbyte2 plTorqueCommand = pidActual + pidOffset;

        me->pidOffset = pidOffset;
        me->plTorqueCommand = plTorqueCommand; 
        me->pidSetpoint = pidSetpoint;
        me->pidActual = pidActual;

    }
    else {
        me-> plState = FALSE;
    }

    sbyte2 plTorqueCommand = me->plTorqueCommand;
    MCM_update_PL_setTorqueCommand(mcm, plTorqueCommand); // we need to change this on mcm.c / pl.c/.h 
    MCM_set_PL_updateState(mcm, me->plState); 

    // in mcm.c input the if statement for the tps
}

#endif

#ifdef NOTDEFINED
/** Power PID **/
void PL_calculateTorqueCommand(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid){
    sbyte4 watts = MCM_getPower(mcm);
    if(watts > POWERLIMIT_INIT) {
        me->plState          = TRUE;
        sbyte2 commandedTQ   = MCM_getCommandedTorque(mcm);
        me->pidOffset           = PID_computeOffset(pid, (float4)watts);
        sbyte2 torqueCommand = (sbyte2)((float4)commandedTQ * (1.0 + ((float4)me->pidOffset / (float4)watts)));
        MCM_update_PL_setTorqueCommand(mcm, torqueCommand);
    }
    else { me->plState    = FALSE; }
    MCM_set_PL_updateState(mcm, me->plState);
}

#endif

#ifdef NOTDEFINED
/** LUT **/
void PL_calculateTorqueCommand(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid){
    // sbyte4 watts = MCM_getPower(mcm);
    if( MCM_getPower(mcm) > KWH_THRESHOLD ){
        // Always set the flag
        me->plState = TRUE;

        //parameters we need for calculations//
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model is 0.027 ohms
        sbyte4 noLoadVoltage = (sbyte4)((float4)mcmCurrent * 0.027) + mcmVoltage;
        sbyte2 pidSetpoint = PL_getTorqueFromLUT(me, me->hashtable, noLoadVoltage, motorRPM);
        sbyte2 pidActual = MCM_getCommandedTorque(mcm);

        PID_updateSetpoint(pid, (float4)pidSetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
        sbyte2 offset =  PID_computeOffset(pid, ((float4)pidActual));
        sbyte2 torqueRequest = pidActual + offset;

        // Setting member values for CAN message debugging. Will change to an if / define to easily toggle in the future.
        me->pidOffset = offset;
        me->plTorqueCommand = torqueRequest;
        me->pidSetpoint = (float4)pidSetpoint;
        me->pidActual = (float4)pidActual;
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);

    }
    else {
        me-> plState = FALSE;
    }
    MCM_set_PL_updateState(mcm, me->plState); 

}

sbyte2 PL_getTorqueFromLUT(PowerLimit* me, HashTable* torqueHashTable, ubyte4 voltage, ubyte4 rpm){    // Find the floor and ceiling values for voltage and rpm
    
    //LUT Lower Bounds
    ubyte4 VOLTAGE_MIN = 280;
    ubyte4 RPM_MIN = 2000;
    
    // Calculating hashtable keys
    ubyte4 rpmInput = rpm - RPM_MIN;
    ubyte4 voltageInput = voltage - VOLTAGE_MIN;
    ubyte4 voltageFloor      = ubyte4_lowerStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 voltageCeiling    = ubyte4_upperStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 rpmFloor          = ubyte4_lowerStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    ubyte4 rpmCeiling        = ubyte4_upperStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    
    // Calculating these now to speed up interpolation later in method
    ubyte2 voltageLowerDiff  = (ubyte2)voltage - voltageFloor;
    ubyte2 voltageUpperDiff  = (ubyte2)voltageCeiling - voltage;
    ubyte2 rpmLowerDiff      = (ubyte2)rpm - rpmFloor;
    ubyte2 rpmUpperDiff      = (ubyte2)rpmCeiling - rpm;

    // Retrieve torque values from the hash table for the four corners
    sbyte2 vFloorRFloor      = (sbyte2)HashTable_getValue(torqueHashTable, voltageFloor, rpmFloor);
    sbyte2 vFloorRCeiling    = (sbyte2)HashTable_getValue(torqueHashTable, voltageFloor, rpmCeiling);
    sbyte2 vCeilingRFloor    = (sbyte2)HashTable_getValue(torqueHashTable, voltageCeiling, rpmFloor);
    sbyte2 vCeilingRCeiling  = (sbyte2)HashTable_getValue(torqueHashTable, voltageCeiling, rpmCeiling);

    // Early escape in case values are the same. May want to make more complex for scenarios such as 2 of the values are the same. However, due to infrequency of hitting exactly on bounds, perhaps not efficeint, except for maybe voltage. should do the math i guess
    /*
    if(vFloorRFloor == vFloorRCeiling && vCeilingRFloor == vCeilingRCeiling)
    {
        me->lutTorque = vFloorRFloor;
        return vFloorRFloor;
    }
    */
    // Calculate interpolation values
    sbyte2 stepDivider          = VOLTAGE_STEP      * RPM_STEP;
    sbyte2 torqueFloorFloor     = vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    sbyte2 torqueFloorCeiling   = vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    sbyte2 torqueCeilingFloor   = vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    sbyte2 torqueCeilingCeiling = vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;

    // Final TQ from LUT
    sbyte2 TQ = (torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider; 
    
    /*
    float4 horizontalInterpolation = (((vCeilingRFloor - vFloorRFloor) / VOLTAGE_STEP) + ((vCeilingRCeiling - vFloorRCeiling) / VOLTAGE_STEP)) / 2.0;
    float4 verticalInterpolation   = (((vFloorRCeiling - vFloorRFloor) / RPM_STEP) + ((vCeilingRCeiling - vCeilingRFloor) / RPM_STEP)) / 2.0;

    // Calculate gains
    float4 gainValueHorizontal = (float4)fmod(voltage, VOLTAGE_STEP);
    float4 gainValueVertical   = (float4)fmod(rpm, RPM_STEP);

    // Combine interpolated values
    me->lutTorque = (gainValueHorizontal * horizontalInterpolation) + (gainValueVertical * verticalInterpolation) + vFloorRFloor;
    */

    me->lutTorque = TQ;
    return me->lutTorque;  // Adjust gain if necessary
}

#endif

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
    const ubyte1 POWER_LIM_LUT[26][26] = {
        {231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {222, 229, 230, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {205, 214, 221, 228, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {187, 198, 206, 214, 221, 227, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {172, 180, 189, 198, 205, 214, 221, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {157, 166, 174, 182, 189, 198, 205, 213, 218, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {144, 152, 160, 168, 175, 183, 190, 199, 205, 213, 218, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {131, 138, 146, 154, 161, 170, 177, 184, 193, 199, 207, 221, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {118, 126, 133, 141, 150, 157, 164, 171, 179, 185, 192, 208, 221, 226, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227},
        {106, 114, 123, 131, 138, 145, 153, 159, 167, 173, 180, 198, 209, 216, 216, 217, 217, 217, 217, 216, 217, 217, 217, 217, 217, 217},
        {95, 104, 112, 120, 127, 133, 140, 148, 155, 162, 169, 185, 199, 205, 207, 207, 207, 208, 208, 208, 208, 208, 208, 208, 208, 208},
        {84, 93, 101, 109, 116, 123, 131, 137, 144, 151, 157, 174, 186, 197, 198, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199},
        {73, 81, 89, 98, 106, 113, 121, 127, 134, 139, 147, 164, 176, 186, 190, 191, 191, 191, 191, 191, 191, 192, 191, 191, 192, 192},
        {61, 71, 79, 88, 95, 103, 110, 117, 124, 131, 138, 153, 166, 177, 182, 183, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184},
        {48, 59, 68, 77, 86, 94, 101, 108, 115, 120, 128, 144, 157, 166, 175, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177},
        {34, 47, 57, 67, 76, 84, 91, 99, 105, 111, 119, 135, 147, 157, 170, 170, 170, 171, 171, 171, 171, 171, 171, 171, 171, 171},
        {12, 33, 44, 56, 66, 74, 82, 89, 96, 103, 110, 126, 138, 150, 159, 164, 164, 165, 165, 165, 165, 165, 165, 165, 165, 165},
        {0, 10, 30, 44, 55, 64, 72, 80, 88, 94, 102, 117, 129, 139, 150, 156, 159, 159, 159, 159, 159, 160, 160, 160, 160, 160},
        {0, 0, 8, 30, 43, 54, 63, 71, 78, 86, 93, 109, 120, 132, 141, 151, 153, 154, 154, 154, 154, 154, 154, 154, 155, 155},
        {0, 0, 0, 6, 29, 42, 52, 61, 69, 77, 85, 101, 113, 125, 134, 142, 148, 149, 149, 150, 149, 150, 150, 150, 150, 150},
        {0, 0, 0, 0, 4, 28, 41, 51, 60, 68, 76, 93, 105, 117, 126, 135, 142, 144, 145, 145, 145, 145, 145, 145, 145, 142},
        {0, 0, 0, 0, 0, 0, 27, 40, 50, 59, 68, 85, 98, 109, 119, 127, 136, 139, 140, 140, 141, 141, 141, 141, 141, 141},
        {0, 0, 0, 0, 0, 0, 0, 26, 39, 49, 59, 77, 90, 101, 111, 120, 128, 134, 136, 136, 137, 137, 137, 137, 137, 137},
        {0, 0, 0, 0, 0, 0, 0, 0, 25, 38, 49, 68, 83, 94, 104, 113, 121, 129, 132, 132, 133, 133, 133, 133, 133, 133},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 38, 60, 75, 87, 97, 106, 115, 121, 127, 129, 129, 129, 129, 129, 129, 129},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 51, 67, 80, 90, 99, 107, 115, 122, 125, 126, 126, 126, 126, 126, 126}};
    const ubyte2 VOLTAGE_MIN = 280;
    const ubyte2 VOLTAGE_MAX = 405;
    const ubyte2 RPM_MIN = 2000;
    const ubyte2 RPM_MAX = 6000;
    const ubyte2 NUM_V = 26;
    const ubyte2 NUM_S = 26;
    for(ubyte2 row = 0; row < NUM_S; ++row) {
        for(ubyte2 column = 0; column < NUM_V; ++column) {
            ubyte2 voltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            ubyte2 rpm   = RPM_MIN + row * RPM_STEP;
            ubyte1 value = lookupTable[(int)row][(int)column];
            HashTable_insertPair(table, voltage, rpm, value);
        }
    }
}

