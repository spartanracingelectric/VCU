
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
#define KWH_LIMIT        (float4) 50000.0  // watts
#define POWERLIMIT_INIT  (sbyte4) 35000  // 5kwh buffer to init PL before PL limit is hit
#define UNIT_CONVERSTION (float4) 95.49    // 9.549 *10.0 to convert to deci-newtonmeters
#define ABSOLUTE_MAX_TORQUE (sbyte2) 2400 //in DNm 

#endif

#define POWERLIMIT_METHOD
#define CAN_VERBOSE         0 // To be implemented later, but idea is want to check if can manager and here to see if we should be setting & transmitting certain values over can for debugging

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));

    me->hashtable = HashTable_new();
    PL_populateHashTable(me->hashtable); 

    me->plState = FALSE;
    me->pidOffset = 0; 
    me->plTorqueCommand = 0; 
    me->pidSetpoint = 0.0; 
    me->pidActual = 0.0;
    me->lutTorque = 0;
     
    return me;
}
// set to NOTDEFINED to invalidate code, to use change to POWERLIMIT_METHOD
#ifdef NOTDEFINED
/** TQ CALCULATIONS **/ 
void PL_calculateTorqueCommand(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
    
    
    // Getting APPS OUTPUT
    ubyte2 torqueMax = 231;
    // MCM_getTorqueMax(mcm);
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

void PL_populateHashTable(HashTable* table)
{
    ubyte1 i = 0;
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

void PL_populateHashTable(HashTable* table)
{
    ubyte1 i = 0;
}
#endif

#ifdef NOTDEFINED
/** LUT **/
void PL_calculateTorqueCommand(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid){
    // sbyte4 watts = MCM_getPower(mcm);
    if( MCM_getPower(mcm) > POWERLIMIT_INIT ){
        // Always set the flag
        me->plState = TRUE;

        //parameters we need for calculations//
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model is 0.027 ohms
        sbyte4 noLoadVoltage = (sbyte4)((float4)mcmCurrent * 0.027) + mcmVoltage;
        sbyte2 pidSetpoint = PL_getTorqueFromLUT(me, me->hashtable, noLoadVoltage, motorRPM);
        ubyte2 pidActual = MCM_getCommandedTorque(mcm);

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
    ubyte4 VOLTAGE_MIN      = 280;
    ubyte4 RPM_MIN          = 2000;
    // Calculating hashtable keys
    ubyte4 rpmInput         = rpm - RPM_MIN;
    ubyte4 voltageInput     = voltage - VOLTAGE_MIN;
    ubyte4 voltageFloor     = ubyte4_lowerStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 voltageCeiling   = ubyte4_upperStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 rpmFloor         = ubyte4_lowerStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    ubyte4 rpmCeiling       = ubyte4_upperStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    
    // Calculating these now to speed up interpolation later in method
    sbyte4 voltageLowerDiff = (sbyte4)(voltage - voltageFloor);
    sbyte4 voltageUpperDiff = (sbyte4)(voltageCeiling - voltage);
    sbyte4 rpmLowerDiff     = (sbyte4)(rpm - rpmFloor);
    sbyte4 rpmUpperDiff     = (sbyte4)(rpmCeiling - rpm);

    // Retrieve torque values from the hash table for the four corners
    sbyte4 vFloorRFloor     = (sbyte4)HashTable_getValue(torqueHashTable, voltageFloor, rpmFloor);
    sbyte4 vFloorRCeiling   = (sbyte4)HashTable_getValue(torqueHashTable, voltageFloor, rpmCeiling);
    sbyte4 vCeilingRFloor   = (sbyte4)HashTable_getValue(torqueHashTable, voltageCeiling, rpmFloor);
    sbyte4 vCeilingRCeiling = (sbyte4)HashTable_getValue(torqueHashTable, voltageCeiling, rpmCeiling);

    // Early escape in case values are the same. May want to make more complex for scenarios such as 2 of the values are the same. However, due to infrequency of hitting exactly on bounds, perhaps not efficeint, except for maybe voltage. should do the math i guess
    /*
    if(vFloorRFloor == vFloorRCeiling && vCeilingRFloor == vCeilingRCeiling)
    {
        me->lutTorque = vFloorRFloor;
        return vFloorRFloor;
    }
    */
    // Calculate interpolation values
    sbyte4 stepDivider          = VOLTAGE_STEP      * RPM_STEP;
    sbyte4 torqueFloorFloor     = vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    sbyte4 torqueFloorCeiling   = vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    sbyte4 torqueCeilingFloor   = vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    sbyte4 torqueCeilingCeiling = vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;

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
	{229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229},
	{213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213},
	{200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200},
	{187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187},
	{177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177},
	{167, 167, 167, 167, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168},
	{158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158},
	{150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150},
	{143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143},
	{136, 136, 136, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137, 137},
	{130, 130, 130, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131},
	{125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125},
	{120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120},
	{115, 115, 115, 115, 115, 115, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116},
	{110, 111, 111, 111, 111, 111, 111, 112, 111, 111, 111, 112, 112, 111, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112},
	{106, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 108, 108, 108, 108, 107, 108, 108, 108, 108, 108, 108, 108, 108, 108},
	{102, 103, 103, 103, 103, 103, 103, 103, 104, 104, 105, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104},
	{98, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
	{89, 96, 96, 96, 96, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97},
	{81, 92, 93, 93, 93, 93, 93, 94, 94, 94, 94, 97, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94},
	{72, 84, 90, 90, 90, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91},
	{63, 75, 86, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 89, 88, 88, 88, 89, 89, 89, 89},
	{54, 67, 78, 85, 85, 86, 85, 85, 85, 85, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86},
	{44, 58, 70, 80, 82, 83, 83, 83, 83, 83, 83, 83, 83, 84, 83, 83, 83, 83, 83, 83, 83, 84, 84, 84, 84, 84},
	{32, 49, 61, 72, 80, 80, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81},
	{15, 38, 53, 64, 74, 78, 78, 78, 78, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79}};
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
#endif

#ifdef POWERLIMIT_METHOD
void PL_calculateTorqueCommand(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid){
    sbyte2 maxTorque;
    float4 tpsPercent;
    sbyte4 watts = MCM_getPower(mcm);
    sbyte2 commandTorque;
    sbyte4 rpm = MCM_getMotorRPM(mcm);

    // sbyte2 plTorqueCommand = me->plTorqueCommand;
    TorqueEncoder_getOutputPercent(tps, &tpsPercent);
    float4 requestedTorque = tpsPercent * maxTorque;
    if(watts < POWERLIMIT_INIT){
        // MCM_update_PL_setTorqueCommand(mcm, requestedTorque);
    }
    else{
        maxTorque = PL_getMaxTorque(rpm);
        MCM_setMaxTorqueDNm(mcm, maxTorque);
    }
    // PL_getTorqueFromLUT(PowerLimit* me, HashTable* torqueHashTable, ubyte4 voltage, ubyte4 rpm){    // Find the floor and ceiling values for voltage and rpm


}

sbyte2 PL_getMaxTorque(sbyte4 rpm){    // Find the floor and ceiling values for voltage and rpm
    sbyte2 torque = (((KWH_LIMIT / rpm) * 2 * PI )/ 60);
    if(torque > (sbyte2)2400){
        return (sbyte2)2400;
    }
    else{
        return torque;
    }
}

void PL_populateHashTable(HashTable* table)
{
    ubyte1 i = 0;
}
#endif