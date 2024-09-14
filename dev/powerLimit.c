
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
//#define NUM_V            (ubyte1) 25
//#define NUM_S            (ubyte1) 25
#define VOLTAGE_STEP     (ubyte2) 5        //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1);
#define RPM_STEP         (ubyte2) 160      //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); 

#define PI               (float4) 3.14159
#define KWH_LIMIT        (float4) 80000.0  // watts
#define PL_INIT          (float4) 55000.0  // 5kwh buffer to init PL before PL limit is hit
#define UNIT_CONVERSTION (float4) 95.49    // 9.549 *10.0 to convert to deci-newtonmeters

#endif


#define POWERLIMIT_METHOD   1 // STATES: 1-3 are for the 3 different PL methods currently in place
#define CAN_VERBOSE         0 // To be implemented later, but idea is want to check if can manager and here to see if we should be setting & transmitting certain values over can for debugging

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    if(POWERLIMIT_METHOD == 3)
    {
        me->hashtable = HashTable_new();
        PowerLimit_populateHashTable(me->hashtable); 
    }

    me->plState = FALSE;   
    me->power = 0.0; 
    me->rpm = 0.0; 

    me->pidOffset = 0.0; 
    me->plfinaltq = 0.0; 
    me->pidsetpoint = 0.0; 
    me->pidactual = 0.0; 
    me->LUTtq = 0.0;
     
    return me;
}

/* tqpid + equation */
#ifdef POWERLIMIT_METHOD == 1
void PL_calculateTorqueCommand(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
    // calc stuff//
    ubyte2 torqueMax = MCM_getTorqueMax(mcm);
    float appsTqPercent;
    TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
    float gain = 9.549;
    float decitq = 10.0;

    //parameters we need for calculations//
    float watts = (float)(MCM_getPower(mcm)); // divide by 1000 to get watts --> kilowatts
    float driversRequestedtq = appsTqPercent*torqueMax; 
    float rpm = (float)MCM_getMotorRPM(mcm);

    
    if(watts > KWH_THRESHOLD)
     {// kwhlimit should be changed to another paramter we make for plthreshold
        me-> plState = TRUE;
        // still need to make/ update all the struct parameters aka values for can validation 
        float pidSetpoint = (float)((KWH_LIMIT*gain/rpm)*decitq);
        float pidActual = (float)((watts*gain/rpm)*decitq);
        PID_updateSetpoint(pid,pidSetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
        float pidOffset =  PID_computeOffset(pid, pidActual);
        float plTorqueCommand = pidActual + pidOffset;

        me->pidOffset = pidOffset;
        me->plTorqueCommand =plTorqueCommand; 
        me->pidSetpoint = pidSetpoint;
        me->pidActual = pidActual;

    }
    else {
        me-> plState = FALSE;
    }

    float plTorqueCommand=  me->plTorqueCommand;
    MCM_update_PowerLimit_TorqueCommand(mcm,  plTorqueCommand); // we need to change this on mcm.c / pl.c/.h 
    MCM_update_PL_setState(mcm, me->plState); 

    // in mcm.c input the if statement for the tps
}

/* powerpid */
#elif POWERLIMIT_METHOD == 2

void PL_calculateTorqueCommand(MotorController* mcm, PowerLimit* me, PID* plPID){
    sbyte4 watts = MCM_getPower(mcm);
    if(watts > PL_INIT) {
        me->plState          = TRUE;
        int maxTQ            = MCM_getTorqueMax(mcm);
        sbyte2 commandedTQ   = MCM_commands_PL_getTorque(me);
        me->offset           = PID_computeOffset(plPID, me->watts);
        ubyte2 torqueCommand = (ubyte2)commandedTQ * (1 + ((ubyte2)(me->offset / me->watts)));
        MCM_update_PL_torqueCommand(mcm, torqueCommand);
    }
    else { me->plState    = FALSE; }
    MCM_set_PL_updateState(mcm, me->plState);
}



#elif POWERLIMIT_METHOD == 3
/** LUT METHOD */
void PL_calculateTorqueCommand(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
    sbyte4 watts = MCM_getPower(mcm);
    if( watts > KWH_THRESHOLD ){
    // Always set the flag
    me-> plState = TRUE;

    // Getting APPS OUTPUT
    ubyte2 torqueMax = MCM_getTorqueMax(mcm);
    float appsPercent;
    TorqueEncoder_getOutputPercent(tps, &appsPercent);
    float appsTorque = appsPercent * torqueMax; 

    //parameters we need for calculations//
    ubyte4 motorRPM = (ubyte4)MCM_getMotorRPM(mcm);
    sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
    sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

    // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model is 0.027 ohms
    ubyte4 noLoadVoltage = (ubyte4)(mcmCurrent * 0.027) + (ubyte4)mcmVoltage;
    float4 pidSetpoint = PL_getTorqueFromLUT(me, me->hashtable, noLoadVoltage, motorRPM);
    float4 pidActual = (float4)MCM_getCommandedTorque(mcm);

    PID_updateSetpoint(pid, pidSetpoint);
    //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
    float offset =  PID_computeOffset(pid, pidActual);
    float torqueRequest = pidActual + offset;
    // Setting member values for CAN message debugging. Will change to an if / define to easily toggle in the future.
    me->pidOffset = offset;
    me->plTorqueCommand = torqueRequest; 
    me->pidSetpoint = pidSetpoint;
    me->pidActual = pidActual;
    MCM_update_PowerLimit_TorqueCommand(mcm, me->plTorqueCommand); 
    }
    else {
        me-> plState = FALSE;
    }
    MCM_update_PL_setState(mcm, me->plState); 

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
    const ubyte1 lookupTable[25][25] = {
        {231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {221, 229, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {204, 214, 221, 228, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {186, 194, 204, 213, 221, 227, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {170, 179, 186, 197, 204, 213, 218, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {153, 164, 172, 179, 187, 197, 204, 213, 218, 224, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {139, 148, 157, 165, 174, 181, 189, 197, 204, 213, 222, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {127, 135, 144, 152, 158, 168, 175, 182, 189, 198, 208, 224, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {114, 122, 132, 138, 147, 155, 161, 170, 176, 184, 197, 213, 221, 223, 224, 223, 224, 224, 223, 224, 224, 224, 224, 224, 224},
        {102, 111, 119, 127, 135, 142, 150, 157, 164, 171, 183, 200, 209, 213, 213, 213, 213, 214, 213, 214, 214, 214, 214, 214, 214},
        {90, 99, 108, 115, 123, 132, 138, 146, 153, 159, 171, 186, 199, 203, 203, 204, 204, 204, 204, 204, 204, 204, 204, 204, 204},
        {79, 88, 97, 104, 113, 120, 127, 135, 141, 148, 161, 176, 189, 194, 195, 195, 195, 195, 195, 196, 196, 196, 196, 196, 196},
        {67, 77, 86, 94, 102, 110, 117, 124, 132, 138, 150, 164, 177, 184, 187, 187, 187, 187, 188, 188, 188, 188, 188, 188, 188},
        {54, 65, 75, 83, 91, 99, 107, 114, 120, 128, 140, 154, 167, 177, 179, 180, 180, 180, 180, 180, 180, 180, 180, 181, 181},
        {40, 53, 63, 73, 81, 89, 97, 104, 111, 118, 130, 144, 157, 166, 172, 173, 173, 173, 173, 173, 174, 174, 174, 174, 174},
        {22, 39, 51, 62, 71, 80, 87, 95, 102, 109, 120, 135, 147, 157, 165, 166, 167, 167, 167, 167, 167, 167, 167, 167, 167},
        {0, 21, 38, 50, 60, 69, 78, 85, 93, 99, 112, 126, 138, 148, 157, 160, 161, 161, 161, 161, 161, 161, 161, 161, 162},
        {0, 0, 19, 36, 49, 59, 68, 76, 83, 91, 103, 117, 129, 140, 150, 155, 155, 155, 155, 156, 156, 156, 156, 156, 156},
        {0, 0, 0, 18, 35, 47, 57, 66, 74, 81, 94, 109, 120, 132, 140, 148, 150, 150, 150, 151, 151, 151, 151, 151, 151},
        {0, 0, 0, 0, 17, 34, 46, 56, 64, 73, 86, 100, 113, 123, 133, 142, 145, 146, 146, 146, 146, 146, 146, 146, 146},
        {0, 0, 0, 0, 0, 15, 33, 45, 55, 63, 77, 92, 104, 116, 125, 134, 140, 141, 141, 141, 141, 141, 141, 142, 142},
        {0, 0, 0, 0, 0, 0, 14, 32, 44, 53, 68, 84, 97, 108, 117, 126, 134, 136, 137, 137, 137, 137, 137, 137, 137},
        {0, 0, 0, 0, 0, 0, 0, 12, 31, 43, 59, 76, 89, 100, 110, 119, 126, 131, 133, 133, 133, 133, 133, 133, 133},
        {0, 0, 0, 0, 0, 0, 0, 0, 11, 30, 49, 68, 81, 93, 103, 112, 120, 127, 129, 129, 129, 129, 129, 129, 129},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 39, 59, 74, 86, 96, 104, 113, 120, 125, 126, 126, 126, 126, 126, 126}};
    const ubyte2 VOLTAGE_MIN = 280;
    const ubyte2 VOLTAGE_MAX = 405;
    const ubyte2 RPM_MIN = 2000;
    const ubyte2 RPM_MAX = 6000;
    const ubyte2 NUM_V = 25;
    const ubyte2 NUM_S = 25;
    for(ubyte2 row = 0; row < NUM_S; ++row) {
        for(ubyte2 column = 0; column < NUM_V; ++column) {
            ubyte2 voltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            ubyte2 rpm   = RPM_MIN + row * RPM_STEP;
            ubyte1 value = lookupTable[(int)row][(int)column];
            HashTable_insertPair(lookupTable, voltage, rpm, value);
        }
    }
}

float PL_getTorqueFromLUTFromLUT(PowerLimit* me, HashTable* torqueHashTable, ubyte4 voltage, ubyte4 rpm){    // Find the floor and ceiling values for voltage and rpm
    
    // Calculating hashtable keys
    ubyte4 voltageFloor      = ubyte4_lowerStepInterval(voltage, VOLTAGE_STEP);
    ubyte4 voltageCeiling    = ubyte4_upperStepInterval(voltage, VOLTAGE_STEP);
    ubyte4 rpmFloor          = ubyte4_lowerStepInterval(rpm, RPM_STEP);
    ubyte4 rpmCeiling        = ubyte4_upperStepInterval(rpm, RPM_STEP);
    
    // Calculating these now to speed up interpolation later in method
    ubyte4 voltageLowerDiff  = voltage - voltageFloor;
    ubyte4 voltageUpperDiff  = voltageCeiling - voltage;
    ubyte4 rpmLowerDiff      = rpm - rpmFloor;
    ubyte4 rpmUpperDiff      = rpmCeiling - rpm;

    // Retrieve torque values from the hash table for the four corners
    float4 vFloorRFloor      = HashTable_getValue(torqueHashTable, voltageFloor, rpmFloor);
    float4 vFloorRCeiling    = HashTable_getValue(torqueHashTable, voltageFloor, rpmCeiling);
    float4 vCeilingRFloor    = HashTable_getValue(torqueHashTable, voltageCeiling, rpmFloor);
    float4 vCeilingRCeiling  = HashTable_getValue(torqueHashTable, voltageCeiling, rpmCeiling);

    // Early escape in case values are the same. May want to make more complex for scenarios such as 2 of the values are the same.
    if(vFloorRFloor == vFloorRCeiling && vCeilingRFloor == vCeilingRCeiling)
    {
        me->lutTorque = vFloorRFloor;
        return vFloorRFloor;
    }

    // Calculate interpolation values
    float4 stepDivider = (float4)(VOLTAGE_STEP * RPM_STEP);
    float4 torqueFloorFloor = vFloorRFloor * (float4)(voltageUpperDiff * rpmUpperDiff);
    float4 torqueFloorCeiling = vFloorRCeiling * (float4)(voltageUpperDiff * rpmLowerDiff);
    float4 torqueCeilingFloor = vCeilingRFloor * (float4)(voltageLowerDiff * rpmUpperDiff);
    float4 torqueCeilingCeiling = vCeilingRCeiling * (float4)(voltageLowerDiff * rpmLowerDiff);

    // Final TQ from LUT
    float4 TQ = (torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider; 
    
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
