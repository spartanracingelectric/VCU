
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

<<<<<<< HEAD
#define POWERLIMIT_METHOD   1 // STATES: 1-3 are for the 3 different PL methods currently in place
#define CAN_VERBOSE         0 // To be implemented later, but idea is want to check if can manager and here to see if we should be setting & transmitting certain values over can for debugging

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    if(POWERLIMIT_METHOD == 3)
    {
        me->hashtable = HashTable_new();
        PowerLimit_populateHashTable(me->hashtable); 
    }

    me->plStatus = FALSE;   
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
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
    // calc stuff//
    ubyte2 maxtq = MCM_getTorqueMax(mcm);
    float appsTqPercent;
    TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
    float gain = 9.549;
    float decitq = 10.0;
=======
void PL_populateHashTable(HashTable* table)
{
    /*
    voltage is x axis
    rpm is y axis 
    values are in tq nM
    */
    ubyte4 lookupTable[25][25] = {
        {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {229.13, 229.46, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
        {205.17, 205.54, 205.98, 208.54, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56},
        {185.49, 185.85, 186.12, 186.31, 186.61, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56},
        {205.67, 169.23, 169.44, 169.65, 169.97, 170.21, 174.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93},
        {194.62, 155.20, 155.34, 155.52, 155.64, 155.92, 156.14, 156.39, 166.35, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74},
        {184.74, 143.12, 143.18, 143.30, 143.45, 143.63, 144.47, 144.03, 144.26, 144.51, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64},
        {175.71, 187.46, 132.69, 132.78, 132.95, 132.97, 133.12, 133.29, 133.55, 133.72, 133.87, 134.19, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10},
        {167.22, 178.99, 123.46, 123.46, 123.50, 123.58, 123.62, 123.85, 123.94, 124.24, 124.51, 124.62, 124.73, 125.10, 136.66, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73},
        {159.06, 171.05, 115.32, 115.24, 115.24, 115.40, 115.29, 115.48, 115.53, 115.68, 115.86, 116.00, 116.45, 116.55, 116.71, 117.05, 126.26, 128.56, 128.56, 128.56, 128.56, 128.56, 128.56, 128.56, 128.56},
        {150.90, 163.48, 171.91, 107.97, 107.91, 107.91, 107.87, 107.95, 108.04, 108.23, 108.30, 108.66, 108.64, 108.93, 109.13, 109.27, 109.60, 109.84, 115.86, 121.44, 121.44, 121.44, 121.44, 121.44, 121.44},
        {142.25, 156.06, 164.78, 171.67, 101.42, 101.26, 101.26, 101.42, 101.37, 101.52, 101.57, 101.70, 101.84, 102.02, 102.20, 102.40, 102.61, 102.85, 103.09, 103.34, 105.36, 114.95, 114.95, 114.95, 114.95},
        {131.93, 148.52, 157.81, 164.91,  95.38,  95.25,  95.28,  95.28,  95.25,  95.38,  96.61,  95.56,  95.68,  95.82,  95.98,  96.16,  96.33,  96.47,  96.74,  96.97,  97.23,  97.48,  97.62,  107.50, 109.22},
        {115.50, 140.37, 150.79, 158.29, 164.43,  89.92,  89.86,  89.82,  89.82,  89.86,  89.92,  90.00,  90.08,  90.21,  90.33,  90.47,  90.62,  90.82,  91.00,  91.20,  91.43,  91.65,  91.90,  92.20,  92.39},
        {115.50, 130.39, 143.39, 151.64, 158.11, 163.61,  84.93,  84.86,  84.85,  84.85,  84.86,  84.92,  84.99,  85.06,  85.17,  85.29,  85.44,  85.58,  85.76,  85.82,  86.06,  86.34,  86.43,  86.79,  86.95},
        {115.50, 115.50, 134.95, 144.67, 151.75, 157.54,  80.48,  80.31,  80.33,  80.30,  80.30,  80.23,  79.83,  80.34,  80.50,  80.59,  80.58,  80.83,  80.91,  81.13,  81.29,  81.41,  81.60,  81.81,  82.11},
        {115.50, 115.50, 122.30, 136.94, 145.10, 151.42, 156.73,  76.27,  76.06,  76.08,  75.98,  75.93,  75.98,  76.02,  76.08,  76.15,  76.25,  76.35,  76.56,  76.37,  76.75,  76.92,  77.28,  77.34,  77.54},
        {115.50, 115.50, 115.50, 126.68, 137.79, 145.02, 150.81, 155.76,  72.28,  72.12,  72.07,  72.03,  72.03,  72.13,  72.16,  72.22,  72.28,  72.37,  72.46,  72.57,  72.62,  72.76,  72.99,  73.15,  73.24}
    };

    for (ubyte1 row = 0; row < NUM_S; ++row) {
        for(ubyte1 column = 0; column < NUM_V; ++column) {
            float4 noLoadVoltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            sbyte4 rpm   = RPM_MIN + row * RPM_STEP;
            ubyte4 value = lookupTable[row][column];
            insert(table, noLoadVoltage, rpm, value);
        }
    }
}

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->hashtable = HashTable_new();
    PL_populateHashTable(me->hashtable);
    me-> plStatus = FALSE;
   // me->pid = PID_new(1, 0, 0, 0);// fill this in  
    me->voltageMCM  = 0.0;
    me->currentMCM  = 0.0;
    me->watts       = 0.0;
    me->motorRPM    = 0.0;
    me->valueLUT    = 0.0;
    me->offset      = 0.0;
    me->estimatedTQ = 0.0;
    me->setpointTQ  = 0.0;
    return me;
}

// this function needs to be HEAVILY debugged for double linear interpolation 
float PL_getTorque(PowerLimit* me, HashTable* torqueHashtable, float noLoadVoltage, float rpm){    // Find the floor and ceiling values for voltage and rpm
    float voltageFloor   = (float)floorToNearestIncrement(noLoadVoltage, VOLTAGE_STEP);
    float voltageCeiling = (float)ceilToNearestIncrement(noLoadVoltage, VOLTAGE_STEP);
    float rpmFloor       = (float)floorToNearestIncrement(rpm, RPM_STEP);
    float rpmCeiling     = (float)ceilToNearestIncrement(rpm, RPM_STEP);
    // Retrieve torque values from the hash table for the four corners, xy convention
    float vFloorRFloor     = (float)get(torqueHashtable, voltageFloor, rpmFloor);
    float vCeilingRFloor   = (float)get(torqueHashtable, voltageCeiling, rpmFloor);
    float vFloorRCeiling   = (float)get(torqueHashtable, voltageFloor, rpmCeiling);
    float vCeilingRCeiling = (float)get(torqueHashtable, voltageCeiling, rpmCeiling);
    // Error check
>>>>>>> fd174af (Among other things, an attent to rename new functions to follow the SYSTEM_actionItem naming convention loosely in place in the VCU)

    //parameters we need for calculations//
    float watts = (float)(MCM_getPower(mcm)); // divide by 1000 to get watts --> kilowatts
    float driversRequestedtq = appsTqPercent*maxtq; 
    float rpm = (float)MCM_getMotorRPM(mcm);

<<<<<<< HEAD
    
    if(watts > KWH_THRESHOLD)
     {// kwhlimit should be changed to another paramter we make for plthreshold
        me-> plStatus = TRUE;
        // still need to make/ update all the struct parameters aka values for can validation 
        float pidsetpoint = (float)((KWH_LIMIT*gain/rpm)*decitq);
        float pidactual = (float)((watts*gain/rpm)*decitq);
        PID_setpointUpdate(pid,pidsetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
        float pidOffset =  PID_computeOffset(pid, pidactual);
        float PLfinalTQ = pidactual + pidOffset;

        me->pidOffset = pidOffset;
        me->plfinaltq =PLfinalTQ; 
        me->pidsetpoint = pidsetpoint;
        me->pidactual = pidactual;

=======
void PL_calculateTorqueOffset(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid){
//-------------------------JUST CHECKING CAN INCASE WE NEED LUT------------------------------------------------------------------------------

    float mcmVoltage   = (float)MCM_getDCVoltage(mcm);
    float mcmCurrent   = (float)MCM_getDCCurrent(mcm);
    me->motorRPM       = (float)MCM_getMotorRPM(mcm);
    me->watts          = (float)MCM_getPower(mcm);
    ubyte2 commandedTQ = MCM_getCommandedTorque(mcm);
    ubyte2 offsetTQ    = 0;

// -------------------------------------no load pack voltage calc: record voltage -------------------------------------
//
//-------------> need to do this this for LUT
//----------------------------------------------------------------------------------------------------------------------------------------------------
//
    ///ubyte2 kwhtovoltage = (ubyte2)((KWH_LIMIT*1000) / current);
    if(me->watts > PL_INIT) {
        me-> plStatus = TRUE;
        ubyte2 maxtq  = MCM_PL_getTorqueMax(mcm);
        float4 appsTqPercent;
        TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
        me->offset = PID_computeOffset(pid, me->watts);
        offsetTQ   = commandedTQ * ((ubyte2)(me->offset / me->watts * 100));
>>>>>>> fd174af (Among other things, an attent to rename new functions to follow the SYSTEM_actionItem naming convention loosely in place in the VCU)
    }
    else {
        me-> plStatus = FALSE;
    }
<<<<<<< HEAD

    float plfinaltq=  me->plfinaltq;
    MCM_update_PowerLimit_TorqueCommand(mcm,  plfinaltq); // we need to change this on mcm.c / pl.c/.h 
    MCM_update_PowerLimit_State(mcm, me->plStatus); 

    // in mcm.c input the if statement for the tps
}

/* powerpid */
#elif POWERLIMIT_METHOD == 2
/*
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
     ubyte2 maxtq = MCM_getTorqueMax(mcm);
       float appsTqPercent;
       TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
        float gain = 9.549;
        float decitq = 10.0;

    //parameters we need for calculations//
    float watts = (float)(MCM_getPower(mcm)); // divide by 1000 to get watts --> kilowatts
    float driversRequestedtq = appsTqPercent*maxtq; 
    float wheelspeed = (float)MCM_getMotorRPM(mcm);

    
    if(watts > KWH_THRESHOLD)
     {// kwhlimit should be changed to another paramter we make for plthreshold
        me-> plStatus = TRUE;
      // still need to make/ update all the struct parameters aka values for can validation 
       float pidsetpoint = (float)(KWH_LIMIT);
       float pidactual = (float)(watts);

       PID_setpointUpdate(pid,pidsetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
       float pidOffset =  PID_computeOffset(pid, pidactual);
       float PLgoalPower = pidactual+ pidOffset;
       float PLfinalTQ = (float)((PLgoalPower*gain/wheelspeed)*decitq);


       
       me->pidOffset = pidOffset;
       me->plfinaltq =PLfinalTQ; 
       me->pidsetpoint = pidsetpoint;
       me->pidactual = pidactual;
    }
    else {
        me-> plStatus = FALSE;
    }
   float plfinaltq=  me->plfinaltq;
    MCM_update_PowerLimit_TorqueCommand(mcm,  plfinaltq); // we need to change this on mcm.c / pl.c/.h 
    MCM_update_PowerLimit_State(mcm, me->plStatus); 

}

*/

#elif POWERLIMIT_METHOD == 3
/** LUT METHOD */
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
    sbyte4 watts = MCM_getPower(mcm);
    if( watts > KWH_THRESHOLD ){
    // Always set the flag
    me-> plStatus = TRUE;

    // Getting APPS OUTPUT
    ubyte2 maxTQ = MCM_getTorqueMax(mcm);
    float appsPercent;
    TorqueEncoder_getOutputPercent(tps, &appsPercent);
    float appsTorque = appsPercent * maxTQ; 

    //parameters we need for calculations//
    ubyte4 motorRPM = (ubyte4)MCM_getMotorRPM(mcm);
    sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
    sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

    // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model is 0.027 ohms
    ubyte4 noLoadVoltage = (ubyte4)(mcmCurrent * 0.027) + (ubyte4)mcmVoltage;
    float4 pidsetpoint = PowerLimit_getTorque(me, me->hashtable, noLoadVoltage, motorRPM);
    float4 pidactual = (float4)MCM_getCommandedTorque(mcm);

    PID_setpointUpdate(pid, pidsetpoint);
    //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
    float offset =  PID_computeOffset(pid, pidactual);
    float torqueRequest = pidactual + offset;
    // Setting member values for CAN message debugging. Will change to an if / define to easily toggle in the future.
    me->pidOffset = offset;
    me->plfinaltq = torqueRequest; 
    me->pidsetpoint = pidsetpoint;
    me->pidactual = pidactual;
    MCM_update_PowerLimit_TorqueCommand(mcm, me->plfinaltq); 
    }
    else {
        me-> plStatus = FALSE;
    }
    MCM_update_PowerLimit_State(mcm, me->plStatus); 
}


void PowerLimit_populateHashTable(HashTable* table)
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

float PowerLimit_getTorque(PowerLimit* me, HashTable* torqueHashTable, ubyte4 voltage, ubyte4 rpm){    // Find the floor and ceiling values for voltage and rpm
    
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
        me->LUTtq = vFloorRFloor;
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
    me->LUTtq = (gainValueHorizontal * horizontalInterpolation) + (gainValueVertical * verticalInterpolation) + vFloorRFloor;
    */

    me->LUTtq = TQ;
    return me->LUTtq;  // Adjust gain if necessary
}
#endif
=======
    MCM_updateTorqueOffset(mcm, offsetTQ);
    MCM_updatePowerLimitState(mcm, me->plStatus);
}
>>>>>>> fd174af (Among other things, an attent to rename new functions to follow the SYSTEM_actionItem naming convention loosely in place in the VCU)
