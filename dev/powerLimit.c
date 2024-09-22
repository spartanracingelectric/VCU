
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"

#ifndef CALCS
#define CALCS

#define VOLTAGE_MIN      (float4) 283.200
#define VOLTAGE_MAX      (float4) 403.200
#define RPM_MIN          (sbyte4) 100
#define RPM_MAX          (sbyte4) 6000
#define NUM_V            (ubyte1) 25
#define NUM_S            (ubyte1) 25
#define VOLTAGE_STEP     (float4) 5        //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1); // 5
#define RPM_STEP         (sbyte4) 245.8333 //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); // 245.8333
#define PI               (float4) 3.14159
#define KWH_LIMIT        (float4) 55000.0  // watts
#define PL_INIT          (float4) 55000.0  // 5kwh buffer to init PL before PL limit is hit
#define UNIT_CONVERSTION (float4) 95.49    // 9.549 *10.0 to convert to deci-newtonmeters

#endif

void populatePLHashTable(HashTable* table)
{
    /*
    voltage is x axis
    rpm is y axis 
    values are in tq nM
    */
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------    
    ubyte4 lookupTable[25][25] = {
    const float4 POWER_LIM_LUT[25][25] = {
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{212.61, 221.28, 225.74, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{185.55, 193.57, 202.84, 212.51, 218.27, 223.91, 230.14, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{161.10, 171.06, 178.66, 185.64, 193.73, 202.82, 208.51, 218.14, 221.57, 228.04, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{140.43, 149.85, 157.31, 165.28, 172.95, 179.33, 188.62, 193.76, 202.80, 209.71, 221.51, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{121.83, 130.78, 137.41, 146.11, 153.49, 160.86, 168.33, 173.86, 181.94, 187.59, 202.88, 218.16, 224.75, 225.33, 225.61, 225.93, 226.02, 226.13, 226.13, 226.23, 226.23, 226.34, 226.34, 226.34, 226.34},
	{104.24, 112.98, 119.96, 127.63, 135.34, 141.80, 149.94, 156.70, 163.78, 170.26, 183.76, 199.21, 206.91, 210.19, 210.41, 210.63, 210.81, 211.02, 211.11, 211.17, 211.20, 211.15, 211.15, 211.15, 211.15},
	{87.79, 95.47, 103.90, 111.85, 119.34, 126.36, 133.36, 140.26, 145.60, 153.37, 166.90, 180.32, 191.86, 196.66, 197.07, 197.38, 197.48, 197.62, 197.81, 197.81, 197.88, 197.91, 198.01, 197.96, 197.96},
	{70.68, 79.67, 88.03, 95.39, 103.30, 110.80, 117.59, 124.60, 131.59, 137.59, 151.21, 164.24, 176.26, 183.83, 185.32, 185.55, 185.73, 185.87, 186.19, 186.09, 186.17, 186.22, 186.27, 186.29, 186.31},
	{52.58, 62.90, 72.01, 80.51, 88.24, 96.03, 103.10, 109.92, 116.50, 122.13, 136.23, 149.90, 161.28, 171.45, 174.41, 175.01, 175.27, 175.53, 175.46, 175.87, 175.64, 175.79, 175.85, 175.70, 175.92},
	{30.59, 44.40, 55.32, 64.53, 73.43, 81.09, 88.77, 95.88, 102.53, 109.18, 121.86, 135.62, 147.52, 157.34, 163.87, 165.70, 165.79, 166.01, 166.05, 166.16, 166.26, 166.41, 166.39, 166.45, 166.48},
	{0.11, 18.65, 35.70, 47.73, 57.67, 66.43, 74.58, 81.62, 89.12, 95.85, 109.38, 122.02, 134.25, 143.99, 153.45, 156.15, 157.36, 157.52, 157.63, 157.74, 157.84, 157.92, 157.86, 158.03, 158.07},
	{0.11, 0.11, 0.11, 25.82, 39.83, 50.40, 59.66, 67.89, 75.57, 82.42, 96.65, 110.18, 121.91, 131.84, 139.95, 149.35, 149.60, 149.86, 149.98, 150.09, 150.05, 150.13, 150.31, 150.38, 150.43},
	{0.11, 0.11, 0.11, 0.11, 12.57, 31.21, 43.23, 52.99, 61.57, 69.21, 84.04, 98.27, 109.87, 119.89, 129.15, 137.39, 141.55, 142.90, 143.01, 143.11, 143.21, 143.29, 143.36, 143.35, 143.48},
	{0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 21.16, 35.57, 46.22, 55.13, 71.45, 86.31, 98.28, 108.44, 117.36, 125.85, 133.56, 135.59, 137.13, 136.71, 136.68, 136.95, 137.07, 137.09, 137.07},
	{0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 4.27, 26.92, 39.20, 58.48, 74.56, 86.97, 97.23, 106.49, 115.06, 122.08, 129.16, 130.37, 130.87, 131.04, 131.04, 131.12, 131.27, 131.33},
	{0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 16.20, 43.99, 62.09, 75.58, 86.50, 96.13, 104.29, 112.74, 119.81, 124.55, 125.38, 125.68, 125.68, 125.75, 125.88, 125.94}};
    for (ubyte1 row = 0; row < NUM_S; ++row) {
        for(ubyte1 column = 0; column < NUM_V; ++column) {
            ubyte2 noLoadVoltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            ubyte2 rpm   = RPM_MIN + row * RPM_STEP;
            ubyte4 value = lookupTable[row][column];
            insert(table, noLoadVoltage, rpm, value);
        }
    }
}

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->hashtable = HashTable_new();
    populatePLHashTable(me->hashtable); 

    me-> PLstatus = FALSE;   
    me->power = 0.0; 
    me->wheelspeed = 0.0; 

    me->piderror = 0.0; 
    me->plfinaltq = 0.0; 
    me->pidsetpoint = 0.0; 
    me->pidactual = 0.0; 
    me->LUTtq=0.0;
     
    return me;
}
//@shaun do this
float noloadvoltagecalc(){
    return -1; 
}
float getTorque(PowerLimit* me, HashTable* torque_hashtable, float voltage, float rpm){    // Find the floor and ceiling values for voltage and rpm
    /*
    float voltageFloor = (float)floorToNearestIncrement(voltage, VOLTAGE_STEP);
    float voltageCeiling = (float)ceilToNearestIncrement(voltage, VOLTAGE_STEP);
    float rpmFloor = (float)floorToNearestIncrement(rpm, RPM_STEP);
    float rpmCeiling = (float)ceilToNearestIncrement(rpm, RPM_STEP);
    // Retrieve torque values from the hash table for the four corners
    float vFloorRFloor = (float)get(torque_hashtable, voltageFloor, rpmFloor);
    float vCeilingRFloor =(float) get(torque_hashtable, voltageCeiling, rpmFloor);
    float vFloorRCeiling = (float)get(torque_hashtable, voltageFloor, rpmCeiling);
    float vCeilingRCeiling = (float)get(torque_hashtable, voltageCeiling, rpmCeiling);
    // Error check

    // Calculate interpolation values
    float horizontalInterpolation = (float)(((vCeilingRFloor - vFloorRFloor) / VOLTAGE_STEP) + ((vCeilingRCeiling - vFloorRCeiling) / VOLTAGE_STEP)) / 2.0;
    float verticalInterpolation   = (float)(((vFloorRCeiling - vFloorRFloor) / RPM_STEP) + ((vCeilingRCeiling - vCeilingRFloor) / RPM_STEP)) / 2.0;
    // Calculate gains
    float gainValueHorizontal = (float)fmod(voltage, VOLTAGE_STEP);
    float gainValueVertical   = (float)fmod(rpm, RPM_STEP);
    // Combine interpolated values
    float calibratedTorque  = (gainValueHorizontal * horizontalInterpolation) + (gainValueVertical * verticalInterpolation) + vFloorRFloor;
    
    me->LUTtq = calibratedTorque; 
    */

     float calibratedTorque = (float)get(torque_hashtable,voltage,rpm);
    return calibratedTorque;  // Adjust gain if necessary
}

/*
// THIS IS THE SUPER OLD CODE THATS REALLY MESSY BE AWARE 
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
  
  

//-------------------------JUST CHECKING CAN INCASE WE NEED LUT------------------------------------------------------------------------------

    float mcmVoltage = (float)MCM_getDCVoltage(mcm);// CHECK THE UNITS FOR THIS
    float mcmCurrent = (float)MCM_getDCCurrent(mcm);
    me->motorRPM     = (float)MCM_getMotorRPM(mcm);
    me->watts        = (float)MCM_getPower(mcm);

//----------------------------------------TESTING-------------------------------------------------
//
//    float appsTqPercent;
//    TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
//    float watts = (float)(appsTqPercent * 100000.0); 
//    float kilowatts = (float)(watts/10.0);
//    float motorRPM = (float)(watts*0.045);
//
//--------------------------------------------------------------------------------------
//
// me->currentMCM = current; 
// me->voltageMCM = voltage; 
//------------------------------------------------------------------------------------------------------------------------------------------------------
//
// -------------------------------------no load pack voltage calc: record voltage -------------------------------------
//
//-------------> need to do this this for LUT
//----------------------------------------------------------------------------------------------------------------------------------------------------
//
    ///ubyte2 kwhtovoltage = (ubyte2)((KWH_LIMIT*1000) / current);
    if(me->watts > PL_INIT) {
        me-> plStatus = TRUE;
        ubyte2 maxtq = MCM_getTorqueMax(mcm);
        float4 appsTqPercent;
        TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
           
        float tqsetpoint  = (float)((PL_INIT/me->motorRPM)*UNIT_CONVERSTION);
        float idealTQ = (float)((me->watts/me->motorRPM)*UNIT_CONVERSTION);
        // float tqsetpoint  = (float)((KWH_LIMIT*gain/motorRPM)*decitq);
        // float predictedtq = (float)((watts*gain/motorRPM)*decitq);

        me->estimatedTQ = idealTQ;
        me->setpointTQ  = tqsetpoint;
        PID_setpointUpdate(pid,tqsetpoint);
        me->error =  PID_compute(pid, idealTQ);
        // float appsTqPercent;
        // TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
        // the torqueMaximumDNm is 2000, scale it accordingly 
        // ubyte2 tq = MCM_getMaxTorqueDNm(mcm);
        // me->PLoffsetpid= (tq * appsTqPercent) + me->error;
    }
    else {
        me-> plStatus = FALSE;
    }
    MCM_update_PowerLimit_TorqueLimit(mcm, me->error);
    MCM_update_PowerLimit_State(mcm, me->PLstatus); 
}
  */


// this is case1: tqpid + equation
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
       // calc stuff//
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
        me-> PLstatus = TRUE;
      // still need to make/ update all the struct parameters aka values for can validation 
       float pidsetpoint = (float)((KWH_LIMIT*gain/wheelspeed)*decitq);
       float pidactual = (float)((watts*gain/wheelspeed)*decitq);
       PID_setpointUpdate(pid,pidsetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
       float piderror =  PID_compute(pid, pidactual);
       float PLfinalTQ = pidactual+ piderror;
       

       me->piderror = piderror;
       me->plfinaltq =PLfinalTQ; 
       me->pidsetpoint = pidsetpoint;
       me->pidactual = pidactual;

    }
    else {
        me-> PLstatus = FALSE;
    }

    float plfinaltq=  me->plfinaltq;
    MCM_update_PowerLimit_TorqueLimit(mcm,  plfinaltq); // we need to change this on mcm.c / pl.c/.h 
    MCM_update_PowerLimit_State(mcm, me->PLstatus); 

    // in mcm.c input the if statement for the tps
}

/* this is case2: powerpid + equation
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
        me-> PLstatus = TRUE;
      // still need to make/ update all the struct parameters aka values for can validation 
       float pidsetpoint = (float)(KWH_LIMIT);
       float pidactual = (float)(watts);

       PID_setpointUpdate(pid,pidsetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
       float piderror =  PID_compute(pid, pidactual);
       float PLgoalPower = pidactual+ piderror;
       float PLfinalTQ = (float)((PLgoalPower*gain/wheelspeed)*decitq);


       
       me->piderror = piderror;
       me->plfinaltq =PLfinalTQ; 
       me->pidsetpoint = pidsetpoint;
       me->pidactual = pidactual;
    }
    else {
        me-> PLstatus = FALSE;
    }
   float plfinaltq=  me->plfinaltq;
    MCM_update_PowerLimit_TorqueLimit(mcm,  plfinaltq); // we need to change this on mcm.c / pl.c/.h 
    MCM_update_PowerLimit_State(mcm, me->PLstatus); 

}

*/
/*
// TODO: write case 3: tqpid+lut
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
       // calc stuff//
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
        me-> PLstatus = TRUE;
      // still need to make/ update all the struct parameters aka values for can validation 
      float noloadvoltage = 300;
       float pidsetpoint = (float)(getTorque(me, me->hashtable,noloadvoltage,wheelspeed));
       float pidactual = (float)((watts*gain/wheelspeed)*decitq);
       PID_setpointUpdate(pid,pidsetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
       float piderror =  PID_compute(pid, pidactual);
       float PLfinalTQ = pidactual+ piderror;
       

       me->piderror = piderror;
       me->plfinaltq =PLfinalTQ; 
       me->pidsetpoint = pidsetpoint;
       me->pidactual = pidactual;

    }
    else {
        me-> PLstatus = FALSE;
    }

    float plfinaltq=  me->plfinaltq;
    MCM_update_PowerLimit_TorqueLimit(mcm, plfinaltq); 
    MCM_update_PowerLimit_State(mcm, me->PLstatus); 


    
}
*/


