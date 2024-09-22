
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
    ubyte4 lookupTable[25][25] = {
 {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
    {219.54, 219.57, 219.73, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55, 219.55},
    {199.31, 199.57, 199.70, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74, 199.74},
    {182.32, 182.62, 182.83, 182.93, 183.00, 183.03, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08, 183.08},
    {168.04, 168.22, 168.56, 168.59, 168.79, 168.96, 168.87, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93, 168.93},
    {155.62, 155.92, 156.14, 156.32, 156.44, 156.55, 156.61, 156.66, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69, 156.69},
    {144.90, 145.20, 145.55, 145.63, 145.75, 145.88, 145.97, 146.02, 146.08, 148.16, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12, 146.12},
    {134.79, 135.79, 136.10, 136.29, 136.43, 136.57, 136.66, 136.85, 136.49, 136.77, 136.90, 136.92, 136.86, 136.86, 136.86, 136.86, 136.86, 136.86, 136.86, 136.86, 136.86, 136.86, 136.86, 136.86, 136.86},
    {124.71, 127.69, 127.76, 127.82, 128.08, 128.40, 128.32, 128.41, 128.56, 128.55, 128.59, 128.59, 128.75, 128.69, 128.78, 129.61, 127.88, 127.88, 127.88, 127.88, 127.88, 127.88, 127.88, 127.88, 127.88},
    {110.11, 119.40, 120.50, 120.58, 119.91, 120.99, 120.98, 121.06, 121.22, 121.30, 121.27, 121.41, 121.13, 121.52, 121.50, 121.44, 121.54, 121.23, 121.23, 121.23, 121.23, 121.23, 121.23, 121.23, 121.23},
    {96.18, 108.77, 113.47, 114.34, 114.25, 114.29, 114.40, 114.50, 114.66, 114.60, 113.89, 114.85, 114.91, 114.85, 114.89, 115.05, 115.03, 115.05, 115.05, 115.05, 115.05, 115.05, 115.05, 115.05, 115.05},
    {82.13, 95.44, 106.47, 108.01, 108.34, 108.38, 108.57, 108.58, 108.74, 108.73, 108.81, 108.87, 108.92, 108.98, 109.01, 109.13, 109.16, 109.11, 109.20, 109.22, 109.22, 109.22, 109.11, 109.11, 109.11},
    {68.37, 82.11, 93.89, 102.29, 102.98, 103.09, 103.18, 103.26, 103.37, 103.44, 103.51, 103.56, 103.62, 103.66, 103.70, 103.76, 104.15, 103.81, 103.84, 103.87, 103.88, 103.84, 103.92, 103.92, 103.92},
    {53.42, 69.00, 81.10, 91.78, 97.55, 98.22, 98.33, 98.42, 98.57, 98.57, 98.66, 98.64, 98.77, 98.83, 98.98, 98.91, 98.95, 98.98, 99.02, 99.10, 99.06, 99.01, 99.10, 99.22, 99.12},
    {36.16, 54.90, 68.48, 79.69, 89.44, 93.52, 93.79, 93.99, 94.06, 94.14, 93.06, 94.29, 94.35, 94.39, 94.64, 94.48, 94.51, 94.56, 94.59, 94.62, 94.58, 94.61, 94.69, 94.70, 94.73},
    {6.53, 38.77, 54.98, 67.36, 77.86, 87.05, 89.30, 89.95, 90.02, 89.98, 90.15, 90.22, 90.28, 90.33, 90.39, 90.43, 90.46, 90.50, 90.53, 90.58, 90.61, 90.62, 90.65, 90.68, 90.69},
    {0.11, 15.10, 39.59, 54.25, 65.79, 75.63, 84.12, 85.94, 86.28, 86.37, 86.42, 86.48, 86.53, 86.39, 86.58, 86.62, 86.73, 86.83, 86.81, 86.84, 86.81, 86.79, 86.93, 86.95, 86.99}};

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


