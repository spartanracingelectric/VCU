
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
#define KWH_LIMIT        (float4) 50000.0  // watts
#define PL_INIT          (float4) 45000.0  // 5kwh buffer to init PL before PL limit is hit
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
    populatePLHashTable(me->hashtable);
    me-> plStatus = FALSE;
   // me->pid = PID_new(1, 0, 0, 0);// fill this in  
    me->voltageMCM  = 0.0;
    me->currentMCM  = 0.0;
    me->watts       = 0.0;
    me->motorRPM    = 0.0;
    me->valueLUT    = 0.0;
    me->error       = 0.0;
    me->estimatedTQ = 0.0;
    me->setpointTQ  = 0.0;
    return me;
}

// this function needs to be HEAVILY debugged for double linear interpolation 
float getTorque(PowerLimit* me, HashTable* torqueHashtable, float noLoadVoltage, float rpm){    // Find the floor and ceiling values for voltage and rpm
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

    // Calculate interpolation values
    float horizontalInterpolation = (float)(((vCeilingRFloor - vFloorRFloor) / VOLTAGE_STEP) + ((vCeilingRCeiling - vFloorRCeiling) / VOLTAGE_STEP)) / 2.0;
    float verticalInterpolation   = (float)(((vFloorRCeiling - vFloorRFloor) / RPM_STEP) + ((vCeilingRCeiling - vCeilingRFloor) / RPM_STEP)) / 2.0;
    // Calculate gains
    float gainValueHorizontal = (float)fmod(noLoadVoltage, VOLTAGE_STEP);
    float gainValueVertical   = (float)fmod(rpm, RPM_STEP);
    // Combine interpolated values
    float calibratedTorque  = 123;
    me->valueLUT = calibratedTorque;
    // return (gainValueHorizontal * horizontalInterpolation) + (gainValueVertical * verticalInterpolation) + lowerFloor;
    return calibratedTorque;  // Adjust gain if necessary
}

void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid){
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
    MCM_update_PowerLimit_State(mcm, me->plStatus); 
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


// TODO: write case 3: tqpid+lut and case 4: powerpid+lut in the same syntax and comment it out
*/

