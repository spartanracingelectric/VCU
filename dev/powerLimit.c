
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

    me->hashtable = HashTable_new();
    PL_populateHashTable(me->hashtable); 

    me->plState = FALSE;

    me->power = 0.0; 
    me->rpm = 0.0; 

    me->pidOffset = 0; 
    me->plTorqueCommand = 0; 
    me->pidSetpoint = 0.0; 
    me->pidActual = 0.0;
    me->lutTorque = 0;
     
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
   float plfinaltq=  me->plfinaltq;
    MCM_update_PowerLimit_TorqueLimit(mcm,  plfinaltq); // we need to change this on mcm.c / pl.c/.h 
    MCM_update_PowerLimit_State(mcm, me->PLstatus); 

}


// TODO: write case 3: tqpid+lut
*/

