
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"

void populatePLHashTable(HashTable* table){

}
PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->hashtable = HashTable_new();
    populatePLHashTable(me->hashtable); 
    me-> PLstatus = FALSE;
    me->pid = PID_new(1,0,0,0);// fill this in  
     me-> powerLimittq = 0.0; 
     me-> PLstatus = FALSE; 
     me-> error = 0.0; 
    return me;
}

void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws){
  
  sbyte4 wheelspeed = MCM_getMotorRPM(mcm);
  sbyte4 kilowatts =  BMS_getPower_W(bms)/1000; // divide by 1000 to get watts --> kilowatts
  //sbyte4 voltage = BMS_getPower_W(bms)/1000;
    if(kilowatts> 78)
    {
        me-> PLstatus = TRUE;
        sbyte2 estimatedtq = (sbyte2) get(me->hashtable,kilowatts,wheelspeed );
        sbyte2 tqsetpoint = (sbyte2) get(me->hashtable,78,wheelspeed );
        
    PID_setpointUpdate(me->pid,tqsetpoint);
    PID_dtUpdate(me->pid, 0.01);
    sbyte2 piderror = PID_compute(me->pid, estimatedtq); 
    me->error = piderror; 
    float4 appsTqPercent;
    TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
// the torqueMaximumDNm is 2000, scale it accordingly 
  ubyte2 tq = MCM_getMaxTorqueDNm(mcm);
    me->powerLimittq= (tq * appsTqPercent) + me->error;
    }
    else
    {
        me-> PLstatus = FALSE;
    }
 MCM_update_PowerLimit_TorqueLimit(mcm, me->powerLimittq);
 MCM_update_PowerLimit_State(mcm, me->PLstatus); 
}