//"Include guard" - prevents this file from being #included more than once
#ifndef _INSTRUMENTCLUSTER_H
#define _INSTRUMENTCLUSTER_H

#include <stdlib.h>
#include "IO_CAN.h"
#include "IO_Driver.h"
#include "motorController.h"
#include "serial.h"

//#include "canManager.h"


typedef struct _InstrumentCluster InstrumentCluster;

InstrumentCluster* InstrumentCluster_new(SerialManager* sm, ubyte2 canMessageBaseID);

//----------------------------------------------------------------------------
// Command Functions
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Update Functions (CAN Inputs)
//----------------------------------------------------------------------------

void IC_parseCanMessage(InstrumentCluster* me, MotorController* mcm, IO_CAN_DATA_FRAME* icCanMessage);
//----------------------------------------------------------------------------
// Status Functions (CAN Inputs)
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//Inter-object functions
//----------------------------------------------------------------------------

ubyte1 IC_PL_getTorqueMapMode(InstrumentCluster *me);
ubyte1 IC_getLaunchControlSensitivity(InstrumentCluster *me);

/*
void IC_to_MCM_setMaxTorqueDNm(MotorController* mcm, ubyte2 newTorque);
void IC_to_MCM_setRegen_TorqueLimitDNm(MotorController* mcm, ubyte2 torqueLimit);
void IC_to_MCM_setRegen_TorqueAtZeroPedalDNm(MotorController* mcm, ubyte2 torqueZero);
void IC_to_MCM_setRegen_PercentBPSForMaxRegen(MotorController* mcm, float4 percentBPS);
void IC_to_MCM_setRegen_PercentAPPSForCoasting(MotorController* mcm, float4 percentAPPS);

ubyte2 IC_to_MCM_getMaxTorqueDNm(MotorController* mcm);
ubyte2 IC_to_MCM_getRegen_TorqueLimitDNm(MotorController* mcm);
ubyte2 IC_to_MCM_getRegen_TorqueAtZeroPedalDNm(MotorController* mcm);
float4 IC_to_MCM_getRegen_PercentBPSForMaxRegen(MotorController* mcm);
float4 IC_to_MCM_getRegen_PercentAPPSForCoasting(MotorController* mcm);
*/

#endif // _INSTRUMENTCLUSTER_H
