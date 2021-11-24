//"Include guard" - prevents this file from being #included more than once
#ifndef _INSTRUMENTCLUSTER_H
#define _INSTRUMENTCLUSTER_H

#include <stdlib.h>
#include "IO_CAN.h"
#include "IO_Driver.h"
#include "serial.h"

//#include "canManager.h"


typedef struct _InstrumentCluster InstrumentCluster;

InstrumentCluster* InstrumentCluster_new(SerialManager* sm, ubyte2 canMessageBaseID);

//----------------------------------------------------------------------------
// Command Functions
//----------------------------------------------------------------------------
void IC_commands_setTorqueDNm(InstrumentCluster* me, ubyte2 torque);
void IC_commands_setRegen_TorqueLimitDNm(InstrumentCluster* me, ubyte2 torqueLimit);
void IC_commands_setRegen_TorqueAtZeroPedalDNm(InstrumentCluster* me, ubyte2 torqueZero);
void IC_commands_setPercentBPSForMaxRegen(InstrumentCluster* me, float4 percentBPS);
void IC_commands_setPercentAPPSForCoasting(InstrumentCluster* me, float4 percentAPPS);

ubyte2 IC_commands_getTorqueDNm(InstrumentCluster* me);
ubyte2 IC_commands_getRegen_TorqueLimitDNm(InstrumentCluster* me);
ubyte2 IC_commands_getRegen_TorqueAtZeroPedalDNm(InstrumentCluster* me);
float4 IC_commands_getPercentBPSForMaxRegen(InstrumentCluster* me);
float4 IC_commands_getPercentAPPSForCoasting(InstrumentCluster* me);

//----------------------------------------------------------------------------
// Update Functions (CAN Inputs)
//----------------------------------------------------------------------------

void IC_parseCanMessage(InstrumentCluster* me, IO_CAN_DATA_FRAME* icCanMessage);

//----------------------------------------------------------------------------
// Status Functions (CAN Inputs)
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//Inter-object functions
//----------------------------------------------------------------------------

ubyte1 IC_getTorqueMapMode(InstrumentCluster *me);
ubyte1 IC_getLaunchControlSensitivity(InstrumentCluster *me);

#endif // _INSTRUMENTCLUSTER_H
