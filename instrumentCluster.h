//"Include guard" - prevents this file from being #included more than once
#ifndef _INSTRUMENTCLUSTER_H
#define _INSTRUMENTCLUSTER_H

#include <stdlib.h>
#include "IO_CAN.h"
#include "IO_Driver.h"
#include "motorController.h"    // need definition of MotorController's struct, not just decaration
#include "serial.h"

//#include "canManager.h"


typedef struct _InstrumentCluster InstrumentCluster;

InstrumentCluster* InstrumentCluster_new(SerialManager* sm, ubyte2 canMessageBaseID);

//----------------------------------------------------------------------------
// Command Functions
//----------------------------------------------------------------------------
void IC_commands_setTorqueDNm(MotorController* mc, ubyte2 torque);
void IC_commands_setRegen_TorqueLimitDNm(MotorController* mc, ubyte2 torqueLimit);
void IC_commands_setRegen_TorqueAtZeroPedalDNm(MotorController* mc, ubyte2 torqueZero);
void IC_commands_setPercentBPSForMaxRegen(MotorController* mc, float4 percentBPS);
void IC_commands_setPercentAPPSForCoasting(MotorController* mc, float4 percentAPPS);

ubyte2 IC_commands_getTorqueDNm(MotorController* mc);
ubyte2 IC_commands_getRegen_TorqueLimitDNm(MotorController* mc);
ubyte2 IC_commands_getRegen_TorqueAtZeroPedalDNm(MotorController* mc);
float4 IC_commands_getPercentBPSForMaxRegen(MotorController* mc);
float4 IC_commands_getPercentAPPSForCoasting(MotorController* mc);
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
