//"Include guard" - prevents this file from being #included more than once
#ifndef _INSTRUMENTCLUSTER_H
#define _INSTRUMENTCLUSTER_H

#include "IO_CAN.h"
#include "IO_Driver.h"
#include "motorController.h"

typedef struct _InstrumentCluster
{
    SerialManager* serialMan;
    
    ubyte2 canMessageBaseId;  //Starting message ID for messages that will come in from this controller

    ubyte1 torqueMapMode;

    //0 = off. Default OFF
    ubyte1 launchControlSensitivity;
    
} InstrumentCluster;

InstrumentCluster* InstrumentCluster_new(ubyte2 canMessageBaseID);

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

ubyte1 IC_getTorqueMapMode(InstrumentCluster *me);
ubyte1 IC_getLaunchControlSensitivity(InstrumentCluster *me);

#endif // _INSTRUMENTCLUSTER_H
