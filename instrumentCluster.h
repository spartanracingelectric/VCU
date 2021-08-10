//"Include guard" - prevents this file from being #included more than once
#ifndef _INSTRUMENTCLUSTER_H
#define _INSTRUMENTCLUSTER_H

#include "IO_CAN.h"
#include "IO_Driver.h"

#include "canManager.h"


typedef struct _InstrumentCluster InstrumentCluster;

InstrumentCluster* InstrumentCluster_new(SerialManager* sm, ubyte2 canMessageBaseID);

//----------------------------------------------------------------------------
// Command Functions
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Update Functions (CAN Inputs)
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Status Functions (CAN Inputs)
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//Inter-object functions
//----------------------------------------------------------------------------

#endif // _INSTRUMENTCLUSTER_H
