#include "IO_Driver.h"

#include "instrumentCluster.h"
#include "motorController.h"    // need definition of MotorController's struct, not just decaration
#include "canManager.h"


struct _InstrumentCluster
{
    SerialManager* serialMan;
    
    ubyte2 canMessageBaseId;  //Starting message ID for messages that will come in from this controller

    ubyte1 torqueMapMode;

    //0 = off. Default OFF
    ubyte1 launchControlSensitivity;
    
};

InstrumentCluster* InstrumentCluster_new(SerialManager* sm, ubyte2 canMessageBaseID)
{
    InstrumentCluster* me = (InstrumentCluster*)malloc(sizeof(struct _InstrumentCluster));

    me->serialMan = sm;

    me->canMessageBaseId = canMessageBaseID;

    me->torqueMapMode=0;

    me->launchControlSensitivity=0;
    
    return me;
}

void IC_parseCanMessage(InstrumentCluster* me, MotorController* mcm, IO_CAN_DATA_FRAME* icCanMessage)
{
    switch (icCanMessage->id)
    {
        case 0x702:
            me->torqueMapMode = icCanMessage->data[0];
            break;
        case 0x703:
            me->launchControlSensitivity = icCanMessage->data[0];
            break;
        // TODO: include data frames for torque mutator's values
    }
}

ubyte1 IC_getTorqueMapMode(InstrumentCluster *me)
{
    return me->torqueMapMode;
}

ubyte1 IC_getLaunchControlSensitivity(InstrumentCluster *me)
{
    return me->launchControlSensitivity;
}

/*****************************************************************************
* Motor Controller module Accessors / Mutators (Set/Get)
****************************************************************************/
void IC_to_MCM_setMaxTorqueDNm(MotorController* mcm, ubyte2 newTorque)
{
    MCM_setMaxTorqueDNm(mcm, newTorque);
}
void IC_to_MCM_setRegen_TorqueLimitDNm(MotorController* mcm, ubyte2 newTorqueLimit)
{
    MCM_setRegen_TorqueLimitDNm(mcm, newTorqueLimit);
}
void IC_to_MCM_setRegen_TorqueAtZeroPedalDNm(MotorController* mcm, ubyte2 newTorqueZero)
{
    MCM_setRegen_TorqueAtZeroPedalDNm(mcm, newTorqueZero);
}
void IC_to_MCM_setRegen_PercentBPSForMaxRegen(MotorController* mcm, float4 percentBPS)
{
    MCM_setRegen_PercentBPSForMaxRegen(mcm, percentBPS);
}
void IC_to_MCM_setRegen_PercentAPPSForCoasting(MotorController* mcm, float4 percentAPPS)
{
    MCM_setRegen_PercentAPPSForCoasting(mcm, percentAPPS);
}

ubyte2 IC_to_MCM_getMaxTorqueDNm(MotorController* mcm)
{
    return MCM_getMaxTorqueDNm(mcm);
}
ubyte2 IC_to_MCM_getRegen_TorqueLimitDNm(MotorController* mcm)
{
    return MCM_getRegen_TorqueLimitDNm(mcm);
}
ubyte2 IC_to_MCM_getRegen_TorqueAtZeroPedalDNm(MotorController* mcm)
{
    return MCM_getRegen_TorqueAtZeroPedalDNm(mcm);
}
float4 IC_to_MCM_getRegen_PercentBPSForMaxRegen(MotorController* mcm)
{
    return MCM_getRegen_PercentBPSForMaxRegen(mcm);
}
float4 IC_to_MCM_getRegen_PercentAPPSForCoasting(MotorController* mcm)
{
    return MCM_getRegen_PercentAPPSForCoasting(mcm);
}