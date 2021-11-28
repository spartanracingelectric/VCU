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

void IC_parseCanMessage(InstrumentCluster* me, IO_CAN_DATA_FRAME* icCanMessage)
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
void IC_to_MCM_setTorqueDNm(MotorController* mc, ubyte2 newTorque)
{
    MCM_setTorqueDNm(mc, newTorque);
}
void IC_to_MCM_setRegen_TorqueLimitDNm(MotorController* mc, ubyte2 newTorqueLimit)
{
    MCM_setRegen_TorqueLimitDNm(mc, newTorqueLimit);
}
void IC_to_MCM_setRegenTorqueAtZeroPedalDNm(MotorController* mc, ubyte2 newTorqueZero)
{
    MCM_setRegen_TorqueAtZeroPedalDNm(mc, newTorqueZero);
}
void IC_to_MCM_setRegen_PercentBPSForMaxRegen(MotorController* mc, float4 percentBPS)
{
    MCM_setRegen_PercentBPSForMaxRegen(mc, percentBPS);
}
void IC_to_MCM_setRegen_PercentAPPSForCoasting(MotorController* mc, float4 percentAPPS)
{
    MCM_setRegen_PercentAPPSForCoasting(mc, percentAPPS);
}

ubyte2 IC_to_MCM_getTorqueDNm(MotorController* mc)
{
    return MCM_getTorqueDNm(mc);
}
ubyte2 IC_to_MCM_getRegen_TorqueLimitDNm(MotorController* mc)
{
    return MCM_getRegen_TorqueLimitDNm(mc);
}
ubyte2 IC_to_MCM_getRegen_TorqueAtZeroPedalDNm(MotorController* mc)
{
    return MCM_getRegen_TorqueAtZeroPedalDNm(mc);
}
float4 IC_to_MCM_getRegen_PercentBPSForMaxRegen(MotorController* mc)
{
    return MCM_getRegen_PercentBPSForMaxRegen(mc);
}
float4 IC_to_MCM_getRegen_PercentAPPSForCoasting(MotorController* mc)
{
    return MCM_getRegen_PercentAPPSForCoasting(mc);
}