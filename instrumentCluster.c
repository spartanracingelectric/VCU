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
* Accessors / Mutators (Set/Get)
****************************************************************************/
void IC_commands_setTorqueDNm(MotorController* mc, ubyte2 newTorque)
{
    mc->torqueMaximumDNm = newTorque;
}
void IC_commands_setRegen_TorqueLimitDNm(MotorController* mc, ubyte2 newTorqueLimit)
{
    if (newTorqueLimit >= 0)
        mc->regen_torqueLimitDNm = newTorqueLimit;
}
void IC_commands_setTorqueAtZeroPedalDNm(MotorController* mc, ubyte2 newTorqueZero)
{
    if(newTorqueZero >= 0)
        mc->regen_torqueAtZeroPedalDNm = newTorqueZero;
}
void IC_commands_setPercentBPSForMaxRegen(MotorController* mc, float4 percentBPS)
{
    if(percentBPS >=0 || percentBPS <= 1)
        mc->regen_percentBPSForMaxRegen = percentBPS;
}
void IC_commands_setPercentAPPSForCoasting(MotorController* mc, float4 percentAPPS)
{
    if(percentAPPS >=0 || percentAPPS <= 1)
        mc->regen_percentAPPSForCoasting = percentAPPS;
}

ubyte2 IC_commands_getTorqueDNm(MotorController* mc)
{
    return mc->torqueMaximumDNm;
}
ubyte2 IC_commands_getRegen_TorqueLimitDNm(MotorController* mc)
{
    return mc->regen_torqueLimitDNm;
}
ubyte2 IC_commands_getTorqueAtZeroPedalDNm(MotorController* mc)
{
    return mc->regen_torqueAtZeroPedalDNm;
}
float4 IC_commands_getPercentBPSForMaxRegen(MotorController* mc)
{
    return mc->regen_percentBPSForMaxRegen;
}
float4 IC_commands_getPercentAPPSForCoasting(MotorController* mc)
{
    return mc->regen_percentAPPSForCoasting;
}