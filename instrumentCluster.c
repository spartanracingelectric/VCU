#include "instrumentCluster.h"

#include "canManager.h"


struct _InstrumentCluster
{
    SerialManager* serialMan;
    
    ubyte2 canMessageBaseId;  //Starting message ID for messages that will come in from this controller

    ubyte1 torqueMapMode;

    //0 = off. Default OFF
    ubyte1 launchControlSensitivity;
    
    ubyte2 torqueMaximumDNm;             //Max torque that can be commanded in deciNewton*meters ("100" = 10.0 Nm)
    ubyte2 regen_torqueLimitDNm;         //Tuneable value.  Regen torque (in Nm) at full regen.  Positive value.
    ubyte2 regen_torqueAtZeroPedalDNm;   //Tuneable value.  Amount of regen torque (in Nm) to apply when both pedals at 0% travel. Positive value.
    float4 regen_percentBPSForMaxRegen;  //Tuneable value.  Amount of brake pedal required for full regen. Value between zero and one.
    float4 regen_percentAPPSForCoasting; //Tuneable value.  Amount of accel pedal required to exit regen.  Value between zero and one.
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
    }
}

/*****************************************************************************
* Accessors / Mutators (Set/Get)
****************************************************************************/
ubyte1 IC_getTorqueMapMode(InstrumentCluster *me)
{
    return me->torqueMapMode;
}

ubyte1 IC_getLaunchControlSensitivity(InstrumentCluster *me)
{
    return me->launchControlSensitivity;
}

// In deci-newton-meters e.g 100 DNm = 10.0 Nm
void IC_commands_setTorqueDNm(InstrumentCluster* me, ubyte2 newTorque) 
{
    me->torqueMaximumDNm = newTorque;
}
void IC_commands_setRegen_TorqueLimitDNm(InstrumentCluster* me, ubyte2 newTorqueLimit) 
{
    me->regen_torqueLimitDNm = newTorqueLimit;
}
void IC_commands_setRegen_TorqueAtZeroPedalDNm(InstrumentCluster* me, ubyte2 newTorqueZero) 
{
    me->regen_torqueAtZeroPedalDNm = newTorqueZero;
}
void IC_commands_setPercentBPSForMaxRegen(InstrumentCluster* me, float4 newPercentBPS) 
{
    me->regen_percentBPSForMaxRegen = newPercentBPS;
}
void IC_commands_setPercentAPPSForCoasting(InstrumentCluster* me, float4 newPercentAPPS) 
{
    me->regen_percentAPPSForCoasting = newPercentAPPS;
}
ubyte2 IC_commands_getTorqueDNm(InstrumentCluster* me) 
{
    return me->torqueMaximumDNm;
}
ubyte2 IC_commands_getRegen_TorqueLimitDNm(InstrumentCluster* me) 
{
    return me->regen_torqueLimitDNm;
}
ubyte2 IC_commands_getRegen_TorqueAtZeroPedalDNm(InstrumentCluster* me) 
{
    return me->regen_torqueAtZeroPedalDNm;
}
float4 IC_commands_getPercentBPSForMaxRegen(InstrumentCluster* me) 
{
    return me->regen_percentBPSForMaxRegen;
}
float4 IC_commands_getPercentAPPSForCoasting(InstrumentCluster* me) 
{
    return me->regen_percentAPPSForCoasting;
}