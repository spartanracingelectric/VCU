#include "IO_Driver.h"

#include "canManager.h"
#include "instrumentCluster.h"
#include "motorController.h" // need definition of MotorController's struct, not just decaration

struct _InstrumentCluster
{
    SerialManager *serialMan;

    ubyte2 canMessageBaseId; // Starting message ID for messages that will come in from this controller

    ubyte1 torqueMapMode;

    // 0 = off. Default OFF
    ubyte1 launchControlSensitivity;
};

InstrumentCluster *InstrumentCluster_new(SerialManager *sm, ubyte2 canMessageBaseID)
{
    InstrumentCluster *me = (InstrumentCluster *) malloc(sizeof(struct _InstrumentCluster));

    me->serialMan = sm;

    me->canMessageBaseId = canMessageBaseID;

    me->torqueMapMode = 0;

    me->launchControlSensitivity = 0;

    return me;
}

void IC_parseCanMessage(InstrumentCluster *me, MotorController *mcm, IO_CAN_DATA_FRAME *icCanMessage)
{
    switch (icCanMessage->id)
    {
    //////////////////////////////////////////////////////////
    //              NON-REGEN TORQUE SETTINGS               //
    // Byte 0-1 LE ubyte2: Set Max Overall Torque Limit     //
    // Byte 2 ubyte1: Set Torque Map Mode                   //
    // Byte 3 ubyte1: Launch control sensitivity (unused)   //
    //////////////////////////////////////////////////////////
    case 0x702:
    {
        MCM_setRegenMode(mcm, icCanMessage->data[0]);
        // MCM_setMaxTorqueDNm(mcm, (ubyte2)icCanMessage->data[1] << 8 | icCanMessage->data[0]);
        // me->torqueMapMode = icCanMessage->data[2];
        //  me->launchControlSensitivity = icCanMessage->data[3];    //unused
        break;
    }

    //////////////////////////////////////////////////////
    //              REGEN TORQUE SETTINGS               //
    // Byte 0-1 LE ubyte2: Set Max Regen Torque Limit   //
    // Byte 2-3 LE ubyte2: Set Torque at Zero Pedal     //
    //////////////////////////////////////////////////////
    case 0x703:
    {
        MCM_setRegen_TorqueLimitDNm(mcm, (icCanMessage->data[0] * 10)); // Nm to DNm
        // MCM_setRegen_TorqueLimitDNm(mcm, (ubyte2)icCanMessage->data[1] << 8 | icCanMessage->data[0]);
        // MCM_setRegen_TorqueAtZeroPedalDNm(mcm, (ubyte2)icCanMessage->data[3] << 8 | icCanMessage->data[2]);
        break;
    }

    //////////////////////////////////////////////////////
    //          REGEN PEDAL PERCENTAGE SETTINGS         //
    // Byte 0-3 LE float4: Set Max Regen Torque Limit   //
    // Byte 4-7 LE float4: Set Torque at Zero Pedal     //
    //////////////////////////////////////////////////////
    case 0x704:
    {
        MCM_setRegen_TorqueAtZeroPedalDNm(mcm, (icCanMessage->data[0] * 10)); // Nm to DNm
        /*
        float4 BPSfloat, APPSfloat;
        // evil bithack avoids float cast errors and keeps code footprint small
        // relies on non-standard behaviour of pointer typecasting
        // more rigorous implementaion will probably involve creating a memcpy-like function
        * (ubyte4 *) &BPSfloat = (ubyte4)icCanMessage->data[3] << 24 | (ubyte4)icCanMessage->data[2] << 16 |
        icCanMessage->data[1] << 8 | icCanMessage->data[0];
        * (ubyte4 *) &APPSfloat = (ubyte4)icCanMessage->data[7] << 24 | (ubyte4)icCanMessage->data[6] << 16 |
        icCanMessage->data[5] << 8 | icCanMessage->data[4]; MCM_setRegen_PercentBPSForMaxRegen(mcm, BPSfloat);
        MCM_setRegen_PercentAPPSForCoasting(mcm, APPSfloat);
        */
        break;
    }
    }
}

ubyte1 IC_getTorqueMapMode(InstrumentCluster *me) { return me->torqueMapMode; }

ubyte1 IC_getLaunchControlSensitivity(InstrumentCluster *me) { return me->launchControlSensitivity; }

/*****************************************************************************
 * Motor Controller module Accessors / Mutators (Set/Get)
 ****************************************************************************/
/*
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
*/