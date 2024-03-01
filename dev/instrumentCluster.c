#include "IO_Driver.h"

#include "instrumentCluster.h"
#include "motorController.h"    // need definition of MotorController's struct, not just decaration
#include "canManager.h"


struct _InstrumentCluster
{
   
    ubyte2 canMessageBaseId;  //Starting message ID for messages that will come in from this controller

    ubyte1 torqueMapMode;

    //0 = off. Default OFF
    ubyte1 launchControlSensitivity;
    
};

InstrumentCluster* InstrumentCluster_new(ubyte2 canMessageBaseID)
{
    InstrumentCluster* me = (InstrumentCluster*)malloc(sizeof(struct _InstrumentCluster));

    me->canMessageBaseId = canMessageBaseID;

    me->torqueMapMode=0;

    me->launchControlSensitivity=0;
    
    return me;
}

void IC_parseCanMessage(InstrumentCluster* me, MotorController* mcm, IO_CAN_DATA_FRAME* icCanMessage)
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
            //MCM_setMaxTorqueDNm(mcm, (ubyte2)icCanMessage->data[1] << 8 | icCanMessage->data[0]);
            //me->torqueMapMode = icCanMessage->data[2];
            // me->launchControlSensitivity = icCanMessage->data[3];    //unused
            break;
        }
        
        //////////////////////////////////////////////////////
        //              REGEN TORQUE SETTINGS               //
        // Byte 0-1 LE ubyte2: Set Max Regen Torque Limit   //
        // Byte 2-3 LE ubyte2: Set Torque at Zero Pedal     //
        //////////////////////////////////////////////////////
        case 0x703:
        {
            MCM_setRegen_TorqueLimitDNm(mcm, (icCanMessage->data[0]*10)); //Nm to DNm
            //MCM_setRegen_TorqueLimitDNm(mcm, (ubyte2)icCanMessage->data[1] << 8 | icCanMessage->data[0]);
            //MCM_setRegen_TorqueAtZeroPedalDNm(mcm, (ubyte2)icCanMessage->data[3] << 8 | icCanMessage->data[2]);
            break;
        }

        //////////////////////////////////////////////////////
        //          REGEN PEDAL PERCENTAGE SETTINGS         //
        // Byte 0-3 LE float4: Set Max Regen Torque Limit   //
        // Byte 4-7 LE float4: Set Torque at Zero Pedal     //
        //////////////////////////////////////////////////////
        case 0x704:
        {
            MCM_setRegen_TorqueAtZeroPedalDNm(mcm, (icCanMessage->data[0]*10)); //Nm to DNm
            break;
        }
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
