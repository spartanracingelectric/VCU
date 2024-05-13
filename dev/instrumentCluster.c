#include "IO_Driver.h"
#include <stdlib.h>
#include "instrumentCluster.h"
#include "motorController.h"    // need definition of MotorController's struct, not just decaration
#include "canManager.h"


void InstrumentCluster_new(InstrumentCluster *me, ubyte2 canMessageBaseID)
{
    me->canMessageBaseId = canMessageBaseID;

    me->torqueMapMode=0;

    me->launchControlSensitivity=0;
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
            /*
            float4 BPSfloat, APPSfloat;
            // evil bithack avoids float cast errors and keeps code footprint small
            // relies on non-standard behaviour of pointer typecasting
            // more rigorous implementation will probably involve creating a memcpy-like function
            * (ubyte4 *) &BPSfloat = (ubyte4)icCanMessage->data[3] << 24 | (ubyte4)icCanMessage->data[2] << 16 | icCanMessage->data[1] << 8 | icCanMessage->data[0];
            * (ubyte4 *) &APPSfloat = (ubyte4)icCanMessage->data[7] << 24 | (ubyte4)icCanMessage->data[6] << 16 | icCanMessage->data[5] << 8 | icCanMessage->data[4];
            MCM_setRegen_PercentBPSForMaxRegen(mcm, BPSfloat);
            MCM_setRegen_PercentAPPSForCoasting(mcm, APPSfloat);
            */
            break;
        }
    }
}
