
#include <stdlib.h> //malloc

#include "IO_Driver.h" 
#include "IO_CAN.h"
#include "IO_RTC.h"

#include "mathFunctions.h"
#include "sensors.h"
#include "canManager.h"
#include "motorController.h"
#include "bms.h"
#include "safety.h"
#include "wheelSpeeds.h"
#include "LaunchControl.h"
#include "drs.h"
#include "main.h"


CanManager* CanManager_new(ubyte2 can0_busSpeed, ubyte1 can0_read_messageLimit, ubyte1 can0_write_messageLimit,
                           ubyte2 can1_busSpeed, ubyte1 can1_read_messageLimit, ubyte1 can1_write_messageLimit,
                           ubyte4 defaultSendDelayus)
{
    CanManager* me = (CanManager*)malloc(sizeof(struct _CanManager));

    for (ubyte4 id = 0; id <= 0x7FF; id++)
    {
        me->canMessageHistory[id] = 0;
    }

    me->sendDelayus = defaultSendDelayus;

    //Activate the CAN channels --------------------------------------------------
    me->ioErr_can0_Init = IO_CAN_Init(IO_CAN_CHANNEL_0, can0_busSpeed, 0, 0, 0);
    me->ioErr_can1_Init = IO_CAN_Init(IO_CAN_CHANNEL_1, can1_busSpeed, 0, 0, 0);

    me->can0_read_messageLimit = can0_read_messageLimit;
    me->can0_write_messageLimit = can0_write_messageLimit;
    me->can1_read_messageLimit = can1_read_messageLimit;
    me->can1_write_messageLimit = can1_write_messageLimit;

    // Configure the FIFO queues
    // This specifies: The handle names for the queues
    // which channel the queue belongs to
    // the # of messages (or maximum count?)
    // the direction of the queue (in/out)
    // the frame size and other stuff?
    IO_CAN_ConfigFIFO(&me->can0_readHandle, IO_CAN_CHANNEL_0, can0_read_messageLimit, IO_CAN_MSG_READ, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can0_writeHandle, IO_CAN_CHANNEL_0, can0_write_messageLimit, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can1_readHandle, IO_CAN_CHANNEL_1, can1_read_messageLimit, IO_CAN_MSG_READ, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can1_writeHandle, IO_CAN_CHANNEL_1, can1_write_messageLimit, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);

    // Assume read/write at error state until used
    me->ioErr_can0_read = IO_E_CAN_BUS_OFF;
    me->ioErr_can0_write = IO_E_CAN_BUS_OFF;
    me->ioErr_can1_read = IO_E_CAN_BUS_OFF;
    me->ioErr_can1_write = IO_E_CAN_BUS_OFF;
    
    ubyte2 messageID;
    ubyte1 emptyData[8] = { 0,0,0,0,0,0,0,0 };
    //Outgoing ----------------------------
    CAN_msg_insert(me->canMessageHistory, 0xC0, emptyData, 25000, 125000, TRUE);  //MCM Command Message

    for (messageID = 0x500; messageID <= 0x515; messageID++)
    {
        CAN_msg_insert(me->canMessageHistory, messageID, emptyData, 50000, 250000, TRUE);
    }

    return me;
}

CanMessageNode *CAN_msg_insert(CanMessageNode **messageHistoryArray, ubyte4 messageID, ubyte1 messageData[8], ubyte4 minTime, ubyte4 maxTime, bool req)
{
    CanMessageNode *message = (CanMessageNode *)malloc(sizeof(CanMessageNode));
    if (message == NULL) //malloc failed
    {
        //fprintf(stderr, "Out of memory!!! (insert)\n");
        //exit(1);
    }
    else
    {
        message->timeBetweenMessages_Min = minTime;
        message->timeBetweenMessages_Max = maxTime;
        IO_RTC_StartTime(&message->lastMessage_timeStamp);
        //To copy an entire array, http://stackoverflow.com/questions/9262784/array-equal-another-array
        memcpy(messageData, message->data, sizeof(messageData));
        message->required = req;
        messageHistoryArray[messageID] = message;
    }
    return message;

}

/*****************************************************************************
* This function takes an array of messages, determines which messages to send
* based on whether or not data has changed since the last time it was sent,
* or if a certain amount of time has passed since the last time it was sent.
*
* Messages that need to be sent are copied to another array and passed to the
* FIFO queue.
*
* Note: http://stackoverflow.com/questions/5573310/difference-between-passing-array-and-array-pointer-into-function-in-c
* http://stackoverflow.com/questions/2360794/how-to-pass-an-array-of-struct-using-pointer-in-c-c
****************************************************************************/
IO_ErrorType CanManager_send(CanManager* me, CanChannel channel, IO_CAN_DATA_FRAME canMessages[], ubyte1 canMessageCount)
{
    bool sendSerialDebug = FALSE;
    bool sendMessage = FALSE;
    ubyte1 messagesToSendCount = 0;
    IO_CAN_DATA_FRAME messagesToSend[canMessageCount];
    ubyte1 emptyData[8] = { 0,0,0,0,0,0,0,0 };

    //----------------------------------------------------------------------------
    // Check if message exists in outgoing message history tree
    //----------------------------------------------------------------------------
    CanMessageNode* lastMessage;  //replace with me->canMessageHistory[ID]
    ubyte1 messagePosition; //used twice
    for (messagePosition = 0; messagePosition < canMessageCount; messagePosition++)
    {
        bool firstTimeMessage = FALSE;
        bool dataChanged = FALSE;
        bool minTimeExceeded = FALSE;
        bool maxTimeExceeded = FALSE;

        ubyte2 outboundMessageID = canMessages[messagePosition].id;
        lastMessage = me->canMessageHistory[outboundMessageID];
        sendMessage = FALSE;

        //----------------------------------------------------------------------------
        // Check if this message exists in the array, if not init it
        //----------------------------------------------------------------------------
        firstTimeMessage = (me->canMessageHistory[outboundMessageID] == NULL);  // pointer is null
        if (firstTimeMessage)
        {
            me->canMessageHistory[outboundMessageID] = (CanMessageNode*)malloc(sizeof(CanMessageNode));
            me->canMessageHistory[outboundMessageID]->timeBetweenMessages_Min = 25000;
            me->canMessageHistory[outboundMessageID]->timeBetweenMessages_Max = 125000;
            me->canMessageHistory[outboundMessageID]->required = TRUE;
            memcpy(emptyData, me->canMessageHistory[outboundMessageID]->data, sizeof(emptyData));
            me->canMessageHistory[outboundMessageID]->lastMessage_timeStamp = 0;
        }

        //----------------------------------------------------------------------------
        // Check if data has changed since last time message was sent
        //----------------------------------------------------------------------------
        //Check each data byte in the data array
        for (ubyte1 dataPosition = 0; dataPosition < 8; dataPosition++)
        {
            //if any data byte is changed, then probably want to send the message
            if (lastMessage->data[dataPosition] != canMessages[messagePosition].data[dataPosition])
            {
                dataChanged = TRUE; //ONLY MODIFY IF CHANGED
            }
        } // end checking each byte in message

        //----------------------------------------------------------------------------
        // Check if time has exceeded
        //----------------------------------------------------------------------------
        minTimeExceeded = ((IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp) >= lastMessage->timeBetweenMessages_Min));
        maxTimeExceeded = ((IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp) >= 50000));
        
        //----------------------------------------------------------------------------
        // If any criteria were exceeded, send the message out
        //----------------------------------------------------------------------------
        if ((firstTimeMessage) || (dataChanged && minTimeExceeded) || (!dataChanged && maxTimeExceeded))
        {
            sendMessage = TRUE;
        }

        //----------------------------------------------------------------------------
        // If we determined that this message should be sent
        //----------------------------------------------------------------------------
        if (sendMessage == TRUE)
        {
            //copy the message that needs to be sent into the outgoing messages array
            //see http://stackoverflow.com/questions/1693853/copying-arrays-of-structs-in-c
            //http://www.socialledge.com/sjsu/index.php?title=ES101_-_Lesson_9_:_Structures
            messagesToSend[messagesToSendCount++] = canMessages[messagePosition];
        }
    } // end of loop for each message in outgoing messages

    IO_UART_Task();
    //----------------------------------------------------------------------------
    // If there are messages to send
    //----------------------------------------------------------------------------
    IO_ErrorType sendResult = IO_E_OK;
    if (messagesToSendCount > 0)
    {
        // Send the messages to send to the appropriate FIFO queue
        sendResult = IO_CAN_WriteFIFO((channel == CAN0_HIPRI) ? me->can0_writeHandle : me->can1_writeHandle, messagesToSend, messagesToSendCount);
        *((channel == CAN0_HIPRI) ? &me->ioErr_can0_write : &me->ioErr_can1_write) = sendResult;

        // Update the outgoing message tree with message sent timestamps
        if ((channel == CAN0_HIPRI ? me->ioErr_can0_write : me->ioErr_can1_write) == IO_E_OK)
        {
            // Loop through the messages that we sent...
            for (messagePosition = 0; messagePosition < messagesToSendCount; messagePosition++)
            {
                IO_RTC_StartTime(&me->canMessageHistory[messagesToSend[messagePosition].id]->lastMessage_timeStamp); // Update the timestamp for when the message was last sent
            }
        }
    }
    return sendResult;
}

/*****************************************************************************
* read
****************************************************************************/
void CanManager_read(CanManager* me, CanChannel channel, MotorController* mcm, InstrumentCluster* ic, BatteryManagementSystem* bms, SafetyChecker* sc)
{
    IO_CAN_DATA_FRAME canMessages[(channel == CAN0_HIPRI ? me->can0_read_messageLimit : me->can1_read_messageLimit)];
    ubyte1 canMessageCount;  // FIFO queue only holds 128 messages max

    // Read messages from hipri channel 
    *(channel == CAN0_HIPRI ? &me->ioErr_can0_read : &me->ioErr_can1_read) =
    IO_CAN_ReadFIFO((channel == CAN0_HIPRI ? me->can0_readHandle : me->can1_writeHandle),
                    canMessages,
                    (channel == CAN0_HIPRI ? me->can0_read_messageLimit : me->can1_read_messageLimit),
                    &canMessageCount);

    // Determine message type based on ID
    for (int currMessage = 0; currMessage < canMessageCount; currMessage++)
    {
        switch (canMessages[currMessage].id)
        {
        //-------------------------------------------------------------------------
        // Motor controller
        //-------------------------------------------------------------------------
        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
            MCM_parseCanMessage(mcm, &canMessages[currMessage]);
            break;
        case 0xA7:
            MCM_parseCanMessage(mcm, &canMessages[currMessage]);
            break;
        case 0xA8:
        case 0xA9:
        case 0xAA:
        case 0xAB:
        case 0xAC:
        case 0x0AD:
        case 0x0AE:
        case 0x0AF:
            MCM_parseCanMessage(mcm, &canMessages[currMessage]);
            break;


        //-------------------------------------------------------------------------
        // BMS
        //-------------------------------------------------------------------------
        case 0x600:
        case 0x602: // Faults
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;
        case 0x604:
        case 0x608:
        case 0x610:
        case 0x611:
        case 0x612:
        case 0x613:
        case 0x620:
        case 0x621:
        case 0x622: // Cell Voltage Summary
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;
        case 0x623: // Cell Temperature Summary
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;
        case 0x624:
        // 1st Module
        case 0x630:
        case 0x631:
        case 0x632:
        // 2nd Module
        case 0x633:
        case 0x634:
        case 0x635:
        // 3rd Module
        case 0x636:
        case 0x637:
        case 0x638:
        // 4th Module
        case 0x639:
        case 0x63A:
        case 0x63B:
        // 5th Module
        case 0x63C:
        case 0x63D:
        case 0x63E:
        // 6th Module
        case 0x63F:
        case 0x640:
        case 0x641:
        // 7th Module
        case 0x642:
        case 0x643:
        case 0x644:
        // 8th Module
        case 0x645:
        case 0x646:
        case 0x647:

        case 0x629:
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;

        case 0x702:
            IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;
        case 0x703:
            IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;
        case 0x704:
            IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;
        //-------------------------------------------------------------------------
        // VCU Debug Control
        //-------------------------------------------------------------------------
        case 0x5FF:
            SafetyChecker_parseCanMessage(sc, &canMessages[currMessage]);
            MCM_parseCanMessage(mcm, &canMessages[currMessage]);
        }
    }
    CanManager_send(me, CAN0_HIPRI, canMessages, canMessageCount);
    CanManager_send(me, CAN1_LOPRI, canMessages, canMessageCount);
}

ubyte1 CanManager_getReadStatus(CanManager* me, CanChannel channel)
{
    return (channel == CAN0_HIPRI) ? me->ioErr_can0_read : me->ioErr_can1_read;
}




/*****************************************************************************
* device-specific functions
****************************************************************************/
/*****************************************************************************
* Standalone Sensor messages
******************************************************************************
* Load sensor values into CAN messages
* Each can message's .data[] holds 1 byte - sensor data must be broken up into separate bytes
* The message addresses are at:
* https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit
****************************************************************************/


//----------------------------------------------------------------------------
// 
//----------------------------------------------------------------------------
void canOutput_sendDebugMessage(CanManager* me, TorqueEncoder* tps, BrakePressureSensor* bps, MotorController* mcm, InstrumentCluster* ic, BatteryManagementSystem* bms, WheelSpeeds* wss, SafetyChecker* sc, LaunchControl* lc, DRS *drs)
{
    IO_CAN_DATA_FRAME canMessages[me->can0_write_messageLimit];
    IO_CAN_DATA_FRAME canMessages1[me->can1_write_messageLimit];
    ubyte2 canMessageCount = 0;
    ubyte2 canMessageID = 0x500;

    //500: TPS 0
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_tps0_can_message(tps);

    //TPS 1
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_tps1_can_message(tps);

    //BPS0
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_bps0_can_message(bps);

    //WSS mm/s output
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_wss_can_message(wss);

    //WSS RPM non-interpolated output
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_wss_rpm1_can_message(wss);

    //WSS RPM interpolated output
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_wss_rpm2_can_message(wss);

    //506: Safety Checker
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_sc_can_message(sc);

    //507: LV Battery
    // canMessageCount++;
    // canMessages[canMessageCount - 1] = get_lvb_can_message();

    //508: MCM Regen settings
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_mcm_regen_can_message(mcm);

    //509: MCM RTD Status
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_mcm_rtd_can_message(mcm);

    //50A: MCM Ground Speed Reference
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_mcm_gsr_can_message(mcm);

    //50B: Launch Control
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_lc_can_message(lc);

    //50C: SAS (Steering Angle Sensor) and DRS
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_drs_can_message(drs);

    //50D: BPS1
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_bps1_can_message(bps);


    //50F: MCM Power Debug
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_mcm_power_can_message(mcm, sc);

    //511: SoftBSPD
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_bspd_can_message(mcm, sc);

    //512: MCM Torque Command
    // canMessageCount++;
    // canMessages[canMessageCount - 1] = get_mcm_pl_can_message(mcm);

    //Motor controller command message
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_mcm_command_can_message(mcm);
    
    //Place the can messages into the FIFO queue ---------------------------------------------------
    CanManager_send(me, CAN0_HIPRI, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)

    //50E: BMS Loopback Test
    canMessages1[0] = get_bms_loopback_can_message(bms);
    CanManager_send(me, CAN1_LOPRI, canMessages1, (ubyte2) 1);

}

IO_CAN_DATA_FRAME get_tps0_can_message(TorqueEncoder* tps) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x500;
    canMessage.data[0] = 0xFF * tps->travelPercent;
    canMessage.data[1] = 0xFF * tps->tps0_percent;
    canMessage.data[2] = tps->tps0_value;
    canMessage.data[3] = tps->tps0_value >> 8;
    canMessage.data[4] = tps->tps0_calibMin;
    canMessage.data[5] = tps->tps0_calibMin >> 8;
    canMessage.data[6] = tps->tps0_calibMax;
    canMessage.data[7] = tps->tps0_calibMax >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_tps1_can_message(TorqueEncoder* tps) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x501;
    canMessage.data[0] = 0xFF * tps->travelPercent;
    canMessage.data[1] = 0xFF * tps->tps1_percent;
    canMessage.data[2] = tps->tps1_value;
    canMessage.data[3] = tps->tps1_value >> 8;
    canMessage.data[4] = tps->tps1_calibMin;
    canMessage.data[5] = tps->tps1_calibMin >> 8;
    canMessage.data[6] = tps->tps1_calibMax;
    canMessage.data[7] = tps->tps1_calibMax >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_bps0_can_message(BrakePressureSensor* bps) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x502;
    canMessage.data[0] = 0xFF * bps->percent;
    canMessage.data[1] = 0;
    canMessage.data[2] = bps->bps0_value;
    canMessage.data[3] = bps->bps0_value >> 8;
    canMessage.data[4] = bps->bps0_calibMin;
    canMessage.data[5] = bps->bps0_calibMin >> 8;
    canMessage.data[6] = bps->bps0_calibMax;
    canMessage.data[7] = bps->bps0_calibMax >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_bps1_can_message(BrakePressureSensor* bps) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x50D;
    canMessage.data[0] = 0xFF * bps->percent;
    canMessage.data[1] = 0;
    canMessage.data[2] = bps->bps1_value;
    canMessage.data[3] = bps->bps1_value >> 8;
    canMessage.data[4] = bps->bps1_calibMin;
    canMessage.data[5] = bps->bps1_calibMin >> 8;
    canMessage.data[6] = bps->bps1_calibMax;
    canMessage.data[7] = bps->bps1_calibMax >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_wss_can_message(WheelSpeeds* wss) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x503;
    canMessage.data[0] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, FL) + 0.5);
    canMessage.data[1] = ((ubyte2)(WheelSpeeds_getWheelSpeed(wss, FL) + 0.5)) >> 8;
    canMessage.data[2] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, FR) + 0.5);
    canMessage.data[3] = ((ubyte2)(WheelSpeeds_getWheelSpeed(wss, FR) + 0.5)) >> 8;
    canMessage.data[4] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, RL) + 0.5);
    canMessage.data[5] = ((ubyte2)(WheelSpeeds_getWheelSpeed(wss, RL) + 0.5)) >> 8;
    canMessage.data[6] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, RR) + 0.5);
    canMessage.data[7] = ((ubyte2)(WheelSpeeds_getWheelSpeed(wss, RR) + 0.5)) >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_wss_rpm1_can_message(WheelSpeeds* wss) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x504;
    canMessage.data[0] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, FALSE) + 0.5);
    canMessage.data[1] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, FALSE) + 0.5)) >> 8;
    canMessage.data[2] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FR, FALSE) + 0.5);
    canMessage.data[3] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FR, FALSE) + 0.5)) >> 8;
    canMessage.data[4] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RL, FALSE) + 0.5);
    canMessage.data[5] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RL, FALSE) + 0.5)) >> 8;
    canMessage.data[6] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, FALSE) + 0.5);
    canMessage.data[7] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, FALSE) + 0.5)) >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_wss_rpm2_can_message(WheelSpeeds* wss) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x505;
    canMessage.data[0] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) + 0.5);
    canMessage.data[1] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) + 0.5)) >> 8;
    canMessage.data[2] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FR, TRUE) + 0.5);
    canMessage.data[3] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FR, TRUE) + 0.5)) >> 8;
    canMessage.data[4] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RL, TRUE) + 0.5);
    canMessage.data[5] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RL, TRUE) + 0.5)) >> 8;
    canMessage.data[6] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, TRUE) + 0.5);
    canMessage.data[7] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, TRUE) + 0.5)) >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_sc_can_message(SafetyChecker* sc) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x506;
    canMessage.data[0] = SafetyChecker_getFaults(sc);
    canMessage.data[1] = SafetyChecker_getFaults(sc) >> 8;
    canMessage.data[2] = SafetyChecker_getFaults(sc) >> 16;
    canMessage.data[3] = SafetyChecker_getFaults(sc) >> 24;
    canMessage.data[4] = SafetyChecker_getWarnings(sc);
    canMessage.data[5] = SafetyChecker_getWarnings(sc) >> 8;
    canMessage.data[6] = SafetyChecker_getNotices(sc);
    canMessage.data[7] = SafetyChecker_getNotices(sc) >> 8;
    canMessage.length = 8;
    return canMessage;
}

// IO_CAN_DATA_FRAME get_lvb_can_message() {
//     float4 LVBatterySOC = lv_battery_soc();
//     Sensor_LVBattery.sensorValue = Sensor_LVBattery.sensorValue + 0.46;
//     IO_CAN_DATA_FRAME canMessage;
//     canMessage.id_format = IO_CAN_STD_FRAME;
//     canMessage.id = 0x507;
//     canMessage.data[0] = (ubyte1)Sensor_LVBattery.sensorValue;
//     canMessage.data[1] = Sensor_LVBattery.sensorValue >> 8;
//     canMessage.data[2] = (sbyte1)(100 * LVBatterySOC);
//     canMessage.data[3] = 0;
//     canMessage.data[4] = 0;
//     canMessage.data[5] = 0;
//     canMessage.data[6] = 0;
//     canMessage.data[7] = 0;
//     canMessage.length = 8;
//     return canMessage;
// }

IO_CAN_DATA_FRAME get_mcm_regen_can_message(MotorController* mcm) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x508;
    canMessage.data[0] = MCM_getRegenMode(mcm);
    canMessage.data[1] = 0;
    canMessage.data[2] = MCM_getMaxTorqueDNm(mcm)/10;
    canMessage.data[3] = MCM_getRegenTorqueLimitDNm(mcm)/10;
    canMessage.data[4] = MCM_getRegenTorqueAtZeroPedalDNm(mcm)/10;
    canMessage.data[5] = 0;
    canMessage.data[6] = MCM_getRegenAPPSForMaxCoastingZeroToFF(mcm);
    canMessage.data[7] = MCM_getRegenBPSForMaxRegenZeroToFF(mcm);
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_mcm_rtd_can_message(MotorController* mcm) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x509;
    canMessage.data[0] = Sensor_HVILTerminationSense.sensorValue;
    canMessage.data[1] = Sensor_HVILTerminationSense.sensorValue >> 8;
    canMessage.data[2] = MCM_getHvilOverrideStatus(mcm);
    canMessage.data[3] = 0;
    canMessage.data[4] = 0;
    canMessage.data[5] = 0; 
    canMessage.data[6] = 0;
    canMessage.data[7] = 0;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_mcm_gsr_can_message(MotorController* mcm) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x50A;
    canMessage.data[0] = (ubyte4)MCM_getGroundSpeedKPH(mcm);
    canMessage.data[1] = (ubyte4)MCM_getGroundSpeedKPH(mcm) >> 8;
    canMessage.data[2] = 0;
    canMessage.data[3] = 0;
    canMessage.data[4] = 0;
    canMessage.data[5] = 0;
    canMessage.data[6] = 0;
    canMessage.data[7] = 0;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_lc_can_message(LaunchControl* lc) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x50B;
    canMessage.data[0] = lc->LCReady;
    canMessage.data[1] = lc->LCStatus;
    canMessage.data[2] = lc->lcTorque;
    canMessage.data[3] = getCalculatedTorque();
    canMessage.data[4] = getCalculatedTorque() >> 8;
    canMessage.data[5] = (sbyte2)lc->slipRatio;
    canMessage.data[6] = (sbyte2)lc->slipRatio >> 8;
    canMessage.data[7] = (ubyte2)lc->lcTorque;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_drs_can_message(DRS* drs) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x50C;
    canMessage.data[0] = steering_degrees();
    canMessage.data[1] = steering_degrees() >> 8;
    canMessage.data[2] = drs->buttonPressed;
    canMessage.data[3] = drs->currentDRSMode;
    canMessage.data[4] = drs->drsFlap;
    canMessage.data[5] = 0;
    canMessage.data[6] = 0;
    canMessage.data[7] = 0;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_bms_loopback_can_message(BatteryManagementSystem* bms) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x50E;
    canMessage.data[0] = BMS_getFaultFlags0(bms);
    canMessage.data[1] = BMS_getFaultFlags1(bms);
    canMessage.data[2] = BMS_getRelayState(bms);
    canMessage.data[3] = BMS_getHighestCellTemp_d_degC(bms);
    canMessage.data[4] = (BMS_getHighestCellTemp_d_degC(bms) >> 8);
    canMessage.data[5] = 0;
    canMessage.data[6] = 0;
    canMessage.data[7] = 0;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_mcm_power_can_message(MotorController* mcm, SafetyChecker* sc) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x50F;
    canMessage.data[0] = MCM_getPower(mcm);
    canMessage.data[1] = MCM_getPower(mcm) >> 8;
    canMessage.data[2] = MCM_getPower(mcm) >> 16;
    canMessage.data[3] = MCM_getPower(mcm) >> 24;
    canMessage.data[4] = SafetyChecker_getWarnings(sc);
    canMessage.data[5] = (SafetyChecker_getWarnings(sc) >> 8);
    canMessage.data[6] = (SafetyChecker_getWarnings(sc) >> 16);
    canMessage.data[7] = (SafetyChecker_getWarnings(sc) >> 24);
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_bspd_can_message(MotorController* mcm, SafetyChecker* sc) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x511;
    canMessage.data[0] = (ubyte1) SafetyChecker_getsoftbspd(sc);
    canMessage.data[1] = 0;
    canMessage.data[2] = 0; 
    canMessage.data[3] = 0; 
    canMessage.data[4] = 0;
    canMessage.data[5] = 0;
    canMessage.data[6] = 0;
    canMessage.data[7] = 0;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_mcm_command_can_message(MotorController* mcm) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0xC0;
    canMessage.data[0] = (ubyte1)MCM_commands_getTorque(mcm);
    canMessage.data[1] =  MCM_commands_getTorque(mcm) >> 8;
    canMessage.data[2] = 0;  //Speed (RPM) 
    canMessage.data[3] = 0 >> 8;  //Speed (RPM) 
    canMessage.data[4] = MCM_commands_getDirection(mcm);
    canMessage.data[5] = (MCM_commands_getInverter(mcm) == ENABLED) ? 1 : 0; // Inverter Enable
    canMessage.data[5] |= 0 << 2; // Speed Mode Override 
    canMessage.data[6] = (ubyte1)MCM_commands_getTorqueLimit(mcm);
    canMessage.data[7] = MCM_commands_getTorqueLimit(mcm) >> 8;
    canMessage.length = 8;
    return canMessage;
}