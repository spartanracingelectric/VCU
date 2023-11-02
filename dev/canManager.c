
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
#include "serial.h"
#include "sensorCalculations.h"
#include "LaunchControl.h"
#include "drs.h"
#include "lut.h"
#include "main.h"

extern LUT* LV_BATT_SOC_LUT;

CanManager* CanManager_new(ubyte2 can0_busSpeed, ubyte1 can0_read_messageLimit, ubyte1 can0_write_messageLimit,
                           ubyte2 can1_busSpeed, ubyte1 can1_read_messageLimit, ubyte1 can1_write_messageLimit,
                           ubyte4 defaultSendDelayus, SerialManager* serialMan)
{
    CanManager* me = (CanManager*)malloc(sizeof(struct _CanManager));
    me->sm = serialMan;

    for (ubyte4 id = 0; id <= 0x7FF; id++)
    {
        me->canMessageHistory[id] = 0;
    }

    me->sendDelayus = defaultSendDelayus;

    //Activate the CAN channels --------------------------------------------------
    me->ioErr_can0_Init = IO_CAN_Init(IO_CAN_CHANNEL_0, can0_busSpeed, 0, 0, 0);
    me->ioErr_can1_Init = IO_CAN_Init(IO_CAN_CHANNEL_1, can1_busSpeed, 0, 0, 0);

    //Configure the FIFO queues
    //This specifies: The handle names for the queues
    //, which channel the queue belongs to
    //, the # of messages (or maximum count?)
    //, the direction of the queue (in/out)
    //, the frame size
    //, and other stuff?
    IO_CAN_ConfigFIFO(&me->can0_readHandle, IO_CAN_CHANNEL_0, can0_read_messageLimit, IO_CAN_MSG_READ, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can0_writeHandle, IO_CAN_CHANNEL_0, can0_write_messageLimit, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can1_readHandle, IO_CAN_CHANNEL_1, can1_read_messageLimit, IO_CAN_MSG_READ, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can1_writeHandle, IO_CAN_CHANNEL_1, can1_write_messageLimit, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);

    //Assume read/write at error state until used
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
    ubyte2 serialMessageID = 0xC0;
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
        else
        {
            if (sendSerialDebug && (serialMessageID == outboundMessageID)) {
                SerialManager_send(me->sm, "This message did not need to be sent.\n");
            }
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

    // 0x0AA
    static const ubyte1 bitInverter = 1;  //bit 1
    static const ubyte1 bitLockout = 128; //bit 7

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
        case 0x0A0:
            //0,1 module A temperature
            //2,3 module B temperature
            //4,5 module C temperature
            //6,7 gate driver board temperature
            break;
        case 0x0A1:
            //0,1 control board temp
            //2,3 rtd 1 temp
            //4,5 rtd 2 temp
            //6,7 rtd 3 temp
            break;
        case 0x0A2:
            //0,1 rtd 4 temp
            //2,3 rtd 5 temp
            //4,5 motor temperature***
            mcm->motor_temp = ((ubyte2)canMessages[currMessage].data[5] << 8 | canMessages[currMessage].data[4]) / 10;
            //6,7 torque shudder
            break;
        case 0x0A3:
            //0,1 voltage analog input #1
            //2,3 voltage analog input #2
            //4,5 voltage analog input #3
            //6,7 voltage analog input #4
            break;
        case 0x0A4:
            // booleans //
            // 0 digital input #1
            // 1 digital input #2
            // 2 digital input #3
            // 4 digital input #5
            // 5 digital input #6
            // 6 digital input #7
            // 7 digital input #8
            break;
        case 0x0A5:
            //0,1 motor angle (electrical)
            //2,3 motor speed*** // in rpms
            //Cast may be required - needs testing
            mcm->motorRPM = reasm_ubyte2(canMessages[currMessage].data, 2);
            //4,5 electrical output frequency
            //6,7 delta resolver filtered
            break;
        case 0x0A6:
            //0,1 Phase A current
            //2,3 Phase B current
            //4,5 Phase C current
            //6,7 DC bus current
            mcm->DC_Current = reasm_ubyte2(canMessages[currMessage].data, 6) / 10;
            break;
        case 0x0A7:
            //0,1 DC bus voltage***
            mcm->DC_Voltage = reasm_ubyte2(canMessages[currMessage].data, 0) / 10;
            //2,3 output voltage
            //4,5 Phase AB voltage
            //6,7 Phase BC voltage
            break;
        case 0x0A8:
            //0,1 Flux Command
            //2,3 flux feedback
            //4,5 id feedback
            //6,7 iq feedback
            break;
        case 0x0A9:
            // 0,1 1.5V reference voltage
            // 2,3 2.5V reference voltage
            // 4,5 5.0V reference voltage
            // 6,7 12V reference voltage
            break;
        case 0x0AA:
            //0,1 VSM state
            //2   Inverter state
            //3   Relay State
            //4   bit-0 inverter run mode
            //4   bit5-7 inverter active discharge state
            //5   inverter command mode

            //6   internal states
            //    bit0 inverter enable state***
            mcm->inverterStatus = (canMessages[currMessage].data[6] & bitInverter) > 0 ? ENABLED : DISABLED;
            //    bit7 inverter enable lockout***
            mcm->lockoutStatus = (canMessages[currMessage].data[6] & bitLockout) > 0 ? ENABLED : DISABLED;

            //7   direction command
            break;
        case 0x0AB: //Faults
            //mcmCanMessage->data;
            //me->faultHistory |= data stuff //????????

            break;
        case 0x0AC:
            //0,1 Commanded Torque
            mcm->commandedTorque = reasm_ubyte2(canMessages[currMessage].data, 0) / 10;
            //2,3 Torque Feedback
            break;
        // case 0x0AD:
        // case 0x0AE:
        // case 0x0AF:

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
            if (canMessages[currMessage].data[1] > 0)
            {
                IO_RTC_StartTime(&mcm->timeStamp_HVILOverrideCommandReceived);
            }
            if (canMessages[currMessage].data[2] == 55)
            {
                IO_RTC_StartTime(&mcm->timeStamp_InverterEnableOverrideCommandReceived);
            }
            if (canMessages[currMessage].data[3] > 0)
            {
                IO_RTC_StartTime(&mcm->timeStamp_InverterDisableOverrideCommandReceived);
            }
                break;
        // default:
        }
    }
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
void canOutput_sendSensorMessages(CanManager* me)
{

}


//----------------------------------------------------------------------------
// 
//----------------------------------------------------------------------------
void canOutput_sendDebugMessage(CanManager* me, TorqueEncoder* tps, BrakePressureSensor* bps, MotorController* mcm, InstrumentCluster* ic, BatteryManagementSystem* bms, WheelSpeeds* wss, SafetyChecker* sc, LaunchControl* lc, DRS *drs)
{
    IO_CAN_DATA_FRAME canMessages[me->can0_write_messageLimit];
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
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_lvb_can_message();

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

    
    //50E: BMS Loopback Test
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_bms_loopback_can_message(bms);

    //50F: MCM Power Debug
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_mcm_power_can_message(mcm, sc);

    //511: SoftBSPD
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_bspd_can_message(mcm, sc);

    //Motor controller command message
    canMessageCount++;
    canMessages[canMessageCount - 1] = get_mcm_command_can_message(mcm);
    
    //Place the can messages into the FIFO queue ---------------------------------------------------
    CanManager_send(me, CAN0_HIPRI, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)

}

IO_CAN_DATA_FRAME get_tps0_can_message(TorqueEncoder* tps) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x500;
    canMessage.data[0] = 0xFF * tps->travelPercent;
    canMessage.data[1] = 0xFF * tps->tps0_percent;
    canMessage.data[2] = Sensor_TPS0.sensorValue;
    canMessage.data[3] = Sensor_TPS0.sensorValue >> 8;
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
    canMessage.data[2] = Sensor_TPS1.sensorValue;
    canMessage.data[3] = Sensor_TPS1.sensorValue >> 8;
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
    canMessage.data[0] = (ubyte2)(wss->speed_FL + 0.5);
    canMessage.data[1] = ((ubyte2)(wss->speed_FL + 0.5)) >> 8;
    canMessage.data[2] = (ubyte2)(wss->speed_FR + 0.5);
    canMessage.data[3] = ((ubyte2)(wss->speed_FR + 0.5)) >> 8;
    canMessage.data[4] = (ubyte2)(wss->speed_RL + 0.5);
    canMessage.data[5] = ((ubyte2)(wss->speed_RL + 0.5)) >> 8;
    canMessage.data[6] = (ubyte2)(wss->speed_RR + 0.5);
    canMessage.data[7] = ((ubyte2)(wss->speed_RR + 0.5)) >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_wss_rpm1_can_message(WheelSpeeds* wss) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x504;
    canMessage.data[0] = (ubyte2)(wss->speed_FL_RPM*60.0f + 0.5);
    canMessage.data[1] = ((ubyte2)(wss->speed_FL_RPM*60.0f + 0.5)) >> 8;
    canMessage.data[2] = (ubyte2)(wss->speed_FR_RPM*60.0f + 0.5);
    canMessage.data[3] = ((ubyte2)(wss->speed_FR_RPM*60.0f + 0.5)) >> 8;
    canMessage.data[4] = (ubyte2)(wss->speed_RL_RPM*60.0f + 0.5);
    canMessage.data[5] = ((ubyte2)(wss->speed_RL_RPM*60.0f + 0.5)) >> 8;
    canMessage.data[6] = (ubyte2)(wss->speed_RR_RPM*60.0f + 0.5);
    canMessage.data[7] = ((ubyte2)(wss->speed_RR_RPM*60.0f + 0.5)) >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_wss_rpm2_can_message(WheelSpeeds* wss) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x505;
    canMessage.data[0] = (ubyte2)(wss->speed_FL_RPM_S*60.0f + 0.5);
    canMessage.data[1] = ((ubyte2)(wss->speed_FL_RPM_S*60.0f + 0.5)) >> 8;
    canMessage.data[2] = (ubyte2)(wss->speed_FR_RPM_S*60.0f + 0.5);
    canMessage.data[3] = ((ubyte2)(wss->speed_FR_RPM_S*60.0f + 0.5)) >> 8;
    canMessage.data[4] = (ubyte2)(wss->speed_RL_RPM_S*60.0f + 0.5);
    canMessage.data[5] = ((ubyte2)(wss->speed_RL_RPM_S*60.0f + 0.5)) >> 8;
    canMessage.data[6] = (ubyte2)(wss->speed_RR_RPM_S*60.0f + 0.5);
    canMessage.data[7] = ((ubyte2)(wss->speed_RR_RPM_S*60.0f + 0.5)) >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_sc_can_message(SafetyChecker* sc) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x506;
    canMessage.data[0] = sc->faults;
    canMessage.data[1] = sc->faults >> 8;
    canMessage.data[2] = sc->faults >> 16;
    canMessage.data[3] = sc->faults >> 24;
    canMessage.data[4] = sc->warnings;
    canMessage.data[5] = sc->warnings >> 8;
    canMessage.data[6] = sc->notices;
    canMessage.data[7] = sc->notices >> 8;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_lvb_can_message() {
    float4 LVBatterySOC = lv_battery_soc();
    Sensor_LVBattery.sensorValue = Sensor_LVBattery.sensorValue + 0.46;
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x507;
    canMessage.data[0] = (ubyte1)Sensor_LVBattery.sensorValue;
    canMessage.data[1] = Sensor_LVBattery.sensorValue >> 8;
    canMessage.data[2] = (sbyte1)(100 * LVBatterySOC);
    canMessage.data[3] = 0;
    canMessage.data[4] = 0;
    canMessage.data[5] = 0;
    canMessage.data[6] = 0;
    canMessage.data[7] = 0;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_mcm_regen_can_message(MotorController* mcm) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x508;
    canMessage.data[0] = mcm->regen_mode;
    canMessage.data[1] = 0;
    canMessage.data[2] = mcm->torqueMaximumDNm/10;
    canMessage.data[3] = mcm->regen_torqueLimitDNm/10;
    canMessage.data[4] = mcm->regen_torqueAtZeroPedalDNm/10;
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
    canMessage.data[2] = mcm->HVILOverride;
    canMessage.data[3] = mcm->startupStage;
    canMessage.data[4] = Sensor_RTDButton.sensorValue;
    canMessage.data[5] = mcm->lockoutStatus;
    canMessage.data[6] = mcm->inverterStatus;
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
    canMessage.data[3] = lc->lcTorque >> 8;
    canMessage.data[4] = (sbyte2)lc->slipRatio;
    canMessage.data[5] = (sbyte2)lc->slipRatio >> 8;
    canMessage.data[6] = (ubyte2)lc->lcTorque;
    canMessage.data[7] = 0;
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
    canMessage.data[0] = bms->faultFlags0;
    canMessage.data[1] = bms->faultFlags1;
    canMessage.data[2] = bms->relayState;
    canMessage.data[3] = bms->highestCellTemperature;
    canMessage.data[4] = bms->highestCellTemperature >> 8;
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
    canMessage.data[4] = sc->warnings;
    canMessage.data[5] = sc->warnings >> 8;
    canMessage.data[6] = 0;
    canMessage.data[7] = 0;
    canMessage.length = 8;
    return canMessage;
}

IO_CAN_DATA_FRAME get_bspd_can_message(MotorController* mcm, SafetyChecker* sc) {
    ubyte1 flags = sc->softBSPD_bpsHigh;
    flags |= sc->softBSPD_kwHigh << 1;
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0x511;
    canMessage.data[0] = sc->softBSPD_fault;
    canMessage.data[1] = flags;
    canMessage.data[2] = 0; //(ubyte1)mcm->kwRequestEstimate;
    canMessage.data[3] = 0; //mcm->kwRequestEstimate >> 8;
    canMessage.length = 4;
    return canMessage;
}

IO_CAN_DATA_FRAME get_mcm_command_can_message(MotorController* mcm) {
    IO_CAN_DATA_FRAME canMessage;
    canMessage.id_format = IO_CAN_STD_FRAME;
    canMessage.id = 0xC0;
    canMessage.data[0] = (ubyte1)mcm->commands_torque;
    canMessage.data[1] = mcm->commands_torque >> 8;
    canMessage.data[2] = 0;  //Speed (RPM?) - not needed - mcu should be in torque mode
    canMessage.data[3] = 0;  //Speed (RPM?) - not needed - mcu should be in torque mode
    canMessage.data[4] = mcm->commands_direction;
    canMessage.data[5] = (mcm->commands_inverter == ENABLED) ? 1 : 0; //unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
    canMessage.data[6] = (ubyte1)mcm->commands_torqueLimit;
    canMessage.data[7] = mcm->commands_torqueLimit >> 8;
    canMessage.length = 8;
    return canMessage;
}

float4 lv_battery_soc() {
    return getValueFromLUT(LV_BATT_SOC_LUT, Sensor_LVBattery.sensorValue/1000.0f/LV_BATT_S);
}