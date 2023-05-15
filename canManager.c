
#include <stdlib.h> //malloc

#include "IO_Driver.h" 
#include "IO_CAN.h"
#include "IO_RTC.h"

#include "mathFunctions.h"
#include "sensors.h"
#include "canManager.h"
#include "avlTree.h"
#include "bms.h"
#include "safety.h"
#include "wheelSpeeds.h"
#include "sensorCalculations.h"
#include "daqSensors.h"


struct _CanManager {
    //AVLNode* incomingTree;
    //AVLNode* outgoingTree;

    ubyte1 canMessageLimit;
    
    //These are our four FIFO queues.  All messages should come/go through one of these queues.
    //Functions shall have a CanChannel enum (see header) parameter.  Direction (send/receive is not
    //specified by this parameter.  The CAN0/CAN1 is selected based on the parameter passed in, and 
    //Read/Write is selected based on the function that is being called (get/send)
    ubyte1 can0_busSpeed;
    ubyte1 can0_readHandle;
    ubyte1 can0_read_messageLimit;
    ubyte1 can0_writeHandle;
    ubyte1 can0_write_messageLimit;

    ubyte1 can1_busSpeed;
    ubyte1 can1_readHandle;
    ubyte1 can1_read_messageLimit;
    ubyte1 can1_writeHandle;
    ubyte1 can1_write_messageLimit;
    
    IO_ErrorType ioErr_can0_Init;
    IO_ErrorType ioErr_can1_Init;

    IO_ErrorType ioErr_can0_fifoInit_R;
    IO_ErrorType ioErr_can0_fifoInit_W;
    IO_ErrorType ioErr_can1_fifoInit_R;
    IO_ErrorType ioErr_can1_fifoInit_W;

    IO_ErrorType ioErr_can0_read;
    IO_ErrorType ioErr_can0_write;
    IO_ErrorType ioErr_can1_read;
    IO_ErrorType ioErr_can1_write;

    ubyte4 sendDelayus;


    //WARNING: These values are not initialized - be careful to only access
    //pointers that have been previously assigned
    //AVLNode* canMessageHistory[0x7FF];
    AVLNode* canMessageHistory[0x7FF];
    //Needs to change to the extended CAN limit
};

//Keep track of CAN message IDs, their data, and when they were last sent.
/*
struct _CanMessageNode
{
    IO_CAN_DATA_FRAME canMessage;
    ubyte4 timeBetweenMessages_Min;
    ubyte4 timeBetweenMessages_Max;
    ubyte1 lastMessage_data[8];
    ubyte4 lastMessage_timeStamp;
    canHistoryNode* left;
    canHistoryNode* right;
};
*/

CanManager* CanManager_new(ubyte2 can0_busSpeed, ubyte1 can0_read_messageLimit, ubyte1 can0_write_messageLimit
                         , ubyte2 can1_busSpeed, ubyte1 can1_read_messageLimit, ubyte1 can1_write_messageLimit
                         , ubyte4 defaultSendDelayus) //ubyte4 defaultMinSendDelay, ubyte4 defaultMaxSendDelay)
{
    CanManager* me = (CanManager*)malloc(sizeof(struct _CanManager));

    //me->sm = serialMan;
    //SerialManager_send(me->sm, "CanManager's reference to SerialManager was created.\n");
    
    //create can history data structure (AVL tree?)
    //me->incomingTree = NULL;
    //me->outgoingTree = NULL;
    //Need to change to extended CAN limit
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
    IO_CAN_ConfigFIFO(&me->can1_readHandle, IO_CAN_CHANNEL_1, can1_read_messageLimit, IO_CAN_MSG_READ, IO_CAN_STD_FRAME, 0, 0); //Only change for CAN1 Read for IMU
    IO_CAN_ConfigFIFO(&me->can1_writeHandle, IO_CAN_CHANNEL_1, can1_write_messageLimit, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);

    //Assume read/write at error state until used
    me->ioErr_can0_read = IO_E_CAN_BUS_OFF;
    me->ioErr_can0_write = IO_E_CAN_BUS_OFF;
    me->ioErr_can1_read = IO_E_CAN_BUS_OFF;
    me->ioErr_can1_write = IO_E_CAN_BUS_OFF;

    //-------------------------------------------------------------------
    //Define default messages
    //-------------------------------------------------------------------
    //AVLNode* insertedMessage;
    //insertedMessage = AVL_insert(me->canMessageHistory, 0x0C0, 0, 50000, 125000, TRUE); //MCM command message


    ubyte2 messageID;
    //Outgoing ----------------------------
    messageID = 0x184;  //Inverter FL 1 Command Message
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0; // us (microseconds)
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 8000; 
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x185;  //Inverter FR 2 Command Message
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 8000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x188;  //Inverter RL 1 Command Message
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 8000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x189;  //Inverter RR 2 Command Message
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 8000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    for (messageID = 0x500; messageID <= 0x515; messageID++)
    {
        me->canMessageHistory[messageID]->timeBetweenMessages_Min = 50000;
        me->canMessageHistory[messageID]->timeBetweenMessages_Max = 250000;
        me->canMessageHistory[messageID]->required = TRUE;
        for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
        IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);
    }

    //Incoming ----------------------------
    /* Currently unused for any timeout errors on VCU side
    //Look into this since the AMKs did timeout
    messageID = 0x283;  //Inverter1FL 1
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0; 
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 500000; 
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x285;  //Inverter1FL 2
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 500000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x284;   //Inverter2FR 1
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 500000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x286;   //Inverter2FR 2
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 500000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x287;   //Inverter3RL 1
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 500000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x289;   //Inverter3RL 2
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 500000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x288;   //Inverter3RR 1
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 500000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x290;   //Inverter3RR 2
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 500000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    //////////////////////////////////////////////////////////////////////////////////

    messageID = 0x623;  //BMS faults
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 5000000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x623;  //BMS faults
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 5000000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);

    messageID = 0x629;  //BMS details
    me->canMessageHistory[messageID]->timeBetweenMessages_Min = 0;
    me->canMessageHistory[messageID]->timeBetweenMessages_Max = 1000000;
    me->canMessageHistory[messageID]->required = TRUE;
    for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID]->data[i] = 0; }
    IO_RTC_StartTime(&me->canMessageHistory[messageID]->lastMessage_timeStamp);
    */

    return me;
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
    //SerialManager_send(me->sm, "Do you even send?\n");
    bool sendSerialDebug = FALSE;
    ubyte2 serialMessageID = 0xC0;
    bool sendMessage = FALSE;
    ubyte1 messagesToSendCount = 0;
    IO_CAN_DATA_FRAME messagesToSend[canMessageCount];//[channel == CAN0_HIPRI ? me->can0_write_messageLimit : me->can1_write_messageLimit];

    //----------------------------------------------------------------------------
    // Check if message exists in outgoing message history tree
    //----------------------------------------------------------------------------
    AVLNode* lastMessage;  //replace with me->canMessageHistory[ID]
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
        // Check if this message exists in the array
        //----------------------------------------------------------------------------
        firstTimeMessage = (me->canMessageHistory[outboundMessageID] == 0);  
        if (firstTimeMessage)
        {
            me->canMessageHistory[outboundMessageID]->timeBetweenMessages_Min = 25000;
            me->canMessageHistory[outboundMessageID]->timeBetweenMessages_Max = 125000;
            me->canMessageHistory[outboundMessageID]->required = TRUE;
            for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[outboundMessageID]->data[i] = 0; }
            //IO_RTC_StartTime(&me->canMessageHistory[outboundMessageID]->lastMessage_timeStamp);
            me->canMessageHistory[outboundMessageID]->lastMessage_timeStamp = 0;
        }

        //----------------------------------------------------------------------------
        // Check if data has changed since last time message was sent
        //----------------------------------------------------------------------------
        //Check each data byte in the data array
        for (ubyte1 dataPosition = 0; dataPosition < 8; dataPosition++)
        {
            ubyte1 oldData = lastMessage->data[dataPosition];
            ubyte1 newData = canMessages[messagePosition].data[dataPosition];
            //if any data byte is changed, then probably want to send the message
            if (oldData == newData)
            {
                //data has not changed.  No action required (DO NOT SET)
                //dataChanged = FALSE;
            }
            else
            {
                dataChanged = TRUE; //ONLY MODIFY IF CHANGED
            }
        }//end checking each byte in message

        //----------------------------------------------------------------------------
        // Check if time has exceeded
        //----------------------------------------------------------------------------
        minTimeExceeded = ((IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp) >= lastMessage->timeBetweenMessages_Min));
        maxTimeExceeded = ((IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp) >= 50000));//lastMessage->timeBetweenMessages_Max));
        
        //----------------------------------------------------------------------------
        // If any criteria were exceeded, send the message out
        //----------------------------------------------------------------------------
        if (  (firstTimeMessage)
           || (dataChanged && minTimeExceeded)
           || (!dataChanged && maxTimeExceeded)
           )
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
                //SerialManager_send(me->sm, "This message did not need to be sent.\n");
            }
        }
    } //end of loop for each message in outgoing messages

    IO_UART_Task();
    //----------------------------------------------------------------------------
    // If there are messages to send
    //----------------------------------------------------------------------------
    IO_ErrorType sendResult = IO_E_OK;
    if (messagesToSendCount > 0)
    {
        //Send the messages to send to the appropriate FIFO queue
        sendResult = IO_CAN_WriteFIFO((channel == CAN0_HIPRI) ? me->can0_writeHandle : me->can1_writeHandle, messagesToSend, messagesToSendCount);
        *((channel == CAN0_HIPRI) ? &me->ioErr_can0_write : &me->ioErr_can1_write) = sendResult;

        //Update the outgoing message tree with message sent timestamps
        if ((channel == CAN0_HIPRI ? me->ioErr_can0_write : me->ioErr_can1_write) == IO_E_OK)
        {
            //Loop through the messages that we sent...
            ///////////AVLNode* messageToUpdate;
            for (messagePosition = 0; messagePosition < messagesToSendCount; messagePosition++)
            {
                //...find the message ID in the outgoing message tree again (big inefficiency here)...
                //////////messageToUpdate = AVL_find(me->outgoingTree, messagesToSend[messagePosition].id);

                //and update the message sent timestamp
                /////////////IO_RTC_GetTimeUS(messageToUpdate->lastMessage_timeStamp); //Update the timestamp for when the message was last sent
                //IO_RTC_GetTimeUS(me->canMessageHistory[messagesToSend[messagePosition].id]->lastMessage_timeStamp);
                IO_RTC_StartTime(&me->canMessageHistory[messagesToSend[messagePosition].id]->lastMessage_timeStamp);
            }
        }
    }
    return sendResult;
}

/*
//Helper functions
ubyte4 CanManager_timeSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage)  //Overflows/resets at 74 min
bool CanManager_enoughTimeSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage) // timesincelast > timeBetweenMessages_Min
bool CanManager_dataChangedSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage) //bitwise comparison for all data bytes
*/


/*****************************************************************************
* read
****************************************************************************/
void CanManager_read(CanManager *me, CanChannel channel, InstrumentCluster *ic, BatteryManagementSystem *bms, SafetyChecker *sc, _DAQSensors *d1, _DriveInverter *inv1, _DriveInverter *inv2)
{
    IO_CAN_DATA_FRAME canMessages[(channel == CAN0_HIPRI ? me->can0_read_messageLimit : me->can1_read_messageLimit)];
    ubyte1 canMessageCount;  //FIFO queue only holds 128 messages max

    //Read messages from hi/lopri channel 
    *(channel == CAN0_HIPRI ? &me->ioErr_can0_read : &me->ioErr_can1_read) =
    IO_CAN_ReadFIFO((channel == CAN0_HIPRI ? me->can0_readHandle : me->can1_readHandle)
                    , canMessages
                    , (channel == CAN0_HIPRI ? me->can0_read_messageLimit : me->can1_read_messageLimit)
                    , &canMessageCount);

    //Determine message type based on ID
    for (int currMessage = 0; currMessage < canMessageCount; currMessage++)
    {
        // Seperate based on CAN message
        switch (canMessages[currMessage].id)
        {
        //-------------------------------------------------------------------------
        //Inverters (Inverter FL and FR are together CAN0 and Inverter RL and RR are together CAN1) 
        //This is to ensure better debug between the two busses
        //-------------------------------------------------------------------------
        case 0x283:
        case 0x285:
            //Inverter FL 1 (CAN0)
            DI_parseCanMessage(inv1, &canMessages[currMessage]);
            break;
        case 0x284:
        case 0x286:
            //Inverter FR 1 (CAN0)
            DI_parseCanMessage(inv2, &canMessages[currMessage]);
            break;
        case 0x287:
        case 0x289:
            //Inverter RL 1 (CAN1)
            DI_parseCanMessage(inv1, &canMessages[currMessage]);
            break;
        case 0x288:
        case 0x290:
            //Inverter RR 1 (CAN1)
            DI_parseCanMessage(inv2, &canMessages[currMessage]);
            break;
        
        //-------------------------------------------------------------------------
        //IMU from DAQ
        //-------------------------------------------------------------------------
        case 0x400:
            DAQ_parseCanMessage(d1, &canMessages[currMessage]);
            break;
        case 0x402:
            DAQ_parseCanMessage(d1, &canMessages[currMessage]);
            break;

        //-------------------------------------------------------------------------
        //BMS
        //-------------------------------------------------------------------------
        case 0x600:
        case 0x602: //Faults
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
        case 0x622: //Cell Voltage Summary
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;
        case 0x623: //Cell Temperature Summary
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;
        case 0x624:
        //1st Module
        case 0x630:
        case 0x631:
        case 0x632:
        //2nd Module
        case 0x633:
        case 0x634:
        case 0x635:
        //3rd Module
        case 0x636:
        case 0x637:
        case 0x638:
        //4th Module
        case 0x639:
        case 0x63A:
        case 0x63B:
        //5th Module
        case 0x63C:
        case 0x63D:
        case 0x63E:
        //6th Module
        case 0x63F:
        case 0x640:
        case 0x641:

        case 0x629:
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;

        case 0x702:
            //Need Updating: IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;
        case 0x703:
            //Need Updating: IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;
        case 0x704:
            //Need Updating: IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;

            
            
        //-------------------------------------------------------------------------
        //VCU Debug Control
        //-------------------------------------------------------------------------
        case 0x5FF:
            SafetyChecker_parseCanMessage(sc, &canMessages[currMessage]);
            //MCM_parseCanMessage(mcm, &canMessages[currMessage]);
            break;
            //default:
        }

        //Parse IMU here
    }
    //IO_CAN_WriteFIFO(me->can1_writeHandle, canMessages, messagesReceived);
    //IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, canMessages);
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
void canOutput_sendDebugMessage0(CanManager* me, TorqueEncoder* tps, BrakePressureSensor* bps, InstrumentCluster* ic, BatteryManagementSystem* bms, WheelSpeeds* wss, SafetyChecker* sc, _DriveInverter *inv1, _DriveInverter *inv2)
{
    IO_CAN_DATA_FRAME canMessages[me->can0_write_messageLimit];
    ubyte1 errorCount;
    float4 tempPedalPercent;   //Pedal percent float (a decimal between 0 and 1
    ubyte1 tps0Percent;  //Pedal percent int   (a number from 0 to 100)
    ubyte1 tps1Percent;
    ubyte2 canMessageCount = 0;
    ubyte2 canMessageID = 0x500;
    ubyte1 byteNum;

    TorqueEncoder_getIndividualSensorPercent(tps, 0, &tempPedalPercent); //borrow the pedal percent variable
    tps0Percent = 0xFF * tempPedalPercent;
    TorqueEncoder_getIndividualSensorPercent(tps, 1, &tempPedalPercent);
    tps1Percent = 0xFF * (tempPedalPercent);
    //tps1Percent = 0xFF * (1 - tempPedalPercent);  //OLD: flipped over pedal percent (this value for display in CAN only)

    TorqueEncoder_getPedalTravel(tps, &errorCount, &tempPedalPercent); //getThrottlePercent(TRUE, &errorCount);
    ubyte1 throttlePercent = 0xFF * tempPedalPercent;

    BrakePressureSensor_getPedalTravel(bps, &errorCount, &tempPedalPercent); //getThrottlePercent(TRUE, &errorCount);
    ubyte1 brakePercent = 0xFF * tempPedalPercent;

    //500: TPS 0
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1; //500
    canMessages[canMessageCount - 1].data[byteNum++] = throttlePercent;
    canMessages[canMessageCount - 1].data[byteNum++] = tps0Percent;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_TPS0.sensorValue; // tps->tps0_value;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_TPS0.sensorValue >> 8; //tps->tps0_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMax >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //TPS 1
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = throttlePercent;
    canMessages[canMessageCount - 1].data[byteNum++] = tps1Percent;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_value;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMax >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //BPS0
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = brakePercent; //This should be bps0Percent, but for now bps0Percent = brakePercent
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_value;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMax >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //WSS mm/s output
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, FL) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeed(wss, FL) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, FR) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeed(wss, FR) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, RL) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeed(wss, RL) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, RR) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeed(wss, RR) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //WSS RPM non-interpolated output
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, FALSE) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, FALSE) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FR, FALSE) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FR, FALSE) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RL, FALSE) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RL, FALSE) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, FALSE) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, FALSE) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    /*
    //TEMP: WSS3
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;  //505
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RL.sensorValue;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RL.sensorValue >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RL.sensorValue >> 16;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RL.sensorValue >> 24;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RR.sensorValue;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RR.sensorValue >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RR.sensorValue >> 16;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RR.sensorValue >> 24;
    canMessages[canMessageCount - 1].length = byteNum;
    */
    /*
    //TEMP: FRONT WSS PIN DEBUG
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;  //505
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.ioErr_signalInit;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.ioErr_signalInit >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.ioErr_signalGet;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.ioErr_signalGet >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FR.ioErr_signalInit;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FR.ioErr_signalInit >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FR.ioErr_signalGet;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FR.ioErr_signalGet >> 8;
    canMessages[canMessageCount - 1].length = byteNum;
    */

    //WSS RPM interpolated output
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FR, TRUE) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FR, TRUE) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RL, TRUE) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RL, TRUE) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, TRUE) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = ((ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, TRUE) + 0.5)) >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //506: Safety Checker
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getFaults(sc);
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getFaults(sc) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getFaults(sc) >> 16;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getFaults(sc) >> 24;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getWarnings(sc);
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getWarnings(sc) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getNotices(sc);
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getNotices(sc) >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //12v battery
    float4 LVBatterySOC = 0;
    if (Sensor_LVBattery.sensorValue < 12730)
        LVBatterySOC = .0 + .1 * getPercent(Sensor_LVBattery.sensorValue, 9200, 12730, FALSE);
    else if (Sensor_LVBattery.sensorValue < 12866)
        LVBatterySOC = .1 + .1 * getPercent(Sensor_LVBattery.sensorValue, 12730, 12866, FALSE);
    else if (Sensor_LVBattery.sensorValue < 12996)
        LVBatterySOC = .2 + .1 * getPercent(Sensor_LVBattery.sensorValue, 12866, 12996, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13104)
        LVBatterySOC = .3 + .1 * getPercent(Sensor_LVBattery.sensorValue, 12996, 13104, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13116)
        LVBatterySOC = .4 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13104, 13116, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13130)
        LVBatterySOC = .5 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13116, 13130, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13160)
        LVBatterySOC = .6 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13130, 13160, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13270)
        LVBatterySOC = .7 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13160, 13270, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13300)
        LVBatterySOC = .8 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13270, 13300, FALSE);
    else //if (Sensor_LVBattery.sensorValue < 14340)
        LVBatterySOC = .9 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13300, 14340, FALSE);

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte1)Sensor_LVBattery.sensorValue;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_LVBattery.sensorValue >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (sbyte1)(100 * LVBatterySOC);
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //508: Regen settings (Need to be updated for the future)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenMode(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getMaxTorqueDNm(mcm)/10;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenTorqueLimitDNm(mcm)/10;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenTorqueAtZeroPedalDNm(mcm)/10;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenAPPSForMaxCoastingZeroToFF(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenBPSForMaxRegenZeroToFF(mcm);
    canMessages[canMessageCount - 1].length = byteNum;

    //509: MCM RTD Status
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_HVILTerminationSense.sensorValue;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_HVILTerminationSense.sensorValue >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50A: Torque Vectoring Loopback (Future Needs)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50B: AMK VCU Debug
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = inv1->startUpStage;
    canMessages[canMessageCount - 1].data[byteNum++] = inv2->startUpStage;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

/* Testing
    ubyte2 longFric[30][30] = {
    { 0, 50, 100, 150, 200, 250}, 
    { 2,  1,   2,   3,   4,   5}, 
    { 4,  6,   7,   8,   9,  10},
    { 6, 11,  12,  13,  14,  15},
    { 8, 16,  17,  18,  19,  20},
    {10, 21,  22,  23,  24,  25},
    {12, 26,  27,  28,  29,  30}
};

ubyte2 i = 0, j = 0;
ubyte2 lookedupRow = 0;
ubyte2 lookedupColumn = 0;
ubyte2 rowFricLookup = LVBatterySOC;
ubyte2 columnFricLookup = LVBatterySOC;

// Find the row and column indices for interpolation
for(i = 0; i < 30; i++) { 
    if(longFric[i][0] > rowFricLookup) {
         lookedupRow = i-1;
         break;
    }
}

for(j = 0; j < 30; j++) {
    if (longFric[0][j] > columnFricLookup) {
        lookedupColumn = j-1;
        break;
    }
}

// Bilinear interpolation
ubyte2 x = rowFricLookup;
ubyte2 x1 = longFric[lookedupRow][0];
ubyte2 x2 = longFric[lookedupRow+1][0];
ubyte2 y = columnFricLookup;
ubyte2 y1 = longFric[0][lookedupColumn];
ubyte2 y2 = longFric[0][lookedupColumn+1];
ubyte2 Q11 = longFric[lookedupRow][lookedupColumn];
ubyte2 Q12 = longFric[lookedupRow][lookedupColumn+1];
ubyte2 Q21 = longFric[lookedupRow+1][lookedupColumn];
ubyte2 Q22 = longFric[i][j];
ubyte2 R1 = Q11*(x2-rowFricLookup)/(x2-x1) + Q21*(rowFricLookup-x1)/(x2-x1);
ubyte2 R2 = Q12*(x2-rowFricLookup)/(x2-x1) + Q22*(rowFricLookup-x1)/(x2-x1);
ubyte2 P = R1*(y2-columnFricLookup)/(y2-y1) + R2*(columnFricLookup-y1)/(y2-y1);
*/

    //50C: SAS (Steering Angle Sensor)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = steering_degrees();
    canMessages[canMessageCount - 1].data[byteNum++] = steering_degrees() >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //P
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50D: BPS1 (TEMPORARY ADDRESS)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = brakePercent; //This should be bps0Percent, but for now bps0Percent = brakePercent
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_value;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_calibMax >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    
    //50E: BMS Loopback Test
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = BMS_getFaultFlags0(bms);
    canMessages[canMessageCount - 1].data[byteNum++] = BMS_getFaultFlags1(bms);
    canMessages[canMessageCount - 1].data[byteNum++] = BMS_getRelayState(bms);
    canMessages[canMessageCount - 1].data[byteNum++] = BMS_getHighestCellTemp_d_degC(bms);
    canMessages[canMessageCount - 1].data[byteNum++] = (BMS_getHighestCellTemp_d_degC(bms) >> 8);
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50F: Power Debug (Need adaption in the future)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getPower(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //(MCM_getPower(mcm) >> 8);
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //(MCM_getPower(mcm) >> 16);
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //(MCM_getPower(mcm) >> 24);
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getWarnings(sc);
    canMessages[canMessageCount - 1].data[byteNum++] = (SafetyChecker_getWarnings(sc) >> 8);
    canMessages[canMessageCount - 1].data[byteNum++] = (SafetyChecker_getWarnings(sc) >> 16);
    canMessages[canMessageCount - 1].data[byteNum++] = (SafetyChecker_getWarnings(sc) >> 24);
    canMessages[canMessageCount - 1].length = byteNum;

    //511: SoftBSPD
    // ubyte1 flags = sc->softBSPD_bpsHigh;
    // flags |= sc->softBSPD_kwHigh << 1;
    // canMessageCount++;
    // byteNum = 0;
    // canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    // canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    // canMessages[canMessageCount - 1].data[byteNum++] = sc->softBSPD_fault;
    // canMessages[canMessageCount - 1].data[byteNum++] = flags;
    // canMessages[canMessageCount - 1].data[byteNum++] = (ubyte1)mcm->kwRequestEstimate;
    // canMessages[canMessageCount - 1].data[byteNum++] = mcm->kwRequestEstimate >> 8;
    // canMessages[canMessageCount - 1].length = byteNum;

    //Inverter 1 FL Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x184;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((inv1->AMK_bInverterOn << 0) | (inv1->AMK_bDcOn << 1) | (inv1->AMK_bEnable << 2) | (inv1->AMK_bErrorReset << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[2] = inv1->AMK_TorqueSetpoint;
    canMessages[canMessageCount - 1].data[3] = inv1->AMK_TorqueSetpoint >> 8;
    canMessages[canMessageCount - 1].data[4] = inv1->AMK_TorqueLimitPositiv;
    canMessages[canMessageCount - 1].data[5] = inv1->AMK_TorqueLimitPositiv >> 8;
    canMessages[canMessageCount - 1].data[6] = inv1->AMK_TorqueLimitNegativ;
    canMessages[canMessageCount - 1].data[7] = inv1->AMK_TorqueLimitNegativ >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //Inverter 2 FR Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x185;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((inv2->AMK_bInverterOn << 0) | (inv2->AMK_bDcOn << 1) | (inv2->AMK_bEnable << 2) | (inv2->AMK_bErrorReset << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[2] = inv2->AMK_TorqueSetpoint;
    canMessages[canMessageCount - 1].data[3] = inv2->AMK_TorqueSetpoint >> 8;
    canMessages[canMessageCount - 1].data[4] = inv2->AMK_TorqueLimitPositiv;
    canMessages[canMessageCount - 1].data[5] = inv2->AMK_TorqueLimitPositiv >> 8;
    canMessages[canMessageCount - 1].data[6] = inv2->AMK_TorqueLimitNegativ;
    canMessages[canMessageCount - 1].data[7] = inv2->AMK_TorqueLimitNegativ >> 8;
    canMessages[canMessageCount - 1].length = 8;
    //----------------------------------------------------------------------------
    //Additional sensors
    //----------------------------------------------------------------------------

    //Place the can messsages into the FIFO queue ---------------------------------------------------
    //IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    CanManager_send(me, CAN0_HIPRI, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    //IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, canMessageCount);  

}

void canOutput_sendDebugMessage1(CanManager *me, TorqueEncoder *tps, BrakePressureSensor *bps, InstrumentCluster *ic, BatteryManagementSystem *bms, WheelSpeeds *wss, SafetyChecker *sc, _DAQSensors *d1, _DriveInverter *inv1, _DriveInverter *inv2)
{
    IO_CAN_DATA_FRAME canMessages[me->can1_write_messageLimit]; 
    ubyte1 errorCount;
    float4 tempPedalPercent;   //Pedal percent float (a decimal between 0 and 1
    ubyte1 tps0Percent;  //Pedal percent int   (a number from 0 to 100)
    ubyte1 tps1Percent;
    ubyte2 canMessageCount = 0;
    ubyte2 canMessageID = 0x500;
    ubyte1 byteNum;

    //Inverter 3 RL Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x188;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((inv1->AMK_bInverterOn << 0) | (inv1->AMK_bDcOn << 1) | (inv1->AMK_bEnable << 2) | (inv1->AMK_bErrorReset << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[2] = inv1->AMK_TorqueSetpoint;
    canMessages[canMessageCount - 1].data[3] = inv1->AMK_TorqueSetpoint >> 8;
    canMessages[canMessageCount - 1].data[4] = inv1->AMK_TorqueLimitPositiv;
    canMessages[canMessageCount - 1].data[5] = inv1->AMK_TorqueLimitPositiv >> 8;
    canMessages[canMessageCount - 1].data[6] = inv1->AMK_TorqueLimitNegativ;
    canMessages[canMessageCount - 1].data[7] = inv1->AMK_TorqueLimitNegativ >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //Inverter 4 RR Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x189;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((inv2->AMK_bInverterOn << 0) | (inv2->AMK_bDcOn << 1) | (inv2->AMK_bEnable << 2) | (inv2->AMK_bErrorReset << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[2] = inv2->AMK_TorqueSetpoint;
    canMessages[canMessageCount - 1].data[3] = inv2->AMK_TorqueSetpoint >> 8;
    canMessages[canMessageCount - 1].data[4] = inv2->AMK_TorqueLimitPositiv;
    canMessages[canMessageCount - 1].data[5] = inv2->AMK_TorqueLimitPositiv >> 8;
    canMessages[canMessageCount - 1].data[6] = inv2->AMK_TorqueLimitNegativ;
    canMessages[canMessageCount - 1].data[7] = inv2->AMK_TorqueLimitNegativ >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //50B: AMK VCU Debug (3/4 Inverters)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x50B;
    canMessages[canMessageCount - 1].data[byteNum++] = inv1->startUpStage;
    canMessages[canMessageCount - 1].data[byteNum++] = inv2->startUpStage;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //Place the can messsages into the FIFO queue ---------------------------------------------------
    //IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    CanManager_send(me, CAN1_LOPRI, canMessages, canMessageCount);  //Send messages to CAN1
    //IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, canMessageCount);  

}