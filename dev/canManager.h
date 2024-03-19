#ifndef _CANMANAGER_H
#define _CANMANAGER_H

#include "IO_Driver.h"
#include "IO_CAN.h"

#include "motorController.h"
#include "instrumentCluster.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "safety.h"
#include "LaunchControl.h"
#include "drs.h"

typedef enum
{
    CAN1_HIPRI,
    CAN2_LOPRI
} CanChannel;
//CAN1: 48 messages per handle (48 read, 48 write)
//CAN2: 16 messages per handle

//Keep track of CAN message IDs, their data, and when they were last sent.
typedef struct _CanMessageNode
{
    ubyte4 timeBetweenMessages_Min;
    ubyte4 timeBetweenMessages_Max;
    ubyte4 lastMessage_timeStamp;
    ubyte1 data[8];
    bool required;
} CanMessageNode;

typedef struct _CanManager {
    CanMessageNode* canMessageHistory[0x7FF];

    ubyte1 canMessageLimit;
    
    //These are our four FIFO queues.  All messages should come/go through one of these queues.
    //Functions shall have a CanChannel enum (see header) parameter.  Direction (send/receive is not
    //specified by this parameter.  The CAN1/CAN2 is selected based on the parameter passed in, and 
    //Read/Write is selected based on the function that is being called (get/send)
    ubyte1 CAN1_busSpeed;
    ubyte1 CAN1_readHandle;
    ubyte1 CAN1_read_messageLimit;
    ubyte1 CAN1_writeHandle;
    ubyte1 CAN1_write_messageLimit;

    ubyte1 CAN2_busSpeed;
    ubyte1 CAN2_readHandle;
    ubyte1 CAN2_read_messageLimit;
    ubyte1 CAN2_writeHandle;
    ubyte1 CAN2_write_messageLimit;
    
    IO_ErrorType ioErr_CAN1_Init;
    IO_ErrorType ioErr_CAN2_Init;

    IO_ErrorType ioErr_CAN1_fifoInit_R;
    IO_ErrorType ioErr_CAN1_fifoInit_W;
    IO_ErrorType ioErr_CAN2_fifoInit_R;
    IO_ErrorType ioErr_CAN2_fifoInit_W;

    IO_ErrorType ioErr_CAN1_read;
    IO_ErrorType ioErr_CAN1_write;
    IO_ErrorType ioErr_CAN2_read;
    IO_ErrorType ioErr_CAN2_write;

    ubyte4 sendDelayus;

    //WARNING: These values are not initialized - be careful to only access
    //pointers that have been previously assigned
} CanManager;

//Note: Sum of messageLimits must be < 128 (hardware only does 128 total messages)
CanManager *CanManager_new(ubyte2 CAN1_busSpeed, ubyte1 CAN1_read_messageLimit, ubyte1 CAN1_write_messageLimit, ubyte2 CAN2_busSpeed, ubyte1 CAN2_read_messageLimit, ubyte1 CAN2_write_messageLimit, ubyte4 defaultSendDelayus);
IO_ErrorType CanManager_send(CanManager *me, CanChannel channel, IO_CAN_DATA_FRAME canMessages[], ubyte1 canMessageCount);

//Reads and distributes can messages to their appropriate subsystem objects so they can updates themselves
void CanManager_read(CanManager *me, CanChannel channel, MotorController *mcm, InstrumentCluster *ic, BatteryManagementSystem *bms, SafetyChecker *sc);
CanMessageNode *CAN_msg_insert(CanMessageNode **messageHistoryArray, ubyte4 messageID, ubyte1 messageData[8], ubyte4 minTime, ubyte4 maxTime, bool req);
void canOutput_sendSensorMessages(CanManager *me);
//void canOutput_sendMCUControl(CanManager* me, MotorController* mcm, bool sendEvenIfNoChanges);
void canOutput_sendDebugMessage(CanManager *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, InstrumentCluster *ic, BatteryManagementSystem *bms, WheelSpeeds *wss, SafetyChecker *sc, LaunchControl *lc, DRS *drs);

ubyte1 CanManager_getReadStatus(CanManager *me, CanChannel channel);

IO_CAN_DATA_FRAME get_tps0_can_message(TorqueEncoder* tps);
IO_CAN_DATA_FRAME get_tps1_can_message(TorqueEncoder* tps);
IO_CAN_DATA_FRAME get_bps0_can_message(BrakePressureSensor* bps);
IO_CAN_DATA_FRAME get_bps1_can_message(BrakePressureSensor* bps);
IO_CAN_DATA_FRAME get_wss_can_message(WheelSpeeds* wss);
IO_CAN_DATA_FRAME get_wss_rpm1_can_message(WheelSpeeds* wss);
IO_CAN_DATA_FRAME get_wss_rpm2_can_message(WheelSpeeds* wss);
IO_CAN_DATA_FRAME get_sc_can_message(SafetyChecker* sc);
IO_CAN_DATA_FRAME get_lvb_can_message();
IO_CAN_DATA_FRAME get_mcm_regen_can_message(MotorController* mcm);
IO_CAN_DATA_FRAME get_mcm_rtd_can_message(MotorController* mcm);
IO_CAN_DATA_FRAME get_mcm_gsr_can_message(MotorController* mcm);
IO_CAN_DATA_FRAME get_lc_can_message(LaunchControl* lc);
IO_CAN_DATA_FRAME get_drs_can_message(DRS* drs);
IO_CAN_DATA_FRAME get_bms_loopback_can_message(BatteryManagementSystem* bms);
IO_CAN_DATA_FRAME get_mcm_power_can_message(MotorController* mcm, SafetyChecker* sc);
IO_CAN_DATA_FRAME get_bspd_can_message(MotorController* mcm, SafetyChecker* sc);
IO_CAN_DATA_FRAME get_mcm_command_can_message(MotorController* mcm);

#endif // _CANMANAGER_H is defined