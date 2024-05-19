#ifndef _CANMANAGER_H
#define _CANMANAGER_H

#include "IO_Driver.h"
#include "IO_CAN.h"

#include "main.h"
#include "motorController.h"
#include "instrumentCluster.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "safety.h"
#include "LaunchControl.h"
#include "drs.h"
#include "timerDebug.h"
#include "torqueEncoder.h"

extern TorqueEncoder *tps;

typedef enum
{
    CAN0,
    CAN1
} CanChannel;
// CAN0: 48 messages per handle (48 read, 48 write)
// CAN1: 16 messages per handle

typedef struct _CAN_MESSAGE_SEND_BUFFER
{
    IO_CAN_DATA_FRAME canMessages0[CAN_WRITE_MESSAGE_LIMIT];
    IO_CAN_DATA_FRAME canMessages1[CAN_WRITE_MESSAGE_LIMIT];
    ubyte1 canMessageCount0;
    ubyte1 canMessageCount1;
} CAN_MESSAGE_SEND_BUFFER;


typedef struct _CanManager
{
    ubyte1 canMessageLimit;

    // These are our four FIFO queues.  All messages should come/go through one of these queues.
    // Functions shall have a CanChannel enum (see header) parameter.  Direction (send/receive is not
    // specified by this parameter.  The CAN0/CAN1 is selected based on the parameter passed in, and
    // Read/Write is selected based on the function that is being called (get/send)
    ubyte1 can0_readHandle;
    ubyte1 can0_writeHandle;

    ubyte1 can1_readHandle;
    ubyte1 can1_writeHandle;

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

    // WARNING: These values are not initialized - be careful to only access
    // pointers that have been previously assigned
} CanManager;

// Note: Sum of messageLimits must be < 128 (hardware only does 128 total messages)
void CanManager_new(CanManager *me);
IO_ErrorType CanManager_send(CanManager *me, CanChannel channel, IO_CAN_DATA_FRAME canMessages[], ubyte1 canMessageCount);

// Reads and distributes can messages to their appropriate subsystem objects so they can updates themselves
void CanManager_read(CanManager *me, CanChannel channel);
void canOutput_sendSensorMessages(CanManager *me);
// void canOutput_sendMCUControl(CanManager* me, MotorController* mcm, bool sendEvenIfNoChanges);
void canOutput_sendDebugMessage(CanManager *me);
IO_ErrorType send_a_fucking_message(CanManager *me);

ubyte1 CanManager_getReadStatus(CanManager *me, CanChannel channel);

IO_CAN_DATA_FRAME get_tps0_can_message();
IO_CAN_DATA_FRAME get_tps1_can_message();
IO_CAN_DATA_FRAME get_bps0_can_message();
IO_CAN_DATA_FRAME get_bps1_can_message();
IO_CAN_DATA_FRAME get_wss_can_message();
IO_CAN_DATA_FRAME get_wss_rpm1_can_message();
IO_CAN_DATA_FRAME get_wss_rpm2_can_message();
IO_CAN_DATA_FRAME get_sc_can_message();
IO_CAN_DATA_FRAME get_lvb_can_message();
IO_CAN_DATA_FRAME get_mcm_regen_can_message();
IO_CAN_DATA_FRAME get_mcm_rtd_can_message();
IO_CAN_DATA_FRAME get_mcm_gsr_can_message();
IO_CAN_DATA_FRAME get_lc_can_message();
IO_CAN_DATA_FRAME get_drs_can_message();
IO_CAN_DATA_FRAME get_bms_loopback_can_message();
IO_CAN_DATA_FRAME get_mcm_power_can_message();
IO_CAN_DATA_FRAME get_bspd_can_message();
IO_CAN_DATA_FRAME get_mcm_pl_can_message();
IO_CAN_DATA_FRAME get_mcm_command_can_message();
IO_CAN_DATA_FRAME get_timer_debug_can_message();
float4 lv_battery_soc();

#endif // _CANMANAGER_H is defined