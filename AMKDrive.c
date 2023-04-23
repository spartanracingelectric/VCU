/*****************************************************************************
 * AMKDrive.c - Drive Inverter (DI)
 ******************************************************************************
 * Stores data from / commands going to AMK drive inverters.
 ****************************************************************************/

#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "IO_Driver.h"
#include "IO_CAN.h"

#include "AMKDrive.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "sensorCalculations.h"
#include "readyToDriveSound.h"

extern Sensor Sensor_RTDButton;
ubyte4 timestamp_Precharge = 0;

_DriveInverter* AmkDriver_new(DI_Location_Address location_address)
{
    _DriveInverter* me = (_DriveInverter*)malloc(sizeof(_DriveInverter));
 
        me->location_address = location_address;

        me->canIdOutgoing = DI_BASE_CAN_ID_OUTGOING + me->location_address;
        me->canIdIncoming = DI_BASE_CAN_ID_INCOMING + me->location_address;

        int startUpStage = 0;

        // Setpoints (commands, outgoing: 0x183 + offset)
        me->AMK_bInverterOn = FALSE;
        me->AMK_bDcOn = FALSE;
        me->AMK_bEnable = FALSE;
        me->AMK_bErrorReset = FALSE;
        me->AMK_TorqueSetpoint = 0;
        me->AMK_TorqueLimitPositiv = 0;
        me->AMK_TorqueLimitNegativ = 0;

        // Actual Values 1 (incoming: 0x282 + offset)
        me->AMK_bSystemReady = FALSE;
        me->AMK_bError = FALSE;
        me->AMK_bWarn = FALSE;
        me->AMK_bQuitDcOn = FALSE;
        me->AMK_bDcOnVal = FALSE;
        me->AMK_bQuitInverterOnVal = FALSE;
        me->AMK_bInverterOnVal = FALSE;
        me->AMK_bDerating = FALSE;
        me->AMK_ActualVelocity = 0;
        me->AMK_TorqueCurrent = 0;
        me->AMK_MagnetizingCurrent = 0;

        // Actual Values 2 (incoming: 0x284 + offset)
        me->AMK_TempMotor = 0; // 0.1degC
        me->AMK_TempInverter = 0;
        me->AMK_ErrorInfo = 0;
        me->AMK_TorqueFeedback = 0; // % 0.1 Nm

    return me;
}

enum InverterStatus {
    RELAY_OFF = 0,
    RELAY_ON_SENDING_CAN = 1,
    PRECHARGE_DC_ENABLE = 2,
    DRIVER_ENABLE = 3,
    READY_TO_DRIVE_INVERTER_ON = 4,
    TORQUE_LIMIT_SET = 5,
    TORQUE_SETPOINTS_ACTIVE = 6
};

void DI_calculateCommands(_DriveInverter* me, TorqueEncoder *tps, BrakePressureSensor *bps){

    sbyte2 torqueOutput = 0;
    sbyte2 appsTorque = 0;
    sbyte2 bpsTorque = 0;

    float4 appsOutputPercent;

    TorqueEncoder_getOutputPercent(tps, &appsOutputPercent);

    appsTorque = me->AMK_TorqueLimitPositiv * getPercent(appsOutputPercent,0, 1, TRUE) - 0 * getPercent(appsOutputPercent, 0, 0, TRUE);
    bpsTorque = 0 - (0 - 0) * getPercent(bps->percent, 0, 0, TRUE);

    torqueOutput = appsTorque + bpsTorque;

    if(torqueOutput > 25){
        torqueOutput = 25;
    } else if (torqueOutput < 0){
        torqueOutput = 0;
    }
    
    DI_commandTorque(torqueOutput, me);
    DI_getCommandedTorque(me);

}

void DI_calculateInverterControl(_DriveInverter* me, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds){
     switch (me->startUpStage){
        case RELAY_OFF:
            if(Sensor_RTDButton.sensorValue == TRUE && bps->percent > 0.10){
                IO_DO_Set(IO_DO_00, TRUE);
                me->startUpStage = 1;
            } else {
                IO_DO_Set(IO_DO_00, FALSE);
                me->startUpStage = 0;
            }
        break;
        //MCM relay on, we can now start sending safe CAN messages
        case RELAY_ON_SENDING_CAN:
            me->AMK_bInverterOn = FALSE;
            me->AMK_bDcOn = FALSE;
            me->AMK_bEnable = FALSE;
            me->AMK_bErrorReset = FALSE;
            me->AMK_TorqueSetpoint = 0;
            me->AMK_TorqueLimitPositiv = 0;
            me->AMK_TorqueLimitNegativ = 0; //No changes until regen is present
            if(me->AMK_bSystemReady == TRUE && me->AMK_bError == FALSE){ 
                me->startUpStage = 2;
                IO_RTC_StartTime(&timestamp_Precharge);
            }
        break;
        //Precharge needs to have occured to now send the new message 
        case PRECHARGE_DC_ENABLE:
            if(HVILTermSense->sensorValue == TRUE && IO_RTC_GetTimeUS(timestamp_Precharge) >= 10000000){ // After 10 Seconds
                me->AMK_bInverterOn = FALSE;
                me->AMK_bDcOn = TRUE;
                me->AMK_bEnable = FALSE;
                me->AMK_bErrorReset = FALSE;
                me->AMK_TorqueSetpoint = 0;
                me->AMK_TorqueLimitPositiv = 0;
                me->AMK_TorqueLimitNegativ = 0;
            }
            if(me->AMK_bQuitDcOn == FALSE && HVILTermSense->sensorValue == TRUE){
                timestamp_Precharge = 0;
                me->AMK_bErrorReset = TRUE;
                if(IO_RTC_GetTimeUS(timestamp_Precharge) >= 5000000) // After 5 Seconds)
                {
                    me->AMK_bErrorReset = FALSE; //We need to retry the sequence
                    me->startUpStage = 1; //Jump after LV
                }
            }
            if(me->AMK_bDcOnVal == TRUE && me->AMK_bQuitDcOn == TRUE){
                me->startUpStage = 3;
            }
        break;
        case DRIVER_ENABLE: 
            if(Sensor_RTDButton.sensorValue == FALSE && tps->calibrated == TRUE && bps->calibrated == TRUE && tps->travelPercent < .05  && bps->percent > .25){
                me->AMK_bInverterOn = FALSE;
                me->AMK_bDcOn = TRUE;
                me->AMK_bEnable = TRUE;
                me->AMK_bErrorReset = FALSE;
                me->AMK_TorqueSetpoint = 0;
                me->AMK_TorqueLimitPositiv = 0;
                me->AMK_TorqueLimitNegativ = 0;
                me->startUpStage = 4;
            }
        break;
        case READY_TO_DRIVE_INVERTER_ON:
            if(Sensor_RTDButton.sensorValue == TRUE && me->AMK_bEnable == TRUE){
                me->AMK_bInverterOn = TRUE;
                me->AMK_bDcOn = TRUE;
                me->AMK_bEnable = TRUE;
                me->AMK_TorqueSetpoint = 0;
                me->AMK_TorqueLimitPositiv = 0;
                me->AMK_TorqueLimitNegativ = 0;
                RTDS_setVolume(rtds, 1, 1500000);
                if(me->AMK_bInverterOnVal == TRUE && me->AMK_bQuitInverterOnVal == TRUE){
                    me->startUpStage = 5;
                } else {
                    me->startUpStage = 1; //Throw to CANbus
                }
            }
        break;
        case TORQUE_LIMIT_SET: 
            me->AMK_bInverterOn = TRUE;
            me->AMK_bDcOn = TRUE;
            me->AMK_bEnable = TRUE;
            me->AMK_TorqueSetpoint = 0;
            me->AMK_TorqueLimitPositiv = 25 * 10; // 25Nm -> Will need to find a way to make this global for the future
            me->AMK_TorqueLimitNegativ = 0;
            if(me->AMK_bError == FALSE){
                me->startUpStage = 6;
            }
        break;
        case TORQUE_SETPOINTS_ACTIVE:
            me->AMK_bInverterOn = TRUE;
            me->AMK_bDcOn = TRUE;
            me->AMK_bEnable = TRUE;
            me->AMK_TorqueLimitPositiv = 25 * 10; // 25Nm -> Will need to find a way to make this global for the future
            me->AMK_TorqueLimitNegativ = 0;
            if(me->AMK_bError == TRUE){
                me->startUpStage = 1;
            }
        break;

        default:
        //We lost track of the sequence
        me->startUpStage = 0;
        break;
     }
}  

void DI_parseCanMessage(IO_CAN_DATA_FRAME* diCanMessage, _DriveInverter* me){

    int address1 = me->canIdIncoming;
    int address2 = me->canIdIncoming + 2;

    if(diCanMessage->id == address1) {
        // System ready status
        me->AMK_bSystemReady = diCanMessage->data[1] >> 0 & 0x01;
        // Error status
        me->AMK_bError = diCanMessage->data[1] >> 1 & 0x01;
        // Warnings status
        me->AMK_bWarn = diCanMessage->data[1] >> 2 & 0x01;
        // Quit DC on status
        me->AMK_bQuitDcOn = diCanMessage->data[1] >> 3 & 0x01;
        // DC on status
        me->AMK_bDcOnVal = diCanMessage->data[1] >> 4 & 0x01;
        // Quit inverter on status
        me->AMK_bQuitInverterOnVal = diCanMessage->data[1] >> 5 & 0x01;
        // Inverter on status
        me->AMK_bInverterOnVal = diCanMessage->data[1] >> 6 & 0x01;
        // Derating value
        me->AMK_bDerating = diCanMessage->data[1] >> 7 & 0x01;
        // Speed value
        me->AMK_ActualVelocity = (double)(diCanMessage->data[3] << 8 | diCanMessage->data[2]);
        me->AMK_ActualVelocity = (double)me->AMK_ActualVelocity * 0.01;
        // Torque current
        me->AMK_TorqueCurrent = (double)(diCanMessage->data[5] << 8 | diCanMessage->data[4]);
        me->AMK_TorqueCurrent = (double) me->AMK_TorqueCurrent * 0.01;
        // Magnetized current
        me->AMK_MagnetizingCurrent = (double)(diCanMessage->data[7] << 8 | diCanMessage->data[6]);
        me->AMK_MagnetizingCurrent = (double) me->AMK_MagnetizingCurrent * 0.01;
        
    } else if(diCanMessage->id == address2) {
         // Motor temperature
        me->AMK_TempMotor = (double)(diCanMessage->data[1] << 8 | diCanMessage->data[0]) / 10.0;
        // Inverter temperature
        me->AMK_TempInverter = (double)(diCanMessage->data[3] << 8 | diCanMessage->data[2]) / 10.0;
        // Diagnostic number
        me->AMK_ErrorInfo = (double)(diCanMessage->data[5] << 8 | diCanMessage->data[4]);
        // Torque feedback
        me->AMK_TorqueFeedback = (double)(diCanMessage->data[7] << 8 | diCanMessage->data[6]) / 10.0;
    }
}

void DI_commandTorque(sbyte2 newTorque, _DriveInverter* me){
     me->AMK_TorqueSetpoint = newTorque * 10; //since in 0.1 offset
}

sbyte2 DI_getCommandedTorque(_DriveInverter* me){
     return me->AMK_TorqueSetpoint;
}