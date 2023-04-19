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
    
    if(me->startUpStage == 6){
       DI_commandTorque(torqueOutput, me);
       DI_getCommandedTorque(me);
    }

}

void DI_calculateRelay(Sensor* HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps){
    // We want to turn on LV to the motor as soon as we are potentially ready to go into HV since we need to send and monitor CAN messages
    if(tps->calibrated == TRUE && bps->calibrated == TRUE){
        IO_DO_Set(IO_DO_00, TRUE);
    } else {
        IO_DO_Set(IO_DO_00, TRUE);
    }
}

void DI_calculateInverterControl(_DriveInverter* me, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds){
     switch (me->startUpStage){
        case 0:
            if(IO_DO_00 == TRUE){
                me->startUpStage = 1;
            }
        break;
        //MCM relay on, we can now start sending safe CAN messages
        case 1:
            me->AMK_bInverterOn = FALSE;
            me->AMK_bDcOn = FALSE;
            me->AMK_bEnable = FALSE;
            me->AMK_bErrorReset = FALSE;
            me->AMK_TorqueSetpoint = 0;
            me->AMK_TorqueLimitPositiv = 0;
            me->AMK_TorqueLimitNegativ = 0;
            if(me->AMK_bSystemReady == TRUE && me->AMK_bError == FALSE){
                me->startUpStage = 2;
            }
        break;
        //Precharge needs to have occured to now send the new message
        case 2:
            if(HVILTermSense->sensorValue == TRUE){
                me->AMK_bInverterOn = FALSE;
                me->AMK_bDcOn = TRUE;
                me->AMK_bEnable = FALSE;
                me->AMK_bErrorReset = FALSE;
                me->AMK_TorqueSetpoint = 0;
                me->AMK_TorqueLimitPositiv = 0;
                me->AMK_TorqueLimitNegativ = 0;
            }
            if(me->AMK_bQuitDcOn == FALSE && HVILTermSense->sensorValue == TRUE){
                me->AMK_bErrorReset = TRUE;
                //Need a delay here since precharge was not done
                me->AMK_bErrorReset = FALSE;
                me->startUpStage = 1;
            }
            if(me->AMK_bDcOnVal == TRUE && me->AMK_bQuitDcOn == TRUE){
                me->startUpStage = 3;
            }
        break;
        case 3: 
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
        case 4:
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
                    me->startUpStage = 1;
                }
            }
        break;
        case 5: 
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
        case 6:
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
        me->AMK_bSystemReady = diCanMessage->data[0] & 0x01;
        // Error status
        me->AMK_bError = (diCanMessage->data[0] >> 1) & 0x01;
        // Warnings status
        me->AMK_bWarn = (diCanMessage->data[0] >> 2) & 0x01;
        // Quit DC on status
        me->AMK_bQuitDcOn = (diCanMessage->data[0] >> 3) & 0x01;
        // DC on status
        me->AMK_bDcOnVal = (diCanMessage->data[0] >> 4) & 0x01;
        // Quit inverter on status
        me->AMK_bQuitInverterOnVal = (diCanMessage->data[0] >> 5) & 0x01;
        // Inverter on status
        me->AMK_bInverterOnVal = (diCanMessage->data[0] >> 6) & 0x01;
        // Derating value
        me->AMK_bDerating = (diCanMessage->data[0] >> 7) & 0x01;
        // Speed value
        me->AMK_ActualVelocity = ((ubyte2)diCanMessage->data[2] << 8 | diCanMessage->data[1]) / 100.0;
        // Torque current
        me->AMK_TorqueCurrent = ((ubyte2)diCanMessage->data[4] << 8 | diCanMessage->data[3]) / 100.0;
        // Magnetized current
        me->AMK_MagnetizingCurrent = ((ubyte2)diCanMessage->data[6] << 8 | diCanMessage->data[5]) / 100.0;
        
    } else if(diCanMessage->id == address2) {
        // Motor temperature
        me->AMK_TempMotor = ((ubyte2)diCanMessage->data[0] << 8 | diCanMessage->data[1]) / 10.0;
        // Inverter temperature
        me->AMK_TempInverter = ((ubyte2)diCanMessage->data[2] << 8 | diCanMessage->data[3]) / 10.0;
        // Diagnostic number
        me->AMK_ErrorInfo = diCanMessage->data[4] & 0x1F;
        // Torque feedback
        me->AMK_TorqueFeedback = ((ubyte2)diCanMessage->data[7] << 8 | diCanMessage->data[6]) / 10.0 - 30.0;
    }
}

void DI_commandTorque(sbyte2 newTorque, _DriveInverter* me){
     me->AMK_TorqueSetpoint = newTorque * 10; //since in 0.1 offset
}

sbyte2 DI_getCommandedTorque(_DriveInverter* me){
     return me->AMK_TorqueSetpoint;
}