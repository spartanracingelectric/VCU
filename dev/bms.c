
#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "mathFunctions.h"

/*********************************************************
 *            *********** CAUTION ***********            *
 * MULTI-BYTE VALUES FOR THE STAFL BMS ARE LITTLE-ENDIAN *
 *                                                       *
 *********************************************************/

BatteryManagementSystem *BMS_new(ubyte2 canMessageBaseID)
{
    BatteryManagementSystem *me = (BatteryManagementSystem *)malloc(sizeof(struct _BatteryManagementSystem));

    me->canMessageBaseId = canMessageBaseID;
    //me->maxTemp = 99;

    me->packCurrent = 0;
    me->packVoltage = 0;
    //Repick a new value, maybe 0xFFFF?
    me->highestCellVoltage = 0;
    me->lowestCellVoltage = 9999;
    me->highestCellTemperature = 0;

    me->faultFlags0 = 0;
    me->faultFlags1 = 0;
    //me->faultFlags0 = 0xFF;
    //me->faultFlags1 = 0xFF;

    me->relayState = FALSE;

    return me;
}

void BMS_parseCanMessage(BatteryManagementSystem *bms, IO_CAN_DATA_FRAME *bmsCanMessage)
{
    ubyte2 utemp16;
    //    sbyte1  temp16;
    ubyte4 utemp32;

    //Subtract BMS Base CAN ID from incoming BMS CAN message ID to get offset
    //Byte extraction DOES NOT INCLUDE SCALING
    //Ex: (bmsCanMessage->id+BMS_MASTER_FAULTS) - bms->canMessageBaseId = BMS_MASTER_FAULTS
    switch (bmsCanMessage->id - bms->canMessageBaseId)
    {
        case BMS_MASTER_FAULTS:
            bms->imminentContactorOpenWarning = bmsCanMessage->data[2];
            bms->faultFlags1                  = bmsCanMessage->data[1];
            bms->faultFlags0                  = bmsCanMessage->data[0];
            break;

        case BMS_MASTER_WARNINGS:
            bms->warningFlags0                = bmsCanMessage->data[0];
            break;

        case BMS_MASTER_SYSTEM_STATUS:
            //bms->reserved                   = bmsCanMessage->data[7];
            bms->state                        = bmsCanMessage->data[6];
            bms->numMonitorBoards             = bmsCanMessage->data[5];
            bms->monitorBoardCommErrFlags     = bmsCanMessage->data[4];
            bms->statusFlags1                 = bmsCanMessage->data[3];
            bms->statusFlags2                 = bmsCanMessage->data[2];
            bms->numFailedThermistors         = bmsCanMessage->data[1];
            //bms->reserved                   = bmsCanMessage->data[0];
            break;

        case BMS_PACK_SAFE_OPERATING_ENVELOPE:
            bms->chargerConstVoltageSetPoint  = reasm_ubyte2(bmsCanMessage->data, 4);
            bms->maxDischargeCurrentAllowed   = reasm_ubyte2(bmsCanMessage->data, 2);
            bms->maxChargeCurrentAllowed      = reasm_ubyte2(bmsCanMessage->data, 0);
            break;

        case BMS_MASTER_LOCAL_BOARD_MEASUREMENTS:
            bms->boardTemperature             = reasm_ubyte2(bmsCanMessage->data, 6);
            bms->powerInputSense_12V_24V      = reasm_ubyte2(bmsCanMessage->data, 4);
            bms->powerInputSense_HVIL         = reasm_ubyte2(bmsCanMessage->data, 2);
            bms->internalRailSense_5V         = reasm_ubyte2(bmsCanMessage->data, 0);
            break;

        case BMS_DIGITAL_INPUTS_AND_OUTPUTS:
            bms->digitalOutputStatus          = bmsCanMessage->data[1];
            bms->digitalInputStatus           = bmsCanMessage->data[0];
            break;

        case BMS_PACK_LEVEL_MEASUREMENTS_1:
            bms->packVoltage                  = reasm_ubyte4(bmsCanMessage->data, 4);
            bms->packCurrent                  = reasm_ubyte4(bmsCanMessage->data, 0);  
            break;

        case BMS_PACK_LEVEL_MEASUREMENTS_2:
            bms->packStateOfCharge            = reasm_ubyte2(bmsCanMessage->data, 6);
            bms->packStateOfHealth            = reasm_ubyte2(bmsCanMessage->data, 4);
            bms->packAmpHoursRemaining        = reasm_ubyte2(bmsCanMessage->data, 2);
            //bms->reserved                   = reasm_ubyte2(bmsCanMessage->data, 0);
            break;
        
        case BMS_CELL_VOLTAGE_SUMMARY:
            bms->highestCellVoltage           = reasm_ubyte2(bmsCanMessage->data, 6);
            bms->lowestCellVoltage            = reasm_ubyte2(bmsCanMessage->data, 4);
            bms->highestCellVoltagePos        = reasm_ubyte2(bmsCanMessage->data, 2);
            bms->lowestCellVoltagePos         = reasm_ubyte2(bmsCanMessage->data, 0);
            break;

        case BMS_CELL_TEMPERATURE_SUMMARY:
            bms->highestCellTemperature       = reasm_ubyte2(bmsCanMessage->data, 6);
            bms->lowestCellTemperature        = reasm_ubyte2(bmsCanMessage->data, 4);
            bms->highestCellTemperaturePos    = reasm_ubyte2(bmsCanMessage->data, 2);
            bms->lowestCellTemperaturePos     = reasm_ubyte2(bmsCanMessage->data, 0);
            break;
        
        case BMS_PACK_LEVEL_MEASUREMENTS_3:
            bms->sumOfCellVoltages            = reasm_ubyte4(bmsCanMessage->data, 4);
            bms->preChargeVoltage             = reasm_ubyte4(bmsCanMessage->data, 0);
            break;
            
        case BMS_CELL_VOLTAGE_DATA:
            bms->cellVoltage_4X_1             = reasm_ubyte2(bmsCanMessage->data, 6);
            bms->cellVoltage_4X_2             = reasm_ubyte2(bmsCanMessage->data, 4);
            bms->cellVoltage_4X_3             = reasm_ubyte2(bmsCanMessage->data, 2);
            bms->cellVoltage_4X_4             = reasm_ubyte2(bmsCanMessage->data, 0);
            break;
            
        case BMS_CELL_TEMPERATURE_DATA:
            bms->cellTemperature_4X_1         = reasm_ubyte2(bmsCanMessage->data, 6);
            bms->cellTemperature_4X_2         = reasm_ubyte2(bmsCanMessage->data, 4);
            bms->cellTemperature_4X_3         = reasm_ubyte2(bmsCanMessage->data, 2);
            bms->cellTemperature_4X_4         = reasm_ubyte2(bmsCanMessage->data, 0);
            break;
            
        case BMS_CELL_SHUNTING_STATUS_1:
            bms->cellShuntingStatusArray1_0   = reasm_ubyte4(bmsCanMessage->data, 4);
            bms->cellShuntingStatusArray1_1   = reasm_ubyte4(bmsCanMessage->data, 0);
            break;
            
        case BMS_CELL_SHUNTING_STATUS_2:
            bms->cellShuntingStatusArray2_0   = reasm_ubyte4(bmsCanMessage->data, 4);
            bms->cellShuntingStatusArray2_1   = reasm_ubyte4(bmsCanMessage->data, 0);
            break;
            
        case BMS_CELL_SHUNTING_STATUS_3:
            bms->cellShuntingStatusArray3_0   = reasm_ubyte4(bmsCanMessage->data, 4);
            bms->cellShuntingStatusArray3_1   = reasm_ubyte4(bmsCanMessage->data, 0);
            break;
            
        case BMS_CELL_SHUNTING_STATUS_4:
            bms->cellShuntingStatusArray4_0   = reasm_ubyte4(bmsCanMessage->data, 4);
            bms->cellShuntingStatusArray4_1   = reasm_ubyte4(bmsCanMessage->data, 0);
            break;
            
        case BMS_CONFIGURATION_INFORMATION:
            //bms->reserved                   = reasm_ubyte2(bmsCanMessage->data, 6);
            //bms->reserved                   = reasm_ubyte2(bmsCanMessage->data, 4);
            bms->numSeriesCells               = reasm_ubyte2(bmsCanMessage->data, 2);
            bms->numThermistors               = reasm_ubyte2(bmsCanMessage->data, 0);
            break;
            
        case BMS_FIRMWARE_VERSION_INFORMATION:
            //bms->reserved                   = bmsCanMessage->data[3]
            bms->fwMajorVerNum                = bmsCanMessage->data[2];
            bms->fwMinorVerNum                = bmsCanMessage->data[1];
            bms->fwRevNum                     = bmsCanMessage->data[0];
            break;
    }
}

IO_ErrorType BMS_relayControl(BatteryManagementSystem *me)
{
    //////////////////////////////////////////////////////////////
    // Digital output to drive a signal to the Shutdown signal  //
    // based on AMS fault detection                             //
    //////////////////////////////////////////////////////////////
    IO_ErrorType err;
    //There is a fault
    if (me->faultFlags0 || me->faultFlags1)
    {
        me->relayState = TRUE;
        err = IO_DO_Set(IO_DO_01, TRUE); //Drive BMS relay true (HIGH)
    }
    //There is no fault
    else
    {
        me->relayState = FALSE;
        err = IO_DO_Set(IO_DO_01, FALSE); //Drive BMS relay false (LOW)
    }
    return err;
}


sbyte2 BMS_getHighestCellTemp_degC(BatteryManagementSystem *me)
{
    //Need to divide by BMS_TEMPERATURE_SCALE at usage to get deciCelsius value into Celsius
    return (me->highestCellTemperature/BMS_TEMPERATURE_SCALE);
}

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower_uW(BatteryManagementSystem *me)
{
    //Need to divide by BMS_POWER_SCALE at usage to get microWatt value into Watts
    return (me->packCurrent * me->packVoltage);
}

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower_W(BatteryManagementSystem *me)
{
    //Need to divide by BMS_POWER_SCALE at usage to get microWatt value into Watts
    return ((me->packCurrent * me->packVoltage)/BMS_POWER_SCALE);
}
