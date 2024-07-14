
#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "serial.h"
#include "mathFunctions.h"

/*********************************************************
 *            *********** CAUTION ***********            *
 * MULTI-BYTE VALUES FOR THE STAFL BMS ARE LITTLE-ENDIAN *
 *                                                       *
 *********************************************************/

struct _BatteryManagementSystem
{

    ubyte2 canMessageBaseId;

    SerialManager *sm;

    //BMS Member Variable format:
    //byte(s), scaling, add'l comments

    // BMS_MASTER_FAULTS //
    //ubyte1 reserved;                          //3
    ubyte1 imminentContactorOpenWarning;        //2
    ubyte1 faultFlags1;                         //1
    ubyte1 faultFlags0;                         //0

    // BMS_MASTER_WARNINGS //
    //ubyte1 reserved;                          //3
    //ubyte1 reserved;                          //2
    //ubyte1 reserved;                          //1
    ubyte1 warningFlags0;                       //0

    // BMS_MASTER_SYSTEM_STATUS //
    //ubyte1 reserved;                          //7
    ubyte1 state;                               //6
    ubyte1 numMonitorBoards;                    //5
    ubyte1 monitorBoardCommErrFlags;            //4
    ubyte1 statusFlags1;                        //3
    ubyte1 statusFlags2;                        //2
    ubyte1 numFailedThermistors;                //1
    //ubyte1 reserved;                          //0

    // BMS_PACK_SAFE_OPERATING_ENVELOPE //
    //ubyte2 reserved;                          //7:6
    ubyte2 chargerConstVoltageSetPoint;         //5:4
    ubyte2 maxDischargeCurrentAllowed;          //3:2
    ubyte2 maxChargeCurrentAllowed;             //1:0

    // BMS_MASTER_LOCAL_BOARD_MEASUREMENTS //
    sbyte2 boardTemperature;                    //7:6
    ubyte2 powerInputSense_12V_24V;             //5:4
    ubyte2 powerInputSense_HVIL;                //3:2
    ubyte2 internalRailSense_5V;                //1:0

    // BMS_DIGITAL_INPUTS_AND_OUTPUTS //
    ubyte1 digitalOutputStatus;                 //1
    ubyte1 digitalInputStatus;                  //0

    // BMS_PACK_LEVEL_MEASUREMENTS_1 //
    ubyte4 packVoltage;                         //7:4, V*1000
    sbyte4 packCurrent;                         //3:0, A*1000, charging=positive discharging=negative

    // BMS_PACK_LEVEL_MEASUREMENTS_2 //
    ubyte2 packStateOfCharge;                   //7:6, %*10
    ubyte2 packStateOfHealth;                   //5:4, %*10
    ubyte2 packAmpHoursRemaining;               //3:2, Ah*10
    //ubyte2 reserved                           //1:0

    // BMS_CELL_VOLTAGE_SUMMARY //
    ubyte4 highestCellVoltage;                  //7:6, V*1000
    ubyte2 lowestCellVoltage;                   //5:4, V*1000
    ubyte2 highestCellVoltagePos;               //3:2, 1-N
    ubyte2 lowestCellVoltagePos;                //1:0, 1-N

    // BMS_CELL_TEMPERATURE_SUMMARY //
    sbyte2 highestCellTemperature;              //7:6, degC*10
    sbyte2 lowestCellTemperature;               //5:4, degC*10
    ubyte2 highestCellTemperaturePos;           //3:2, 1-N
    ubyte2 lowestCellTemperaturePos;            //1:0, 1-N

    // BMS_PACK_LEVEL_MEASUREMENTS_3 //
    ubyte4 sumOfCellVoltages;                   //7:4, V*1000
    ubyte4 preChargeVoltage;                    //3:0, V*1000

    // BMS_CELL_VOLTAGE_DATA //
    // Use these variables as temporary buffers for now
    // Maybe transfer to array in future?
    ubyte2 cellVoltage_4X_1;                    //7:6, V*1000, X from 0 to 63 (targeted cell group voltage)
    ubyte2 cellVoltage_4X_2;                    //5:4, V*1000, X from 0 to 63 (targeted cell group voltage)
    ubyte2 cellVoltage_4X_3;                    //3:2, V*1000, X from 0 to 63 (targeted cell group voltage)
    ubyte2 cellVoltage_4X_4;                    //1:0, V*1000, X from 0 to 63 (targeted cell group voltage)

    // BMS_CELL_TEMPERATURE_DATA //
    // Use these variables as temporary buffers for now
    // Maybe transfer to array in future?
    ubyte2 cellTemperature_4X_1;                //7:6, degC*10, X from 0 to 63 (targeted cell group voltage)
    ubyte2 cellTemperature_4X_2;                //5:4, degC*10, X from 0 to 63 (targeted cell group voltage)
    ubyte2 cellTemperature_4X_3;                //3:2, degC*10, X from 0 to 63 (targeted cell group voltage)
    ubyte2 cellTemperature_4X_4;                //1:0, degC*10, X from 0 to 63 (targeted cell group voltage)

    // BMS_CELL_SHUNTING_STATUS_1 //
    //No ubyte8 exists, so we need to split it into two ubyte4
    ubyte4 cellShuntingStatusArray1_0;          //7:4, , bit0=1 - shunting active for cell 1 | bit31=1 - shunting active for cell 32 
    ubyte4 cellShuntingStatusArray1_1;          //3:0, , bit0=1 - shunting active for cell 33 | bit31=1 - shunting active for cell 64 

    // BMS_CELL_SHUNTING_STATUS_2 //
    ubyte4 cellShuntingStatusArray2_0;          //7:4, , bit0=1 - shunting active for cell 65 | bit31=1 - shunting active for cell 96 
    ubyte4 cellShuntingStatusArray2_1;          //3:0, , bit0=1 - shunting active for cell 97 | bit31=1 - shunting active for cell 128 

    // BMS_CELL_SHUNTING_STATUS_3 //
    ubyte4 cellShuntingStatusArray3_0;          //7:4, , bit0=1 - shunting active for cell 129 | bit31=1 - shunting active for cell 160 
    ubyte4 cellShuntingStatusArray3_1;          //3:0, , bit0=1 - shunting active for cell 161 | bit31=1 - shunting active for cell 192

    // BMS_CELL_SHUNTING_STATUS_4 //
    ubyte4 cellShuntingStatusArray4_0;          //7:4, , bit0=1 - shunting active for cell 193 | bit31=1 - shunting active for cell 224
    ubyte4 cellShuntingStatusArray4_1;          //3:0, , bit0=1 - shunting active for cell 225 | bit31=1 - shunting active for cell 256 

    // BMS_CONFIGUATION_INFORMATION //
    //ubyte2 reserved;                          //7:6
    //ubyte2 reserved;                          //5:4
    ubyte2 numSeriesCells;                      //3:2
    ubyte2 numThermistors;                      //1:0

    // BMS_FIRMWARE_VERSION_INFORMATION //
    //ubyte1 reserved;                          //3
    ubyte1 fwMajorVerNum;                       //2, , X.0.0
    ubyte1 fwMinorVerNum;                       //1, , 0.X.0
    ubyte1 fwRevNum;                            //0, , 0.0.X

    bool relayState;

    // signed = 2's complement: 0XfFF = -1, 0x00 = 0, 0x01 = 1
};

BatteryManagementSystem *BMS_new(SerialManager *serialMan, ubyte2 canMessageBaseID)
{

    BatteryManagementSystem *me = (BatteryManagementSystem *)malloc(sizeof(struct _BatteryManagementSystem));

    me->canMessageBaseId = canMessageBaseID;
    me->sm = serialMan;
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
            bms->chargerConstVoltageSetPoint  = ( ((ubyte2)bmsCanMessage->data[5] << 8)
                                                | ((ubyte2)bmsCanMessage->data[4])
                                                );
            bms->maxDischargeCurrentAllowed   = ( ((ubyte2)bmsCanMessage->data[3] << 8)
                                                | ((ubyte2)bmsCanMessage->data[2])
                                                );
            bms->maxChargeCurrentAllowed      = ( ((ubyte2)bmsCanMessage->data[1] << 8)
                                                | ((ubyte2)bmsCanMessage->data[0])
                                                );
            break;

        case BMS_MASTER_LOCAL_BOARD_MEASUREMENTS:
            bms->boardTemperature             = ( ((ubyte2)bmsCanMessage->data[7] << 8)
                                                | ((ubyte2)bmsCanMessage->data[6])
                                                );
            bms->powerInputSense_12V_24V      = ( ((ubyte2)bmsCanMessage->data[5] << 8)
                                                | ((ubyte2)bmsCanMessage->data[4])
                                                );
            bms->powerInputSense_HVIL         = ( ((ubyte2)bmsCanMessage->data[3] << 8)
                                                | ((ubyte2)bmsCanMessage->data[2])
                                                );
            bms->internalRailSense_5V         = ( ((ubyte2)bmsCanMessage->data[1] << 8)
                                                | ((ubyte2)bmsCanMessage->data[0])
                                                );
            break;

        case BMS_DIGITAL_INPUTS_AND_OUTPUTS:
            bms->digitalOutputStatus          = bmsCanMessage->data[1];
            bms->digitalInputStatus           = bmsCanMessage->data[0];
            break;

        case BMS_PACK_LEVEL_MEASUREMENTS_1:
            bms->packVoltage                  = ( ((ubyte4)bmsCanMessage->data[7] << 24)
                                                | ((ubyte4)bmsCanMessage->data[6] << 16)
                                                | ((ubyte4)bmsCanMessage->data[5] << 8)
                                                | ((ubyte4)bmsCanMessage->data[4])
                                                ) ;
            bms->packCurrent                  = ( ((ubyte4)bmsCanMessage->data[3] << 24)
                                                | ((ubyte4)bmsCanMessage->data[2] << 16)
                                                | ((ubyte4)bmsCanMessage->data[1] << 8)
                                                | ((ubyte4)bmsCanMessage->data[0])
                                                );  
            break;

        case BMS_PACK_LEVEL_MEASUREMENTS_2:
            bms->packStateOfCharge            = ( ((ubyte2)bmsCanMessage->data[7] << 8)
                                                | ((ubyte2)bmsCanMessage->data[6])
                                                );
            bms->packStateOfHealth            = ( ((ubyte2)bmsCanMessage->data[5] << 8)
                                                | ((ubyte2)bmsCanMessage->data[4])
                                                );
            bms->packAmpHoursRemaining        = ( ((ubyte2)bmsCanMessage->data[3] << 8)
                                                | ((ubyte2)bmsCanMessage->data[2])
                                                );
            //bms->reserved                   = ( ((ubyte2)bmsCanMessage->data[1] << 8)
            //                                  | ((ubyte2)bmsCanMessage->data[0])
            //                                  );
            break;
        
        case BMS_CELL_VOLTAGE_SUMMARY:
            bms->highestCellVoltage           = ( ((ubyte2)bmsCanMessage->data[7] << 8)
                                                | ((ubyte2)bmsCanMessage->data[6])
                                                );
            bms->lowestCellVoltage            = ( ((ubyte2)bmsCanMessage->data[5] << 8)
                                                | ((ubyte2)bmsCanMessage->data[4])
                                                );
            bms->highestCellVoltagePos        = ( ((ubyte2)bmsCanMessage->data[3] << 8)
                                                | ((ubyte2)bmsCanMessage->data[2])
                                                );
            bms->lowestCellVoltagePos         = ( ((ubyte2)bmsCanMessage->data[1] << 8)
                                                | ((ubyte2)bmsCanMessage->data[0])
                                                );
            break;

        case BMS_CELL_TEMPERATURE_SUMMARY:
            bms->highestCellTemperature       = ( ((ubyte2)bmsCanMessage->data[7] << 8)
                                                | ((ubyte2)bmsCanMessage->data[6])
                                                );
            bms->lowestCellTemperature        = ( ((ubyte2)bmsCanMessage->data[5] << 8)
                                                | ((ubyte2)bmsCanMessage->data[4])
                                                );
            bms->highestCellTemperaturePos    = ( ((ubyte2)bmsCanMessage->data[3] << 8)
                                                | ((ubyte2)bmsCanMessage->data[2])
                                                );
            bms->lowestCellTemperaturePos     = ( ((ubyte2)bmsCanMessage->data[1] << 8)
                                                | ((ubyte2)bmsCanMessage->data[0])
                                                );
            break;
        
        case BMS_PACK_LEVEL_MEASUREMENTS_3:
            bms->sumOfCellVoltages            = ( ((ubyte4)bmsCanMessage->data[7] << 24)
                                                | ((ubyte4)bmsCanMessage->data[6] << 16)
                                                | ((ubyte4)bmsCanMessage->data[5] << 8)
                                                | ((ubyte4)bmsCanMessage->data[4])
                                                );
            bms->preChargeVoltage             = ( ((ubyte4)bmsCanMessage->data[3] << 24)
                                                | ((ubyte4)bmsCanMessage->data[2] << 16)
                                                | ((ubyte4)bmsCanMessage->data[1] << 8)
                                                | ((ubyte4)bmsCanMessage->data[0])
                                                );
            break;
            
        case BMS_CELL_VOLTAGE_DATA:
            bms->cellVoltage_4X_1             = ( ((ubyte2)bmsCanMessage->data[7] << 8)
                                                | ((ubyte2)bmsCanMessage->data[6])
                                                );
            bms->cellVoltage_4X_2             = ( ((ubyte2)bmsCanMessage->data[5] << 8)
                                                | ((ubyte2)bmsCanMessage->data[4])
                                                );
            bms->cellVoltage_4X_3             = ( ((ubyte2)bmsCanMessage->data[3] << 8)
                                                | ((ubyte2)bmsCanMessage->data[2])
                                                );
            bms->cellVoltage_4X_4             = ( ((ubyte2)bmsCanMessage->data[1] << 8)
                                                | ((ubyte2)bmsCanMessage->data[0])
                                                );
            break;
            
        case BMS_CELL_TEMPERATURE_DATA:
            bms->cellTemperature_4X_1         = ( ((ubyte2)bmsCanMessage->data[7] << 8)
                                                | ((ubyte2)bmsCanMessage->data[6])
                                                );
            bms->cellTemperature_4X_2         = ( ((ubyte2)bmsCanMessage->data[5] << 8)
                                                | ((ubyte2)bmsCanMessage->data[4])
                                                );
            bms->cellTemperature_4X_3         = ( ((ubyte2)bmsCanMessage->data[3] << 8)
                                                | ((ubyte2)bmsCanMessage->data[2])
                                                );
            bms->cellTemperature_4X_4         = ( ((ubyte2)bmsCanMessage->data[1] << 8)
                                                | ((ubyte2)bmsCanMessage->data[0])
                                                );
            break;
            
        case BMS_CELL_SHUNTING_STATUS_1:
            bms->cellShuntingStatusArray1_0   = ( ((ubyte4)bmsCanMessage->data[7] << 24)
                                                | ((ubyte4)bmsCanMessage->data[6] << 16)
                                                | ((ubyte4)bmsCanMessage->data[5] << 8)
                                                | ((ubyte4)bmsCanMessage->data[4])
                                                );
            bms->cellShuntingStatusArray1_1   = ( ((ubyte4)bmsCanMessage->data[3] << 24)
                                                | ((ubyte4)bmsCanMessage->data[2] << 16)
                                                | ((ubyte4)bmsCanMessage->data[1] << 8)
                                                | ((ubyte4)bmsCanMessage->data[0])
                                                );
            break;
            
        case BMS_CELL_SHUNTING_STATUS_2:
            bms->cellShuntingStatusArray2_0   = ( ((ubyte4)bmsCanMessage->data[7] << 24)
                                                | ((ubyte4)bmsCanMessage->data[6] << 16)
                                                | ((ubyte4)bmsCanMessage->data[5] << 8)
                                                | ((ubyte4)bmsCanMessage->data[4])
                                                );
            bms->cellShuntingStatusArray2_1   = ( ((ubyte4)bmsCanMessage->data[3] << 24)
                                                | ((ubyte4)bmsCanMessage->data[2] << 16)
                                                | ((ubyte4)bmsCanMessage->data[1] << 8)
                                                | ((ubyte4)bmsCanMessage->data[0])
                                                );
            break;
            
        case BMS_CELL_SHUNTING_STATUS_3:
            bms->cellShuntingStatusArray3_0   = ( ((ubyte4)bmsCanMessage->data[7] << 24)
                                                | ((ubyte4)bmsCanMessage->data[6] << 16)
                                                | ((ubyte4)bmsCanMessage->data[5] << 8)
                                                | ((ubyte4)bmsCanMessage->data[4])
                                                );
            bms->cellShuntingStatusArray3_1   = ( ((ubyte4)bmsCanMessage->data[3] << 24)
                                                | ((ubyte4)bmsCanMessage->data[2] << 16)
                                                | ((ubyte4)bmsCanMessage->data[1] << 8)
                                                | ((ubyte4)bmsCanMessage->data[0])
                                                );
            break;
            
        case BMS_CELL_SHUNTING_STATUS_4:
            bms->cellShuntingStatusArray4_0   = ( ((ubyte4)bmsCanMessage->data[7] << 24)
                                                | ((ubyte4)bmsCanMessage->data[6] << 16)
                                                | ((ubyte4)bmsCanMessage->data[5] << 8)
                                                | ((ubyte4)bmsCanMessage->data[4])
                                                );
            bms->cellShuntingStatusArray4_1   = ( ((ubyte4)bmsCanMessage->data[3] << 24)
                                                | ((ubyte4)bmsCanMessage->data[2] << 16)
                                                | ((ubyte4)bmsCanMessage->data[1] << 8)
                                                | ((ubyte4)bmsCanMessage->data[0])
                                                );
            break;
            
        case BMS_CONFIGUATION_INFORMATION:
            //bms->reserved                   = ( ((ubyte2)bmsCanMessage->data[7] << 8)
            //                                  | ((ubyte2)bmsCanMessage->data[6])
            //                                  );
            //bms->reserved                   = ( ((ubyte2)bmsCanMessage->data[5] << 8)
            //                                  | ((ubyte2)bmsCanMessage->data[4])
            //                                  );
            bms->numSeriesCells               = ( ((ubyte2)bmsCanMessage->data[3] << 8)
                                                | ((ubyte2)bmsCanMessage->data[2])
                                                );
            bms->numThermistors               = ( ((ubyte2)bmsCanMessage->data[1] << 8)
                                                | ((ubyte2)bmsCanMessage->data[0])
                                                );
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
    if (BMS_getFaultFlags0(me) || BMS_getFaultFlags1(me))
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

/*
sbyte1 BMS_getAvgTemp(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "AvgPackTemp: %i\n", me->avgTemp);
    return (me->avgTemp);
}
*/

ubyte4 BMS_getHighestCellVoltage_mV(BatteryManagementSystem *me)
{
    return (me->highestCellVoltage);
}

ubyte2 BMS_getLowestCellVoltage_mV(BatteryManagementSystem *me)
{
    return (me->lowestCellVoltage);
}

ubyte4 BMS_getPackVoltage(BatteryManagementSystem *me)

{
    return (me->packVoltage); 
}

ubyte4 BMS_getPackCurrent(BatteryManagementSystem *me){
    return (me->packCurrent);
}
//Split into
sbyte2 BMS_getHighestCellTemp_d_degC(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "highestCellTemp (degC*10): %i\n", (me->highestCellTemperature));

    //Need to divide by BMS_TEMPERATURE_SCALE at usage to get deciCelsius value into Celsius
    return (me->highestCellTemperature);
}

sbyte2 BMS_getHighestCellTemp_degC(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "highestCellTemp (degC): %i\n", (me->highestCellTemperature/BMS_TEMPERATURE_SCALE));

    //Need to divide by BMS_TEMPERATURE_SCALE at usage to get deciCelsius value into Celsius
    return (me->highestCellTemperature/BMS_TEMPERATURE_SCALE);
}

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower_uW(BatteryManagementSystem *me)
{
    //char buffer[32];
    //sprintf(buffer, "power (uW): %f\n", (me->packCurrent * me->packVoltage));

    //Need to divide by BMS_POWER_SCALE at usage to get microWatt value into Watts
    return (me->packCurrent * me->packVoltage);
}

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower_W(BatteryManagementSystem *me)
{
    //char buffer[32];
    //sprintf(buffer, "power (W): %f\n", ((me->packCurrent * me->packVoltage)/BMS_POWER_SCALE));

    //Need to divide by BMS_POWER_SCALE at usage to get microWatt value into Watts
    return ((me->packCurrent * me->packVoltage)/BMS_POWER_SCALE);
}

ubyte1 BMS_getFaultFlags0(BatteryManagementSystem *me) {
    //Flag 0x01: Isolation Leakage Fault
    //Flag 0x02: BMS Monitor Communication Fault
    //Flag 0x04: Pre-charge Fault
    //Flag 0x08: Pack Discharge Operating Envelope Exceeded
    //Flag 0x10: Pack Charge Operating Envelope Exceeded
    //Flag 0x20: Failed Thermistor Fault
    //Flag 0x40: HVIL Fault
    //Flag 0x80: Emergency Stop Fault
    return me->faultFlags0;
}

ubyte1 BMS_getFaultFlags1(BatteryManagementSystem *me) {
    //Flag 0x01: Cell Over-Voltage Fault
    //Flag 0x02: Cell Under-Voltage Fault
    //Flag 0x04: Cell Over-Temperature Fault
    //Flag 0x08: Cell Under-Temperature Fault
    //Flag 0x10: Pack Over-Voltage Fault
    //Flag 0x20: Pack Under-Voltage Fault
    //Flag 0x40: Over-Current Discharge Fault
    //Flag 0x80: Over-Current Charge Fault
    return me->faultFlags1;
}

bool BMS_getRelayState(BatteryManagementSystem *me) {
    //Return state of shutdown board relay
    return me->relayState;
}

/*
ubyte2 BMS_getPackTemp(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "PackTemp: %i\n", me->packTemp);
    return (me->packTemp);
}
*/