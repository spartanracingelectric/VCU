#ifndef _BATTERYMANAGEMENTSYSTEM_H
#define _BATTERYMANAGEMENTSYSTEM_H

#include <stdio.h>
#include <stdint.h>

#include "serial.h"
#include "IO_CAN.h"

// BMS Base Address
#define BMS_BASE_ADDRESS                    0x600

// BMS Transmitted Messages (BMS --> VCU)
#define BMS_SAFETY_CHECKER                  0x000   //4 bytes
#define BMS_CELL_SUMMARY                    0x022   //8 bytes


// BMS Scaling factors
// X/SCALE
#define BMS_VOLTAGE_SCALE                   1000    //V*1000, milliVolts to Volts
#define BMS_CURRENT_SCALE                   1000    //A*1000, milliAmps to Amps
#define BMS_POWER_SCALE                     BMS_VOLTAGE_SCALE*BMS_CURRENT_SCALE //(V*1000)*(A*1000), microWatts to Watts
#define BMS_TEMPERATURE_SCALE               10      //degC*10, deciCelsius to Celsius
#define BMS_PERCENT_SCALE                   10      //%*10, percent*10 to percent
#define BMS_AMP_HOURS_SCALE                 10      //Ah*10, deciAmpHours to AmpHours

// BMS Specific Fault Bits/Flags (within faultFlag0 and 1)
#define BMS_CELL_OVER_VOLTAGE_FLAG          0x01
#define BMS_CELL_UNDER_VOLTAGE_FLAG         0x02
#define BMS_CELL_OVER_TEMPERATURE_FLAG      0x04

typedef struct _BatteryManagementSystem
{

    ubyte2 canMessageBaseId;

    // BMS_CELL_VOLTAGE_SUMMARY //
    ubyte2 highestCellVoltage;
    ubyte2 lowestCellVoltage;                   

    // BMS_CELL_TEMPERATURE_SUMMARY
    sbyte2 highestCellTemperature;
    sbyte2 lowestCellTemperature;
   
    // BMS_FAULTS
    ubyte1 faultFlags0;                       
    bool relayState;
} BatteryManagementSystem;

BatteryManagementSystem* BMS_new(ubyte2 canMessageBaseID);
void BMS_parseCanMessage(BatteryManagementSystem* bms, IO_CAN_DATA_FRAME* bmsCanMessage);

// BMS COMMANDS // 

IO_ErrorType BMS_relayControl(BatteryManagementSystem *me);
bool BMS_getRelayState(BatteryManagementSystem *me);

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
ubyte2 BMS_getHighestCellVoltage_mV(BatteryManagementSystem *me);   //Millivolts
ubyte2 BMS_getLowestCellVoltage_mV(BatteryManagementSystem *me);   //Millivolts
sbyte2 BMS_getHighestCellTemp_d_degC(BatteryManagementSystem* me);  //deciCelsius (higher resolution)
sbyte2 BMS_getHighestCellTemp_degC(BatteryManagementSystem* me);    //Celsius
ubyte1 BMS_getFaultFlags0(BatteryManagementSystem *me);



#endif // _BATTERYMANAGEMENTSYSTEM_H

