#ifndef _BATTERYMANAGEMENTSYSTEM_H
#define _BATTERYMANAGEMENTSYSTEM_H

#include <stdio.h>
#include <stdint.h>
#include "IO_CAN.h"
#include "serial.h"

//Max mismatch voltage, in volts
//To determine VCU-side fault
#define BMS_MAX_CELL_MISMATCH_V 1.00f
#define BMS_MIN_CELL_VOLTAGE_WARNING 3.20f
#define BMS_MAX_CELL_TEMPERATURE_WARNING 55.0f

///////////////////////////////////////////////////////////////////////////////////
// STAFL BMS CAN PROTOCOL CONSTANTS, offsets from base address                   //
// Ex: canMessageBaseId + BMS_MASTER_FAULTS = Address 0x(canMessageBaseId+0x002) //
// CAN Protocol datasheet can be found in:                                       //
// SRE drive -> SRE Software -> Documentation -> Datasheets -> BMS -> Stafl      //
///////////////////////////////////////////////////////////////////////////////////

// BMS Base Address
#define BMS_BASE_ADDRESS                    0x600

// BMS Received Messages (VCU --> BMS)
#define BMS_STATE_COMMAND                   0x000   //2 bytes, Control BMS state transition
#define BMS_CHARGER_COMMAND                 0x008   //4 bytes, Control BMS charging behavior

// BMS Transmitted Messages (BMS --> VCU)
#define BMS_MASTER_FAULTS                   0x002   //4 bytes
#define BMS_MASTER_WARNINGS                 0x004   //4 bytes 
#define BMS_MASTER_SYSTEM_STATUS            0x010   //8 bytes
#define BMS_PACK_SAFE_OPERATING_ENVELOPE    0x011   //8 bytes
#define BMS_MASTER_LOCAL_BOARD_MEASUREMENTS 0x012   //8 bytes
#define BMS_DIGITAL_INPUTS_AND_OUTPUTS      0x013   //2 bytes
#define BMS_PACK_LEVEL_MEASUREMENTS_1       0x020   //8 bytes
#define BMS_PACK_LEVEL_MEASUREMENTS_2       0x021   //8 bytes
#define BMS_CELL_VOLTAGE_SUMMARY            0x022   //8 bytes
#define BMS_CELL_TEMPERATURE_SUMMARY        0x023   //8 bytes
#define BMS_PACK_LEVEL_MEASUREMENTS_3       0x024   //8 bytes
#define BMS_CELL_VOLTAGE_DATA               0x030   //8 bytes
#define BMS_CELL_TEMPERATURE_DATA           0x080   //8 bytes
#define BMS_CELL_SHUNTING_STATUS_1          0x0D0   //8 bytes
#define BMS_CELL_SHUNTING_STATUS_2          0x0D1   //8 bytes
#define BMS_CELL_SHUNTING_STATUS_3          0x0D2   //8 bytes
#define BMS_CELL_SHUNTING_STATUS_4          0x0D3   //8 bytes
#define BMS_CONFIGURATION_INFORMATION        0x0FC   //8 bytes
#define BMS_FIRMWARE_VERSION_INFORMATION    0x0FE   //4 bytes

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
    //Flag 0x01: Cell Over-Voltage Fault
    //Flag 0x02: Cell Under-Voltage Fault
    //Flag 0x04: Cell Over-Temperature Fault
    //Flag 0x08: Cell Under-Temperature Fault
    //Flag 0x10: Pack Over-Voltage Fault
    //Flag 0x20: Pack Under-Voltage Fault
    //Flag 0x40: Over-Current Discharge Fault
    //Flag 0x80: Over-Current Charge Fault
    ubyte1 faultFlags0;                         //0
    //Flag 0x01: Isolation Leakage Fault
    //Flag 0x02: BMS Monitor Communication Fault
    //Flag 0x04: Pre-charge Fault
    //Flag 0x08: Pack Discharge Operating Envelope Exceeded
    //Flag 0x10: Pack Charge Operating Envelope Exceeded
    //Flag 0x20: Failed Thermistor Fault
    //Flag 0x40: HVIL Fault
    //Flag 0x80: Emergency Stop Fault

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

    // BMS_CONFIGURATION_INFORMATION //
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

typedef struct _BatteryManagementSystem BatteryManagementSystem;

BatteryManagementSystem* BMS_new(ubyte2 canMessageBaseID);
void BMS_parseCanMessage(BatteryManagementSystem* bms, IO_CAN_DATA_FRAME* bmsCanMessage);

// BMS COMMANDS // 
IO_ErrorType BMS_relayControl(BatteryManagementSystem *me);

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower_uW(BatteryManagementSystem* me);                //microWatts (higher resolution)
sbyte4 BMS_getPower_W(BatteryManagementSystem* me);                 //Watts

typedef enum
{
    relayFault = 0x08,
    contactorK3Status = 0x04,
    contactorK2Status = 0x02,
    contactorK1Status = 0x01,
    faultState = 0x00,

} systemState;

typedef enum
{
    DrivingOffWhilePluggedIn = 0x01,	// Driving off while plugged in
    InterlockTripped = 0x02,			// Inter-lock is tripped
    CommunicationFault = 0x04,			// Communication fault with a bank or cell
    ChargeOverCurrent = 0x08,			// Charge over-current
    DischargeOverCurrent = 0x10,        // Discharge over-current
    OverTemperature = 0x20,				// Over-temperature fault
    UnderVoltage = 0x40,				// Under voltage
    OverVoltage = 0x80,					// Over voltage

    // CUSTOM MESSAGES //

    BMSNotDetected = 0x100,
    InitFailed = 0x200,

} faultOptions;

typedef enum
{
    StoredNoFault = 0x0,						// No fault
    StoredDrivingOffWhilePluggedIn = 0x01,		// Driving off while plugged in
    StoredInterlockTripped = 0x02,				// Interlock is tripped
    StoredCommFault = 0x03,						// Communication fault with a bank or cell
    StoredChargeOverCurrent = 0x04,				// Charge over-current
    StoredDischargeOverCurrent = 0x05,			// Discharge over-current
    StoredOverTemperature = 0x06,				// Over-temperature fault
    StoredUnderVoltage = 0x07,					// Under voltage
    StoredOverVoltage = 0x08,					// Over voltage

    StoredNoBatteryVoltage = 0x09,              // No battery voltage
    StoredHighVoltageBMinusLeak = 0xA,			// High voltage B- leak to chassis
    StoredHighVoltageBPlusLeak = 0xB,			// High voltage B+ leak to chassis
    StoredContactorK1Shorted = 0xC,				// Contactor K1 shorted
    StoredContactorK2Shorted = 0xD,				// Contactor K2 shorted
    StoredContactorK3Shorted = 0xE,				// Contactor K3 shorted
    StoredNoPrecharge = 0xF,					// No precharge
    StoredOpenK2 = 0x10,						// Open K2
    StoredExcessivePrechargeTime = 0x11,		// Excessive precharge time
    StoredEEPROMStackOverflow = 0x12,			// EEPROM stack overflow

} storedFaults;

typedef enum {
    /*
     * Using hex to represent each bit position
     */

    IOFlagPowerFromSource = 0x01,				// There is power from the source
    IOFlagPowerFromLoad = 0x02,					// There is power from the load
    IOFlagInterlockedTripped = 0x04,			// The inter-lick is tripped
    IOFlagHardWireContactorRequest = 0x08,		// There is a hard-wire contactor request
    IOFlagCANContactorRequest = 0x10,			// There is a CAN contactor request
    IOFlagHighLimitSet = 0x20,					// The HLIM is set
    IOFlagLowLimitSet = 0x40,					// The LLIM is set
    IOFlagFanIsOn = 0x80,						// The fan is on

} IOFlags;

typedef enum {
    LimitCauseErrorReadingValue = -1,
    LimitCauseNone = 0,							// No limit
    LimitCausePackVoltageTooLow,				// Pack voltage too low
    LimitCausePackVoltageTooHigh,				// Pack voltage too high
    LimitCauseCellVoltageTooLow,				// Cell voltage too low
    LimitCauseCellVoltageTooHigh,				// Cell voltage too high
    LimitCauseTempTooHighToCharge,				// Temperature too high for charging
    LimitCauseTempTooLowToCharge,				// Temperature too low for charging
    LimitCauseTempTooHighToDischarge,			// Temperature too high for discharging
    LimitCauseTempTooLowToDischarge,			// Temperature too low for discharging
    LimitCauseChargingCurrentPeakTooLong,		// Charging current peak lasting too long
    LimitCauseDischargingCurrentPeakTooLong,	// Discharging current peak lasted too long

} LimitCause;

#define ERROR_READING_LIMIT_VALUE = -1

#endif // _BATTERYMANAGEMENTSYSTEM_H



