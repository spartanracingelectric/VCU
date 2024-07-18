#ifndef _BATTERYMANAGEMENTSYSTEM_H
#define _BATTERYMANAGEMENTSYSTEM_H

#include <stdio.h>
#include <stdint.h>

#include "serial.h"
#include "IO_CAN.h"

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
#define BMS_CONFIGUATION_INFORMATION        0x0FC   //8 bytes
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

typedef struct _BatteryManagementSystem BatteryManagementSystem;

BatteryManagementSystem* BMS_new(SerialManager* serialMan, ubyte2 canMessageBaseID);
void BMS_parseCanMessage(BatteryManagementSystem* bms, IO_CAN_DATA_FRAME* bmsCanMessage);

// BMS COMMANDS // 

IO_ErrorType BMS_relayControl(BatteryManagementSystem *me);
bool BMS_getRelayState(BatteryManagementSystem *me);

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower_uW(BatteryManagementSystem* me);                //microWatts (higher resolution)
sbyte4 BMS_getPower_W(BatteryManagementSystem* me);                 //Watts
ubyte2 BMS_getPackTemp(BatteryManagementSystem* me);
sbyte1 BMS_getAvgTemp(BatteryManagementSystem* me);
ubyte4 BMS_getHighestCellVoltage_mV(BatteryManagementSystem *me);   //Millivolts
ubyte2 BMS_getLowestCellVoltage_mV(BatteryManagementSystem *me);   //Millivolts
sbyte2 BMS_getHighestCellTemp_d_degC(BatteryManagementSystem* me);  //deciCelsius (higher resolution)
sbyte2 BMS_getHighestCellTemp_degC(BatteryManagementSystem* me);    //Celsius
ubyte1 BMS_getFaultFlags0(BatteryManagementSystem *me);
ubyte1 BMS_getFaultFlags1(BatteryManagementSystem *me);
ubyte4 BMS_getPackVoltage(BatteryManagementSystem *me); //Millivolts

ubyte1 BMS_getCCL(BatteryManagementSystem* me);
ubyte1 BMS_getDCL(BatteryManagementSystem* me);

ubyte4 BMS_getPackCurrent(BatteryManagementSystem *me);
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
    CommuncationFault = 0x04,			// Communication fault with a bank or cell
    ChargeOverCurrent = 0x08,			// Charge over-current
    DischargeOverCurrent = 0x10,        // Discharge over-current
    OverTemperture = 0x20,				// Over-temperature fault
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
    StoredInterockTripped = 0x02,				// Interlock is tripped
    StoredCommFault = 0x03,						// Communication fault with a bank or cell
    StoredChargeOverCurrent = 0x04,				// Charge over-current
    StoredDischargeOverCurrent = 0x05,			// Dishcarge over-current
    StoredOverTemperture = 0x06,				// Over-temperature fault
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




typedef enum LimitCause{

    LimitCauseErrorReadingValue = -1,
    LimitCauseNone = 0,							// No limit
    LimitCausePackVoltageTooLow,				// Pack voltage too low
    LimitCausePackVolageTooHigh,				// Pack voltage too high
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



