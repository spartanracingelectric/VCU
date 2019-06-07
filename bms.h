#ifndef _BATTERYMANAGEMENTSYSTEM_H
#define _BATTERYMANAGEMENTSYSTEM_H

/**************************************************************************
 * 	REVISION HISTORY:
 *
 *	2016-5-11 - Rabeel Elahi - Added bms_commands_getPower();
 *							 - Added bms_commands_getPackTemp();
 *
 *	2016-4-6  - Rabeel Elahi - Added constructor and BMS data struct
 *							 - Initially added helper functions to update variables,
 *							   but decided to update variables by passing BMS pointer to
 *							   canInput_readMesagges(BMS* bms). Commented out for future use.
 *							 - Added functions to help with endian conversion.
 *							   Possibly move to mathFunctions?
 *
 *	2016-3-28 - Rabeel Elahi - Created this file.
 *
 *
 **************************************************************************/

#include <stdio.h>
#include <stdint.h>

#include "serial.h"
#include "IO_CAN.h"


typedef struct _BatteryManagementSystem BatteryManagementSystem;

BatteryManagementSystem* BMS_new(SerialManager* serialMan, ubyte2 canMessageBaseID);
void BMS_parseCanMessage(BatteryManagementSystem* bms, IO_CAN_DATA_FRAME* bmsCanMessage);

// BMS COMMANDS // 

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower(BatteryManagementSystem* me);
ubyte2 BMS_getPackTemp(BatteryManagementSystem* me);
sbyte1 BMS_getAvgTemp(BatteryManagementSystem* me);
sbyte1 BMS_getMaxTemp(BatteryManagementSystem* me);

ubyte1 BMS_getCCL(BatteryManagementSystem* me);
ubyte1 BMS_getDCL(BatteryManagementSystem* me);

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






//
//// ELITHION BMS OPTIONS //
//
//ubyte1  updateState();
//ubyte2 updateTimer();
//ubyte1  updateFlags();
//ubyte1  updateFaultCode();
//ubyte1  updateLevelFaults();
//
//// PACK //
//
//ubyte2 updatePackVoltage(); 	// volts
//ubyte1  updateMinVtg(); 		// volts; individual cell voltage
//ubyte1  updateMaxVtg();
//ubyte1  updateMinVtgCell();
//ubyte1  updateMaxVtgCell();
//
//
//// CURRENT //
//
//sbyte2  updatePackCurrent(); 	 			// amps
//ubyte2 updateChargeLimit();				// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//ubyte2 updateDischargeLimit();			// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//
//// BATTERY //
//
//ubyte4 batteryEnergyIn();
//ubyte4 batteryEnergyOut();
//
//
//ubyte1  updateSOC();
//ubyte2 updateDOD();
//ubyte2 updateCapacity();
//ubyte1  updateSOH();
//
//// TEMP //
//
//sbyte1  updatePackTemp();			     // average pack temperature
//sbyte1  updateMinTemp();			     // Temperature of coldest sensor
//sbyte1  updateMinTempCell(); 		     // ID of cell with lowest temperature
//sbyte1  updateMaxTemp();			     // Temperature of hottest sensor
//sbyte1  updateMaxTempCell(); 		     // ID of cell with highest temperature
//
//
//// RESISTANCE //
//
//ubyte2 updatePackRes();				// resistance of entire pack
//ubyte1  updateMinRes();  			// resistance of lowest resistance cells
//ubyte1  updateMinResCell();          // ID of cell with lowest resistance
//ubyte1  updateMaxRes();				// resistance of highest resistance cells
//ubyte1  updateMaxResCell();			// ID of cell with highest resistance
//
//LimitCause updateChargeLimitCause();
//LimitCause updateDischargeLimitCause();

//
////void getFaults(FaultOptions *presentFaults, StoredFault *storedFault, FaultOptions *presentWarnings);
//void clearStoredFault();
//
//IOFlags getIOFlags();
//


#endif // _BATTERYMANAGEMENTSYSTEM_H



