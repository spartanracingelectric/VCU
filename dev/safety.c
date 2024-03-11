#include <stdlib.h> //malloc
//#include <math.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "IO_CAN.h"

#include "safety.h"
#include "mathFunctions.h"

#include "sensors.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"

#include "motorController.h"
#include "bms.h"

//TODO #162 Add in CAN Address to tell which Safeties are on or off

//----------------------------------------------------------------------------
//Faults
//last flag is 0x 8000 0000 (32 flags, 8 hex characters)
//----------------------------------------------------------------------------
//nibble 1
static const ubyte4 F_tpsOutOfRange = 1;
static const ubyte4 F_bpsOutOfRange = 2;
static const ubyte4 F_tpsPowerFailure = 4;
static const ubyte4 F_bpsPowerFailure = 8;
//nibble 2
static const ubyte4 F_tpsSignalFailure = 0x10;
static const ubyte4 F_bpsSignalFailure = 0x20;
static const ubyte4 F_tpsNotCalibrated = 0x40;
static const ubyte4 F_bpsNotCalibrated = 0x80;

//nibble 3
static const ubyte4 F_tpsOutOfSync = 0x100;
static const ubyte4 F_bpsOutOfSync = 0x200; //NOT USED
static const ubyte4 F_tpsbpsImplausible = 0x400;
//static const ubyte4 UNUSED = 0x800;

//nibble 4
static const ubyte4 F_softBSPDFault = 0x1000;
//static const ubyte4 F_ = 0x2000;
//static const ubyte4 F_ = 0x4000;
//static const ubyte4 F_ = 0x8000;

//nibble 5
static const ubyte4 F_lvsBatteryVeryLow = 0x10000;
//static const ubyte4 F_ = 0x20000;
//static const ubyte4 F_ = 0x40000;
//static const ubyte4 F_ = 0x80000;

//nibble 6
static const ubyte4 F_bmsOverVoltageFault = 0x100000;
static const ubyte4 F_bmsUnderVoltageFault = 0x200000;
static const ubyte4 F_bmsOverTemperatureFault = 0x400000;
//static const ubyte4 F_bmsOtherFault = 0x800000;

//nibble 7
static const ubyte4 F_bmsCellMismatchFault = 0x1000000;
//static const ubyte4 F_ = 0x2000000;
//static const ubyte4 F_ = 0x4000000;
//static const ubyte4 F_ = 0x8000000;

//nibble 8
//                             nibble: 87654321
static const ubyte4 F_unusedFaults = 0xFFFEF800;

//Warnings -------------------------------------------
static const ubyte2 W_lvsBatteryLow = 1;
static const ubyte2 W_hvilOverrideEnabled = 0x40; //This flag indicates HVIL bypass (MCM turn on)
static const ubyte2 W_safetyBypassEnabled = 0x80; //This flag controls the safety bypass
static const ubyte2 W_bmsOverVoltageWarning = 0x100; 
static const ubyte2 W_bmsUnderVoltageWarning = 0x200; 
static const ubyte2 W_bmsOverTemperatureWarning = 0x400; 

//Notices
static const ubyte2 N_HVILTermSenseLost = 1;

static const ubyte2 N_Over75kW_BMS = 0x10;
static const ubyte2 N_Over75kW_MCM = 0x20;

ubyte4 timestamp_SoftBSPD = 0;

/*****************************************************************************
* SafetyChecker object
******************************************************************************
* ToDo: change to ubyte1[8] (64 flags possible)
* 1 = fault
* 0 = no fault
****************************************************************************/
struct _SafetyChecker
{
    //Problems that require motor torque to be disabled
    ubyte4 faults;
    ubyte2 warnings;
    ubyte2 notices;
    ubyte1 maxAmpsCharge;
    ubyte1 maxAmpsDischarge;

    bool softBSPD_bpsHigh;
    bool softBSPD_kwHigh;
    bool softBSPD_fault;

    bool bypass;
    ubyte4 timestamp_bypassSafetyChecks;
    ubyte4 bypassSafetyChecksTimeout_us;
};

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
SafetyChecker *SafetyChecker_new(ubyte2 maxChargeAmps, ubyte2 maxDischargeAmps)
{
    SafetyChecker *me = (SafetyChecker *)malloc(sizeof(struct _SafetyChecker));
    me->faults = 0;
    me->warnings = 0;

    me->maxAmpsCharge = maxChargeAmps;
    me->maxAmpsDischarge = maxDischargeAmps;

    me->softBSPD_bpsHigh = TRUE;
    me->softBSPD_kwHigh = TRUE;
    me->softBSPD_fault = TRUE;

    me->bypass = FALSE;
    me->timestamp_bypassSafetyChecks = 0;
    me->bypassSafetyChecksTimeout_us = 500000; //If safety bypass command is not neceived in this time then safety is re-enabled
    //Note: The safety bypass warning flag is the determining factor in bypassing the multiplier.
    return me;
}

void SafetyChecker_parseCanMessage(SafetyChecker *me, IO_CAN_DATA_FRAME *canMessage)
{
    switch (canMessage->id)
    {
    case 0x5FF:
        //If the safety bypass code (0xC4) is received on the VCU debug address (0x5FF) at byte 0 (data[0])
        if (canMessage->data[0] == 0xC4)
        {
            IO_RTC_StartTime(&me->timestamp_bypassSafetyChecks);
        }
        break;
    }
}

//Updates all values based on sensor readings, safety checks, etc
void SafetyChecker_update(SafetyChecker *me, MotorController *mcm, BatteryManagementSystem *bms, TorqueEncoder *tps, BrakePressureSensor *bps, Sensor *HVILTermSense, Sensor *LVBattery)
{
    ubyte1 *message[50]; //For sprintf'ing variables to print in serial
    //SerialManager_send(me->serialMan, "Entered SafetyChecker_update().\n");
    /*****************************************************************************
    * Faults
    ****************************************************************************/
    //===================================================================
    // Get calibration status
    //===================================================================
    //me->faults = 0xFFFF; //Set ALL faults by default.  only clear if truly safe
    me->faults &= ~F_unusedFaults;

    //me->faults = 1;
    if (tps->calibrated == FALSE)
    {
        me->faults |= F_tpsNotCalibrated;
    }
    else
    {
        me->faults &= ~F_tpsNotCalibrated;
    }

    if (bps->calibrated == FALSE)
    {
        me->faults |= F_bpsNotCalibrated;
    }
    else
    {
        me->faults &= ~F_bpsNotCalibrated;
    }

    //===================================================================
    // Check if VCU was able to get a TPS/BPS reading
    //===================================================================
    if (tps->tps0->ioErr_powerInit != IO_E_OK || tps->tps1->ioErr_powerInit != IO_E_OK || tps->tps0->ioErr_powerSet != IO_E_OK || tps->tps1->ioErr_powerSet != IO_E_OK)
    {
        me->faults |= F_tpsPowerFailure;
    }
    else
    {
        me->faults &= ~F_tpsPowerFailure;
    }

    if (tps->tps0->ioErr_signalInit != IO_E_OK || tps->tps1->ioErr_signalInit != IO_E_OK || tps->tps0->ioErr_signalGet != IO_E_OK || tps->tps1->ioErr_signalGet != IO_E_OK)
    {
        //me->faults |= F_tpsSignalFailure;
    }
    else
    {
        me->faults &= ~F_tpsSignalFailure;
    }

    if (bps->bps0->ioErr_powerInit != IO_E_OK || bps->bps0->ioErr_powerSet != IO_E_OK)
    {
        me->faults |= F_bpsPowerFailure;
    }
    else
    {
        me->faults &= ~F_bpsPowerFailure;
    }

    if (bps->bps0->ioErr_signalInit != IO_E_OK || bps->bps0->ioErr_signalGet != IO_E_OK)
    {
        me->faults |= F_bpsSignalFailure;
    }
    else
    {
        me->faults &= ~F_bpsSignalFailure;
    }

    //===================================================================
    // Make sure raw sensor readings are within operating range
    //===================================================================
    //RULE: EV2.3.10 - signal outside of operating range is considered a failure
    //  This refers to SPEC SHEET values, not calibration values
    //Note: IC cars may continue to drive for up to 100ms until valid readings are restored, but EVs must immediately cut power
    //Note: We need to decide how to report errors and how to perform actions when those errors occur.  For now, I'm calling an imaginary Err.Report function

    //-------------------------------------------------------------------
    //Torque Encoder
    //-------------------------------------------------------------------
    if (tps->tps0->sensorValue < tps->tps0->specMin || tps->tps0->sensorValue > tps->tps0->specMax || tps->tps1->sensorValue < tps->tps1->specMin || tps->tps1->sensorValue > tps->tps1->specMax)
    {
        // me->faults |= F_tpsOutOfRange;
    }
    else
    {
        me->faults &= ~F_tpsOutOfRange;
    }

    //-------------------------------------------------------------------
    //Brake Pressure Sensor
    //-------------------------------------------------------------------
    if (bps->bps0->sensorValue < bps->bps0->specMin || bps->bps0->sensorValue > bps->bps0->specMax)
    {
        // me->faults |= F_bpsOutOfRange;
    }
    else
    {
        me->faults &= ~F_bpsOutOfRange;
    }

    //===================================================================
    // Make sure calibrated TPS readings are in sync with each other
    //===================================================================
    // EV2.3.5 If an implausibility occurs between the values of these two sensors
    //  the power to the motor(s) must be immediately shut down completely. It is not necessary
    //  to completely deactivate the tractive system, the motor controller(s) shutting down the
    //  power to the motor(s) is sufficient.
    // EV2.3.6 Implausibility is defined as a deviation of more than 10 % pedal travel between the sensors.
    //-------------------------------------------------------------------

    //Check for implausibility (discrepancy > 10%)
    //RULE: EV2.3.6 Implausibility is defined as a deviation of more than 10% pedal travel between the sensors.
    float4 tps0Percent; //Pedal percent float (a decimal between 0 and 1
    float4 tps1Percent;

    TorqueEncoder_getIndividualSensorPercent(tps, 0, &tps0Percent); //borrow the pedal percent variable
    TorqueEncoder_getIndividualSensorPercent(tps, 1, &tps1Percent);

    if ((tps1Percent - tps0Percent) > .1 || (tps1Percent - tps0Percent) < -.1) //Note: Individual TPS readings don't go negative, otherwise this wouldn't work
    {
        // me->faults |= F_tpsOutOfSync;
    }
    else
    {
        me->faults &= ~F_tpsOutOfSync;
    }

    //Only one BPS right now - this fault doesn't happen
    me->faults &= ~F_bpsOutOfSync;

    //===================================================================
    // 2024 Rev 1 EV.4.7 APPS / Brake Pedal Plausibility Check
    //===================================================================
    // Must monitor for the two conditions:
    //  • The mechanical brakes are engaged EV.4.6, T.3.2.4
    //  • The APPS signals more than 25% Pedal Travel EV.4.5
    //  EV.4.7.2 If the two conditions in EV.4.7.1 occur at the same time:
    //      a. Power to the Motor(s) must be immediately and completely shut down
    //      b. The Motor shut down must stay active until the APPS signals less than 5% Pedal Travel, with or without brake operation
    // EV.5.7.2 The Motor shut down must remain active until the APPS signals less than 5% pedal travel, with or without brake operation.
    //-------------------------------------------------------------------
    bool tpsAbove25Percent = (tps->travelPercent > .25); //Rules is 25% this is a hack that is made to check

    //If mechanical brakes actuated && tps > 25%
    if (bps->brakesAreOn && tpsAbove25Percent)
    {
        // Set the TPS/BPS implaisibility VCU fault
        me->faults |= F_tpsbpsImplausible;
    }
    else if (tps->travelPercent < .05) //TPS is reduced to < 5%
    {
        // There is no implausibility if TPS is below 5%
        me->faults &= ~(F_tpsbpsImplausible); //turn off the implausibility flag
    }

    //===================================================================
    // 2022 EV.8.3 / Accumulator Management System Fault
    //===================================================================
    // EV.8.3.4 The AMS must monitor for:
    //      a. Voltage values outside the allowable range EV.8.4.2              - Yup
    //      b. Voltage sense Overcurrent Protection device(s) blown or tripped  - Yup?
    //      c. Temperature values outside the allowable range EV.8.5.2          - Yup
    //      d. Missing or interrupted voltage or temperature measurements       - Yup?
    //      e. A fault in the AMS                                               - Yup
    // EV.8.3.5 If the AMS detects one or more of the conditions of EV.8.3.4 above, the AMS must:
    //      a. Open the Shutdown Circuit EV.8.2.2 (drive fault IO)              - TODO
    //      b. Turn on the AMS Indicator Light (handled by Shutdown circuit)    - Handled by Shutdown Circuit
    //-------------------------------------------------------------------

    //If over voltage fault detected
    if (BMS_getFaultFlags1(bms) & BMS_CELL_OVER_VOLTAGE_FLAG)
    {
        //me->faults |= F_bmsOverVoltageFault;
    }
    else
    {
        me->faults &= ~(F_bmsOverVoltageFault);
    }

    //If under voltage fault detected
    if (BMS_getFaultFlags1(bms) & BMS_CELL_UNDER_VOLTAGE_FLAG)
    {
        me->faults |= F_bmsUnderVoltageFault;
    }
    else
    {
        me->faults &= ~(F_bmsUnderVoltageFault);
    }

    //If over temperature fault detected
    if (BMS_getFaultFlags1(bms) & BMS_CELL_OVER_TEMPERATURE_FLAG)
    {
        me->faults |= (F_bmsOverTemperatureFault);
    }
    else
    {
        me->faults &= ~(F_bmsOverTemperatureFault);
    }

    //If mismatch greater than specified mismatch value
    //BMS cell voltage data members are in mV, 
    if ( (BMS_getHighestCellVoltage_mV(bms)-BMS_getLowestCellVoltage_mV(bms)) > (BMS_MAX_CELL_MISMATCH_V*1000) )
    {
        //me->faults |= F_bmsCellMismatchFault;
    }
    else
    {
        me->faults &= ~(F_bmsCellMismatchFault);
    }

    /*****************************************************************************
    * Warnings
    ****************************************************************************/
    //===================================================================
    // LVS Battery Check - FAULTS LATCH UNTIL RETURN TO PREVIOUS STAGE
    //===================================================================
    //  IO_ADC_UBAT: 0..40106  (0V..40.106V)
    //-------------------------------------------------------------------
    if (LVBattery->sensorValue <= 9200) //12730 = 10% SOC but hard to tell under load. 9200 = empty
    {
        me->faults |= F_lvsBatteryVeryLow;
        me->warnings |= W_lvsBatteryLow;
    }
    else if (LVBattery->sensorValue <= 12730) //13100 = Recharge percentage, per Shorai
    {
        me->faults &= ~F_lvsBatteryVeryLow;
        me->warnings |= W_lvsBatteryLow;
    }
    else
    {
        me->warnings &= ~F_lvsBatteryVeryLow;
        me->warnings &= ~W_lvsBatteryLow;
    }

    
    //===================================================================
    // softBSPD - Software implementation of Brake System Plausibility Device
    //===================================================================
    // Danny (e-tech judge) suggested implementing a BSPD check in software as
    // an alternative to making a hardware adjustment to the BSPD requiring us to re-tech.
    // By tripping a software BSPD slightly before the actual BSPD would trip, we can avoid
    // the BSPD actually shutting down the car and ending our run.
    // 2021 BSPD looks for 2.75V from BPS
    // and ?? kW (approximation from a current sensor assuming nominal pack voltage)
    // We will use 2.0V (TBD) BPS voltage and even the tiniest amount of torque command
    
    // Formula for relating kW to Nm:
    //(kW) = torque (Nm) x speed (RPM) / 9.5488
    
    // MCM readings <--> REQUESTED torque * rpm / 9.5488
    // 259*158 = 225 * 2556 / 9.5488
    // 40922 = 60227 <-- This discrepancy is because we don't get all of the requested torque 


    me->softBSPD_bpsHigh = bps->bps0->sensorValue > 1900;
    me->softBSPD_kwHigh = MCM_getPower(mcm) > 4000;

    // Note: this is using the FUTURE torque request with the PREVIOUS RPM
    if (me->softBSPD_bpsHigh && me->softBSPD_kwHigh)
    {
        IO_RTC_StartTime(&timestamp_SoftBSPD);
        me->softBSPD_fault = FALSE;
        //me->faults |= F_softBSPDFault;
        // Light_set(Light_dashEco, 1);  // For testing only
    }
    else if (IO_RTC_GetTimeUS(timestamp_SoftBSPD) >= 500000 || IO_RTC_GetTimeUS(timestamp_SoftBSPD) == 0)
    {
        timestamp_SoftBSPD = 0;
        me->softBSPD_fault = FALSE;
        me->faults &= ~F_softBSPDFault;
        // Light_set(Light_dashEco, 0);  // For testing only
    }


    //===================================================================
    // Safety checker bypass
    //===================================================================
    // The safety checker should only be bypassed by a CAN message sent by
    // the PCAN Explorer dashboard.  This is only used during debugging.
    //-------------------------------------------------------------------
    //In case CAN communication is lost, the bypass should be disabled after some time,
    if (IO_RTC_GetTimeUS(me->timestamp_bypassSafetyChecks) < me->bypassSafetyChecksTimeout_us)
    {
        me->warnings |= W_safetyBypassEnabled;
    }
    else
    {
        me->warnings &= ~W_safetyBypassEnabled;
    }

    //===================================================================
    // HVIL Override
    //===================================================================
    if (MCM_getHvilOverrideStatus(mcm) == TRUE)
    {
        me->warnings |= W_hvilOverrideEnabled;
    }
    else
    {
        me->warnings &= ~W_hvilOverrideEnabled;
    }

    //===================================================================
    // 2022 EV.8.3 / Accumulator Management System Warning
    //===================================================================

    //If under voltage fault detected
    if (BMS_getLowestCellVoltage_mV(bms) < (BMS_MIN_CELL_VOLTAGE_WARNING*BMS_VOLTAGE_SCALE))
    {
        // me->warnings |= W_bmsUnderVoltageWarning;
    }
    else
    {
        me->warnings &= ~(W_bmsUnderVoltageWarning);
    }

    //If over temperature fault detected
    if (BMS_getHighestCellTemp_d_degC(bms) > (BMS_MAX_CELL_TEMPERATURE_WARNING*BMS_TEMPERATURE_SCALE))
    {
        me->warnings |= W_bmsOverTemperatureWarning;
    }
    else
    {
        me->warnings &= ~(W_bmsOverTemperatureWarning);
    }

    /*****************************************************************************
    * Notices
    ****************************************************************************/

    //===================================================================
    // HVIL Term Sense Check
    //===================================================================
    // If HVIL term sense goes low (because HV went down), motor torque
    // command should be set to zero before turning off the controller
    //-------------------------------------------------------------------
    if (HVILTermSense->sensorValue == FALSE)
    {
        me->notices |= N_HVILTermSenseLost;
    }
    else
    {
        me->notices &= ~N_HVILTermSenseLost;
    }

    if (BMS_getPower_W(bms) > 75000)
    {
        me->notices |= N_Over75kW_BMS;
    }
    else
    {
        me->notices &= ~N_Over75kW_BMS;
    }

    if (MCM_getPower(mcm) > 75000)
    {
        me->notices |= N_Over75kW_MCM;
    }
    else
    {
        me->notices &= ~N_Over75kW_MCM;
    }
}

//Updates all values based on sensor readings, safety checks, etc
bool SafetyChecker_allSafe(SafetyChecker *me)
{
    return (me->faults == 0);
}

//Updates all values based on sensor readings, safety checks, etc
ubyte4 SafetyChecker_getFaults(SafetyChecker *me)
{
    return (me->faults);
}

//Updates all values based on sensor readings, safety checks, etc
ubyte4 SafetyChecker_getWarnings(SafetyChecker *me)
{
    return (me->warnings);
}

//Updates all values based on sensor readings, safety checks, etc
ubyte4 SafetyChecker_getNotices(SafetyChecker *me)
{
    return (me->notices);
}

void SafetyChecker_reduceTorque(SafetyChecker *me, MotorController *mcm, BatteryManagementSystem *bms, WheelSpeeds *wss)
{
    float4 multiplier = 1;
    //float4 tempMultiplier = 1;
    //Get ground speed in KPH using only FL WSS
    //sbyte1 groundSpeedKPH = (sbyte1)WheelSpeeds_getGroundSpeedKPH(wss, 1);
    sbyte2 groundSpeedKPH = MCM_getGroundSpeedKPH(mcm);


    if (me->faults > 0) //Any VCU fault exists
    {
        multiplier = 0;
    }
    
    // If HVIL is open, we must command 0 torque before opening the motor controller relay
    if ((me->notices & N_HVILTermSenseLost) > 0)
    {
       multiplier = 0;
    }    

    if ((me->warnings & W_safetyBypassEnabled) == W_safetyBypassEnabled)
    {
        multiplier = 1;
    }
    MCM_commands_setTorqueDNm(mcm, MCM_commands_getTorque(mcm) * multiplier);
}

//-------------------------------------------------------------------
// 80kW Limit Check
//-------------------------------------------------------------------
//Change this to return a multiplier instead of torque value
//ubyte2 checkPowerDraw(BatteryManagementSystem* bms, MotorController* mcm)
//{
//    ubyte2 torqueThrottle = 0;
//
//    // if either the bms or mcm goes over 75kw, limit torque
//    if ((BMS_getPower(bms) > 75000) || (MCM_getPower(mcm) > 75000))
//    {
//        // using bmsPower since closer to e-meter
//        torqueThrottle = MCM_getCommandedTorque(mcm) - (((BMS_getPower(bms) - 80000) / 80000) * MCM_getCommandedTorque(mcm));
//    }
//
//    return torqueThrottle;
//}