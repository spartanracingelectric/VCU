#include <stdlib.h> //malloc
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
#include "serial.h"
#include "main.h"

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

extern Sensor Sensor_LVBattery;
extern Button Sensor_HVILTerminationSense;

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
SafetyChecker *SafetyChecker_new(SerialManager *sm, ubyte2 maxChargeAmps, ubyte2 maxDischargeAmps)
{
    SafetyChecker *me = (SafetyChecker *)malloc(sizeof(struct _SafetyChecker));

    me->serialMan = sm;
    me->faults = 0;
    me->warnings = 0;

    me->maxAmpsCharge = maxChargeAmps;
    me->maxAmpsDischarge = maxDischargeAmps;

    me->softBSPD_bpsHigh = TRUE;
    me->softBSPD_kwHigh = TRUE;
    me->softBSPD_fault = TRUE;

    me->bypass = FALSE;
    me->timestamp_bypassSafetyChecks = 0;
    me->bypassSafetyChecksTimeout_us = 500000; //If safety bypass command is not received in this time then safety is re-enabled
    // Note: The safety bypass warning flag is the determining factor in bypassing the multiplier.
    return me;
}

// Updates all values based on sensor readings, safety checks, etc
void SafetyChecker_update(SafetyChecker *me, MotorController *mcm, BatteryManagementSystem *bms, TorqueEncoder *tps, BrakePressureSensor *bps)
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

    set_flags(&me->faults, F_tpsNotCalibrated, tps->calibrated == FALSE);

    set_flags(&me->faults, F_bpsNotCalibrated, bps->calibrated == FALSE);

    //===================================================================
    // Check if VCU was able to get a TPS/BPS reading
    //===================================================================
    set_flags(&me->faults, F_tpsPowerFailure,  tps->tps0->ioErr_powerSet != IO_E_OK || tps->tps1->ioErr_powerSet != IO_E_OK); // tps->tps0->ioErr_powerInit != IO_E_OK || tps->tps1->ioErr_powerInit != IO_E_OK || 
    
    set_flags(&me->faults, F_tpsSignalFailure, tps->tps0->ioErr_signalInit != IO_E_OK || tps->tps1->ioErr_signalInit != IO_E_OK || tps->tps0->ioErr_signalGet != IO_E_OK || tps->tps1->ioErr_signalGet != IO_E_OK);

    set_flags(&me->faults, F_bpsPowerFailure, bps->bps0->ioErr_powerSet != IO_E_OK); // bps->bps0->ioErr_powerInit != IO_E_OK || 
    
    set_flags(&me->faults, F_bpsSignalFailure, bps->bps0->ioErr_signalInit != IO_E_OK || bps->bps0->ioErr_signalGet != IO_E_OK);

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
    set_flags(&me->faults, F_tpsOutOfRange, tps->tps0->sensorValue < tps->tps0->specMin || tps->tps0->sensorValue > tps->tps0->specMax || tps->tps1->sensorValue < tps->tps1->specMin || tps->tps1->sensorValue > tps->tps1->specMax);

    //-------------------------------------------------------------------
    //Brake Pressure Sensor
    //-------------------------------------------------------------------
    set_flags(&me->faults, F_bpsOutOfRange, bps->bps0->sensorValue < bps->bps0->specMin || bps->bps0->sensorValue > bps->bps0->specMax);

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

    set_flags(&me->faults, F_tpsOutOfSync, (tps->tps1_percent - tps->tps0_percent) > .1 || (tps->tps1_percent - tps->tps0_percent) < -.1);

    //Only one BPS right now - this fault doesn't happen
    me->faults &= ~F_bpsOutOfSync;

    //===================================================================
    // 2021 EV.5.7 APPS / Brake Pedal Plausibility Check
    //===================================================================
    // EV.5.7.1 The power to the Motor(s) must be immediately and completely shut down when both of the following exist at the same time:
    //     • The mechanical brakes are actuated
    //     • The APPS signals more than 25% pedal travel
    //     This must be demonstrated at Technical Inspection
    // EV.5.7.2 The Motor shut down must remain active until the APPS signals less than 5% pedal travel, with or without brake operation.
    //-------------------------------------------------------------------

    //If mechanical brakes actuated && tps > 25%
    if (bps->brakesAreOn && (tps->travelPercent > .25) && SOFT_BSPD_ENABLE)
    {
        // Set the TPS/BPS implausibility VCU fault
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
    set_flags(&me->faults, F_bmsOverVoltageFault, bms->faultFlags1 & BMS_CELL_OVER_VOLTAGE_FLAG);

    //If under voltage fault detected
    set_flags(&me->faults, F_bmsUnderVoltageFault, bms->faultFlags1 & BMS_CELL_UNDER_VOLTAGE_FLAG);

    //If over temperature fault detected
    set_flags(&me->faults, F_bmsOverTemperatureFault, bms->faultFlags1 & BMS_CELL_OVER_TEMPERATURE_FLAG);

    //If mismatch greater than specified mismatch value
    //BMS cell voltage data members are in mV
    // NOTE: This should be redone to take into account voltage sag under load
    // set_flags(&me->faults, F_bmsCellMismatchFault, (bms->highestCellVoltage-bms->lowestCellVoltage) > (BMS_MAX_CELL_MISMATCH_V*1000));

    /*****************************************************************************
    * Warnings
    ****************************************************************************/
    //===================================================================
    // LVS Battery Check - FAULTS LATCH UNTIL RETURN TO PREVIOUS STAGE
    //===================================================================
    //  IO_ADC_UBAT: 0..40106  (0V..40.106V)
    //-------------------------------------------------------------------
    set_flags(&me->faults, F_lvsBatteryVeryLow, FALSE); // This should be based on SOC, can be enabled later
    set_flags(&me->warnings, W_lvsBatteryLow, FALSE); // This should be based on SOC, can be enabled later

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
    // (kW) = torque (Nm) x speed (RPM) / 9.5488
    
    // MCM readings <--> REQUESTED torque * rpm / 9.5488
    // 259*158 = 225 * 2556 / 9.5488
    // 40922 = 60227 <-- This discrepancy is because we don't get all of the requested torque 

    me->softBSPD_bpsHigh = bps->bps0->sensorValue > 1900;
    me->softBSPD_kwHigh = MCM_getPower(mcm) > 4000;

    // Note: this is using the FUTURE torque request with the PREVIOUS RPM
    if (me->softBSPD_bpsHigh && me->softBSPD_kwHigh && SOFT_BSPD_ENABLE)
    {
        IO_RTC_StartTime(&timestamp_SoftBSPD);
        me->softBSPD_fault = TRUE;
        me->faults |= F_softBSPDFault;
    }
    else if (IO_RTC_GetTimeUS(timestamp_SoftBSPD) >= 500000 || IO_RTC_GetTimeUS(timestamp_SoftBSPD) == 0)
    {
        timestamp_SoftBSPD = 0;
        me->softBSPD_fault = FALSE;
        me->faults &= ~F_softBSPDFault;
    }

    //===================================================================
    // Safety checker bypass
    //===================================================================
    // The safety checker should only be bypassed by a CAN message sent by
    // the PCAN Explorer dashboard.  This is only used during debugging.
    //-------------------------------------------------------------------
    // In case CAN communication is lost, the bypass should be disabled after some time,
    set_flags(&me->warnings, W_safetyBypassEnabled, IO_RTC_GetTimeUS(me->timestamp_bypassSafetyChecks) < me->bypassSafetyChecksTimeout_us);

    //===================================================================
    // HVIL Override
    //===================================================================
    set_flags(&me->warnings, W_hvilOverrideEnabled, mcm->HVILOverride == TRUE);

    //===================================================================
    // 2022 EV.8.3 / Accumulator Management System Warning
    //===================================================================

    //If under voltage fault detected
    set_flags(&me->warnings, W_bmsUnderVoltageWarning, bms->lowestCellVoltage < (BMS_MIN_CELL_VOLTAGE_WARNING*BMS_VOLTAGE_SCALE));

    //If over temperature fault detected
    set_flags(&me->warnings, W_bmsOverTemperatureWarning, bms->highestCellTemperature > (BMS_MAX_CELL_TEMPERATURE_WARNING*BMS_TEMPERATURE_SCALE));

    /*****************************************************************************
    * Notices
    ****************************************************************************/

    //===================================================================
    // HVIL Term Sense Check
    //===================================================================
    // If HVIL term sense goes low (because HV went down), motor torque
    // command should be set to zero before turning off the controller
    //-------------------------------------------------------------------
    set_flags(&me->notices, N_HVILTermSenseLost, Sensor_HVILTerminationSense.sensorValue == FALSE);

    set_flags(&me->notices, N_Over75kW_BMS, BMS_getPower_W(bms) > 75000);
    
    set_flags(&me->notices, N_Over75kW_MCM, MCM_getPower(mcm) > 75000);
}

void SafetyChecker_reduceTorque(SafetyChecker *me, MotorController *mcm, BatteryManagementSystem *bms, WheelSpeeds *wss)
{
    float4 multiplier = 1;
    //float4 tempMultiplier = 1;
    //Get ground speed in KPH using only FL WSS
    //sbyte1 groundSpeedKPH = (sbyte1)WheelSpeeds_getGroundSpeedKPH(wss, 1);
    sbyte2 groundSpeedKPH = MCM_getGroundSpeedKPH(mcm);


    //-------------------------------------------------------------------
    // Critical conditions - set 0 torque
    //-------------------------------------------------------------------

    //if ((me->warnings & W_bmsOverTemperatureWarning) > 0)
    //{
    //   multiplier = 0.80;
    //}

    //if ((me->warnings & W_bmsUnderVoltageWarning) > 0)
    //{
    //   multiplier = 0.70;
    //}

    if (me->faults > 0) //Any VCU fault exists
    {
        multiplier = 0;
    }
    
    // If HVIL is open, we must command 0 torque before opening the motor controller relay
    if ((me->notices & N_HVILTermSenseLost) > 0)
    {
       multiplier = 0;
       SerialManager_send(me->serialMan, "HVIL term sense low\n");
    }    

    //-------------------------------------------------------------------
    // Other limits (% reduction) - set torque to the lowest of all these
    // IMPORTANT: Be aware of direction-sensitive situations (accel/regen)
    //-------------------------------------------------------------------
    //80kW limit ---------------------------------
    // if either the bms or mcm goes over 75kw, limit torque
    //////////if ((BMS_getPower(bms) > 75000) || (MCM_getPower(mcm) > 75000))
    //////////{
    //////////    // using bmsPower since closer to e-meter
    //////////    tempMultiplier = 1 - getPercent(max(BMS_getPower(bms), MCM_getPower(mcm)), 75000, 80000, TRUE);
    //////////    SerialManager_send(me->serialMan, "SC.Mult: 80kW\n");
    //////////}
    //////////if (tempMultiplier < multiplier) { multiplier = tempMultiplier; }

    //CCL/DCL from BMS --------------------------------
    //why the DCL/CCL could be limited:
    //0: No limit
    //1 : Pack voltage too low
    //2 : Pack voltage high
    //3 : Cell voltage low
    //4 : Cell voltage high
    //5 : Temperature high for charging
    //6 : Temperature too low for charging
    //7 : Temperature high for discharging
    //8 : Temperature too low for discharging
    //9 : Charging current peak lasted too long
    //10 = A : Discharging current peak lasted too long
    //11 = B : Power up delay(Charge testing)
    //12 = C : Fault
    //13 = D : Contactors are off
    ////////if (MCM_commands/*_getTorque(mcm) >= 0)
    ////////{*/
    ///////////*tempMultiplier = getPercent(BMS_getDCL(bms), 0, me->maxAmpsDischarge, TRUE);
    //////////if (tempMultiplier < 1)
    //////////{
    //////////    SerialManager_send(me->serialMan, "SC.Mult: DCL\n");
    //////////}*/
    //////////}
    //////////else //regen - Pick the lowest of CCL and speed reductions
    //////////{
    //////////    tempMultiplier = getPercent(BMS_getCCL(bms), 0, me->maxAmpsCharge, TRUE);
    //////////    if (tempMultiplier < 1)
    //////////    {
    //////////        SerialManager_send(me->serialMan, "SC.Mult: CCL\n");
    //////////    }
    //////////    //Also, regen should be ramped down as speed approaches minimum
    
    
    //////////        if (tempMultiplier < 1) { SerialManager_send(me->serialMan, "SC.Mult: Regen < 15kph\n"); }
    //////////    }
    ////////if (tempMultiplier < multiplier) { multiplier = tempMultiplier; }

   /* if ( groundSpeedKPH < MCM_getRegenRampdownStartSpeed(mcm))
    {
        float4 regenMultiplier = 1 - getPercent(groundSpeedKPH, MCM_getRegenMinSpeed(mcm), MCM_getRegenRampdownStartSpeed(mcm), TRUE);
        //float4 regenMultiplier = 1 - getPercent(WheelSpeeds_getGroundSpeed(wss), MCM_getRegenMinSpeed(mcm), MCM_getRegenRampdownStartSpeed(mcm), TRUE);
            //USE FOR WHEEL SPEEDS TO DETECT RAMP DOWN
        if (regenMultiplier < multiplier ) { multiplier = regenMultiplier; } // Use regenMultiplier if it is lower
    }
    */
    //Reduce the torque command.  Multiplier should be a percent value (between 0 and 1)

    //If the safety bypass is enabled, then override the multiplier to 100% (no reduction)
    if ((me->warnings & W_safetyBypassEnabled) == W_safetyBypassEnabled)
    {
        multiplier = 1;
    }
    MCM_commands_setTorqueDNm(mcm, mcm->commands_torque * multiplier);
}

void set_flags(ubyte4 *fault, ubyte4 flag, bool condition) {
    if (condition)
    {
        *fault |= flag;
    }
    else
    {
        *fault &= ~flag;
    }
}

