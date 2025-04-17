#include "IO_RTC.h"
#include <math.h>
#include <stdlib.h> //Needed for malloc

#include "brakePressureSensor.h"
#include "mathFunctions.h"

#include "sensors.h"
// extern Sensor Sensor_BPS0;
// extern Sensor Sensor_BenchTPS1;

/*****************************************************************************
 * Brake Pressure Sensor (BPS) functions
 ****************************************************************************/

// TODO: #94 Make this CAN configurable and store in EEPROM
// This value is used for controlling the brake light and triggering the TPS-BPS implausibility fault
#define BRAKES_ON_PERCENT .08

BrakePressureSensor *BrakePressureSensor_new(void)
{
    BrakePressureSensor *me = (BrakePressureSensor *) malloc(sizeof(struct _BrakePressureSensor));
    // me->bench = benchMode;

    // TODO: Make sure the main loop is running before doing this
    me->bps0 = &Sensor_BPS0;
    me->bps1 = &Sensor_BPS1;
    // me->tps1 = (benchMode == TRUE) ? &Sensor_BenchTPS1 : &Sensor_TPS1;

    // Max/min values from the datasheet, including inaccuracy (important since our BPS sits slightly below 0.5V but
    // still within range) If voltage exceeds these values, a fault is thrown in safety.c. Accuracy below 100PSI is +/-
    // 0.5% of the full scale span (4V), which is +/- 0.2V
    Sensor_BPS0.specMin = 450 - (4000 * .005); // 450 - (4000 * .005);
    Sensor_BPS1.specMin = 450 - (4000 * .005); // 450 - (4000 * .005);

    // Accuracy above 100PSI is +/- 0.25% of the full scale span (4V), which is +/- 0.1V
    Sensor_BPS0.specMax = 4500 + (4000 * .0025);
    Sensor_BPS1.specMax = 4500 + (4000 * .0025);

    me->bps0_reverse = FALSE;
    me->bps1_reverse = FALSE;

    // BPS0 only
    me->percent        = 0;
    me->brakesAreOn    = FALSE;
    me->runCalibration = FALSE; // Do not run the calibration at the next main loop cycle

    me->bps0_calibMin = 489;
    me->bps0_calibMax = 2290;
    me->calibrated    = TRUE;

    return me;
}

// Updates all values based on sensor readings, safety checks, etc
void BrakePressureSensor_update(BrakePressureSensor *me, bool bench)
{
    me->bps0_value = me->bps0->sensorValue;
    me->bps1_value = me->bps1->sensorValue;

    // This function runs before the calibration cycle function.  If calibration is currently
    // running, then set the percentage to zero for safety purposes.
    if (me->runCalibration == TRUE || me->calibrated == FALSE)
    {
        me->bps0_percent = 0;
        me->bps1_percent = 0;
        me->percent      = 0;
        me->brakesAreOn  = FALSE; // Blocks Ready To Drive
    }
    else
    {
        me->bps0_percent = getPercent(me->bps0_value, me->bps0_calibMin, me->bps0_calibMax, TRUE);
        me->bps1_percent = getPercent(me->bps1_value, me->bps1_calibMin, me->bps1_calibMax, TRUE);
        // BPS0 only
        me->percent     = me->bps0_percent; // Note: If we had redundant sensors we would average them here
        me->brakesAreOn = me->percent > BRAKES_ON_PERCENT;
    }

    // Turn brake light on or off
    if (me->brakesAreOn)
    {
        Light_set(Light_brake, 1);
    }
    else if (bench == FALSE)
    {
        Light_set(Light_brake, 0);
    }
    else
    {
        if (me->percent > 0 && me->percent < .02)
        {
            Light_set(Light_brake, .20);
        }
        else if (me->percent >= .02 && me->percent < .30)
        {
            Light_set(Light_brake, .30);
        }
        else if (me->percent >= .30)
        {
            Light_set(Light_brake, me->percent);
        }
    }
}

// Sets initial/calibrated values
void BrakePressureSensor_resetCalibration(BrakePressureSensor *me)
{
    me->calibrated    = FALSE;
    me->bps0_calibMin = me->bps0->sensorValue;
    me->bps0_calibMax = me->bps0->sensorValue;
    // me->bps1_calibMin = me->bps1->sensorValue;
    // me->bps1_calibMax = me->bps1->sensorValue;
}

void BrakePressureSensor_saveCalibrationToEEPROM(BrakePressureSensor *me) {}

void BrakePressureSensor_loadCalibrationFromEEPROM(BrakePressureSensor *me) {}

void BrakePressureSensor_startCalibration(BrakePressureSensor *me, ubyte1 secondsToRun)
{
    if (me->runCalibration == FALSE) // Ignore the button if calibration is already running
    {
        me->runCalibration = TRUE;
        BrakePressureSensor_resetCalibration(me);
        me->calibrated = FALSE;
        IO_RTC_StartTime(&(me->timestamp_calibrationStart));
        me->calibrationRunTime = secondsToRun;
    }
    else
    {
        IO_RTC_StartTime(&(me->timestamp_calibrationStart)); // extend the calibration time
    }
}

/*-------------------------------------------------------------------
* CalibrateTPS
* Description: Records TPS minimum/maximum voltages (when?) and stores them (where?), or flags that calibration is
complete
* Parameters:
* Inputs:
* Returns:
* Notes:
* Throws:
-------------------------------------------------------------------*/
// Physical pedal travel will only occur across the center (about 1/2) of the actual sensor's range of travel
// The rules (especially EV2.3.6) are written about % of PEDAL travel, not percent of sensor range, so we must calculate
// pedal travel by recording the min/max voltages at min/max throttle positions
void BrakePressureSensor_calibrationCycle(BrakePressureSensor *me, ubyte1 *errorCount)
{
    if (me->runCalibration == TRUE)
    {
        if (IO_RTC_GetTimeUS(me->timestamp_calibrationStart) < (ubyte4) (me->calibrationRunTime) * 1000 * 1000)
        {
            // The calibration itself
            if (me->bps0->sensorValue < me->bps0_calibMin)
            {
                me->bps0_calibMin = me->bps0->sensorValue;
            }
            if (me->bps0->sensorValue > me->bps0_calibMax)
            {
                me->bps0_calibMax = me->bps0->sensorValue;
            }

            if (me->bps1->sensorValue < me->bps1_calibMin)
            {
                me->bps1_calibMin = me->bps1->sensorValue;
            }
            if (me->bps1->sensorValue > me->bps1_calibMax)
            {
                me->bps1_calibMax = me->bps1->sensorValue;
            }
        }
        else // Calibration shutdown
        {
            float4 pedalTopPlay    = 1.05;
            float4 pedalBottomPlay = .95;

            me->bps0_calibMin *= me->bps0_reverse ? pedalBottomPlay : pedalTopPlay;
            me->bps0_calibMax *= me->bps0_reverse ? pedalTopPlay : pedalBottomPlay;
            me->bps1_calibMin *= me->bps1_reverse ? pedalBottomPlay : pedalTopPlay;
            me->bps1_calibMax *= me->bps1_reverse ? pedalTopPlay : pedalBottomPlay;

            me->runCalibration = FALSE;
            me->calibrated     = TRUE;
            Light_set(Light_dashEco, 0);
        }
    }
    else
    {
        // TODO: Throw warning: calibrationCycle helper function was called but calibration should not be running
    }

    // TODO: Write calibration data to EEPROM

    // TODO: Check for valid/reasonable calibration data

    // TODO: Do something on the display to show that voltages are being recorded

    // Idea: Display "bars" filling up on right segment (for gas pedal) _=E=_=E...
    //       Once calibration data makes sense, show pedal location (0-10%, 10-90%, 90-100%) with bars
}

void BrakePressureSensor_getIndividualSensorPercent(BrakePressureSensor *me, ubyte1 sensorNumber, float4 *percent)
{
    // Sensor* bps;
    // ubyte2 calMin;
    // ubyte2 calMax;

    switch (sensorNumber)
    {
    case 0:
        *percent = me->bps0_percent;
        // bps = me->bps0;
        // calMin = me->bps0_calibMin;
        // calMax = me->bps0_calibMax;
        break;
    case 1:
        *percent = me->bps1_percent;
        // bps = me->bps1;
        // calMin = me->bps1_calibMin;
        // calMax = me->bps1_calibMax;
        break;
    }
    // float4 TPS0PedalPercent = getPercent(me->tps0->sensorValue, calMin, calMax, TRUE); //Analog Input 0
}

/*-------------------------------------------------------------------
* GetThrottlePosition
* Description: Reads TPS Pin voltages and returns % of throttle pedal travel.
* Parameters:  None
* Inputs:      Assumes TPS#.sensorValue has been set by main loop
* Returns:     Throttle value in percent (from 0 to 1)
* Notes:       Valid pedal travel is from 10% (0.10) to 90% (0.90), not including mechanical limits.
* Throws:      000 - TPS0 voltage out of range
*              001 - TPS1 voltage out of range, 002
-------------------------------------------------------------------*/
void BrakePressureSensor_getPedalTravel(BrakePressureSensor *me, ubyte1 *errorCount, float4 *pedalPercent)
{
    *pedalPercent = me->percent;

    // What about other error states?
    // Voltage outside of calibration range
    // Voltages off center

    //    if (errorCount > 0)
    //    {
    //        return 0;
    //    }
    //    else
    //    {
    // return (TPS0PedalPercent + TPS1PedalPercent) / 2;
    //    }
}