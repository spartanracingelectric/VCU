#include <stdlib.h>  //Needed for malloc
#include <math.h>
#include "IO_RTC.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"
#include "sensors.h"

extern DigitalOutput Eco_Light;

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
void TorqueEncoder_new(TorqueEncoder *me)
{
    me->tps0 = &TPS0;
    me->tps1 = &TPS1;

    //Where/should these be hardcoded?
    me->tps0_reverse = FALSE;
    me->tps1_reverse = TRUE;

    me->outputCurveExponent = 1.0;

    me->travelPercent = 0;
    me->runCalibration = FALSE;  //Do not run the calibration at the next main loop cycle

    //me->calibrated = FALSE;
    //TorqueEncoder_resetCalibration(me);

    //New Rotary Sensor Datasheet limits (used by safety checker / rules requirement)
    //It is literally a potentiometer, no sensor operating range in theory?
    //That would mean we could probably make our own ranges up
    me->tps0->specMin = 100; // Target 0% = ~250
    me->tps0->specMax = 2500; // Target 100% = ~2000
    me->tps1->specMin = 2500; // Target 0% = ~2650
    me->tps1->specMax = 4900; // Target 100% = ~4700

    
    me->tps0_calibMin = 500;
    me->tps0_calibMax = 1440;
    me->tps1_calibMin = 3350;
    me->tps1_calibMax = 4400;
    me->calibrated = TRUE;
}

//Updates all values based on sensor readings, safety checks, etc
void TorqueEncoder_update(TorqueEncoder* me)
{
    me->tps0_value = me->tps0->sensorValue;
    me->tps1_value = me->tps1->sensorValue;

    me->travelPercent = 0;
    
    //This function runs before the calibration cycle function.  If calibration is currently
    //running, then set the percentage to zero for safety purposes.
    if (me->runCalibration == FALSE)
    {
        //getPedalTravel = 0;

        //-------------------------------------------------------------------
        // Make sure the sensors have been calibrated
        //-------------------------------------------------------------------
        //if ((Sensor_TPS0.isCalibrated == FALSE) || (Sensor_TPS1.isCalibrated == FALSE))
        if (me->calibrated == FALSE)
        {
            me->tps0_percent = 0;
            me->tps1_percent = 0;
        }
        else
        {
            //Calculate individual throttle percentages
            //Percent = (Voltage - CalibMin) / (CalibMax - CalibMin)
            me->tps0_percent = getPercent((float4)me->tps0_value, (float4)me->tps0_calibMin, (float4)me->tps0_calibMax, TRUE);
            me->tps1_percent = getPercent((float4)me->tps1_value, (float4)me->tps1_calibMin, (float4)me->tps1_calibMax, TRUE);

            me->travelPercent = (me->tps0_percent + me->tps1_percent) / 2;
        }
    }
}

void TorqueEncoder_resetCalibration(TorqueEncoder* me)
{
    me->calibrated = FALSE;
    
    me->tps0_calibMin = me->tps0->sensorValue;
    me->tps0_calibMax = me->tps0->sensorValue;
    me->tps1_calibMin = me->tps1->sensorValue;
    me->tps1_calibMax = me->tps1->sensorValue;
}

void TorqueEncoder_startCalibration(TorqueEncoder* me, ubyte1 secondsToRun)
{
    if (me->runCalibration == FALSE) //Ignore the button if calibration is already running
    {
        me->runCalibration = TRUE;
        TorqueEncoder_resetCalibration(me);
        me->calibrated = FALSE;
        IO_RTC_StartTime(&(me->timestamp_calibrationStart));
        me->calibrationRunTime = secondsToRun;
    }
    else
    {
        IO_RTC_StartTime(&(me->timestamp_calibrationStart));  //extend the calibration time
    }
}

/*-------------------------------------------------------------------
* CalibrateTPS
* Description: Records TPS minimum/maximum voltages (when?) and stores them (where?), or flags that calibration is complete
* Parameters:
* Inputs:
* Returns:
* Notes:
* Throws:
-------------------------------------------------------------------*/
// Physical pedal travel will only occur across the center (about 1/2) of the actual sensor's range of travel
// The rules (especially EV2.3.6) are written about % of PEDAL travel, not percent of sensor range, so we must calculate pedal travel by recording the min/max voltages at min/max throttle positions
void TorqueEncoder_calibrationCycle(TorqueEncoder* me, ubyte1* errorCount)
//THIS FUNCTION SHOULD NOT BE CALLED FROM MAIN
{
    if (me->runCalibration == TRUE)
    {
        if (IO_RTC_GetTimeUS(me->timestamp_calibrationStart) < (ubyte4)(me->calibrationRunTime) * 1000 * 1000)
        {
            //The calibration itself
            if (me->tps0->sensorValue < me->tps0_calibMin) { me->tps0_calibMin = me->tps0->sensorValue; }
            if (me->tps0->sensorValue > me->tps0_calibMax) { me->tps0_calibMax = me->tps0->sensorValue; }

            if (me->tps1->sensorValue < me->tps1_calibMin) { me->tps1_calibMin = me->tps1->sensorValue; }
            if (me->tps1->sensorValue > me->tps1_calibMax) { me->tps1_calibMax = me->tps1->sensorValue; }

        }
        else  //Calibration shutdown
        {
            float4 shrink0 = (me->tps0_calibMax - me->tps0_calibMin) * .05;
            float4 shrink1 = (me->tps1_calibMax - me->tps1_calibMin) * .05;
            me->tps0_calibMin += shrink0;
            me->tps0_calibMax -= shrink0;
            me->tps1_calibMin += shrink1;
            me->tps1_calibMax -= shrink1;

            me->runCalibration = FALSE;
            me->calibrated = TRUE;
            DigitalOutput_set(&Eco_Light, FALSE);
        }

    }
    else
    {
        //TODO: Throw warning: calibrationCycle helper function was called but calibration should not be running
    }

    //TODO: Write calibration data to EEPROM

    //TODO: Check for valid/reasonable calibration data

    //TODO: Do something on the display to show that voltages are being recorded

    //Idea: Display "bars" filling up on right segment (for gas pedal) _=E=_=E...
    //      Once calibration data makes sense, show pedal location (0-10%, 10-90%, 90-100%) with bars

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

float4 TorqueEncoder_getOutputPercent(TorqueEncoder* me)
{
    return powf(me->travelPercent, me->outputCurveExponent);
}
