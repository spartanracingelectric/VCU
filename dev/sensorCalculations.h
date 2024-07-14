#ifndef _SENSORCALCULATIONS_H
#define _SENSORCALCULATIONS_H

/*****************************************************************************
* Output Calculations
******************************************************************************
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-11-16 - Rusty Pedrosa -
*****************************************************************************/

//VCU/C headers
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file

#include "sensors.h"

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/

// Physical pedal travel will only occur across the center (about 1/2) of the actual sensor's range of travel
// The rules (especially EV2.3.6) are written about % of PEDAL travel, not percent of sensor range, so we must calculate pedal travel by recording the min/max voltages at min/max throttle positions

/*-------------------------------------------------------------------
* CalibrateTPS
* Description: Records TPS minimum/maximum voltages (when?) and stores them (where?)
* Parameters:
* Inputs:
* Returns:
* Notes:
* Throws:
-------------------------------------------------------------------*/
void calibrateTPS(bool bench, ubyte1 secondsToRunCalibration);

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
float4 getThrottlePercent(bool bench, ubyte1* errorCount);

/*****************************************************************************
* BPS (Brake Pressure Sensor) functions
*
****************************************************************************/
//Input: Voltage
//Outputs: ???

/*-------------------------------------------------------------------
* CalibrateBPS
* Description: Records brake system pressure at point where wheels
*              lock up [THIS IS A SUGGESTION - maybe we just use the
*              DAQ to determine this and hardcode a value
* Parameters:
* Inputs:      Pin151 = Analog Input 2
* Returns:
* Notes:
* Throws:
-------------------------------------------------------------------*/
void CalibrateBPS(void);

/*-------------------------------------------------------------------
* GetBrakePosition
* Description: Reads TPS Pin voltages and returns % of throttle pedal travel.
* Parameters:  None
* Inputs:
* Returns:     Brake pedal travel in percent (from 0 to 1), where
*              0 = no pedal travel and 100 = //TODO: 4 locked wheels???
*              NOTE: This means values above 100% are both allowed
*                    and EXPECTED during e.g. brake tests & extreme lockups
* Notes:
* Throws:
-------------------------------------------------------------------*/
void GetBrakePosition(void);

/*****************************************************************************
* Wheel Speed Sensor functions
* FR = Pin275 = Digital Timer Input 0
* FL = Pin268 = Digital Timer Input 1
* RR = Pin274 = Digital Timer Input 2
* RL = Pin267 = Digital Timer Input 3
****************************************************************************/
//Inputs: Wheel speed sensor frequency
//Outputs: 4 speeds, RPM, ground speed
//4 funcs, one to get each wheel's speed
//16 bumps per rotation, 16 hz = 1 rotation per second
#define NUM_BUMPS 16
#define WHEEL_DIAMETER 18 //Inches    /// change to 18 check it with DAQ
#define NUM_BUMPS_D (double)NUM_BUMPS
#define WHEEL_DIAMETER_D (double)WHEEL_DIAMETER

double freq_to_rpm(double freq);

//Theoretical ground speed
//63360 inches per mile. Wish we could use metric.
double rpm_to_mph(double rpm);

/*****************************************************************************
* Shock pot(iometer) functions - FOUR NEEDED
* FR = Pin150 = Analog Input 4
* FL = Pin138 = Analog Input 5
* RR = Pin149 = Analog Input 6
* RL = Pin137 = Analog Input 7
* 0 = ride height
****************************************************************************/
//Input: Ohms
//Outputs: ???
//See VCU Manual section 5.8.8 - there are different outputs depending on sensor resistance
//Example sensor: Active Sensors CLS0950
//Resistive range: 0.4 to 6.0 kohm
//ShockPot.

/*****************************************************************************
* Steering Angle Sensor (SAS)
Input: Voltage
Output: Degrees
****************************************************************************/
sbyte4 steering_degrees();

/*****************************************************************************
* Doppler speed sensor functions
*
****************************************************************************/
//Input: Frequency
//Outputs: ???

/*****************************************************************************
* Water temp sensor functions
*
****************************************************************************/
//Input: Ohms
//Outputs: ???

/*****************************************************************************
* Dash button functions
*
****************************************************************************/
//TCS
//ECO
//RTD
//blablalba



/*****************************************************************************
* Error Reporting Stuff
*
* TODO: Write error reporting functions
****************************************************************************/
//Err.Report(ErrCode, ErrDesc, ErrAction) {
//Write an error to the log
//Add error code to dash
//000 = TPS0 out of range
//001 = TPS1 out of range
//}

#endif //  _SENSORCALCULATIONS_H