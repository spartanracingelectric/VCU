/*****************************************************************************
* Sensors
******************************************************************************
* bla bla bla.
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-12-01 - Rusty Pedrosa - Changed loading of sensor data to switch
*                              statement inside of a loop
*****************************************************************************/

#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_ADC.h"
#include "IO_PWD.h"
#include "IO_PWM.h"
#include "IO_DIO.h"

#include "sensors.h"
#include "mathFunctions.h"

extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
//extern Sensor Sensor_BPS1;
extern Sensor Sensor_WSS_FL;
extern Sensor Sensor_WSS_FR;
extern Sensor Sensor_WSS_RL;
extern Sensor Sensor_WSS_RR;
extern Sensor Sensor_WPS_FL;
extern Sensor Sensor_WPS_FR;
extern Sensor Sensor_WPS_RL;
extern Sensor Sensor_WPS_RR;
extern Sensor Sensor_SAS;
extern Sensor Sensor_LVBattery;

extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;

extern Sensor Sensor_RTDButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_TCSSwitchA;
extern Sensor Sensor_TCSSwitchB;
extern Sensor Sensor_HVILTerminationSense;

/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
//----------------------------------------------------------------------------
// Read sensors values from ADC channels
// The sensor values should be stored in sensor objects.
//----------------------------------------------------------------------------
void sensors_updateSensors(void)
{
    //TODO: Handle errors (using the return values for these Get functions)

    //TODO: RTDS

    //Torque Encoders ---------------------------------------------------
	Sensor_TPS0.ioErr_signal = IO_PWD_FreqGet(IO_PWM_00, &Sensor_TPS0.sensorValue);
	Sensor_TPS1.ioErr_signal = IO_PWD_FreqGet(IO_PWM_01, &Sensor_TPS1.sensorValue);

    //Brake Position Sensor ---------------------------------------------------
	Sensor_BPS0.ioErr_signal = IO_ADC_Get(IO_ADC_5V_02, &Sensor_BPS0.sensorValue, &Sensor_BPS0.fresh);

    //?? - For future use ---------------------------------------------------
    //IO_ADC_Get(IO_ADC_5V_03, &Sensor_BPS1.sensorValue, &Sensor_BPS1.fresh);

    //Bench TPS ---------------------------------------------------
    //Read TPS values in from sensor/ADC
	Sensor_BenchTPS0.ioErr_signal = IO_ADC_Get(IO_ADC_5V_00, &Sensor_BenchTPS0.sensorValue, &Sensor_BenchTPS0.fresh);
	Sensor_BenchTPS1.ioErr_signal = IO_ADC_Get(IO_ADC_5V_01, &Sensor_BenchTPS1.sensorValue, &Sensor_BenchTPS1.fresh);
    //Give TPS values to TPS object

    //Shock pots ---------------------------------------------------
    /*IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_FL.sensorValue, &Sensor_WPS_FL.fresh);
    IO_ADC_Get(IO_ADC_5V_05, &Sensor_WPS_FR.sensorValue, &Sensor_WPS_FR.fresh);
    IO_ADC_Get(IO_ADC_5V_06, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RL.fresh);
    IO_ADC_Get(IO_ADC_5V_07, &Sensor_WPS_RR.sensorValue, &Sensor_WPS_RR.fresh);
	*/

    //Wheel speed sensors ---------------------------------------------------
	Sensor_WSS_FL.ioErr_signal = IO_PWD_FreqGet(IO_PWD_08, &Sensor_WSS_FL.sensorValue);
	Sensor_WSS_FR.ioErr_signal = IO_PWD_FreqGet(IO_PWD_09, &Sensor_WSS_FR.sensorValue);
	Sensor_WSS_RL.ioErr_signal = IO_PWD_FreqGet(IO_PWD_10, &Sensor_WSS_RL.sensorValue);
	Sensor_WSS_RR.ioErr_signal = IO_PWD_FreqGet(IO_PWD_11, &Sensor_WSS_RR.sensorValue);

    //Switches / Digital ---------------------------------------------------
	Sensor_RTDButton.ioErr_signal = IO_DI_Get(IO_DI_00, &Sensor_RTDButton.sensorValue);
	Sensor_EcoButton.ioErr_signal = IO_DI_Get(IO_DI_01, &Sensor_EcoButton.sensorValue);
	Sensor_TCSSwitchA.ioErr_signal = IO_DI_Get(IO_DI_02, &Sensor_TCSSwitchA.sensorValue);
	Sensor_TCSSwitchB.ioErr_signal = IO_DI_Get(IO_DI_03, &Sensor_TCSSwitchB.sensorValue);
	Sensor_HVILTerminationSense.ioErr_signal = IO_DI_Get(IO_DI_07, &Sensor_HVILTerminationSense.sensorValue);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
	Sensor_LVBattery.ioErr_signal = IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);


}

//----------------------------------------------------------------------------
//Testing MCM relay control
//----------------------------------------------------------------------------
void setMCMRelay(bool turnOn)
{
    IO_DO_Set(IO_DO_00, turnOn);
}

void dashLight_set(DashLight light, bool turnOn)
{
    ubyte2 duty = turnOn ? 65535 : 0;

    switch (light)
    {
    case dash_EcoLight:
        IO_PWM_SetDuty(IO_PWM_04, duty, NULL);  //Pin 116
        break;

    case dash_ErrorLight:
        IO_PWM_SetDuty(IO_PWM_05, duty, NULL);  //Pin 104
        break;

    case dash_RTDLight:
        IO_PWM_SetDuty(IO_PWM_06, duty, NULL);  //Pin 115
        break;

    case dash_TCSLight:
        IO_PWM_SetDuty(IO_PWM_03, duty, NULL);  //Pin 105
        break;
    }

}

/*****************************************************************************
* Output Calculations
******************************************************************************
* Takes properties from devices (such as raw sensor values [ohms, voltage],
* MCU/BMS CAN messages, etc), performs calculations with that data, and updates
* the relevant objects' properties.
*
* This includes sensor calculations, motor controller control calculations,
* traction control, BMS/safety calculations, etc.
* (May need to split this up later)
*
* For example: GetThrottlePosition() takes the raw TPS voltages from the TPS
* sensor objects and returns the throttle pedal percent.  This function does
* NOT update the sensor objects, but it would be acceptable for another
* function in this file to do so.
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-11-16 - Rusty Pedrosa -
*****************************************************************************/
