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

#include "IO_ADC.h"
#include "IO_DIO.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "IO_PWD.h"
#include "IO_PWM.h"

#include "mathFunctions.h"
#include "sensors.h"

extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
extern Sensor Sensor_BPS1;
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
extern Sensor Sensor_TestButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_DRSButton;
extern Sensor Sensor_DRSKnob;
extern Sensor Sensor_LCButton;
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
    // TODO: Handle errors (using the return values for these Get functions)

    // TODO: RTDS

    // Torque Encoders ---------------------------------------------------
    // Sensor_BenchTPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_00, &Sensor_BenchTPS0.sensorValue,
    // &Sensor_BenchTPS0.fresh); Sensor_BenchTPS1.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_01,
    // &Sensor_BenchTPS1.sensorValue, &Sensor_BenchTPS1.fresh);
    Sensor_TPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_00, &Sensor_TPS0.sensorValue, &Sensor_TPS0.fresh);
    Sensor_TPS1.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_01, &Sensor_TPS1.sensorValue, &Sensor_TPS1.fresh);
    // Sensor_TPS0.ioErr_signalGet = IO_PWD_PulseGet(IO_PWM_00, &Sensor_TPS0.sensorValue);
    // Sensor_TPS1.ioErr_signalGet = IO_PWD_PulseGet(IO_PWM_01, &Sensor_TPS1.sensorValue);

    // Brake Position Sensor ---------------------------------------------------
    Sensor_BPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_02, &Sensor_BPS0.sensorValue, &Sensor_BPS0.fresh);
    Sensor_BPS1.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_03, &Sensor_BPS1.sensorValue, &Sensor_BPS1.fresh);

    // TCS Knob
    //  Sensor_TCSKnob.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_04, &Sensor_TCSKnob.sensorValue, &Sensor_TCSKnob.fresh);

    // Shock pots ---------------------------------------------------
    /*IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_FL.sensorValue, &Sensor_WPS_FL.fresh);
    IO_ADC_Get(IO_ADC_5V_05, &Sensor_WPS_FR.sensorValue, &Sensor_WPS_FR.fresh);
    IO_ADC_Get(IO_ADC_5V_06, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RL.fresh);
    IO_ADC_Get(IO_ADC_5V_07, &Sensor_WPS_RR.sensorValue, &Sensor_WPS_RR.fresh);
    */

    // Wheel speed sensors ---------------------------------------------------

    // For input smoothing
    if (Sensor_WSS_FL.sensorValue > 0) // If non-zero reading, update displayed val
    {
        Sensor_WSS_FL.heldSensorValue = Sensor_WSS_FL.sensorValue;
        IO_RTC_StartTime(&Sensor_WSS_FL.timestamp); // Reset time
    }
    else if (IO_RTC_GetTimeUS(Sensor_WSS_FL.timestamp) >
             750000) // Has been longer than 750000ms (timeout, reset heldSensorValue to 0)
    {
        Sensor_WSS_FL.heldSensorValue = 0;
    }

    if (Sensor_WSS_FR.sensorValue > 0)
    {
        Sensor_WSS_FR.heldSensorValue = Sensor_WSS_FR.sensorValue;
        IO_RTC_StartTime(&Sensor_WSS_FR.timestamp);
    }
    else if (IO_RTC_GetTimeUS(Sensor_WSS_FR.timestamp) >
             750000) // Has been longer than 750000ms (timeout, reset heldSensorValue to 0)
    {
        Sensor_WSS_FR.heldSensorValue = 0;
    }

    if (Sensor_WSS_RL.sensorValue > 0)
    {
        Sensor_WSS_RL.heldSensorValue = Sensor_WSS_RL.sensorValue;
        IO_RTC_StartTime(&Sensor_WSS_RL.timestamp);
    }
    else if (IO_RTC_GetTimeUS(Sensor_WSS_RL.timestamp) >
             750000) // Has been longer than 750000ms (timeout, reset heldSensorValue to 0)
    {
        Sensor_WSS_RL.heldSensorValue = 0;
    }

    if (Sensor_WSS_RR.sensorValue > 0)
    {
        Sensor_WSS_RR.heldSensorValue = Sensor_WSS_RR.sensorValue;
        IO_RTC_StartTime(&Sensor_WSS_RR.timestamp);
    }
    else if (IO_RTC_GetTimeUS(Sensor_WSS_RR.timestamp) >
             750000) // Has been longer than 750000ms (timeout, reset heldSensorValue to 0)
    {
        Sensor_WSS_RR.heldSensorValue = 0;
    }

    ubyte4 pulseTrash;
    Sensor_WSS_FL.ioErr_signalGet = IO_PWD_ComplexGet(IO_PWD_10, &Sensor_WSS_FL.sensorValue, &pulseTrash, NULL);
    Sensor_WSS_FR.ioErr_signalGet = IO_PWD_ComplexGet(IO_PWD_08, &Sensor_WSS_FR.sensorValue, &pulseTrash, NULL);
    Sensor_WSS_RL.ioErr_signalGet = IO_PWD_ComplexGet(IO_PWD_09, &Sensor_WSS_RL.sensorValue, &pulseTrash, NULL);
    Sensor_WSS_RR.ioErr_signalGet = IO_PWD_ComplexGet(IO_PWD_11, &Sensor_WSS_RR.sensorValue, &pulseTrash, NULL);

    // Switches / Digital ---------------------------------------------------
    Sensor_RTDButton.ioErr_signalGet = IO_DI_Get(IO_DI_00, &Sensor_RTDButton.sensorValue);
    Sensor_EcoButton.ioErr_signalGet = IO_DI_Get(IO_DI_01, &Sensor_EcoButton.sensorValue);
    // Sensor_TCSSwitchUp.ioErr_signalGet = IO_DI_Get(IO_DI_02, &Sensor_TCSSwitchUp.sensorValue);
    Sensor_LCButton.ioErr_signalGet             = IO_DI_Get(IO_DI_03, &Sensor_LCButton.sensorValue);
    Sensor_HVILTerminationSense.ioErr_signalGet = IO_DI_Get(IO_DI_07, &Sensor_HVILTerminationSense.sensorValue);
    Sensor_DRSButton.ioErr_signalGet            = IO_DI_Get(IO_DI_04, &Sensor_DRSButton.sensorValue);
    Sensor_TestButton.ioErr_signalGet           = IO_DI_Get(IO_DI_02, &Sensor_TestButton.sensorValue); // ! unoccupied

    // Other stuff ---------------------------------------------------
    // Battery voltage (at VCU internal electronics supply input)
    Sensor_LVBattery.ioErr_signalGet = IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);
    // Steering Angle Sensor
    Sensor_SAS.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_04, &Sensor_SAS.sensorValue, &Sensor_SAS.fresh);

    // DRS Knob
    Sensor_DRSKnob.ioErr_signalGet = IO_ADC_Get(IO_ADC_VAR_00, &Sensor_DRSKnob.sensorValue, &Sensor_DRSKnob.fresh);
}

void Light_set(Light light, float4 percent)
{
    ubyte2 duty = 65535 * percent; // For Cooling_RadFans

    bool power = duty > 5000 ? TRUE : FALSE; // Even though it's a lowside output, TRUE = on

    switch (light)
    {
    // PWM devices
    case Light_brake: IO_DO_Set(IO_ADC_CUR_00, power); break;

    case Cooling_waterPump: IO_DO_Set(IO_DO_02, power); break;

    case Cooling_RadFans: // Radiator Fans
        IO_PWM_SetDuty(IO_PWM_02, duty, NULL);
        IO_DO_Set(IO_DO_03, TRUE);
        break;

    case Cooling_batteryFans:
        // IO_DO_Set(IO_DO_04, power);
        break;

        //--------------------------------------------
        // These devices moved from PWM to DIO

    case Light_dashTCS: break;

    case Light_dashEco: IO_DO_Set(IO_ADC_CUR_01, power); break;

    case Light_dashError: IO_DO_Set(IO_ADC_CUR_02, power); break;

    case Light_dashRTD: IO_DO_Set(IO_ADC_CUR_03, power); break;
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
