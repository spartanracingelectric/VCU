/*****************************************************************************
* Sensors
*****************************************************************************/

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "IO_ADC.h"
#include "IO_PWD.h"
#include "IO_PWM.h"
#include "IO_DIO.h"

#include "sensors.h"
#include "mathFunctions.h"

extern Sensor TPS0;
extern Sensor TPS1;
extern Sensor BPS0;
extern Sensor BPS1;
extern PWDSensor WSS_FL;
extern PWDSensor WSS_FR;
extern PWDSensor WSS_RL;
extern PWDSensor WSS_RR;
extern Sensor Sensor_SAS;
extern Sensor Sensor_LVBattery;

extern Button RTD_Button;
extern Button Cal_Button;
extern Button DRS_Button;
extern Sensor Sensor_DRSKnob;
extern Button LC_Button;
extern Button Sensor_HVILTerminationSense;

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
    //Torque Encoders ---------------------------------------------------
    TPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_00, &TPS0.sensorValue, &TPS0.fresh);
    TPS1.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_01, &TPS1.sensorValue, &TPS1.fresh);

    //Brake Position Sensor ---------------------------------------------------
    BPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_02, &BPS0.sensorValue, &BPS0.fresh);
    BPS1.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_03, &BPS1.sensorValue, &BPS1.fresh);
   
    //Wheel speed sensors ---------------------------------------------------
    PWDSensor_read(&WSS_FL);
    PWDSensor_read(&WSS_FR);
    PWDSensor_read(&WSS_RL);
    PWDSensor_read(&WSS_RR);

    //Switches / Digital ---------------------------------------------------
    Button_read(&RTD_Button);
    Button_read(&Cal_Button);
    Button_read(&LC_Button);
    Button_read(&Sensor_HVILTerminationSense);
    Button_read(&DRS_Button);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
    Sensor_LVBattery.ioErr_signalGet = IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);
    //Steering Angle Sensor
    Sensor_SAS.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_04, &Sensor_SAS.sensorValue, &Sensor_SAS.fresh);

    //DRS Knob
    Sensor_DRSKnob.ioErr_signalGet = IO_ADC_Get(IO_ADC_VAR_00, &Sensor_DRSKnob.sensorValue, &Sensor_DRSKnob.fresh);
}

Button* Button_new(ubyte1 pin, bool inverted) {
    Button* button = malloc(sizeof(Button));
    IO_RTC_StartTime(&button->timestamp);
    IO_RTC_StartTime(&button->heldTimestamp);
    button->sensorValue = 0;
    button->heldSensorValue = 0;
    button->sensorAddress = pin;
    button->inverted = inverted;
    button->ioErr_signalInit = IO_DI_Init(pin, IO_DI_PU_10K);
    return button;
}

void Button_read(Button* button) {
    button->ioErr_signalGet = IO_DI_Get(button->sensorAddress, &button->sensorValue);
    IO_RTC_StartTime(&button->timestamp);
    if (button->inverted) {
        button->sensorValue = !button->sensorValue;
    }
    // Look and see if the button has changed state, if so calculate the time since the last change
    if (button->sensorValue != button->heldSensorValue) {
        button->heldSensorValue = button->sensorValue;
        button->heldTime = button->timestamp - button->heldTimestamp;
        IO_RTC_StartTime(&button->heldTimestamp);
    }
}

PWDSensor* PWDSensor_new(ubyte1 pin) {
    PWDSensor* sensor = malloc(sizeof(PWDSensor));
    IO_RTC_StartTime(&sensor->timestamp);
    sensor->sensorAddress = pin;
    sensor->heldSensorValue = 0;
    sensor->ioErr_signalInit = IO_PWD_ComplexInit(pin, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_3_2, 1, IO_PWD_THRESH_1_25V, IO_PWD_PD_10K, NULL);
    return sensor;
}

void PWDSensor_read(PWDSensor* sensor) {
    ubyte4 pulseTrash;
    ubyte2 frequency_now;
    sensor->ioErr_signalGet = IO_PWD_ComplexGet(sensor->sensorAddress, &frequency_now, &pulseTrash, NULL);
    if(sensor->ioErr_signalGet != IO_E_PWD_NOT_FINISHED) {
        sensor->sensorValue = frequency_now;
        IO_RTC_StartTime(&sensor->timestamp);
        sensor->heldSensorValue = sensor->sensorValue;
    } else if (IO_RTC_GetTimeUS(sensor->timestamp) > 750000) //Has been longer than 750000us (timeout, reset heldSensorValue to 0)
    { 
        sensor->heldSensorValue=0;
    }
}

void Light_set(Light light, float4 percent)
{
    ubyte2 duty = 65535 * percent; //For Cooling_RadFans

    bool power = duty > 5000 ? TRUE : FALSE; //Even though it's a lowside output, TRUE = on

    switch (light)
    {
    //PWM devices
    case Light_brake:
        IO_DO_Set(IO_ADC_CUR_00, power);
        break;

    case Cooling_waterPump:
        IO_DO_Set(IO_DO_02, power);
        break;

    case Cooling_RadFans:  // Radiator Fans
        IO_PWM_SetDuty(IO_PWM_02, duty, NULL);
        break;

    case Cooling_batteryFans:
        IO_DO_Set(IO_DO_04, power);
        break;

        //--------------------------------------------
        //These devices moved from PWM to DIO

    case Light_dashTCS:
        break;

    case Light_dashEco:
        IO_DO_Set(IO_ADC_CUR_01, power);
        break;

    case Light_dashError:
        IO_DO_Set(IO_ADC_CUR_02, power);
        break;

    case Light_dashRTD:
        IO_DO_Set(IO_ADC_CUR_03, power);
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
*****************************************************************************/

/*****************************************************************************
* Steering Angle Sensor (SAS)
Input: Voltage
Output: Degrees
****************************************************************************/
sbyte4 steering_degrees(){
    sbyte4 min_voltage = 960;    // Adjusted minimum voltage to 0 mV
    sbyte4 max_voltage = 2560; // Adjusted maximum voltage to 5000 mV
    sbyte4 min_angle = -90;
    sbyte4 max_angle = 90;
    
    sbyte4 voltage_range = max_voltage - min_voltage;
    sbyte4 angle_range = max_angle - min_angle;
    sbyte4 voltage = Sensor_SAS.sensorValue;

    sbyte4 deg = min_angle + (angle_range * (voltage - min_voltage)) / voltage_range;
    return deg;
}
