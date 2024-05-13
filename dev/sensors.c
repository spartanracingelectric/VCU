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
extern Sensor SAS;
extern Sensor LVBattery;

extern Button RTD_Button;
extern Button Cal_Button;
extern Button DRS_Button;
extern Sensor DRSKnob;
extern Button LC_Button;
extern Button HVILTerminationSense;

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
    Sensor_read(&TPS0);
    Sensor_read(&TPS1);

    //Brake Position Sensor ---------------------------------------------------
    Sensor_read(&BPS0);
    Sensor_read(&BPS1);
   
    //Wheel speed sensors ---------------------------------------------------
    PWDSensor_read(&WSS_FL);
    PWDSensor_read(&WSS_FR);
    PWDSensor_read(&WSS_RL);
    PWDSensor_read(&WSS_RR);

    //Switches / Digital ---------------------------------------------------
    Button_read(&RTD_Button);
    Button_read(&Cal_Button);
    Button_read(&LC_Button);
    Button_read(&HVILTerminationSense);
    Button_read(&DRS_Button);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
    Sensor_read(&LVBattery);
    //Steering Angle Sensor
    Sensor_read(&SAS);

    //DRS Knob
    Sensor_read(&DRSKnob);
}

Sensor* Sensor_new(ubyte1 pin, ubyte1 power) {
    Sensor* sensor = malloc(sizeof(Sensor));
    IO_RTC_StartTime(&sensor->timestamp);
    sensor->sensorAddress = pin;
    sensor->powerAddress = power;
    sensor->ioErr_signalInit = IO_ADC_ChannelInit(pin, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
    return sensor;
}

void Sensor_power_set(Sensor* sensor) {
    sensor->ioErr_powerSet = IO_POWER_Set(sensor->powerAddress, IO_POWER_ON);;
}

void Sensor_read(Sensor* sensor) {
    sensor->ioErr_signalGet = IO_ADC_Get(sensor->sensorAddress, &sensor->sensorValue, &sensor->fresh);
    IO_RTC_StartTime(&sensor->timestamp);
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

DigitalOutput* DigitalOutput_new(ubyte1 pin, bool inverted) {
    DigitalOutput* output = malloc(sizeof(DigitalOutput));
    output->outputAddress = pin;
    output->inverted = inverted;
    IO_DO_Init(pin);
    IO_DO_Set(pin, inverted ? TRUE : FALSE);
    return output;
}

void DigitalOutput_set(DigitalOutput* output, bool value) {
    IO_DO_Set(output->outputAddress, output->inverted ? !value : value);
}

PWMOutput* PWMOutput_new(ubyte1 pin, ubyte2 frequency, float4 duty) {
    PWMOutput* output = malloc(sizeof(PWMOutput));
    output->outputAddress = pin;
    output->frequency = frequency;
    output->duty = duty * 0xFFFF;
    IO_PWM_Init(pin, frequency, TRUE, FALSE, 0, FALSE, NULL);
    IO_PWM_SetDuty(pin, duty * 0xFFFF, NULL);
    return output;
}

void PWMOutput_set(PWMOutput* output, float4 duty) {
    IO_PWM_SetDuty(output->outputAddress, duty * 0xFFFF, NULL);
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
    sbyte4 voltage = SAS.sensorValue;

    sbyte4 deg = min_angle + (angle_range * (voltage - min_voltage)) / voltage_range;
    return deg;
}
