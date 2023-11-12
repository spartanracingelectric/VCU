/**********************************************************************//**
 * \file sensors.h
 *
 * \brief Sensor object definitions and basic functions
 *
 *      The IO Driver high level interface provides a general
 *      initialization function, a version API and general task functions
 *      which shall wrap the whole user application.
 *
 **************************************************************************/

//"Include guard" - prevents this file from being #included more than once
#ifndef _SENSORS_H
#define _SENSORS_H

#include "IO_Driver.h"



typedef enum 
{ 
    Light_dashEco,        //on/off
    Light_dashError,      //on/off
    Light_dashRTD,        //on/off
    Light_dashTCS,        //on/off
    Light_brake,          //PWM
    Cooling_waterPump,    //PWM
    Cooling_RadFans,      //PWM
    Cooling_batteryFans   //on/off
} Light;


//----------------------------------------------------------------------------
// Sensor Object Definitions
//----------------------------------------------------------------------------
// Parameters:
//
// specMin/Max values should come from each sensor's datasheets, but it is not
// required for all sensors.
//
//----------------------------------------------------------------------------
typedef struct _Sensor {
    //Sensor values / properties
    ubyte4 specMin;
    ubyte4 specMax;
    
    ubyte2 sensorValue;
    ubyte4 heldSensorValue;
    ubyte4 timestamp;
    ubyte1 sensorAddress;
    ubyte1 powerAddress;
    bool fresh;
    //bool isCalibrated;
    IO_ErrorType ioErr_powerInit;
    IO_ErrorType ioErr_powerSet;
    IO_ErrorType ioErr_signalInit;
    IO_ErrorType ioErr_signalGet;
} Sensor;

typedef struct _Button {
    bool sensorValue;
    bool heldSensorValue;
    ubyte4 timestamp;
    ubyte4 heldTime;
    ubyte4 heldTimestamp;
    ubyte1 sensorAddress;
    bool fresh;
    bool inverted;
    IO_ErrorType ioErr_signalInit;
    IO_ErrorType ioErr_signalGet;
} Button;

typedef struct _PWDSensor {
    ubyte2 sensorValue;
    ubyte2 heldSensorValue;
    ubyte4 timestamp;
    ubyte1 sensorAddress;
    bool fresh;
    IO_ErrorType ioErr_signalInit;
    IO_ErrorType ioErr_signalGet;
} PWDSensor;

//----------------------------------------------------------------------------
// Sensor Object Declarations
//----------------------------------------------------------------------------
//Initialize all sensors in vcu.c
//TODO: Read stored calibration data from EEPROM

//Torque Encoders (TPS is not really accurate since there's no throttle to position in an EV)
extern Sensor TPS0;  // = { 0, 0.5, 4.5 };
extern Sensor TPS1;  // = { 0, 4.5, 0.5 };

//Brake Position Sensors
extern Sensor BPS0;  // = { 1, 0.5, 4.5 };  //Brake system pressure (or front only in the future)
extern Sensor BPS1;  // = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)

//Wheel Speed Sensors (like an ABS sensor)
extern PWDSensor WSS_FL;  // = { 2 };
extern PWDSensor WSS_FR;  // = { 2 };
extern PWDSensor WSS_RL;  // = { 2 };
extern PWDSensor WSS_RR;  // = { 2 };

//Steering angle Sensor (SAS) - continuous rotation sensor, works like TPS, probably ratiometric
extern Sensor Sensor_SAS;  // = { 4 };

//Switches
//precharge failure
extern Button RTD_Button;
extern Button Cal_Button;
extern Sensor Sensor_TCSSwitchUp;
extern Button LC_Button;
extern Sensor Sensor_TCSKnob;
extern Button DRS_Button;
extern Sensor Sensor_DRSKnob;

extern Button Sensor_HVILTerminationSense;


//Other
extern Sensor Sensor_LVBattery; // = { 0xA };  //Note: There will be no init for this "sensor"

Sensor* Sensor_new(ubyte1 pin, ubyte1 power);
Button* Button_new(ubyte1 pin, bool inverted);
PWDSensor* PWDSensor_new(ubyte1 pin);

//----------------------------------------------------------------------------
// Sensor Functions
//----------------------------------------------------------------------------
void sensors_updateSensors(void);

void setMCMRelay(bool turnOn);
void Sensor_power_set(Sensor* sensor);
void Sensor_read(Sensor* sensor);
void Button_read(Button* button);
void PWDSensor_read(PWDSensor* sensor);

//----------------------------------------------------------------------------
// Outputs
//----------------------------------------------------------------------------
void Light_set(Light light, float4 percent);
/*****************************************************************************
* Steering Angle Sensor (SAS)
Input: Voltage
Output: Degrees
****************************************************************************/
sbyte4 steering_degrees();

#endif // _SENSORS_H