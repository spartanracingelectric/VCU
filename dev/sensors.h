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
// TODO: What about having default calibration values?  (Probably useless)
//----------------------------------------------------------------------------
typedef struct _Sensor {
    //Sensor values / properties
    ubyte4 specMin;
    ubyte4 specMax;
    
    //ubyte2 calibMin;
    //ubyte2 calibMax;
    //ubyte2 calibNormal;  //zero value or normal position

    //ubyte2 calibratedValue;
    ubyte4 sensorValue;
    ubyte4 heldSensorValue;
    ubyte4 timestamp;
    bool fresh;
    //bool isCalibrated;
    IO_ErrorType ioErr_powerInit;
    IO_ErrorType ioErr_powerSet;
    IO_ErrorType ioErr_signalInit;
    IO_ErrorType ioErr_signalGet;
} Sensor;

//----------------------------------------------------------------------------
// Sensor Object Declarations
//----------------------------------------------------------------------------
//Initialize all sensors in vcu.c
//TODO: Read stored calibration data from EEPROM

//Torque Encoders (TPS is not really accurate since there's no throttle to position in an EV)
extern Sensor Sensor_TPS0;  // = { 0, 0.5, 4.5 };
extern Sensor Sensor_TPS1;  // = { 0, 4.5, 0.5 };

//Brake Position Sensors
extern Sensor Sensor_BPS0;  // = { 1, 0.5, 4.5 };  //Brake system pressure (or front only in the future)
extern Sensor Sensor_BPS1;  // = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)

//Wheel Speed Sensors (like an ABS sensor)
extern Sensor Sensor_WSS_FL;  // = { 2 };
extern Sensor Sensor_WSS_FR;  // = { 2 };
extern Sensor Sensor_WSS_RL;  // = { 2 };
extern Sensor Sensor_WSS_RR;  // = { 2 };

//Steering angle Sensor (SAS) - continuous rotation sensor, works like TPS, probably ratiometric
extern Sensor Sensor_SAS;  // = { 4 };

//Switches
//precharge failure
extern Sensor Sensor_RTDButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_TCSSwitchUp;
extern Sensor Sensor_LCButton;
extern Sensor Sensor_TCSKnob;
extern Sensor Sensor_DRSButton;
extern Sensor Sensor_DRSKnob;
extern Sensor Sensor_TEMP_BrakingSwitch;

extern Sensor Sensor_HVILTerminationSense;


//Other
extern Sensor Sensor_LVBattery; // = { 0xA };  //Note: There will be no init for this "sensor"


//----------------------------------------------------------------------------
// Sensor Functions
//----------------------------------------------------------------------------
void sensors_updateSensors(void);


void setMCMRelay(bool turnOn);


//----------------------------------------------------------------------------
// Outputs
//----------------------------------------------------------------------------
void Light_set(Light light, float4 percent);

#endif // _SENSORS_H