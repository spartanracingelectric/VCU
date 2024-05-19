/**********************************************************************/ /**
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
#include "motorController.h"
#include "bms.h"
#include "safety.h"
#include "wheelSpeeds.h"
#include "serial.h"
#include "LaunchControl.h"
#include "drs.h"
#include "lut.h"
#include "watch_dog.h"

typedef struct _Sensor
{
    // Sensor values / properties
    ubyte4 specMin;
    ubyte4 specMax;

    ubyte2 sensorValue;
    ubyte4 heldSensorValue;
    ubyte4 timestamp;
    ubyte1 sensorAddress;
    ubyte1 powerAddress;
    bool fresh;

    IO_ErrorType ioErr_powerInit;
    IO_ErrorType ioErr_powerSet;
    IO_ErrorType ioErr_signalInit;
    IO_ErrorType ioErr_signalGet;

} Sensor;

typedef struct _Button
{
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

typedef struct _PWDSensor
{
    ubyte2 sensorValue;
    ubyte2 heldSensorValue;
    ubyte4 timestamp;
    ubyte1 sensorAddress;
    bool fresh;
    IO_ErrorType ioErr_signalInit;
    IO_ErrorType ioErr_signalGet;
} PWDSensor;

typedef struct _DigitalOutput
{
    ubyte1 outputAddress;
    bool inverted;
} DigitalOutput;

typedef struct _PWMOutput
{
    ubyte1 outputAddress;
    ubyte2 duty;
    ubyte2 frequency;
} PWMOutput;

//----------------------------------------------------------------------------
// Sensor Object Declarations
//----------------------------------------------------------------------------
// Initialize all sensors in vcu.c
// TODO: Read stored calibration data from EEPROM

// Torque Encoders (TPS is not really accurate since there's no throttle to position in an EV)
extern Sensor TPS0;
extern Sensor TPS1;

// Brake Position Sensors
extern Sensor BPS0; // Brake system pressure (or front only in the future)
extern Sensor BPS1; // Rear brake system pressure (separate address in case used for something else)

// Wheel Speed Sensors (like an ABS sensor)
extern PWDSensor WSS_FL;
extern PWDSensor WSS_FR;
extern PWDSensor WSS_RL;
extern PWDSensor WSS_RR;

// Steering angle Sensor (SAS) - continuous rotation sensor, works like TPS, probably ratiometric
extern Sensor SAS;

// Switches
// precharge failure
extern Button RTD_Button;
extern Button Cal_Button;
extern Sensor TCSSwitchUp;
extern Button LC_Button;
extern Sensor TCSKnob;
extern Button DRS_Button;
extern Sensor DRSKnob;

extern Button HVILTerminationSense;
extern Sensor LVBattery;

extern DigitalOutput Brake_Light;
extern DigitalOutput TCS_Light;
extern DigitalOutput Eco_Light;
extern DigitalOutput Err_Light;
extern DigitalOutput RTD_Light;
extern DigitalOutput MCM_Power;
extern DigitalOutput VCU_BMS_Power; // I have no idea what this is for
extern DigitalOutput Water_Pump;
extern DigitalOutput Other_Fans;
extern DigitalOutput Accum_Fans;
extern DigitalOutput Bullshit;
extern DigitalOutput DRS_Open;
extern DigitalOutput DRS_Close;
extern DigitalOutput RTD_Sound;

// extern PWMOutput RTD_Sound;
extern PWMOutput Rad_Fans;
extern PWMOutput Accum_Fan;

extern LUT *LV_BATT_SOC_LUT;
extern WatchDog wd;
extern TorqueEncoder *tps;
extern BrakePressureSensor *bps;
extern ReadyToDriveSound *rtds;
extern MotorController *mcm;
extern InstrumentCluster *ic;
extern BatteryManagementSystem *bms;
extern SafetyChecker *sc;
extern LaunchControl *lc;
extern DRS *drs;
extern TimerDebug *td;
extern WheelSpeeds *wss;

Sensor *Sensor_new(ubyte1 pin, ubyte1 power);
Button *Button_new(ubyte1 pin, bool inverted);
PWDSensor *PWDSensor_new(ubyte1 pin);
DigitalOutput *DigitalOutput_new(ubyte1 pin, bool inverted);
PWMOutput *PWMOutput_new(ubyte1 pin, ubyte2 frequency, float4 duty);

void sensors_updateSensors(void);

void setMCMRelay(bool turnOn);
void Sensor_power_set(Sensor *sensor);
void Sensor_read(Sensor *sensor);
void Button_read(Button *button);
void PWDSensor_read(PWDSensor *sensor);
void DigitalOutput_set(DigitalOutput *output, bool value);
void PWMOutput_set(PWMOutput *output, float4 duty);

/*****************************************************************************
* Steering Angle Sensor (SAS)
Input: Voltage
Output: Degrees
****************************************************************************/
sbyte4 steering_degrees();

#endif // _SENSORS_H