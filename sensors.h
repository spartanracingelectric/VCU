/**********************************************************************//**
 * \file sensors.h
 *
 * \brief Sensor object definitions and basic functions
 *
 *      The IO Driver high level interface provides a general
 *      initialization function, a version API and general task functions
 *      which shall wrap the whole user application.
 *
 * \section appl_structure Basic structure of an application
 *
 *      The IO Driver API provides two different types of functions:
 *      - Initialization functions:
 *              These functions are designed to be called once at the
 *              beginning of an application.
 *      - Task functions:
 *              These functions are designed to be called periodically at
 *              runtime.
 *
 *      The function IO_Driver_Init() needs to be the first function, called
 *      during the initialization.
 *
 *      All task functions need to be enclosed by the functions
 *      IO_Driver_TaskBegin() and IO_Driver_TaskEnd()
 *
 *      Example of an application:
 *
 *      \code
 *      void task (void)
 *      {
 *          IO_Driver_TaskBegin();
 *
 *          // User Application
 *          // and calls to driver task functions.
 *
 *          IO_Driver_TaskEnd();
 *      }
 *
 *
 *      void main (void)
 *      {
 *          ubyte4 timestamp
 *
 *          //--------------------------------//
 *          // start of driver initialization //
 *          //--------------------------------//
 *
 *          // IO_Driver_Init() is the first function:
 *          IO_Driver_Init(NULL);  // no safety critical application
 *
 *          //------------------------------//
 *          // end of driver initialization //
 *          //------------------------------//
 *
 *
 *          //--------------------------------------------//
 *          // from now on only task functions are called //
 *          //--------------------------------------------//
 *          while (1){
 *              IO_RTC_StartTime(&timestamp);
 *
 *              task();
 *
 *              while (IO_RTC_GetTimeUS(timestamp) < 5000);
 *          }
 *      }
 *
 *      \endcode
 *
 *      The \c task function is called every 5000us = 5ms. Please
 *      refer to the \ref IO_RTC.h "Real Time Clock" documentation for
 *      details on how to use the RTC functions.
 *
 * \section startup_limits Limitations during startup
 *         During the first cycle the measurement values (eg. ADC, PWD, etc.) are
 *         invalid, which is indicated by the corresponding fresh flag when reading the value.
 *         Starting with the second cycle the actual measurement values are available.
 *
 *         Further limitations for safety controllers:
 *         The safety controllers (\safetycontrollers) perform various tests at startup.
 *         For this reason the measurement values can hold unexpected values during this process.
 *         The application software should wait until the CPU has entered
 *         the Main state before evaluating the input variables.
 *         For details on CPU states and how to retrieve the current state,
 *         refer to \ref DIAG_Functions.h "Functions for ECU diagnostics"
 *
 * \section general_limits General limitations
 *         When the IO Driver is configured as safety critical(safety controllers \safetycontrollers),
 *         all the variables modified by an ISR shall be placed in internal RAM only, otherwise a
 *         safety state could be triggered during the execution of the External RAM Memory check.
 *
 **************************************************************************/


//"Include guard" - prevents this file from being #included more than once
#ifndef _SENSORS_H
#define _SENSORS_H

#include "IO_Driver.h"



typedef enum 
{ 
      Light_dashEco        //on/off
    , Light_dashError      //on/off
    , Light_dashRTD        //on/off
    , Light_dashTCS        //on/off
    , Light_brake          //PWM
    , Cooling_waterPump    //PWM
    , Cooling_motorFans    //on/off
    , Cooling_batteryFans  //on/off
} Light;


//----------------------------------------------------------------------------
// Sensor Object Definitions
//----------------------------------------------------------------------------
// Parameters:
//
// specMin/Max values should come from each sensor's datasheets, but it is not
// required for all sensors.
//
// TODO: What about having default calbiration values?  (Probably useless)
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
//extern Sensor Sensor_BPS1;  // = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)

//Wheel Speed Sensors (like an ABS sensor)
extern Sensor Sensor_WSS_FL;  // = { 2 };
extern Sensor Sensor_WSS_FR;  // = { 2 };
extern Sensor Sensor_WSS_RL;  // = { 2 };
extern Sensor Sensor_WSS_RR;  // = { 2 };

//Wheel Position Sensors (Shock pots)
extern Sensor Sensor_WPS_FL;  // = { 3 };
extern Sensor Sensor_WPS_FR;  // = { 3 };
extern Sensor Sensor_WPS_RL;  // = { 3 };
extern Sensor Sensor_WPS_RR;  // = { 3 };

//Steering position Sensor (SPS) - continuous rotation sensor, works like TPS, probably ratiometric
extern Sensor Sensor_SAS;  // = { 4 };

//Switches
//precharge failure
extern Sensor Sensor_RTDButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_TCSSwitchUp;
extern Sensor Sensor_TCSSwitchDown;
extern Sensor Sensor_TCSKnob;
//TEMP BENCH SWITCHES
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