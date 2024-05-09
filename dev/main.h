/*****************************************************************************
* SR-14 Vehicle Control Firmware for the TTTech HY-TTC 60 Controller (VCU)
*****************************************************************************/

//VCU/C headers
#include <stdio.h>
#include <string.h>
#include "APDB.h"
#include "IO_DIO.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
#include "IO_UART.h"

//Our code
#include "initializations.h"
#include "sensors.h"
#include "canManager.h"
#include "motorController.h"
#include "instrumentCluster.h"
#include "readyToDriveSound.h"
#include "wheelSpeeds.h"
#include "safety.h"
#include "cooling.h"
#include "bms.h"
#include "LaunchControl.h"
#include "drs.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "timerDebug.h"#include <IO_Driver.h>

#define CYCLE_TIME_US 10000 // in microseconds
#define CYCLE_TIME (1000000.0 / CYCLE_TIME_US) // in seconds
#define CAN_0_BAUD 500 // in kbps
#define CAN_1_BAUD 500 // in kbps
#define CAN_READ_MESSAGE_LIMIT 64 // in messages
#define CAN_WRITE_MESSAGE_LIMIT 32 // in messages
//16 bumps per rotation, 16 hz = 1 rotation per second
#define F_WSS_TICKS 26 // number of ticks on the tone wheel
#define R_WSS_TICKS 22 // number of ticks on the tone wheel
#define WHEEL_DIAMETER 18 //Inches
#define LV_BATT_S 8 // number of cells in series for the LV Battery
#define LC_STEERING_THRESHOLD 1500.0f // in degrees

#define SOFT_BSPD_ENABLE 0
#define POWER_LIMIT 1

