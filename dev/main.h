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
#include "sensorCalculations.h"
#include "cooling.h"
#include "bms.h"
#include "LaunchControl.h"
#include "drs.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"