#ifndef _DERATING_H
#define _DERATING_H
#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "derating.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"
#include "sensorCalculations.h"

typedef struct _Derating{
    enum {OFF, READYTODERATE, PUSHTOPASS, ON } Derating_status;
    float Derating_cellTempLim; //Once cell temmp is reached Derating will activate
    float Derating_socLim; //Once soc is reached Derating will activate
    sbyte2 Derating_torqueLim;
    float Derating_powerLim;
}Derating;

Derating* Derating_new();


#endif