#ifndef _DAQSENSORS_H
#define _DAQSENSORS_H

#include <stdlib.h> 
#include "IO_CAN.h"
#include "IO_Driver.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"

typedef struct DAQSensors {
    
    float AccelX;
    float AccelY;
    float GyroZ;
    float LoadCell_1;
    float LoadCell_2;
    float LoadCell_3;
    float LoadCell_4;

} _DAQSensors;

_DAQSensors* DAQ_Sensor_new();

void DAQ_parseCanMessage(_DAQSensors* daq1, IO_CAN_DATA_FRAME* daqCANMessage);

#endif