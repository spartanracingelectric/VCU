#include <stdlib.h> 
#include "IO_CAN.h"
#include "IO_Driver.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "daqSensors.h"

_DAQSensors* DAQ_Sensor_new(){

    _DAQSensors* me = (_DAQSensors*)malloc(sizeof(_DAQSensors));

    me->AccelX = 0;
    me->AccelY = 0;
    me->GyroZ = 0;
    me->LoadCell_1 = 0;
    me->LoadCell_2 = 0;
    me->LoadCell_3 = 0;
    me->LoadCell_4 = 0;

    return me;

}

void DAQ_parseCanMessage(_DAQSensors* me, IO_CAN_DATA_FRAME* daqCANMessage){

}