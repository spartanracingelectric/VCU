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

    return me;

}

void DAQ_parseCanMessage(_DAQSensors* me, IO_CAN_DATA_FRAME* daqCANMessage){

    //Issue with it not subtracting the value correctly due to the DBC (offset required) -> -320 and -250 dont work
    //Compare it with 320 and 250

    if(daqCANMessage->id == 0x400) {

        me->AccelX = (((ubyte4)daqCANMessage->data[1] << 8 | daqCANMessage->data[0]) - 320.0); //m/s2
        me->AccelY = (((ubyte4)daqCANMessage->data[3] << 8 | daqCANMessage->data[2]) - 320.0); //m/s2
        
    } else if(daqCANMessage->id == 0x402) {
  
        me->GyroZ = (((ubyte4)daqCANMessage->data[5] << 8 | daqCANMessage->data[4]) - 250.0); //deg/s

    }
}