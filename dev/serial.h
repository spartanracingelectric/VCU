#ifndef _SERIALMANAGER_H
#define _SERIALMANAGER_H

#include "IO_Driver.h" 
#include "IO_UART.h"

typedef struct _SerialManager
{
    //Init stuff
    //speed
    //packet size
    //???

    //More stuff
    ubyte1 size; //This value is thrown away
} SerialManager;

//Make serialMan available globally
//SerialManager* serialMan;

//Note: Sum of messageLimits must be < 128 (hardware only does 128 total messages)
SerialManager* SerialManager_new(void);

//usage:
//ubyte1* message = "my message";
//Write(serialMan, message);
IO_ErrorType SerialManager_send(SerialManager* me, const ubyte1* data);
//IO_ErrorType SerialManager_sendLen(SerialManager* me, const ubyte1* data, ubyte1* dataLength);

IO_ErrorType SerialManager_sprintf(SerialManager* me, const ubyte1* message, void* dataValue);
#endif // This header has been defined before