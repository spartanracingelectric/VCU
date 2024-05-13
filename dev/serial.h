#ifndef _SERIALMANAGER_H
#define _SERIALMANAGER_H

#include "IO_Driver.h" 
#include "IO_UART.h"

#define MAX_PRINT_CHARS 32

void serial_init(void);

//usage:
//ubyte1* message = "my message";
//Write(serialMan, message);
IO_ErrorType serial_send(const ubyte1* data);
//IO_ErrorType SerialManager_sendLen(SerialManager* me, const ubyte1* data, ubyte1* dataLength);

IO_ErrorType serial_sprintf(const ubyte1* message, void* dataValue);
#endif // This header has been defined before