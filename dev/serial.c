#include <stdlib.h> //Needed for malloc
#include <stdio.h>  //sprintf
#include <string.h>
#include "IO_Driver.h"
#include "IO_UART.h"
#include "serial.h"
#include "mathFunctions.h"

void serial_init(void)
{
    IO_UART_Init(IO_UART_RS232, 115200, 8, IO_UART_PARITY_NONE, 1);
}

IO_ErrorType serial_send(const ubyte1 *data)
{
    IO_ErrorType err = IO_UART_Write(IO_UART_CH0, data, strlen(data), min_ew(MAX_PRINT_CHARS, strlen(data)));
    return err;
}

IO_ErrorType serial_sprintf(const ubyte1 *message, void *dataValue)
{
    ubyte1 *temp[64];
    sprintf(&temp, message, dataValue);
    IO_ErrorType err = IO_UART_Write(IO_UART_CH0, temp, strlen(temp), min_ew(MAX_PRINT_CHARS, strlen(temp)));
    return err;
}

// IO_ErrorType SerialManager_sendLen(SerialManager* me, const ubyte1* data, ubyte1* dataLength)
//{
//     return IO_UART_Write(IO_UART_CH0, data, dataLength, &dataLength);
// }
