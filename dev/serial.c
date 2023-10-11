#include <stdlib.h> //Needed for malloc
#include <stdio.h>  //sprintf
#include <string.h>
#include "IO_Driver.h"
#include "IO_UART.h"
#include "serial.h"

SerialManager *SerialManager_new(void)
{
    SerialManager *me = (SerialManager *)malloc(sizeof(struct _SerialManager));
    IO_UART_Init(IO_UART_RS232, 115200, 8, IO_UART_PARITY_NONE, 1);

    return me;
}

IO_ErrorType SerialManager_send(SerialManager *me, const ubyte1 *data)
{
    IO_ErrorType err = IO_UART_Write(IO_UART_CH0, data, strlen(data), &me->size);
    return err;
}

IO_ErrorType SerialManager_sprintf(SerialManager *me, const ubyte1 *message, void *dataValue)
{
    ubyte1 *temp[64];
    sprintf(&temp, message, dataValue);
    IO_ErrorType err = IO_UART_Write(IO_UART_CH0, temp, strlen(temp), &me->size);
    return err;
}

//IO_ErrorType SerialManager_sendLen(SerialManager* me, const ubyte1* data, ubyte1* dataLength)
//{
//    return IO_UART_Write(IO_UART_CH0, data, dataLength, &dataLength);
//}
