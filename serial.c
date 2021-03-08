#include <stdlib.h> //Needed for malloc
#include <stdio.h>  //sprintf
#include <string.h>
#include "IO_Driver.h"
#include "IO_UART.h"
#include "serial.h"

struct _SerialManager
{
    //Init stuff
    //speed
    //packet size
    //???

    //More stuff
    ubyte1 size; //This value is thrown away
};

SerialManager *SerialManager_new(void)
{
    SerialManager *me = (SerialManager *)malloc(sizeof(struct _SerialManager));
    IO_UART_Init(IO_UART_RS232, 115200, 8, IO_UART_PARITY_NONE, 1);

    return me;
}

// IO_ErrorType SerialManager_send(SerialManager *me, const ubyte1 *data)
// {
//     IO_ErrorType err = IO_UART_Write(IO_UART_CH0, data, strlen(data), &me->size);
//     return err;
// }

void SerialManager_send(SerialManager* me, const ubyte1* data)
{
    IO_UART_Write(IO_UART_CH0, data, strlen(data), &me->size);
    do
    {
        IO_UART_GetTxStatus(IO_UART_CH0, &me->size);
        IO_UART_Task();
    } while (me->size != 0);
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
