/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  UART driver
 *
 *  Implements an echo application.
 *  Anything received via the RS232 interface is sent back.
 *
 **************************************************************************/

#include "IO_Driver.h"
#include "IO_UART.h"
#include "APDB.h"

APDB appl_db =
          { 0                      /* ubyte4 versionAPDB        */
          , {0}                    /* BL_T_DATE flashDate       */
                                   /* BL_T_DATE buildDate                   */
          , { (ubyte4) (((((ubyte4) RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
          , 0                      /* ubyte4 nodeType           */
          , 0                      /* ubyte4 startAddress       */
          , 0                      /* ubyte4 codeSize           */
          , 0                      /* ubyte4 legacyAppCRC       */
          , 0                      /* ubyte4 appCRC             */
          , 1                      /* ubyte1 nodeNr             */
          , 0                      /* ubyte4 CRCInit            */
          , 0                      /* ubyte4 flags              */
          , 0                      /* ubyte4 hook1              */
          , 0                      /* ubyte4 hook2              */
          , 0                      /* ubyte4 hook3              */
          , APPL_START             /* ubyte4 mainAddress        */
          , {0, 1}                 /* BL_T_CAN_ID canDownloadID */
          , {0, 2}                 /* BL_T_CAN_ID canUploadID   */
          , 0                      /* ubyte4 legacyHeaderCRC    */
          , 0                      /* ubyte4 version            */
          , 500                    /* ubyte2 canBaudrate        */
          , 0                      /* ubyte1 canChannel         */
          , {0}                    /* ubyte1 reserved[8*4]      */
          , 0                      /* ubyte4 headerCRC          */
          };


#define IO_SPI_BUFFER_LEN 32


void wait(ubyte2 time)
{
    ubyte2 i,j;

    for (i = 0; i < time; i++)
    {
        j = 0;
        do{
            IO_UART_Task();
            j--;
            __nop();
        }while (j > 0);
    }
}


void main (void)
{
    ubyte1 data[40] = {0};
    ubyte1 size;

    IO_Driver_Init( NULL );

    IO_UART_Init(IO_UART_RS232, 115200, 8, IO_UART_PARITY_NONE, 1);

    while (1){

        IO_UART_GetRxStatus(IO_UART_RS232, &size);
        if (size > 0)
        {
            IO_UART_Read(IO_UART_RS232, data, 40, &size);

            IO_UART_Write(IO_UART_RS232, data, size, &size);

            do
            {
                IO_UART_GetTxStatus(IO_UART_RS232, &size);
                IO_UART_Task();
            }while (size != 0);
        }

        /* wait and task */
        wait (0x0001);
    }
}
















