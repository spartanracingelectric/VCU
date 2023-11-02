#include <stdio.h>
#include <string.h>
#include "IO_Driver.h"
#include "IO_UART.h"
#include "IO_PWD.h"
#include "IO_RTC.h"
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


volatile ubyte4 count = 0;
ubyte4 timestamp = 0;

void main(void)
{
    ubyte1 data[128] = {0};
    ubyte1 size, len_tx;
    IO_ErrorType ret0, ret1;
    ubyte2 val0, val1;

    IO_UART_Init( IO_UART_RS232
                , 115200
                , 8
                , IO_UART_PARITY_NONE
                , 1 );

    IO_Driver_Init( NULL );

    IO_PWD_IncInit( IO_PWD_08
                  , IO_PWD_INC_1_COUNT
                  , 0x7FFF
                  , IO_PWD_THRESH_1_25V
                  , IO_PWD_PD_110
                  , NULL );

    IO_PWD_IncInit( IO_PWD_11
                  , IO_PWD_INC_2_COUNT
                  , 0x7FFF
                  , IO_PWD_THRESH_2_5V
                  , IO_PWD_PD_1K8
                  , NULL );

    IO_PWD_IncDeInit( IO_PWD_08 );
    IO_PWD_IncDeInit( IO_PWD_10 );

    IO_PWD_IncInit( IO_PWD_08
                  , IO_PWD_INC_1_COUNT
                  , 0x7FFF
                  , IO_PWD_THRESH_1_25V
                  , IO_PWD_PD_110
                  , NULL );

    IO_PWD_IncInit( IO_PWD_11
                  , IO_PWD_INC_2_COUNT
                  , 0x7FFF
                  , IO_PWD_THRESH_2_5V
                  , IO_PWD_PD_1K8
                  , NULL );

    /* For Incremental Decoder supply */
    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);

    while (1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin();

        ret0 = IO_PWD_IncGet ( IO_PWD_08
                             , &val0 );

        ret1 = IO_PWD_IncGet ( IO_PWD_11
                             , &val1 );

        sprintf ((char *) data, "\x0C%02X\t%04X\t%02X\t%04X\n\r", ret0
                                                                , val0
                                                                , ret1
                                                                , val1 );

        len_tx = (ubyte1)strlen((char *) data);
        IO_UART_Write( IO_UART_RS232
                     , data
                     , len_tx
                     , &size );
        do
        {
            IO_UART_GetTxStatus( IO_UART_RS232
                               , &size );
            IO_UART_Task();
        }while (size != 0);

        count++;

        IO_Driver_TaskEnd();

        while (IO_RTC_GetTimeUS(timestamp) < 20000)
        {
        }
    }
}

