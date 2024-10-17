#include <stdio.h>
#include <string.h>
#include "IO_Driver.h"
#include "IO_UART.h"
#include "IO_DIO.h"
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
    ubyte1 val = 0;
    IO_ErrorType ret0, ret1, ret2, ret3, ret4, ret5, ret6, ret7, ret8, ret9, ret10, ret11, ret12, ret13, ret14, ret15, ret16, ret17, ret18, ret19;

    IO_UART_Init( IO_UART_RS232
                , 115200
                , 8
                , IO_UART_PARITY_NONE
                , 1 );

    IO_Driver_Init( NULL );

    IO_DO_Init ( IO_DO_00 );
    IO_DO_Init ( IO_DO_01 );
    IO_DO_Init ( IO_DO_02 );
    IO_DO_Init ( IO_DO_03 );
    IO_DO_Init ( IO_DO_04 );
    IO_DO_Init ( IO_DO_05 );
    IO_DO_Init ( IO_DO_06 );
    IO_DO_Init ( IO_DO_07 );
    IO_DO_Init ( IO_DO_08 );
    IO_DO_Init ( IO_DO_09 );
    IO_DO_Init ( IO_DO_10 );
    IO_DO_Init ( IO_DO_11 );
    IO_DO_Init ( IO_DO_12 );
    IO_DO_Init ( IO_DO_13 );
    IO_DO_Init ( IO_DO_14 );
    IO_DO_Init ( IO_DO_15 );
    IO_DO_Init ( IO_DO_16 );
    IO_DO_Init ( IO_DO_17 );
    IO_DO_Init ( IO_DO_18 );
    IO_DO_Init ( IO_DO_19 );

    IO_DO_DeInit ( IO_DO_00 );
    IO_DO_DeInit ( IO_DO_01 );
    IO_DO_DeInit ( IO_DO_02 );
    IO_DO_DeInit ( IO_DO_03 );
    IO_DO_DeInit ( IO_DO_04 );
    IO_DO_DeInit ( IO_DO_05 );
    IO_DO_DeInit ( IO_DO_06 );
    IO_DO_DeInit ( IO_DO_07 );
    IO_DO_DeInit ( IO_DO_08 );
    IO_DO_DeInit ( IO_DO_09 );
    IO_DO_DeInit ( IO_DO_10 );
    IO_DO_DeInit ( IO_DO_11 );
    IO_DO_DeInit ( IO_DO_12 );
    IO_DO_DeInit ( IO_DO_13 );
    IO_DO_DeInit ( IO_DO_14 );
    IO_DO_DeInit ( IO_DO_15 );
    IO_DO_DeInit ( IO_DO_16 );
    IO_DO_DeInit ( IO_DO_17 );
    IO_DO_DeInit ( IO_DO_18 );
    IO_DO_DeInit ( IO_DO_19 );

    IO_DO_Init ( IO_DO_00 );
    IO_DO_Init ( IO_DO_01 );
    IO_DO_Init ( IO_DO_02 );
    IO_DO_Init ( IO_DO_03 );
    IO_DO_Init ( IO_DO_04 );
    IO_DO_Init ( IO_DO_05 );
    IO_DO_Init ( IO_DO_06 );
    IO_DO_Init ( IO_DO_07 );
    IO_DO_Init ( IO_DO_08 );
    IO_DO_Init ( IO_DO_09 );
    IO_DO_Init ( IO_DO_10 );
    IO_DO_Init ( IO_DO_11 );
    IO_DO_Init ( IO_DO_12 );
    IO_DO_Init ( IO_DO_13 );
    IO_DO_Init ( IO_DO_14 );
    IO_DO_Init ( IO_DO_15 );
    IO_DO_Init ( IO_DO_16 );
    IO_DO_Init ( IO_DO_17 );
    IO_DO_Init ( IO_DO_18 );
    IO_DO_Init ( IO_DO_19 );

    while (1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin();

        ret0 = IO_DO_Set ( IO_DO_00
                         , TRUE );

        ret1 = IO_DO_Set ( IO_DO_01
                         , FALSE );

        ret2 = IO_DO_Set ( IO_DO_02
                         , TRUE );

        ret3 = IO_DO_Set ( IO_DO_03
                         , FALSE );

        ret4 = IO_DO_Set ( IO_DO_00
                         , (bool) (val) );

        ret5 = IO_DO_Set ( IO_DO_05
                         , (bool) ((~val) & 0x01) );

        ret6 = IO_DO_Set ( IO_DO_06
                         , (bool) (val) );

        ret7 = IO_DO_Set ( IO_DO_07
                         , (bool) ((~val) & 0x01) );

        ret8 = IO_DO_Set ( IO_DO_08
                         , TRUE );

        ret9 = IO_DO_Set ( IO_DO_09
                         , FALSE );

        ret10 = IO_DO_Set ( IO_DO_10
                          , TRUE );

        ret11 = IO_DO_Set ( IO_DO_11
                          , FALSE );

        ret12 = IO_DO_Set ( IO_DO_12
                          , (bool) (val) );

        ret13 = IO_DO_Set ( IO_DO_13
                          , (bool) ((~val) & 0x01) );

        ret14 = IO_DO_Set ( IO_DO_14
                          , (bool) (val) );

        ret15 = IO_DO_Set ( IO_DO_15
                          , (bool) ((~val) & 0x01) );

        ret16 = IO_DO_Set ( IO_DO_16
                          , (bool) (val) );

        ret17 = IO_DO_Set ( IO_DO_17
                          , (bool) ((~val) & 0x01) );

        ret18 = IO_DO_Set ( IO_DO_18
                          , (bool) (val) );

        ret19 = IO_DO_Set ( IO_DO_19
                          , (bool) ((~val) & 0x01) );


        if ((count % 45) == 0)
        {
            val = (~val) & 0x01;
        }

        sprintf ((char *) data, "\x0C%02X %02X %02X %02X %02X %02X %02X %02X\n\r%02X %02X %02X %02X\n\r%02X %02X %02X %02X %02X %02X %02X %02X\n\r", ret0
                                                                                                                                                   , ret1
                                                                                                                                                   , ret2
                                                                                                                                                   , ret3
                                                                                                                                                   , ret4
                                                                                                                                                   , ret5
                                                                                                                                                   , ret6
                                                                                                                                                   , ret7
                                                                                                                                                   , ret8
                                                                                                                                                   , ret9
                                                                                                                                                   , ret10
                                                                                                                                                   , ret11
                                                                                                                                                   , ret12
                                                                                                                                                   , ret13
                                                                                                                                                   , ret14
                                                                                                                                                   , ret15
                                                                                                                                                   , ret16
                                                                                                                                                   , ret17
                                                                                                                                                   , ret18
                                                                                                                                                   , ret19 );
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


        while (IO_RTC_GetTimeUS(timestamp) < 50000)
        {
        }
    }
}

