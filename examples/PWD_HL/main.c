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
    IO_ErrorType ret0, ret1, ret2, ret3, ret4, ret5, ret6, ret7, ret8, ret9, ret10, ret11;
    ubyte4 val1, val3, val5, val7, val8_1, val9_1, val10_1, val11_1;
    ubyte2 val8_0, val9_0, val10_0, val11_0, val0, val2, val4, val6;

    IO_UART_Init( IO_UART_RS232
                , 115200
                , 8
                , IO_UART_PARITY_NONE
                , 1 );

    IO_Driver_Init( NULL );

    IO_PWD_FreqInit( IO_PWD_00
                   , IO_PWD_FALLING_VAR );

    IO_PWD_PulseInit( IO_PWD_01
                    , IO_PWD_HIGH_TIME );

    IO_PWD_FreqInit( IO_PWD_02
                   , IO_PWD_FALLING_VAR );

    IO_PWD_PulseInit( IO_PWD_03
                    , IO_PWD_LOW_TIME );

    IO_PWD_FreqInit( IO_PWD_04
                   , IO_PWD_FALLING_VAR );

    IO_PWD_PulseInit( IO_PWD_05
                    , IO_PWD_HIGH_TIME );

    IO_PWD_FreqInit( IO_PWD_06
                  , IO_PWD_FALLING_VAR );

    IO_PWD_PulseInit( IO_PWD_07
                    , IO_PWD_LOW_TIME );

    IO_PWD_FreqDeInit( IO_PWD_06 );
    IO_PWD_PulseDeInit( IO_PWD_01 );
    IO_PWD_FreqDeInit( IO_PWD_00 );
    IO_PWD_PulseDeInit( IO_PWD_07 );
    IO_PWD_FreqDeInit( IO_PWD_04 );
    IO_PWD_PulseDeInit( IO_PWD_05 );
    IO_PWD_FreqDeInit( IO_PWD_02 );
    IO_PWD_PulseDeInit( IO_PWD_03 );

    IO_PWD_FreqInit( IO_PWD_00
                   , IO_PWD_FALLING_VAR );

    IO_PWD_PulseInit( IO_PWD_01
                    , IO_PWD_HIGH_TIME );

    IO_PWD_FreqInit( IO_PWD_02
                   , IO_PWD_FALLING_VAR );

    IO_PWD_PulseInit( IO_PWD_03
                    , IO_PWD_LOW_TIME );

    IO_PWD_FreqInit( IO_PWD_04
                   , IO_PWD_FALLING_VAR );

    IO_PWD_PulseInit( IO_PWD_05
                    , IO_PWD_HIGH_TIME );

    IO_PWD_FreqInit( IO_PWD_06
                   , IO_PWD_FALLING_VAR );

    IO_PWD_PulseInit( IO_PWD_07
                    , IO_PWD_LOW_TIME );


    IO_PWD_ComplexInit( IO_PWD_08
                      , IO_PWD_HIGH_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_0_8
                      , 0
                      , IO_PWD_THRESH_1_25V
                      , IO_PWD_PU_10K
                      , NULL );

    IO_PWD_ComplexInit( IO_PWD_09
                      , IO_PWD_LOW_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_1_6
                      , 0
                      , IO_PWD_THRESH_1_25V
                      , IO_PWD_PD_10K
                      , NULL );

    IO_PWD_ComplexInit( IO_PWD_10
                      , IO_PWD_HIGH_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_3_2
                      , 0
                      , IO_PWD_THRESH_1_25V
                      , IO_PWD_PD_1K8
                      , NULL );

    IO_PWD_ComplexInit( IO_PWD_11
                      , IO_PWD_LOW_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_3_2
                      , 0
                      , IO_PWD_THRESH_1_25V
                      , IO_PWD_PD_110
                      , NULL );

    IO_PWD_ComplexDeInit( IO_PWD_10 );
    IO_PWD_ComplexDeInit( IO_PWD_11 );
    IO_PWD_ComplexDeInit( IO_PWD_08 );
    IO_PWD_ComplexDeInit( IO_PWD_09 );

    IO_PWD_ComplexInit( IO_PWD_08
                      , IO_PWD_HIGH_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_0_2
                      , 0
                      , IO_PWD_THRESH_1_25V
                      , IO_PWD_PU_10K
                      , NULL );

    IO_PWD_ComplexInit( IO_PWD_09
                      , IO_PWD_LOW_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_3_2
                      , 0
                      , IO_PWD_THRESH_1_25V
                      , IO_PWD_PD_10K
                      , NULL );

    IO_PWD_ComplexInit( IO_PWD_10
                      , IO_PWD_HIGH_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_0_2
                      , 8
                      , IO_PWD_THRESH_1_25V
                      , IO_PWD_PD_1K8
                      , NULL );

    IO_PWD_ComplexInit( IO_PWD_11
                      , IO_PWD_LOW_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_0_8
                      , 0
                      , IO_PWD_THRESH_1_25V
                      , IO_PWD_PD_10K
                      , NULL );

    while (1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin();

        ret0 = IO_PWD_FreqGet( IO_PWD_00
                             , &val0 );

        ret1 = IO_PWD_PulseGet( IO_PWD_01
                              , &val1 );

        ret2 = IO_PWD_FreqGet( IO_PWD_02
                             , &val2 );

        ret3 = IO_PWD_PulseGet( IO_PWD_03
                              , &val3 );

        ret4 = IO_PWD_FreqGet( IO_PWD_04
                             , &val4 );

        ret5 = IO_PWD_PulseGet( IO_PWD_05
                              , &val5 );

        ret6 = IO_PWD_FreqGet( IO_PWD_06
                             , &val6 );

        ret7 = IO_PWD_PulseGet( IO_PWD_07
                              , &val7 );

        ret8 = IO_PWD_ComplexGet( IO_PWD_08
                                , &val8_0
                                , &val8_1
                                , NULL );

        ret9 = IO_PWD_ComplexGet( IO_PWD_09
                                , &val9_0
                                , &val9_1
                                , NULL );

        ret10 = IO_PWD_ComplexGet( IO_PWD_10
                                 , &val10_0
                                 , &val10_1
                                 , NULL );

        ret11 = IO_PWD_ComplexGet( IO_PWD_11
                                 , &val11_0
                                 , &val11_1
                                 , NULL );

        sprintf ((char *) data, "\x0C%4.1f\t%4.1f\t%4.1f\t%4.1f\t%4.1f\t%4.1f\t%4.1f\t%4.1f\n\r%02X|%02X|%02X|%02X|%02X|%02X|%02X|%02X\n\r\n\r", (float) val0
                                                                                                                                         , (float) val1 / 1000.0
                                                                                                                                         , (float) val2
                                                                                                                                         , (float) val3 / 1000.0
                                                                                                                                         , (float) val4
                                                                                                                                         , (float) val5 / 1000.0
                                                                                                                                         , (float) val6
                                                                                                                                         , (float) val7 / 1000.0
                                                                                                                                         , ret0
                                                                                                                                         , ret1
                                                                                                                                         , ret2
                                                                                                                                         , ret3
                                                                                                                                         , ret4
                                                                                                                                         , ret5
                                                                                                                                         , ret6
                                                                                                                                         , ret7 );

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

        sprintf ((char *) data, "%4.1f\t%4.1f\t%4.1f\t%4.1f\t%4.1f\t%4.1f\t%4.1f\t%4.1f\n\r%02X|%02X|%02X|%02X", (float) val8_0
                                                                                                                 , (float) val8_1
                                                                                                                 , (float) val9_0
                                                                                                                 , (float) val9_1
                                                                                                                 , (float) val10_0
                                                                                                                 , (float) val10_1
                                                                                                                 , (float) val11_0
                                                                                                                 , (float) val11_1
                                                                                                                 , ret8
                                                                                                                 , ret9
                                                                                                                 , ret10
                                                                                                                 , ret11 );


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

        while (IO_RTC_GetTimeUS(timestamp) < 50000);
        {
        }
    }
}

