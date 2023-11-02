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

bool di_val_0, di_val_1, di_val_2, di_val_3, di_val_4, di_val_5, di_val_6, di_val_7, di_val_8, di_val_k15, di_val_9, di_val_10, di_val_11, di_val_12, di_val_13, di_val_14, di_val_15, di_val_16, di_val_17, di_val_18, di_val_19, di_val_20, di_val_21;

void main(void)
{
    ubyte1 data[128] = {0};
    ubyte1 size, len_tx;

    IO_Driver_Init( NULL );

    IO_UART_Init( IO_UART_RS232
                , 115200
                , 8
                , IO_UART_PARITY_NONE
                , 1 );

    IO_DI_Init( IO_DI_00
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_01
              , IO_DI_PD_10K );
    IO_DI_Init( IO_DI_02
              , IO_DI_PD_1K8 );
    IO_DI_Init( IO_DI_03
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_04
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_05
              , IO_DI_PD_10K );
    IO_DI_Init( IO_DI_06
              , IO_DI_PD_1K8 );
    IO_DI_Init( IO_DI_07
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_K15
              , 0 );

    IO_DI_DeInit( IO_DI_00 );
    IO_DI_DeInit( IO_DI_01 );
    IO_DI_DeInit( IO_DI_02 );
    IO_DI_DeInit( IO_DI_03 );
    IO_DI_DeInit( IO_DI_04 );
    IO_DI_DeInit( IO_DI_05 );
    IO_DI_DeInit( IO_DI_06 );
    IO_DI_DeInit( IO_DI_07 );
    IO_DI_DeInit( IO_DI_K15 );

    IO_DI_Init( IO_DI_00
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_01
              , IO_DI_PD_10K );
    IO_DI_Init( IO_DI_02
              , IO_DI_PD_1K8 );
    IO_DI_Init( IO_DI_03
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_04
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_05
              , IO_DI_PD_10K );
    IO_DI_Init( IO_DI_06
              , IO_DI_PD_1K8 );
    IO_DI_Init( IO_DI_07
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_K15
              , 0 );



    IO_DI_Init( IO_DI_08
              , 0 );
    IO_DI_Init( IO_DI_09
              , 0 );
    IO_DI_Init( IO_DI_10
              , 0 );
    IO_DI_Init( IO_DI_11
              , 0 );
    IO_DI_Init( IO_DI_12
              , 0 );
    IO_DI_Init( IO_DI_13
              , 0 );
    IO_DI_Init( IO_DI_14
              , 0 );
    IO_DI_Init( IO_DI_15
              , 0 );

    IO_DI_DeInit( IO_DI_08 );
    IO_DI_DeInit( IO_DI_09 );
    IO_DI_DeInit( IO_DI_10 );
    IO_DI_DeInit( IO_DI_11 );
    IO_DI_DeInit( IO_DI_12 );
    IO_DI_DeInit( IO_DI_13 );
    IO_DI_DeInit( IO_DI_14 );
    IO_DI_DeInit( IO_DI_15 );

    IO_DI_Init( IO_DI_08
              , 0 );
    IO_DI_Init( IO_DI_09
              , 0 );
    IO_DI_Init( IO_DI_10
              , 0 );
    IO_DI_Init( IO_DI_11
              , 0 );
    IO_DI_Init( IO_DI_12
              , 0 );
    IO_DI_Init( IO_DI_13
              , 0 );
    IO_DI_Init( IO_DI_14
              , 0 );
    IO_DI_Init( IO_DI_15
              , 0 );



    IO_DI_Init( IO_DI_16
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_17
              , IO_DI_PD_10K );
    IO_DI_Init( IO_DI_18
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_19
              , IO_DI_PD_1K8 );

    IO_DI_DeInit( IO_DI_16 );
    IO_DI_DeInit( IO_DI_17 );
    IO_DI_DeInit( IO_DI_18 );
    IO_DI_DeInit( IO_DI_19 );

    IO_DI_Init( IO_DI_16
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_17
              , IO_DI_PD_10K );
    IO_DI_Init( IO_DI_18
              , IO_DI_PU_10K );
    IO_DI_Init( IO_DI_19
              , IO_DI_PD_1K8 );

    while (1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_DI_Get( IO_DI_00
                 , &di_val_0 );
        IO_DI_Get( IO_DI_01
                 , &di_val_1 );
        IO_DI_Get( IO_DI_02
                 , &di_val_2 );
        IO_DI_Get( IO_DI_03
                 , &di_val_3 );
        IO_DI_Get( IO_DI_04
                 , &di_val_4 );
        IO_DI_Get( IO_DI_05
                 , &di_val_5 );
        IO_DI_Get( IO_DI_06
                 , &di_val_6 );
        IO_DI_Get( IO_DI_07
                 , &di_val_7 );
        IO_DI_Get( IO_DI_K15
                 , &di_val_k15 );

        IO_DI_Get( IO_DI_08
                 , &di_val_8 );
        IO_DI_Get( IO_DI_09
                 , &di_val_9 );
        IO_DI_Get( IO_DI_10
                 , &di_val_10 );
        IO_DI_Get( IO_DI_11
                 , &di_val_11 );
        IO_DI_Get( IO_DI_12
                 , &di_val_12 );
        IO_DI_Get( IO_DI_13
                 , &di_val_13 );
        IO_DI_Get( IO_DI_14
                 , &di_val_14 );
        IO_DI_Get( IO_DI_15
                 , &di_val_15 );

        IO_DI_Get( IO_DI_16
                 , &di_val_16 );
        IO_DI_Get( IO_DI_17
                 , &di_val_17 );
        IO_DI_Get( IO_DI_18
                 , &di_val_18 );
        IO_DI_Get( IO_DI_19
                 , &di_val_19 );


        sprintf ((char *) data, "\x0C%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\n\r%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\n\r%4d\t%4d\t%4d\t%4d\n\r", (ubyte1) di_val_0
                                                                                                                                                     , (ubyte1) di_val_1
                                                                                                                                                     , (ubyte1) di_val_2
                                                                                                                                                     , (ubyte1) di_val_3
                                                                                                                                                     , (ubyte1) di_val_4
                                                                                                                                                     , (ubyte1) di_val_5
                                                                                                                                                     , (ubyte1) di_val_6
                                                                                                                                                     , (ubyte1) di_val_7
                                                                                                                                                     , (ubyte1) di_val_k15
                                                                                                                                                     , (ubyte1) di_val_8
                                                                                                                                                     , (ubyte1) di_val_9
                                                                                                                                                     , (ubyte1) di_val_10
                                                                                                                                                     , (ubyte1) di_val_11
                                                                                                                                                     , (ubyte1) di_val_12
                                                                                                                                                     , (ubyte1) di_val_13
                                                                                                                                                     , (ubyte1) di_val_14
                                                                                                                                                     , (ubyte1) di_val_15
                                                                                                                                                     , (ubyte1) di_val_16
                                                                                                                                                     , (ubyte1) di_val_17
                                                                                                                                                     , (ubyte1) di_val_18
                                                                                                                                                     , (ubyte1) di_val_19 );
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

