/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  ADC driver
 *
 *  Setup all ADC's and print the values on the serial interface
 *
 **************************************************************************/

#include <stdio.h>
#include <string.h>
#include "IO_Driver.h"
#include "IO_UART.h"
#include "IO_ADC.h"
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
    ubyte2 adc_val_0, adc_val_1, adc_val_2, adc_val_3, adc_val_4, adc_val_5, adc_val_6, adc_val_7, adc_val_8, adc_val_9, adc_val_10, adc_val_11, adc_val_12, adc_val_13, adc_val_14, adc_val_15, adc_val_16, adc_val_17, adc_val_18, adc_val_19, adc_val_20, adc_val_21, adc_val_22, adc_val_23, adc_val_24, adc_val_25, adc_val_26, adc_val_27, adc_val_28;
    bool adc_fresh_0, adc_fresh_1, adc_fresh_2, adc_fresh_3, adc_fresh_4, adc_fresh_5, adc_fresh_6, adc_fresh_7, adc_fresh_8, adc_fresh_9, adc_fresh_10, adc_fresh_11, adc_fresh_12, adc_fresh_13, adc_fresh_14, adc_fresh_15, adc_fresh_16, adc_fresh_17, adc_fresh_18, adc_fresh_19, adc_fresh_20, adc_fresh_21, adc_fresh_22, adc_fresh_23, adc_fresh_24, adc_fresh_25, adc_fresh_26, adc_fresh_27, adc_fresh_28;

    IO_Driver_Init(NULL);

    IO_UART_Init( IO_UART_RS232
                , 115200
                , 8
                , IO_UART_PARITY_NONE
                , 1 );

    IO_POWER_Set( IO_PIN_269
                , IO_POWER_8_5_V );

    /* 5V ADC inputs */
    IO_ADC_ChannelInit( IO_ADC_5V_00
                      , IO_ADC_CURRENT
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_01
                      , IO_ADC_CURRENT
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_02
                      , IO_ADC_RESISTIVE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_03
                      , IO_ADC_RESISTIVE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_04
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_05
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_06
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_07
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelDeInit( IO_ADC_5V_00 );
    IO_ADC_ChannelDeInit( IO_ADC_5V_01 );
    IO_ADC_ChannelDeInit( IO_ADC_5V_02 );
    IO_ADC_ChannelDeInit( IO_ADC_5V_03 );
    IO_ADC_ChannelDeInit( IO_ADC_5V_04 );
    IO_ADC_ChannelDeInit( IO_ADC_5V_05 );
    IO_ADC_ChannelDeInit( IO_ADC_5V_06 );
    IO_ADC_ChannelDeInit( IO_ADC_5V_07 );

    IO_ADC_ChannelInit( IO_ADC_5V_00
                      , IO_ADC_CURRENT
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_01
                      , IO_ADC_CURRENT
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_02
                      , IO_ADC_RESISTIVE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_03
                      , IO_ADC_RESISTIVE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_04
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_05
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_06
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_5V_07
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);
    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);
    IO_POWER_Set (IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);

    /* ADC inputs with variable range selection */
    IO_ADC_ChannelInit( IO_ADC_VAR_00
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_10V
                      , IO_ADC_PU_10K
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_01
                      , IO_ADC_ABSOLUTE
                      , IO_ADC_RANGE_10V
                      , IO_ADC_PD_10K
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_02
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_10V
                      , IO_ADC_PU_10K
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_03
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_10V
                      , IO_ADC_PD_10K
                      , IO_SENSOR_SUPPLY_VAR
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_04
                      , IO_ADC_ABSOLUTE
                      , IO_ADC_RANGE_25V
                      , IO_ADC_PU_10K
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_05
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_25V
                      , IO_ADC_PD_10K
                      , IO_SENSOR_SUPPLY_VAR
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_06
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_25V
                      , IO_ADC_PU_10K
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_07
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_25V
                      , IO_ADC_PD_10K
                      , IO_SENSOR_SUPPLY_VAR
                      , NULL );

    IO_ADC_ChannelDeInit( IO_ADC_VAR_00 );
    IO_ADC_ChannelDeInit( IO_ADC_VAR_01 );
    IO_ADC_ChannelDeInit( IO_ADC_VAR_02 );
    IO_ADC_ChannelDeInit( IO_ADC_VAR_03 );
    IO_ADC_ChannelDeInit( IO_ADC_VAR_04 );
    IO_ADC_ChannelDeInit( IO_ADC_VAR_05 );
    IO_ADC_ChannelDeInit( IO_ADC_VAR_06 );
    IO_ADC_ChannelDeInit( IO_ADC_VAR_07 );

    IO_ADC_ChannelInit( IO_ADC_VAR_00
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_5V
                      , IO_ADC_PU_10K
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_01
                      , IO_ADC_ABSOLUTE
                      , IO_ADC_RANGE_5V
                      , IO_ADC_PD_10K
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_02
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_5V
                      , IO_ADC_PU_10K
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_03
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_5V
                      , IO_ADC_PD_10K
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_04
                      , IO_ADC_ABSOLUTE
                      , IO_ADC_RANGE_15V
                      , IO_ADC_PU_10K
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_05
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_15V
                      , IO_ADC_PD_10K
                      , IO_SENSOR_SUPPLY_VAR
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_06
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_15V
                      , IO_ADC_PU_10K
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_VAR_07
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_15V
                      , IO_ADC_PD_10K
                      , IO_SENSOR_SUPPLY_VAR
                      , NULL );

    /* normal ADC inputs */
    IO_ADC_ChannelInit( IO_ADC_00
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_01
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_02
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_03
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_04
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_05
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_06
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_07
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelDeInit( IO_ADC_00 );
    IO_ADC_ChannelDeInit( IO_ADC_01 );
    IO_ADC_ChannelDeInit( IO_ADC_02 );
    IO_ADC_ChannelDeInit( IO_ADC_03 );
    IO_ADC_ChannelDeInit( IO_ADC_04 );
    IO_ADC_ChannelDeInit( IO_ADC_05 );
    IO_ADC_ChannelDeInit( IO_ADC_06 );
    IO_ADC_ChannelDeInit( IO_ADC_07 );

    IO_ADC_ChannelInit( IO_ADC_00
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_01
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_02
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_03
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_04
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_05
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_06
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_1
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_07
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_BOARD_TEMP
                      , 0
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_SENSOR_SUPPLY_VAR
                      , 0
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_SENSOR_SUPPLY_0
                      , 0
                      , 0
                      , 0
                      , 0
                      , NULL );

    IO_ADC_ChannelInit( IO_ADC_SENSOR_SUPPLY_1
                      , 0
                      , 0
                      , 0
                      , 0
                      , NULL );

    while (1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin ();

        /* 5V ADC inputs */
        IO_ADC_Get( IO_ADC_5V_00
                  , &adc_val_0
                  , &adc_fresh_0 );

        IO_ADC_Get( IO_ADC_5V_01
                  , &adc_val_1
                  , &adc_fresh_1 );

        IO_ADC_Get( IO_ADC_5V_02
                  , &adc_val_2
                  , &adc_fresh_2 );

        IO_ADC_Get( IO_ADC_5V_03
                  , &adc_val_3
                  , &adc_fresh_3 );

        IO_ADC_Get( IO_ADC_5V_04
                  , &adc_val_4
                  , &adc_fresh_4 );

        IO_ADC_Get( IO_ADC_5V_05
                  , &adc_val_5
                  , &adc_fresh_5 );

        IO_ADC_Get( IO_ADC_5V_06
                  , &adc_val_6
                  , &adc_fresh_6 );

        IO_ADC_Get( IO_ADC_5V_07
                  , &adc_val_7
                  , &adc_fresh_7 );

        /* ADC inputs with variable range selection */
        IO_ADC_Get( IO_ADC_VAR_00
                  , &adc_val_8
                  , &adc_fresh_8 );

        IO_ADC_Get( IO_ADC_VAR_01
                  , &adc_val_9
                  , &adc_fresh_9 );

        IO_ADC_Get( IO_ADC_VAR_02
                  , &adc_val_10
                  , &adc_fresh_10 );

        IO_ADC_Get( IO_ADC_VAR_03
                  , &adc_val_11
                  , &adc_fresh_11 );

        IO_ADC_Get( IO_ADC_VAR_04
                  , &adc_val_12
                  , &adc_fresh_12 );

        IO_ADC_Get( IO_ADC_VAR_05
                  , &adc_val_13
                  , &adc_fresh_13 );

        IO_ADC_Get( IO_ADC_VAR_06
                  , &adc_val_14
                  , &adc_fresh_14 );

        IO_ADC_Get( IO_ADC_VAR_07
                  , &adc_val_15
                  , &adc_fresh_15 );

        /* normal ADC inputs */
        IO_ADC_Get( IO_ADC_00
                  , &adc_val_16
                  , &adc_fresh_16 );

        IO_ADC_Get( IO_ADC_01
                  , &adc_val_17
                  , &adc_fresh_17 );

        IO_ADC_Get( IO_ADC_02
                  , &adc_val_18
                  , &adc_fresh_18 );

        IO_ADC_Get( IO_ADC_03
                  , &adc_val_19
                  , &adc_fresh_19 );

        IO_ADC_Get( IO_ADC_04
                  , &adc_val_20
                  , &adc_fresh_20 );

        IO_ADC_Get( IO_ADC_05
                  , &adc_val_21
                  , &adc_fresh_21 );

        IO_ADC_Get( IO_ADC_06
                  , &adc_val_22
                  , &adc_fresh_22 );

        IO_ADC_Get( IO_ADC_07
                  , &adc_val_23
                  , &adc_fresh_23 );

        IO_ADC_Get( IO_BOARD_TEMP
                  , &adc_val_24
                  , &adc_fresh_24 );

        IO_ADC_Get( IO_SENSOR_SUPPLY_VAR
                  , &adc_val_25
                  , &adc_fresh_25 );

        IO_ADC_Get( IO_ADC_SENSOR_SUPPLY_0
                  , &adc_val_26
                  , &adc_fresh_26 );

        IO_ADC_Get( IO_ADC_SENSOR_SUPPLY_1
                  , &adc_val_27
                  , &adc_fresh_27 );

        IO_ADC_Get( IO_ADC_UBAT
                  , &adc_val_28
                  , &adc_fresh_28 );

        sprintf ((char *) data, "\x0C%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\n\r%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\n\r%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d", adc_val_0
                                                                                                                                                                , adc_val_1
                                                                                                                                                                , adc_val_2
                                                                                                                                                                , adc_val_3
                                                                                                                                                                , adc_val_4
                                                                                                                                                                , adc_val_5
                                                                                                                                                                , adc_val_6
                                                                                                                                                                , adc_val_7
                                                                                                                                                                , adc_val_8
                                                                                                                                                                , adc_val_9
                                                                                                                                                                , adc_val_10
                                                                                                                                                                , adc_val_11
                                                                                                                                                                , adc_val_12
                                                                                                                                                                , adc_val_13
                                                                                                                                                                , adc_val_14
                                                                                                                                                                , adc_val_15
                                                                                                                                                                , adc_val_16
                                                                                                                                                                , adc_val_17
                                                                                                                                                                , adc_val_18
                                                                                                                                                                , adc_val_19
                                                                                                                                                                , adc_val_20
                                                                                                                                                                , adc_val_21
                                                                                                                                                                , adc_val_22
                                                                                                                                                                , adc_val_23 );
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

        sprintf ((char *) data, "\n\r%4d\t%4d\t%4d\t%4d\t%4d", adc_val_24
                                                             , adc_val_25
                                                             , adc_val_26
                                                             , adc_val_27
                                                             , adc_val_28 );
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

