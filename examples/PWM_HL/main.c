#include <stdio.h>
#include <string.h>
#include "IO_Driver.h"
#include "IO_PWM.h"
#include "IO_RTC.h"
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

#define DUTY_LITTLE_STEP    0x100
#define DUTY_BIG_STEP       0x2000

volatile ubyte4 count = 0;
volatile ubyte2 duty = 0;
volatile ubyte2 dual_duty = 0x7FFF;
volatile ubyte2 duty_step = 0;
volatile sbyte2 dir = 0;
ubyte4 timestamp = 0;

void main(void)
{
    ubyte1 data[128] = {0};
    ubyte1 size, len_tx;
    IO_ErrorType ret0, ret1, ret2, ret3, ret4, ret5;
    ubyte2 curr0, curr1, curr2, curr3;
    bool direction = TRUE;


    IO_Driver_Init( NULL );

    IO_UART_Init( IO_UART_RS232
                , 115200
                , 8
                , IO_UART_PARITY_NONE
                , 1 );


    /* IO_PWM_00 */
    IO_PWM_Init( IO_PWM_00
               , 100
               , TRUE
               , TRUE
               , IO_ADC_CUR_00
               , FALSE
               , NULL );

    /* IO_PWM_01 */
    IO_PWM_Init( IO_PWM_01
               , 100
               , FALSE
               , TRUE
               , IO_ADC_CUR_01
               , TRUE
               , NULL );

    IO_PWM_DeInit( IO_PWM_01 );

    IO_PWM_Init( IO_PWM_01
               , 100
               , FALSE
               , TRUE
               , IO_ADC_CUR_01
               , TRUE
               , NULL );

    /* IO_PWM_02 */
    IO_PWM_Init( IO_PWM_02
               , 150
               , TRUE
               , FALSE
               , 0
               , TRUE
               , NULL );

    /* IO_PWM_03 */
    IO_PWM_Init( IO_PWM_03
               , 200
               , FALSE
               , FALSE
               , 0
               , TRUE
               , NULL );

    /* IO_PWM_04 + IO_PWM_05 */
    IO_PWM_DualInit( IO_PWM_05
                   , 200
                   , FALSE
                   , IO_ADC_CUR_03
                   , NULL );

    /* IO_PWM_06 + IO_PWM_07 */
    IO_PWM_DualInit( IO_PWM_06
                   , 100
                   , TRUE
                   , IO_ADC_CUR_02
                   , NULL );

    /* IO_PWM_04 + IO_PWM_05 */
    IO_PWM_DualDeInit( IO_PWM_05 );

    IO_PWM_DualInit( IO_PWM_05
                   , 200
                   , FALSE
                   , IO_ADC_CUR_03
                   , NULL );

    while (1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin();

        ret0 = IO_PWM_SetDuty( IO_PWM_00
                             , duty_step
                             , NULL );

        ret1 = IO_PWM_SetDuty( IO_PWM_01
                             , 0x7FFF
                             , NULL );

        ret2 = IO_PWM_SetDuty( IO_PWM_02
                             , duty
                             , NULL );

        ret3 = IO_PWM_SetDuty( IO_PWM_03
                             , duty
                             , NULL );

        ret4 = IO_PWM_DualSetDuty( IO_PWM_05
                                 , dual_duty
                                 , direction
                                 , NULL
                                 , NULL );

        ret5 = IO_PWM_DualSetDuty( IO_PWM_06
                                 , dual_duty
                                 , direction
                                 , NULL
                                 , NULL );

        IO_PWM_GetCur( IO_PWM_00
                     , &curr0 );

        IO_PWM_GetCur( IO_PWM_01
                     , &curr1 );

        IO_PWM_GetCur( IO_PWM_06
                     , &curr2 );

        IO_PWM_GetCur( IO_PWM_05
                     , &curr3 );

        if (duty >= 0xFFFF)
        {
            dir = -DUTY_LITTLE_STEP;
        }
        if (duty <= 0x0)
        {
            dir = DUTY_LITTLE_STEP;
        }
        duty += dir;

        if ((count % 150) == 0)
        {
            direction = (direction == TRUE) ? FALSE : TRUE;
        }
        dual_duty += DUTY_LITTLE_STEP;




        if ((count % 150) == 0)
        {
            duty_step += DUTY_BIG_STEP;
        }

        sprintf ((char *) data, "\x0C%02X\t%02X\t%02X\t%02X\t%02X\t%02X\n\r\n\r%3.1f%%\t%4d\t%4d\t%4d\t%4d", ret0
                                                                                                           , ret1
                                                                                                           , ret2
                                                                                                           , ret3
                                                                                                           , ret4
                                                                                                           , ret5
                                                                                                           , ((float) duty_step) / 0xFFFF * 100.0
                                                                                                           , curr0
                                                                                                           , curr1
                                                                                                           , curr2
                                                                                                           , curr3 );
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

        sprintf ((char *) data, "\n\r\n\r%04X\t%04X\t%04X", duty
                                                          , duty_step
                                                          , dual_duty );
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

        while (IO_RTC_GetTimeUS(timestamp) < 10000);
        {
        }
    }
}

