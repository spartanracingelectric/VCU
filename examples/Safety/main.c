/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  EEPROM driver
 *
 *  Safety example
 *
 **************************************************************************/

#include <stdio.h>
#include <string.h>
#include "IO_Driver.h"
#include "DIAG_Constants.h"
#include "DIAG_Functions.h"
#include "APDB.h"
#include "IO_ADC.h"
#include "IO_PWM.h"

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


void appl_init (void)
{
    IO_ADC_SAFETY_CONF temp_adc;
    IO_PWM_SAFETY_CONF temp_pwm;
    IO_PWD_CPLX_SAFETY_CONF temp_pwd;
    IO_PWD_INC_SAFETY_CONF temp_inc;

    /* 5V ADC init */
    temp_adc.adc_val_upper = 90;
    temp_adc.adc_val_lower = 10;
    temp_adc.redundant_channel = IO_ADC_NON_REDUNDANT;
    IO_ADC_ChannelInit( IO_ADC_5V_00
                      , IO_ADC_RATIOMETRIC
                      , 0
                      , 0
                      , IO_ADC_SENSOR_SUPPLY_0
                      , &temp_adc );


    /* ADC var init */
    temp_adc.adc_val_upper = 95;
    temp_adc.adc_val_lower = 5;
    temp_adc.redundant_channel = IO_ADC_NON_REDUNDANT;
    IO_ADC_ChannelInit( IO_ADC_VAR_03
                      , IO_ADC_RATIOMETRIC
                      , IO_ADC_RANGE_20V
                      , IO_ADC_PD_10K
                      , IO_ADC_SENSOR_SUPPLY_0
                      , &temp_adc );


    /* PWM init */
    temp_pwm.resistance = 10;
    IO_PWM_Init( IO_PWM_00
               , 200
               , TRUE
               , TRUE
               , IO_ADC_CUR_02
               , TRUE
               , &temp_pwm );

    /* PWD init */
    temp_pwd.pwd_freq_val_upper = 10000;
    temp_pwd.pwd_freq_val_lower = 100;
    temp_pwd.pwd_pulse_val_upper = 10000;
    temp_pwd.pwd_pulse_val_lower = 100;
    IO_PWD_ComplexInit( IO_PWD_10
                      , IO_PWD_HIGH_TIME
                      , IO_PWD_FALLING_VAR
                      , IO_PWD_RESOLUTION_0_2
                      , 0
                      , IO_PWD_THRESH_2_5V
                      , IO_PWD_PD_1K8
                      , &temp_pwd );

    /* PWD incremental init */
    temp_inc.pwd_cnt_val_lower = 1000;
    temp_inc.pwd_cnt_val_upper = 50000;
    IO_PWD_IncInit( IO_PWD_08
                  , IO_PWD_INC_1_COUNT
                  , 0x7FFF
                  , IO_PWD_THRESH_1_25V
                  , IO_PWD_PD_110
                  , &temp_inc );
}

void task (void)
{
    ubyte2 pwd_inc, pwd_comp_freq, adc_5v, adc_32v, curr;
    ubyte4 pwd_comp_pulse;
    bool bl_temp;
    ubyte1 diag_state;
    ubyte1 watchdog_state;
    DIAG_ERRORCODE diag_error;
    DIAG_ERRORCODE watchdog_error;
    ubyte1 data[128] = {0};
    ubyte1 size, len_tx;

    DIAG_Status( &diag_state
               , &watchdog_state
               , &diag_error
               , &watchdog_error );



    /* ADC 5V inputs */
    IO_ADC_Get( IO_ADC_5V_00
              , &adc_5v
              , &bl_temp );

    /* ADC var inputs */
    IO_ADC_Get( IO_ADC_VAR_03
              , &adc_32v
              , &bl_temp );

    /* PWM outputs */
    IO_PWM_SetDuty( IO_PWM_00
                  , 0x7FFF
                  , NULL );

    IO_PWM_GetCur( IO_PWM_00
                 , &curr );

    /* PWD inputs */
    IO_PWD_ComplexGet( IO_PWD_10
                     , &pwd_comp_freq
                     , &pwd_comp_pulse
                     , NULL );

    /* PWD incremental */
    IO_PWD_IncGet ( IO_PWD_08
                  , &pwd_inc );

    sprintf ((char *) data, "%02X\t%02X\n\r", diag_state, watchdog_state);
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
}


void main (void)
{
    ubyte4 timestamp;
    IO_DRIVER_SAFETY_CONF safety_conf;

    /* Driver init */
    safety_conf.glitch_filter_time = 180;
    safety_conf.command_period = 10000;
    safety_conf.timeout_count = 3;
    safety_conf.CPU_check_loopcount = 1;
    IO_Driver_Init(&safety_conf);

    /* RS232 init */
    IO_UART_Init( IO_UART_RS232
                , 115200
                , 8
                , IO_UART_PARITY_NONE
                , 1 );

    appl_init();

    while (1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin();

        task();

        IO_Driver_TaskEnd();

        /* wait */
        while (IO_RTC_GetTimeUS(timestamp) < safety_conf.command_period);
    }
}

