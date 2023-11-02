/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  Minimodule for Current Measurement
 *
 *  Configures current measurement channels and digital output
 *  channels on the minimodule.
 *  The current is retrieved and stored in a global variable
 *  The digital ouptuts are toggled.
 *
 **************************************************************************/


#include "IO_Driver.h"
#include "IO_RTC.h"
#include "MM_MCM.h"

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



IO_ErrorType task_err;
IO_ErrorType get_err[4];

ubyte2 channel_value[4];
bool channel_fresh[4];

ubyte2 counter = 0;

bool do_state[4];

void main (void)
{
    ubyte4 timestamp;

    IO_Driver_Init(NULL);

    MM_MCM_ChannelInit(IO_MCM_CHANNEL_00, IO_MCM_CONFIG_CURRENT);
    MM_MCM_ChannelInit(IO_MCM_CHANNEL_01, IO_MCM_CONFIG_DO);
    MM_MCM_ChannelInit(IO_MCM_CHANNEL_02, IO_MCM_CONFIG_CURRENT);
    MM_MCM_ChannelInit(IO_MCM_CHANNEL_03, IO_MCM_CONFIG_DO);

    while (1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin();

        if (counter == 5000)
        {
            do_state[1] = (do_state[1] == TRUE) ? FALSE : TRUE;
        }
        if (counter == 10000)
        {
            do_state[1] = (do_state[1] == TRUE) ? FALSE : TRUE;
            do_state[3] = (do_state[3] == TRUE) ? FALSE : TRUE;
            counter = 0;
        }


        get_err[0] = MM_MCM_GetCurr (IO_MCM_CHANNEL_00, &(channel_value[0]), &(channel_fresh[0]));
        get_err[1] = MM_MCM_SetOutput (IO_MCM_CHANNEL_01, do_state[1]);
        get_err[2] = MM_MCM_GetCurr (IO_MCM_CHANNEL_02, &(channel_value[2]), &(channel_fresh[2]));
        get_err[3] = MM_MCM_SetOutput (IO_MCM_CHANNEL_03, do_state[3]);


        task_err = MM_MCM_Task();
        IO_Driver_TaskEnd();

        if (counter < 0xFFFF)
        {
            counter++;
        }

        /* wait */
        while (IO_RTC_GetTimeUS(timestamp) < 1000);
    }
}
















