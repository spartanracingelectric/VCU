/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  EEPROM driver
 *
 *  Writes data to EEPROM an reads it back again
 *
 **************************************************************************/

#include "IO_POWER.h"
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

void wait (ubyte2 time)
{
    ubyte2 i,j;

    for (i = 0; i < time; i++)
    {
        j = 0;
        do
        {
            j--;
            __nop();
        } while (j > 0);
    }
}

void main (void)
{
    ubyte1 i, state;

    IO_POWER_Init();

    i = 3;
    state = 1;
    while (1)
    {
        IO_Driver_TaskBegin();

        if (state == 1)
        {
            IO_POWER_Set (IO_PIN_136, IO_POWER_OFF);
            i--;
            if (i == 0)
            {
                state = 2;
                i = 3;
            }
        }
        else if (state == 2)
        {
            IO_POWER_Set (IO_PIN_136, IO_POWER_ON);
            i--;
            if (i == 0)
            {
                state = 3;
                i = 3;
            }
        }
        else if (state == 3)
        {
            IO_POWER_Set (IO_PIN_135, IO_POWER_OFF);
            i--;
            if (i == 0)
            {
                state = 4;
                i = 3;
            }
        }
        else if (state == 4)
        {
            IO_POWER_Set (IO_PIN_135, IO_POWER_ON);
            i--;
            if (i == 0)
            {
                state = 5;
                i = 3;
            }
        }
        else if (state == 5)
        {
            IO_POWER_Set (IO_PIN_269, IO_POWER_8_5_V);
            i--;
            if (i == 0)
            {
                state = 6;
                i = 3;
            }
        }
        else if (state == 6)
        {
            IO_POWER_Set (IO_PIN_269, IO_POWER_10_0_V);
            i--;
            if (i == 0)
            {
                state = 7;
                i = 3;
            }
        }
        else if (state == 7)
        {
            IO_POWER_Set (IO_PIN_269, IO_POWER_14_5_V);
            i--;
            if (i == 0)
            {
                state = 8;
                i = 3;
            }
        }
        else
        {
            IO_POWER_Set (IO_PIN_271, IO_POWER_OFF);
        }

        IO_Driver_TaskEnd();

        /* wait */
        wait (0x003F);
    }
}

