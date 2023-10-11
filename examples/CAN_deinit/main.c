/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  CAN driver
 *
 *  Implements an Echo application which uses standard message buffers
 *  As soon as anything is received it is sent back to the can bus.
 *
 **************************************************************************/

#include "IO_CAN.h"
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


void main (void)
{
    ubyte1 handle_w, handle_r, handle_fifo, size;
    IO_CAN_DATA_FRAME can_frames[25];

    IO_CAN_Init( IO_CAN_CHANNEL_0
               , 500
               , 0     //default
               , 0     //default
               , 0);   //default

    IO_CAN_ConfigMsg( &handle_w
                    , IO_CAN_CHANNEL_1
                    , IO_CAN_MSG_WRITE
                    , IO_CAN_STD_FRAME
                    , 0
                    , 0);

    IO_CAN_ConfigFIFO ( &handle_fifo
                      , IO_CAN_CHANNEL_1
                      , 10
                      , IO_CAN_MSG_READ
                      , IO_CAN_STD_FRAME
                      , 0
                      , 0 );

    IO_CAN_ConfigMsg( &handle_r
                    , IO_CAN_CHANNEL_1
                    , IO_CAN_MSG_READ
                    , IO_CAN_STD_FRAME
                    , 0
                    , 0);

    IO_CAN_DeInitHandle (handle_r);
    IO_CAN_DeInitHandle (handle_w);

    IO_CAN_ConfigFIFO( &handle_w
                    , IO_CAN_CHANNEL_1
                    , 20
                    , IO_CAN_MSG_WRITE
                    , IO_CAN_STD_FRAME
                    , 0
                    , 0);

    IO_CAN_DeInitHandle ( handle_fifo );

    IO_CAN_ConfigFIFO( &handle_r
                     , IO_CAN_CHANNEL_1
                     , 20
                     , IO_CAN_MSG_READ
                     , IO_CAN_STD_FRAME
                     , 0
                     , 0);

    while (1){
        if (IO_CAN_FIFOStatus(handle_r) == IO_E_OK)
        {
            /* read message */
            IO_CAN_ReadFIFO(handle_r, can_frames, 25, &size);

            /* and send back */
            IO_CAN_WriteFIFO(handle_w, can_frames, size);

            /* wait for successful transmission */
            while (IO_CAN_FIFOStatus(handle_w) != IO_E_OK);
        }

    }

}
















