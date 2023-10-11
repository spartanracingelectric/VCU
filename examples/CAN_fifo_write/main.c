/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  CAN driver
 *
 *  Demonstrates how to transmit CAN messages using FIFO buffers
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
    volatile ubyte1 i = 1;
    ubyte1 handle_fifo;
    IO_CAN_DATA_FRAME can_frame[20] = {{{0}}};

    IO_CAN_Init( IO_CAN_CHANNEL_0
               , 500
               , 0     //default
               , 0     //default
               , 0);   //default


    IO_CAN_ConfigFIFO( &handle_fifo
                     , IO_CAN_CHANNEL_0
                     , 10
                     , IO_CAN_MSG_WRITE
                     , IO_CAN_STD_FRAME
                     , 0
                     , 0 );

   can_frame[0].id = 1;
   can_frame[0].id_format = IO_CAN_STD_FRAME;
   can_frame[0].length = 4;
   can_frame[0].data[0] = 1;
   can_frame[0].data[1] = 2;
   can_frame[0].data[2] = 3;
   can_frame[0].data[3] = 4;

   can_frame[1].id = 2;
   can_frame[1].id_format = IO_CAN_STD_FRAME;
   can_frame[1].length = 8;
   can_frame[1].data[0] = 5;
   can_frame[1].data[1] = 6;
   can_frame[1].data[2] = 7;
   can_frame[1].data[3] = 8;
   can_frame[1].data[4] = 9;
   can_frame[1].data[5] = 10;
   can_frame[1].data[6] = 11;
   can_frame[1].data[7] = 12;

   can_frame[2].id = 3;
   can_frame[2].id_format = IO_CAN_STD_FRAME;
   can_frame[2].length = 4;
   can_frame[2].data[0] = 9;
   can_frame[2].data[1] = 10;
   can_frame[2].data[2] = 11;
   can_frame[2].data[3] = 12;

   IO_CAN_WriteFIFO(handle_fifo, can_frame, 3);

   IO_CAN_WriteFIFO(handle_fifo, can_frame, 3);

   IO_CAN_WriteFIFO(handle_fifo, can_frame, 3);

   IO_CAN_WriteFIFO(handle_fifo, can_frame, 3);

   IO_CAN_WriteFIFO(handle_fifo, can_frame, 3);

   while(i);
}

