/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
 /**********************************************************************//**
 * \file APDB.h
 *
 * \brief APDB define for boot loader
 *
 *      Contains the definition for the application database.
 *      This database is used by the boot loader.
 *
 *      The boot loader needs this information to determine
 *      where the application actually starts. For this reason the field
 *      "mainAddress" must be provided by the application.
 *
 *  \section apdb_example Code example
 *
 *      Example for APDB definition in an application:
 *      \code
 *      APDB appl_db =
 *          { 0                      // ubyte4 versionAPDB
 *          , {0}                    // BL_T_DATE flashDate
 *                                   // BL_T_DATE buildDate
 *          , { (ubyte4) (((((ubyte4) RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
 *                        ((((ubyte4) RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
 *                        ((((ubyte4) RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
 *                        ((((ubyte4) RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
 *                        ((((ubyte4) RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
 *          , 0                      // ubyte4 nodeType
 *          , 0                      // ubyte4 startAddress
 *          , 0                      // ubyte4 codeSize
 *          , 0                      // ubyte4 legacyAppCRC
 *          , 0                      // ubyte4 appCRC
 *          , 1                      // ubyte1 nodeNr
 *          , 0                      // ubyte4 CRCInit
 *          , 0                      // ubyte4 flags
 *          , 0                      // ubyte4 hook1
 *          , 0                      // ubyte4 hook2
 *          , 0                      // ubyte4 hook3
 *          , APPL_START             // ubyte4 mainAddress
 *          , {0, 1}                 // BL_T_CAN_ID canDownloadID
 *          , {0, 2}                 // BL_T_CAN_ID canUploadID
 *          , 0                      // ubyte4 legacyHeaderCRC
 *          , 0                      // ubyte4 version
 *          , 500                    // ubyte2 canBaudrate
 *          , 0                      // ubyte1 canChannel
 *          , 0                      // ubyte4 password
 *          , 0                      // ubyte4 magicSeed
 *          , {0}                    // ubyte1 reserved[6*4]
 *          , 0                      // ubyte4 headerCRC
 *          };
 *      \endcode
 *
 **************************************************************************/
#ifndef _APDB_H
#define _APDB_H

#include "IO_Driver.h"

/* prototype for cstart */
extern void _cstart( void );

/**************************************************************************
 *
 * D E F I N I T I O N S
 *
 **************************************************************************/

#define APPL_START      ((ubyte4) &_cstart)

#define APDB_ADDRESS    0xC10000

#define APDB volatile __huge const BL_APDB __at(APDB_ADDRESS)


/**************************************************************************
 *
 * D A T A   S T R U C T U R E
 *
 **************************************************************************/

/**
 *
 * \brief Date structure
 *
 *  Data structure for saving dates like flash or build date.
 *
 */
typedef struct _bl_t_date
{
    ubyte4 date;          /**< date.  */
}BL_T_DATE;


/**
 *
 * \brief CAN ID structure
 *
 *
 */
typedef struct _bl_t_can_id
{
    ubyte4 extended;        /**< if bit is set to 1, extended CAN ID is used    */
    ubyte4 ID;              /**< right-aligned CAN-ID, LSB must start at bit 0 */
}BL_T_CAN_ID;


/**
 *
 * \brief APDB structure
 *
 *  Data structure for accessing the Application Descriptor Block.
 *
 */
typedef struct _bl_apdb
{
    ubyte4 versionAPDB;         /**< [8bit.8bit] version of the APDB           */
    BL_T_DATE flashDate;        /**< date when the application has
                                     been flashed.                             */
    BL_T_DATE buildDate;        /**< date when the application has
                                     been built (only if used by the customer) */
    ubyte4 nodeType;            /**< Hardware type (\ttc50, \ttc60, ..)
                                     To match software to hardware in PC-Tool  */
    ubyte4 startAddress;        /**< start address: address where the code
                                     in the memory starts                      */
    ubyte4 codeSize;            /**< size of the application in bytes          */
    ubyte4 legacyAppCRC;        /**< legacy application crc for flash checker  */
    ubyte4 appCRC;              /**< CRC-32 of the application                 */
    ubyte4 nodeNr;              /**< node number                               */
    ubyte4 CRCInit;             /**< init value for certain CRC checks         */
    ubyte4 flags;               /**< various flags                             */
    ubyte4 hook1;               /**< custom hook 1                             */
    ubyte4 hook2;               /**< custom hook 2                             */
    ubyte4 hook3;               /**< custom hook 3                             */
    ubyte4 mainAddress;         /**< vector address of the application, main() */
    BL_T_CAN_ID canDownloadID;  /**< CAN ID for download                       */
    BL_T_CAN_ID canUploadID;    /**< CAN ID for upload                         */
    ubyte4 legacyHeaderCRC;     /**< legacy header crc for flash checker       */
    ubyte4 version;             /**< [8bit.8bit.16bit] version of the application
                                     (only if used by the customer)            */
    ubyte4 canBaudrate;         /**< baud rate for CAN communication [kbit/s]  */
    ubyte4 canChannel;          /**< channel for CAN communication             */
    ubyte4 password;            /**< hash-value of password for memory access  */
    ubyte4 magicSeed;           /**< Initialise seed value for CRC calculation
                                     with the MCHK HW module                   */
    ubyte1 reserved[6*4];       /**< reserved for future use                   */
    ubyte4 headerCRC;           /**< CRC-32 of the header section
                                     (including APPCRC)                        */
}BL_APDB;


#endif /* _APDB_H */
