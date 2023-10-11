/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_EEPROM.h
 *
 * \brief IO Driver functions for EEPROM
 *
 *      The EEPROM driver functions allow writing to the SPI
 *      EEPROM as well as reading from it.
 *
 *      The EEPROM driver is a high level SPI device driver and
 *      uses the low level SPI driver.
 *
 *      The communication with the EEPROM is handled in a cyclic manner.
 *      EEPROM operations take plenty of time. A write as well as a
 *      read process is only triggered by calling the respective function.
 *      A status function is provided which returns whether the driver
 *      has finished the last task or not.
 *
 *  \section eeprom_examples EEPROM code examples
 *
 *      EEPROM initialization example: \n
 *      EEPROM does not need an explicit initialization.
 *      The EEPROM driver is initialized by IO_Driver_Init().
 *      \code
 *      IO_Driver_Init();
 *      \endcode
 *
 *      EEPROM write example:
 *      \code
 *      ubyte1 data[6] = {0, 1, 2, 3, 4, 5};
 *
 *      // check if EEPROM is busy
 *      if (IO_EEPROM_GetStatus() == IO_E_OK)
 *      {
 *          // if not busy write data
 *          IO_EEPROM_Write(0, 6, data);
 *      }
 *
 *      // write is complete when IO_EEPROM_GetStatus() returns IO_E_OK again.
 *      \endcode
 *
 *      EEPROM read example:
 *      \code
 *      ubyte1 data[2000] = {0};
 *
 *      // check if EEPROM is busy
 *      if (IO_EEPROM_GetStatus() == IO_E_OK)
 *      {
 *          // if not busy start reading
 *          IO_EEPROM_Read(0, 2000, data);
 *      }
 *
 *      // data is not yet available!!
 *      // data is available when IO_EEPROM_GetStatus() returns IO_E_OK again.
 *      \endcode
 *
 **************************************************************************/
#ifndef _IO_EEPROM_H
#define _IO_EEPROM_H

/**************************************************************************
 *
 * I N C L U D E S
 *
 **************************************************************************/

#include "IO_Driver.h"

/**************************************************************************
 *
 * D E F I N I T I O N S
 *
 **************************************************************************/

/**************************************************************************
 *
 * D A T A   S T R U C T U R E
 *
 **************************************************************************/

/**************************************************************************
 *
 * F U N C T I O N S
 *
 **************************************************************************/
/**********************************************************************//**
 *
 * \brief Initialization of the EEPROM driver.
 *
 *   Initialization of EEPROM driver.
 *      - Initializes internal data structure
 *      - Configures SPI driver
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_CHANNEL_BUSY        module has been initialized before
 *
 **************************************************************************
 *
 * \remarks
 *   Module is initialized only once.
 *   To re-initialize the module, the function IO_EEPROM_DeInit()
 *   needs to be called.
 *
 **************************************************************************/
 IO_ErrorType IO_EEPROM_Init(void);

/**********************************************************************//**
 *
 * \brief Deinitializes the EEPROM driver.
 *
 *   Deinitializes the module.
 *   Allows re-initialization by IO_EEPROM_Init()
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the module is not initialized
 *
 **************************************************************************/
 IO_ErrorType IO_EEPROM_DeInit(void);

/**********************************************************************//**
 *
 * \brief Read data from the EEPROM
 *
 *   The function only triggers a read operation.
 *   The read operation can take several cycles to be completed,
 *   depending on the SPI load and the amount of data to be read.
 *
 *   The read data is available on the address where the data parameter
 *   points to as soon as the read operation is finished.
 *   The state can be polled using the IO_EEPROM_GetStatus() function.
 *
 *
 * \param   offset      EEPROM memory offset (0..8191)
 * \param   length      Length of data to be read (0..8192)
 * \param   data        Pointer to data
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_BUSY                EEPROM module is still busy
 * \retval IO_E_EEPROM_RANGE        invalid address offset or range
 * \retval IO_E_NULL_POINTER        a null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the module is not initialized
 *
 **************************************************************************
 *
 * \remarks
 *   On hardware versions below V 2.0 a 2 kByte EEPROM is mounted.
 *   Do not execute read/write operations beyond address 2047 on these
 *   devices.
 *
 ***************************************************************************/
 IO_ErrorType IO_EEPROM_Read( ubyte2 offset
                            , ubyte2 length
                            , ubyte1 * const data);

/**********************************************************************//**
 *
 * \brief Write data to the EEPROM
 *
 *   The function triggers a write operation.
 *   The write operation can take several cycles to be completed,
 *   depending on the SPI load and the amount of data to be written.
 *
 *   The write operation is completed as soon as the SPI communication
 *   has been finished.
 *   The state can be polled using the IO_EEPROM_GetStatus() function.
 *
 *
 * \param   offset      EEPROM memory offset (0..8191)
 * \param   length      Length of data to be write (0..8192)
 * \param   data        Pointer to data
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_BUSY                EEPROM module is still busy
 * \retval IO_E_EEPROM_RANGE        invalid address offset or range
 * \retval IO_E_NULL_POINTER        a null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the module is not initialized
 *
 **************************************************************************
 *
 * \remarks
 *   On hardware versions below V 2.0 a 2 kByte EEPROM is mounted.
 *   Do not execute read/write operations beyond address 2047 on these
 *   devices.
 *
 ***************************************************************************/
 IO_ErrorType IO_EEPROM_Write( ubyte2 offset
                             , ubyte2 length
                             , const ubyte1 * const data);

/**********************************************************************//**
 *
 * \brief Returns the status of the EEPROM driver
 *
 *   Returns whether the EEPROM is idle or if a read or write operation
 *   is ongoing.
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK      everything fine, driver is idle
 * \retval IO_E_BUSY    a read or a write operation is ongoing, driver is busy.
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the module is not initialized
 *
 **************************************************************************/
 IO_ErrorType IO_EEPROM_GetStatus(void);

#endif /* _IO_EEPROM_H */
