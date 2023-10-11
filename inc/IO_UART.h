/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_UART.h
 *
 * \brief IO Driver functions for UART communication.
 *
 *      The Universal Serial  Interface Channel module (USIC) is a
 *      flexible interface module covering several serial
 *      communication protocols.
 *
 *      A USIC module contains two independent communication channels
 *      named UxC0 and UxC1, with x being the number of the USIC module.
 *
 *      The UART communication driver uses the USIC module as
 *      universal asynchronous receiver transmitter.
 *
 *  \section uart_buffers The UART buffers
 *
 *      The XC2000 has 16 byte transmit and receive buffers for
 *      every UART channel.
 *      The UART driver additionally implements a SW buffer of 128 bytes.
 *
 *      This means that a maximum of 128 bytes can be written, using the
 *      IO_UART_Write() function. The UART Task function sequentially copies
 *      the data to the HW buffer and sends it. The function IO_UART_GetTxStatus()
 *      returns the number of remaining bytes in the SW buffer. The write function
 *      can be called any time, as long as the SW buffer is not full.
 *
 *      The UART Task function is called automatically every SW cycle by IO_Driver_TaskEnd(). \n
 *      To speed up the transmission the application can call the IO_UART_Task() function
 *      at any time.
 *
 *      In every software cycle a maximum of 16 bytes (size of HW buffer)
 *      can be received via a UART interface. \n The data is copied to the
 *      SW buffer by the UART task function. To increase the number of bytes
 *      which can be received within a cycle the application can call the
 *      function IO_UART_Task() during the cycle. \n
 *      The function IO_UART_GetRxStatus()
 *      returns the number of bytes in the SW buffer which is available
 *      for reading.
 *
 *  \section uart_examples UART code examples
 *
 *      UART initialization example:
 *      \code
 *      // setup UART channel 1:
 *      IO_UART_Init( IO_UART_RS232
 *                  , 115200                // 115200 baud
 *                  , 8                     // 8 data bits
 *                  , IO_UART_PARITY_NONE   // no parity
 *                  , 1 );                  // 1 stop bit
 *      \endcode
 *
 *      UART read example:
 *      \code
 *      ubyte1 data[40] = {0};
 *      ubyte1 size;
 *
 *      // check if new bytes have been received
 *      IO_UART_GetRxStatus(IO_UART_RS232, &size);
 *      if (size > 0)
 *      {
 *          // read a maximum of 40 bytes
 *          IO_UART_Read(IO_UART_RS232, data, 40, &size);
 *
 *          // data now holds the received data
 *          // size holds the number of actually read bytes.
 *      }
 *      \endcode
 *
 *      UART write example:
 *      \code
 *      ubyte1 data[5] = {0, 1, 2, 3, 4};
 *      ubyte1 size;
 *
 *      // write data to UART buffer:
 *      IO_UART_Write(IO_UART_RS232, data, 5, &size);
 *      // size holds the number of actually written bytes.
 *
 *      // check if data has been transmitted
 *      IO_UART_GetTxStatus(IO_UART_RS232, &size);
 *      // when size returns 0 all the data has been transmitted
 *      \endcode
 *
 **************************************************************************/
#ifndef _IO_UART_H
#define _IO_UART_H

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

/**
 * \name UART channels
 *
 * Defines for the available UART channels
 */
/*@{*/
#define IO_UART_CH0     IO_INT_PIN_UART_CH0   /**< UART Channel 0 */
#define IO_UART_CH1     IO_INT_PIN_UART_CH1   /**< UART Channel 1 */
#define IO_UART_CH2     IO_INT_PIN_UART_CH2   /**< UART Channel 2 */

/* aliases */
#define IO_UART_RS232   IO_UART_CH0   /**< alias for UART Channel 0 */
#define IO_UART_LIN     IO_UART_CH1   /**< alias for UART Channel 1 */
#define IO_UART_MiniMod IO_UART_CH2   /**< alias for UART Channel 2 */
/*@}*/

/**
 * \name Parity configuration
 *
 * Defines for the UART parity configuration
 */
/*@{*/
#define IO_UART_PARITY_NONE     0x0 /**< no parity   */
#define IO_UART_PARITY_EVEN     0x2 /**< even parity */
#define IO_UART_PARITY_ODD      0x3 /**< odd parity  */
/*@}*/

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
 * \brief Initialization of the UART communication driver.
 *
 *   Initialization of UART Serial Communication Driver
 *      - Enables module
 *      - Configures module for ASC
 *      - Initializes SW queue
 *
 * \param   channel         UART Channel, one of:
 *                              - \c IO_UART_CH0, \c IO_UART_RS232
 *                              - \c IO_UART_CH1, \c IO_UART_LIN
 *                              - \c IO_UART_CH2, \c IO_UART_MiniMod
 * \param   baudrate        Baud rate in baud/s:
 *                              - 1200 - 115200 baud/s (\c IO_UART_CH0, \c IO_UART_RS232)
 *                              - 1200 - 20000 baud/s (\c IO_UART_CH1, \c IO_UART_LIN)
 *                              - 1200 - 115200 baud/s (\c IO_UART_CH2, \c IO_UART_MiniMod)
 * \param   dbits           Number of data bits per frame (1 ... 8)
 * \param   par             Parity configuration, one of:
 *                              - \c IO_UART_PARITY_NONE
 *                              - \c IO_UART_PARITY_EVEN
 *                              - \c IO_UART_PAEITY_ODD
 * \param   sbits           Number of stop bits per frame (1 .. 2)
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_INVALID_CHANNEL_ID  channel number is invalid
 * \retval IO_E_INVALID_PARAMETER   a given parameter is out of range
 * \retval IO_E_CHANNEL_BUSY        the channel is already initialized
 *
 **************************************************************************
 *
 * \remarks
 *   Module is initialized only once.
 *   To re-initialize the module, the function IO_UART_DeInit()
 *   needs to be called.
 *
 * \remarks
 *   When LIN is selected (i.e. channel = IO_UART_LIN), the parameters
 *   dbits, par and sbits will be ignored. LIN always uses 1 stop bit
 *   no parity and 8 data bits (except for the break frame).
 *
 **************************************************************************/
 IO_ErrorType IO_UART_Init( ubyte1 channel
                          , ubyte4 baudrate
                          , ubyte1 dbits
                          , ubyte1 par
                          , ubyte1 sbits);

/**********************************************************************//**
 *
 * \brief Deinitialization of the UART channel
 *
 *   Allows re-initialization by IO_UART_Init()
 *
 * \param   channel         UART Channel, one of:
 *                              - \c IO_UART_CH0, \c IO_UART_RS232
 *                              - \c IO_UART_CH1, \c IO_UART_LIN
 *                              - \c IO_UART_CH2, \c IO_UART_MiniMod
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_INVALID_CHANNEL_ID  channel number is invalid
 * \retval IO_E_CHANNEL_NOT_CONFIGURED channel has not been initialized
 *
 **************************************************************************/
 IO_ErrorType IO_UART_DeInit(ubyte1 channel);

/**********************************************************************//**
 *
 * \brief Task function for UART communication
 *
 *   The task function shall be called every SW cycle.
 *   It copies the data to be sent from the SW buffer to the HW buffer
 *   and the received data from the HW buffer to the SW buffer.
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_UART_BUFFER_FULL    SW receive queue is full and data
 *                                  has been lost
 * \retval IO_E_UART_OVERFLOW       HW receive buffer overrun
 * \retval IO_E_UART_PARITY         parity check failed
 *
 *
 **************************************************************************/
 IO_ErrorType IO_UART_Task(void);

/**********************************************************************//**
 *
 * \brief Read data from serial interface
 *
 *   Reads the data from the SW buffer.
 *
 * \param   channel         UART Channel, one of:
 *                              - \c IO_UART_CH0, \c IO_UART_RS232
 *                              - \c IO_UART_CH1, \c IO_UART_LIN
 *                              - \c IO_UART_CH2, \c IO_UART_MiniMod
 * \param   data            Data array
 * \param   len             Maximum size of data array
 * \param   rx_len          Actually read bytes
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_UART_BUFFER_FULL    SW receive queue is full and data
 *                                  has been lost
 * \retval IO_E_UART_OVERFLOW       HW receive buffer overrun
 * \retval IO_E_UART_PARITY         parity check failed
 * \retval IO_E_INVALID_CHANNEL_ID  invalid channel ID has been passed
 * \retval IO_E_NULL_POINTER        null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED channel has not been initialized
 *
 **************************************************************************
 *
 * \remarks
 *   The received serial data is not processed even when LIN is selected
 *   (i.e. channel = IO_UART_LIN), the data is copied raw into the data array.
 **************************************************************************/
 IO_ErrorType IO_UART_Read( ubyte1 channel
                          , ubyte1 * const data
                          , ubyte1 len
                          , ubyte1 * const rx_len);

/**********************************************************************//**
 *
 * \brief Write data to serial interface
 *
 *   Writes the data to the SW buffer and starts the transmission.
 *
 * \param   channel         UART Channel, one of:
 *                              - \c IO_UART_CH0, \c IO_UART_RS232
 *                              - \c IO_UART_CH1, \c IO_UART_LIN
 *                              - \c IO_UART_CH2, \c IO_UART_MiniMod
 * \param   data            Data array
 * \param   len             Number of bytes in data array
 * \param   tx_len          Actually written bytes
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_UART_BUFFER_FULL    SW transmit queue is full, no data
 *                                    has been written
 * \retval IO_E_INVALID_CHANNEL_ID  invalid channel ID has been passed
 * \retval IO_E_NULL_POINTER        null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED channel has not been initialized
 *
 **************************************************************************
 *
 * \remarks
 *   When LIN is selected (i.e. channel = IO_UART_LIN), the driver will
 *   add a break and a sync symbol to every data frame.
 *   i.e. for every call of IO_UART_Write the driver will first transmit
 *   a break and a sync symbol, before transmitting the data passed to the
 *   function.\n
 *   The bytes in the data array include the number of information data bytes
 *   to be transmitted (D0...Dx), the ID field and the CRC field.\n
 *   The ID field needs to be sent as the first byte, the CRC needs to be
 *   calculated by the application and sent as the last byte.\n
 * \n
 * \remarks
 *   | Break | Sync |  ID  |  D0  |  D1  |  ....  | Dx-1 |  Dx  |  CRC  |
 * \remarks
 * \n
 *   For additional information about the ID and CRC fields please refer to
 *   the LIN specification.
 **************************************************************************/
 IO_ErrorType IO_UART_Write( ubyte1 channel
                           , const ubyte1 * const data
                           , ubyte1 len
                           , ubyte1 * const tx_len);

/**********************************************************************//**
 *
 * \brief Retrieve the status of the receive buffer
 *
 *   Returns the current status of the receive buffer: new data available, error has occured...
 *
 * \param   channel         UART Channel, one of:
 *                              - \c IO_UART_CH0, \c IO_UART_RS232
 *                              - \c IO_UART_CH1, \c IO_UART_LIN
 *                              - \c IO_UART_CH2, \c IO_UART_MiniMod
 * \param   rx_len          Number of received data frames in receive buffer
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_UART_BUFFER_FULL    SW receive queue is full and data
 *                                  has been lost
 * \retval IO_E_UART_OVERFLOW       HW receive buffer overrun
 * \retval IO_E_UART_PARITY         parity check failed
 * \retval IO_E_INVALID_CHANNEL_ID  invalid channel ID has been passed
 * \retval IO_E_NULL_POINTER        null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED channel has not been initialized
 *
 **************************************************************************/
 IO_ErrorType IO_UART_GetRxStatus( ubyte1 channel
                                 , ubyte1 * const rx_len );

/**********************************************************************//**
 *
 * \brief Retrieve the status of the transmit buffer
 *
 *   Returns the number of remaining bytes in the SW buffer.
 *
 * \param   channel         UART Channel, one of:
 *                              - \c IO_UART_CH0, \c IO_UART_RS232
 *                              - \c IO_UART_CH1, \c IO_UART_LIN
 *                              - \c IO_UART_CH2, \c IO_UART_MiniMod
 * \param   tx_len          Number of data frames in the transmit buffer
 *                          which have not yet been transmitted
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_INVALID_CHANNEL_ID  invalid channel ID has been passed
 * \retval IO_E_NULL_POINTER        null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED channel has not been initialized
 *
 **************************************************************************/
 IO_ErrorType IO_UART_GetTxStatus( ubyte1 channel
                                 , ubyte1 * const tx_len );


#endif /* _IO_UART_H */
