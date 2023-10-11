/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_CAN.h
 *
 * \brief IO Driver functions for CAN communication.
 *
 *      The CAN driver uses the MultiCAN module of the XC2000 CPU.
 *
 *      The CAN driver supports up to 4 CAN interfaces, so called <em>nodes</em>.
 *      The number of available nodes depends on the HW variant.
 *
 * \if TTC94E
 * \verbatim[table]
 * ================== ===================================
 *   HW variant             No of CAN interfaces
 * ------------------ -----------------------------------
 *   \ttc50                 2
 * ------------------ -----------------------------------
 *   \ttc60                 2
 * ------------------ -----------------------------------
 *   \ttc90                 2
 * ------------------ -----------------------------------
 *   \ttc94                 4
 * ------------------ -----------------------------------
 *   \ttc94e                4
 * ------------------ -----------------------------------
 * \endverbatim
 * \else
 * \verbatim[table]
 * ================== ===================================
 *   HW variant             No of CAN interfaces
 * ------------------ -----------------------------------
 *   \ttc50                 2
 * ------------------ -----------------------------------
 *   \ttc60                 2
 * ------------------ -----------------------------------
 *   \ttc90                 2
 * ------------------ -----------------------------------
 *   \ttc94                 4
 * ------------------ -----------------------------------
 * \endverbatim
 * \endif
 *
 *      All CAN nodes share a common set of message objects.
 *      For further details on message objects
 *      see \ref can_handles "CAN handles and message objects".
 *
 *      The functions 'remote acknowledge' and 'remote request' are not
 *      supported.
 *
 *  \section can_ac_mask_usage Usage of the acceptance masks:
 *
 *      The acceptance mask defines the relevant bits of the CAN ID.
 *      A binary 1 marks a relevant bit in the CAN ID on the same position.
 *
 *      Setting all bits of the acceptance mask (0x1FFFFFFF) only accepts
 *      the ID set with the ID parameter and rejects all other IDs.
 *      Setting the acceptance mask to 0 causes the message buffer to
 *      accept any IDs.
 *
 *      Using this mechanism a message buffer can be used to accept a
 *      range of CAN IDs.
 *
 *      Example:
 *      \code
 *          ac_mask = 0x1FFFFF00 = 0 b 1 1111 1111 1111 1111 1111 0000 0000
 *          id      = 0x00000200 = 0 b 0 0000 0000 0000 0000 0010 0000 0000
 *      \endcode
 *      in this example all messages with an id between 0x200 and 0x2FF
 *      are accepted.
 *
 *  \section can_handles CAN handles and message objects
 *
 *      The MultiCAN module of the XC2000 CPU provides a certain number
 *      of so called <em>CAN message objects</em>. A message object is a dedicated
 *      memory area which is used to store a CAN message.
 *      Every message object has its own identifier and acceptance mask.
 *      The number of available message objects depends on the HW variant as follows:
 *
 * \verbatim[table]
 * ================== ===================================
 *   HW variant             message objects
 * ------------------ -----------------------------------
 *   \ttc50                 128
 * ------------------ -----------------------------------
 *   \ttc60                 128
 * ------------------ -----------------------------------
 *   \ttc90                 128
 * ------------------ -----------------------------------
 *   \ttc94                 256
 * ------------------ -----------------------------------
 * \if TTC94E
 *   \ttc94e                256
 * ------------------ -----------------------------------
 * \endif
 *
 * \endverbatim
 *
 *      The available message objects are shared between the available
 *      CAN interfaces and they may be individually allocated to one of those
 *      interfaces as required by the application.
 *
 *      For receiving and transmitting CAN messages a message object
 *      needs to be configured.
 *      During the configuration the driver will assign a message object to the
 *      respective CAN interface and will return a so called <em>CAN handle</em>.
 *
 *      The <em>CAN handle</em> is a reference to the configured message object
 *      and is used used to exchange data with the driver.
 *      There are two types of CAN handles:
 *       - Standard message handles
 *       - FIFO buffer handles
 *
 *      The number of CAN handles is internally limited to a <b>maximum of 128 Handles (for the \ttc94e to a maximum of 256 Handles)</b>.
 *
 *      \subsection can_handles_std Standard message handles
 *        A standard message handle is configured by calling the driver function
 *        IO_CAN_ConfigMsg(). It references to a single message object which
 *        can store <em>one</em> CAN message.
 *
 *        For receiving/transmitting data via a single message object the driver
 *        functions IO_CAN_ReadMsg() and IO_CAN_WriteMsg() are provided.
 *
 *        data overflow handling:
 *        - when \em transmitting: If the previously provided CAN message has not been
 *                  transmitted completely, the driver will not accept a new message
 *                  and will return #IO_E_BUSY which means that the CAN message
 *                  has not been transmitted.
 *                  The application should send the data again as soon as the transmission
 *                  is finished.
 *        - when \em receiving: If a CAN message is received when the previously received
 *                  message has not yet been retrieved by the application, the old message
 *                  will be overwritten.
 *                  The driver function will return #IO_E_CAN_OVERFLOW indicating that
 *                  at least one message has been lost.
 *                  If no CAN message is received between two consecutive read attempts
 *                  of the application, the driver will return #IO_E_CAN_OLD_DATA
 *                  indicating that the retrieved data is old. 
 *      \warning
 *      If two ECUs are connected via CAN(one ECU sends CAN messages the other one receives them) there is
 *      a possibility that at some point CAN messages will get lost using the function IO_CAN_ReadMsg(). 
 *      The reason for that behavior can be described as follows. The cycle times of both ECUs are not 
 *      exactly the same due to production tolerances of the components. This leads to a very slow drift 
 *      between the software cycles of the ECUs. There is a critical range of the phase shift between the 
 *      software cycles of the ECUs in which the CAN messages are received while the software reads them from 
 *      the hardware buffer. This can last for several seconds if the drift is very slow. During that time CAN 
 *      messages will get lost. To avoid this behavior it is highly recommended to use the function IO_CAN_ReadFIFO()
 *      with an appropriate buffer length instead of the function IO_CAN_ReadMsg() to receive the corresponding
 *      CAN message.
 *
 *        The status of a standard message handle can be checked with the function
 *        IO_CAN_MsgStatus().
 *
 *      \subsection can_handles_fifo FIFO buffer handles
 *
 *        The driver supports FIFO buffers. A FIFO buffer can be created by
 *        linking several message objects to one FIFO buffer which is able
 *        to store multiple CAN message.
 *
 *        A FIFO buffer handle is configured by calling the function
 *        IO_CAN_ConfigFIFO(). The desired size of the buffer is passed to the
 *        function via a parameter.
 *
 *        For receiving/transmitting data via FIFO buffers the driver provides
 *        the functions IO_CAN_ReadFIFO() and IO_CAN_WriteFIFO(). With this functions
 *        multiple CAN messages can be retrieved/sent with one function call.
 *
 *        data overflow handling:
 *        - when \em transmitting: If the number of free message objects in a FIFO
 *                  buffer is lower than the number of messages the application wants
 *                  to transmit, the CAN driver will not accept any data and returns
 *                  #IO_E_CAN_FIFO_FULL.
 *                  The application should send the data again as soon as there is
 *                  enough free space in the FIFO buffer.
 *        - when \em receiving: If a CAN message is received and the FIFO buffer is full,
 *                  the oldest message in the FIFO queue will be overwritten.
 *                  The driver function will return #IO_E_CAN_FIFO_FULL which indicates
 *                  that at least one message has been lost.
 *                  If no CAN message is received between two consecutive read attempts
 *                  of the application, the driver will return #IO_E_CAN_OLD_DATA
 *                  but without copying any data to the application buffers.
 
 *  \section can_example CAN code examples*
 *        The status of a FIFO buffer handle can be checked with the function
 *        IO_CAN_FIFOStatus().
 *
 *
 *      Examples for CAN initialization:
 *      \code
 *      ubyte1 handle_w, handle_r, handle_fifo_w, handle_fifo_r;
 *
 *      IO_CAN_Init( IO_CAN_CHANNEL_0
 *                 , 500
 *                 , 0     //default
 *                 , 0     //default
 *                 , 0);   //default
 *
 *      // standard message buffers //
 *
 *      IO_CAN_ConfigMsg( &handle_w
 *                      , IO_CAN_CHANNEL_0  // channel 0
 *                      , IO_CAN_MSG_WRITE  // transmit message buffer
 *                      , IO_CAN_STD_FRAME  // standard ID
 *                      , 0
 *                      , 0);
 *
 *      IO_CAN_ConfigMsg( &handle_r
 *                      , IO_CAN_CHANNEL_0  // channel 0
 *                      , IO_CAN_MSG_READ   // receive message buffer
 *                      , IO_CAN_STD_FRAME  // standard ID
 *                      , 1
 *                      , 0x1FFFFFFF);      // accept only id 1
 *
 *      // FIFO message buffer //
 *
 *      IO_CAN_ConfigFIFO( &handle_fifo_w
 *                       , IO_CAN_CHANNEL_0 // channel 0
 *                       , 20               // 20 items
 *                       , IO_CAN_MSG_WRITE // transmit fifo buffer
 *                       , IO_CAN_STD_FRAME // standard ID
 *                       , 0
 *                       , 0 );
 *
 *      IO_CAN_ConfigFIFO( &handle_fifo_r
 *                       , IO_CAN_CHANNEL_0 // channel 0
 *                       , 20               // 20 items
 *                       , IO_CAN_MSG_READ  // receive fifo buffer
 *                       , IO_CAN_STD_FRAME // standard ID
 *                       , 0
 *                       , 0 );             // accept every id
 *
 *      \endcode
 *
 *      Examples for CAN task function calls:
 *      \code
 *      IO_CAN_DATA_FRAME can_frame;
 *
 *      // check if new message has been received
 *      if (IO_CAN_MsgStatus(handle_r) == IO_E_OK)
 *      {
 *          // if message has been received, read the message from the buffer
 *          IO_CAN_ReadMsg(handle_r, &can_frame);
 *
 *          // received message is now stored in can_frame
 *          // and can be used by the application
 *      }
 *
 *
 *      // assemble CAN frame:
 *      can_frame.id = 1;
 *      can_frame.id_format = IO_CAN_STD_FRAME;
 *      can_frame.length = 4;
 *      can_frame.data[0] = 1;
 *      can_frame.data[1] = 2;
 *      can_frame.data[2] = 3;
 *      can_frame.data[3] = 4;
 *
 *      // transmit message
 *      IO_CAN_WriteMsg(handle_w, &can_frame);
 *
 *      // wait until the transmission has been finished:
 *      while (IO_CAN_MsgStatus(handle_w) != IO_E_OK);
 *
 *      \endcode
 *
 **************************************************************************/
#ifndef _IO_CAN_H
#define _IO_CAN_H

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
 * \name CAN channels
 * \anchor can_channels_def
 *
 * Selects the CAN communication channel.
 */
/*@{*/
#define IO_CAN_CHANNEL_0    IO_INT_PIN_CAN_CH0   /**< CAN Channel 0 */
#define IO_CAN_CHANNEL_1    IO_INT_PIN_CAN_CH1   /**< CAN Channel 1 */
#if (defined(TTC94) || defined(TTC94E) || defined(TTC94R))
    #define IO_CAN_CHANNEL_2    IO_INT_PIN_CAN_CH2   /**< CAN Channel 2 (only for \ttc94) */
    #define IO_CAN_CHANNEL_3    IO_INT_PIN_CAN_CH3   /**< CAN Channel 3 (only for \ttc94) */
#endif /* TTC94 || TTC94E || TTC94R */
/*@}*/

/**
 * \name Message buffer direction
 * \anchor can_buffer_direction
 *
 * Selects the transmission direction of a CAN message buffer
 */
/*@{*/
#define IO_CAN_MSG_READ     0   /**< used to setup a message buffer for receiving    */
#define IO_CAN_MSG_WRITE    1   /**< used to setup a message buffer for transmitting */
/*@}*/

/**
 * \name CAN frame format
 * \anchor can_buffer_direction
 *
 * Selects the format for a CAN frame
 */
/*@{*/
#define IO_CAN_STD_FRAME    0   /**< the id parameter holds a standard (11-bit) ID  */
#define IO_CAN_EXT_FRAME    1   /**< the id parameter holds an extended (29-bit) ID */
/*@}*/

/**************************************************************************
 *
 * D A T A   S T R U C T U R E
 *
 **************************************************************************/

/**
 *
 * \brief CAN data frame
 *
 *  Stores a data frame for the CAN communication.
 *
 */
typedef struct _io_can_data_frame
{
    ubyte1 data[8];      /**< data buffer                          */
    ubyte1 length;       /**< number of words in transmit buffer   */
    ubyte1 id_format;    /**< standard or extended format          */
    ubyte4 id;           /**< ID for CAN communication             */
} IO_CAN_DATA_FRAME;


/**************************************************************************
 *
 * F U N C T I O N S
 *
 **************************************************************************/


/**********************************************************************//**
 *
 * \brief Initialization of the CAN communication driver.
 *
 *   The function
 *      - Enables the module
 *      - Sets the module clock to 45MHz
 *      - Sets up the bit timing for the given channel
 *
 *
 * \param   channel         CAN channel, one of:
 *                               - \c IO_CAN_CHANNEL_0
 *                               - \c IO_CAN_CHANNEL_1
 *                               - \c IO_CAN_CHANNEL_2
 *                               - \c IO_CAN_CHANNEL_3
 * \param   baudrate        Baud rate in kbit/s (125 ... 1000)
 * \param   tseg1           Time segment before sample point (3 ... 16)
 * \param   tseg2           Time segment after sample point (2 ... 8)
 * \param   sjw             Synchronization jump width (1 ... 4)
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_INVALID_PARAMETER   invalid parameter has been passed
 * \retval IO_E_INVALID_CHANNEL_ID  invalid channel ID has been passed
 * \retval IO_E_CHANNEL_BUSY        channel has been initialized before
 *
 **************************************************************************
 *
 * \remarks
 *   Module is initialized only once.
 *   To re-initialize the module, the function IO_CAN_DeInit()
 *   needs to be called.
 *
 * \remarks
 *   Set the parameters tseg1, tseg2 and sjw to zero (0)
 *   to set the parameters to default values.
 *
 * \remarks
 * \if TTC94E
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available only
 *   for \ttc94 and \ttc94e
 * \else
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available only
 *   for \ttc94 
 * \endif
 **************************************************************************/
 IO_ErrorType IO_CAN_Init( ubyte1 channel
                         , ubyte2 baudrate
                         , ubyte1 tseg1
                         , ubyte1 tseg2
                         , ubyte1 sjw );

/**********************************************************************//**
 *
 * \brief Deinitializes a single message handle
 *
 *   Allows re-initialization by \c IO_CAN_Config...()
 *
 *
 * \param   handle      CAN message handle
 *                          (retrieved from \c IO_CAN_ConfigMsg
 *                           or \c IO_CAN_ConfigFIFO)
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_PARAMETER       invalid parameter has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  handle has not been initialized
 *
 **************************************************************************/
 IO_ErrorType IO_CAN_DeInitHandle(ubyte1 handle);

/**********************************************************************//**
 *
 * \brief Deinitializes the given CAN channel
 *
 *   Allows re-initialization by \c IO_CAN_Init()
 *
 *
 * \param   channel         CAN channel, one of:
 *                               - \c IO_CAN_CHANNEL_0
 *                               - \c IO_CAN_CHANNEL_1
 *                               - \c IO_CAN_CHANNEL_2
 *                               - \c IO_CAN_CHANNEL_3
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                     everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      invalid parameter has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED channel has not been initialized
 *
 **************************************************************************
 *
 * \remarks
 * \if TTC94E
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available
 *   only for \ttc94 and \ttc94e
 * \else
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available
 *   only for \ttc94
 * \endif
 **************************************************************************/
 IO_ErrorType IO_CAN_DeInit(ubyte1 channel);

/**************************************************************************
 * NORMAL CAN MESSAGES
 **************************************************************************/

/**********************************************************************//**
 *
 * \brief Configures a message object.
 *
 *   Configures a message object for the given CAN channel and
 *   returns a message object handle.
 *
 *
 * \param   handle         Returns the message object handle
 * \param   channel         CAN channel, one of:
 *                               - \c IO_CAN_CHANNEL_0
 *                               - \c IO_CAN_CHANNEL_1
 *                               - \c IO_CAN_CHANNEL_2
 *                               - \c IO_CAN_CHANNEL_3
 * \param   mode           Mode for CAN Message, one of:
 *                          - \c IO_CAN_MSG_READ,
 *                          - \c IO_CAN_MSG_WRITE
 * \param   id_format      Format of message identifier, one of:
 *                          - \c IO_CAN_STD_FRAME,
 *                          - \c IO_CAN_EXT_FRAME
 * \param   id             CAN message identifier
 * \param   ac_mask        CAN acceptance mask, refer to \ref can_ac_mask_usage
 *                              "Usage of the acceptance mask" for further details.
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_CAN_MAX_MO_REACHED      no more HW message objects are available
 * \retval IO_E_CAN_MAX_HANDLES_REACHED no more handles are available
 * \retval IO_E_NULL_POINTER            null Pointer has been passed
 * \retval IO_E_INVALID_CHANNEL_ID      invalid Channel ID has been passed
 * \retval IO_E_INVALID_PARAMETER       invalid Parameter has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel was not initialized
 *
 **************************************************************************
 *
 * \remarks
 * \if TTC94E
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available only
 *   for \ttc94 and \ttc94e
 * \else
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available only
 *   for \ttc94 
 * \endif
 **************************************************************************/
 IO_ErrorType IO_CAN_ConfigMsg( ubyte1 * const handle
                              , ubyte1 channel
                              , ubyte1 mode
                              , ubyte1 id_format
                              , ubyte4 id
                              , ubyte4 ac_mask );

/**********************************************************************//**
 *
 * \brief Returns the data of a message object
 *
 *   Reads a message from a given message object.
 *   Returns whether the message is new or not.
 *
 *
 * \param   handle      CAN message object handle
 *                          (retrieved from IO_CAN_ConfigMsg())
 * \param   buffer      Pointer to data buffer structure.
 *                          The received frame will be stored there.
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                     everything fine
 * \retval IO_E_CAN_OVERFLOW           overflow of message object
 * \retval IO_E_CAN_OLD_DATA           no new data has been received since the last read
 * \retval IO_E_CAN_WRONG_HANDLE       invalid handle has been passed
 * \retval IO_E_NULL_POINTER           null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the given handle was not configured
 * \retval IO_E_CAN_INVALID_DATA       the received data is invalid - read again to get the valid data
 *
 **************************************************************************/
 IO_ErrorType IO_CAN_ReadMsg( ubyte1 handle
                            , IO_CAN_DATA_FRAME * const buffer );

/**********************************************************************//**
 *
 * \brief Transmits a CAN Message
 *
 *   Transmits a CAN message, using the given channel and message object.
 *   Returns whether the transmission has been started successfully or not.
 *
 *
 * \param   handle         CAN message object handle
 *                              (retrieved from IO_CAN_ConfigMsg())
 * \param   data           Pointer to data structure.
 *                              The data in this structure will be transmitted.
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                       everything fine
 * \retval IO_E_BUSY                     message object busy - no data has been transmitted
 * \retval IO_E_CAN_WRONG_HANDLE         invalid handle has been passed
 * \retval IO_E_NULL_POINTER             null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED   the given handle has not been configured
 *
 **************************************************************************/
 IO_ErrorType IO_CAN_WriteMsg( ubyte1 handle
                             , const IO_CAN_DATA_FRAME * const data );


/**************************************************************************
 * FIFO BUFFERED CAN MESSAGES
 *************************************************************************/

/**********************************************************************//**
 *
 * \brief Configures a FIFO buffer.
 *
 *   Configures a FIFO buffer for the given CAN channel.
 *
 *
 * \param   handle         Returns the FIFO buffer handle
 * \param   channel         CAN channel, one of:
 *                               - \c IO_CAN_CHANNEL_0
 *                               - \c IO_CAN_CHANNEL_1
 *                               - \c IO_CAN_CHANNEL_2
 *                               - \c IO_CAN_CHANNEL_3
 * \param   size           Size of FIFO buffer (number of frames).
 * \param   mode           Mode for CAN Message, one of:
 *                              - \c IO_CAN_MSG_READ
 *                              - \c IO_CAN_MSG_WRITE
 * \param   id_format      Format of message identifier, one of:
 *                              - \c IO_CAN_STD_FRAME
 *                              - \c IO_CAN_EXT_FRAME
 * \param   id             CAN message identifier
 * \param   ac_mask        CAN acceptance mask, refer to \ref can_ac_mask_usage
 *                              "Usage of the acceptance mask" for further details.
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_CAN_MAX_MO_REACHED      no more HW messgae objects are available
 * \retval IO_E_CAN_MAX_HANDLES_REACHED no more free handles are available
 * \retval IO_E_NULL_POINTER            null pointer has been passed
 * \retval IO_E_INVALID_CHANNEL_ID      invalid channel number has been passed
 * \retval IO_E_INVALID_PARAMETER       a parameter is out of range
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  channel has not been initialized
 *
 **************************************************************************
 *
 * \remarks
 * \if TTC94E
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available only
 *   for \ttc94 and \ttc94e
 * \else
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available only
 *   for \ttc94
 * \endif
 **************************************************************************/
 IO_ErrorType IO_CAN_ConfigFIFO( ubyte1 * const handle
                               , ubyte1 channel
                               , ubyte1 size
                               , ubyte1 mode
                               , ubyte1 id_format
                               , ubyte4 id
                               , ubyte4 ac_mask );

/**********************************************************************//**
 *
 * \brief Reads the data from a FIFO buffer.
 *
 *   Copies received CAN frames from a given FIFO buffer to a SW frame buffer.
 *
 *
 * \param   handle         CAN FIFO buffer handle
 *                    (retrieved from IO_CAN_ConfigFIFO())
 * \param   buffer         Pointer to data buffer structure.
 *                    The received frame will be stored there.
 * \param   buffer_size    Size of "buffer", maximum number of frames to be
 *                    written to the buffer.
 * \param   rx_frames      Number of frames which have been copied to "buffer"
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                everything fine
 * \retval IO_E_NULL_POINTER      null pointer has been passed to function
 * \retval IO_E_CAN_FIFO_FULL     overflow of FIFO buffer
 * \retval IO_E_CAN_WRONG_HANDLE  invalid handle has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the given handle has not been configured
 * \retval IO_E_CAN_OLD_DATA      no data has been received
 *
 **************************************************************************/
 IO_ErrorType IO_CAN_ReadFIFO( ubyte1 handle
                             , IO_CAN_DATA_FRAME * const buffer
                             , ubyte1 buffer_size
                             , ubyte1 * const rx_frames );

/**********************************************************************//**
 *
 * \brief Writes CAN frames to a FIFO buffer.
 *
 *   Copies CAN Frames from a SW frame buffer to a transmit FIFO buffer.
 *   The CAN transmission will be started.
 *
 *
 * \param   handle         CAN message object handle
 *                    (retrieved from IO_CAN_ConfigFIFO())
 * \param   data           Pointer to data structure.
 *                    The data in this structure will be transmitted.
 * \param   tx_length      Number of frames in "data" which shall be transmitted.
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                everything fine
 * \retval IO_E_CAN_FIFO_FULL     FIFO is full - no data has been transmitted
 * \retval IO_E_CAN_WRONG_HANDLE  invalid handle has been passed
 * \retval IO_E_NULL_POINTER      null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the given channel has not been configured
 *
 **************************************************************************/
 IO_ErrorType IO_CAN_WriteFIFO( ubyte1 handle
                              , const IO_CAN_DATA_FRAME * const data
                              , ubyte1 tx_length );

/**************************************************************************
 * AUXILIARY FUNCTIONS
 **************************************************************************/

/**********************************************************************//**
 *
 * \brief Returns the error counters of the CAN channel.
 *
 *   Returns the transmit and receive error counters of the selected CAN channel.
 *
 *
 * \param   channel           CAN channel, one of:
 *                               - \c IO_CAN_CHANNEL_0
 *                               - \c IO_CAN_CHANNEL_1
 *                               - \c IO_CAN_CHANNEL_2
 *                               - \c IO_CAN_CHANNEL_3
 * \param   rx_error_counter  Value of the receive error counter
 * \param   tx_error_counter  Value of the transmit error counter
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_CAN_ERROR_PASSIVE   controller is in error pasive state
 * \retval IO_E_CAN_BUS_OFF         controller is in bus off state
 * \retval IO_E_INVALID_CHANNEL_ID  wrong channel number has been passed
 * \retval IO_E_NULL_POINTER        null pointer has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the given channel has not been initialized
 *
 **************************************************************************
 *
 * \remarks
 * \if TTC94E
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available only
 *   for \ttc94 and \ttc94e
 * \else
 *   CAN channels IO_CAN_CHANNEL_2 and IO_CAN_CHANNEL_3 are available only
 *   for \ttc94
 * \endif
 *
 **************************************************************************/
 IO_ErrorType IO_CAN_Status( ubyte1 channel
                           , ubyte1 * const rx_error_counter
                           , ubyte1 * const tx_error_counter );

/**********************************************************************//**
 *
 * \brief Returns the status of a message buffer object.
 *
 *
 * \param   handle         CAN message object handle
 *                          (retrieved from IO_CAN_ConfigMsg())
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                everything fine
 * \retval IO_E_BUSY              transmission is ongoing
 * \retval IO_E_CAN_OVERFLOW      message object overflow
 * \retval IO_E_CAN_OLD_DATA      no new data received
 * \retval IO_E_CAN_WRONG_HANDLE  invalid handle has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the given handle has not been configured
 *
 **************************************************************************
 *
 * \remarks
 *   Calling this function also starts the Bus Off recovery sequence
 *   if the respective CAN node is in Bus Off.
 *
 **************************************************************************/
 IO_ErrorType IO_CAN_MsgStatus( ubyte1 handle );

/**********************************************************************//**
 *
 * \brief Returns the status of a FIFO buffer.
 *
 *
 * \param   handle         CAN message object handle
 *                          (retrieved from IO_CAN_ConfigFIFO())
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                everything fine
 * \retval IO_E_BUSY              transmission is ongoing
 * \retval IO_E_CAN_FIFO_FULL     FIFO is full - no data has been transmitted
 * \retval IO_E_CAN_OLD_DATA      no new data has been received
 * \retval IO_E_CAN_WRONG_HANDLE  invalid handle has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED the given handle has not been configured
 *
 **************************************************************************
 *
 * \remarks
 *   Calling this function also starts the Bus Off recovery sequence
 *   if the respective CAN node is in Bus Off.
 *
 **************************************************************************/
 IO_ErrorType IO_CAN_FIFOStatus( ubyte1 handle );

#endif /* _IO_CAN_H */
