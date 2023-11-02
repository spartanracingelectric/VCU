/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_DIO.h
 *
 * \brief IO Driver functions for Digital Input/Output.
 *
 *      Contains all service functions for the digital in/outputs.\n
 *      Note: The digital inputs are connected directly to a CPU pin,
 *      therefore these inputs reflect the current status of the input
 *      at the point in time where the function is called.\n
 *      The digital outputs are controlled over SPI shift registers, therefore
 *      the output will be set in the Driver Task function at the end
 *      of the cycle. This takes two cycles (two Driver Task end function calls)
 *      until the output is updated.\n
 *      Additionally a open load and short circuit detection is implemented on
 *      the digital outputs.
 *
 *  \section dio_examples DIO Code Examples
 *
 *      DIO initialization examples:
 *      \code
 *      IO_DI_Init( IO_DI_00            // digital input
 *                , IO_DI_PU_10K );     // enable 10kOhm pull up
 *
 *      IO_DO_Init( IO_DO_00 );         // digital output
 *      \endcode
 *
 *      DIO task function examples:
 *      \code
 *      bool di_val_0;
 *
 *      IO_DI_Get( IO_DI_00             // read value of digital input
 *               , &di_val_0 );
 *
 *      IO_DO_Set( IO_DO_00             // set digital output value
 *               , TRUE );
 *
 *      \endcode
 *
 **************************************************************************/

#ifndef _IO_DIO_H
#define _IO_DIO_H

/**************************************************************************
 *
 * I N C L U D E S
 *
 **************************************************************************/

#include "IO_Driver.h"
#include "IO_PWD.h"

/**************************************************************************
 *
 * D E F I N I T I O N S
 *
 **************************************************************************/

/**
 * \name Pull up / down settings
 * \anchor dio_pupd_settings
 *
 * Selects a pull-up or pull-down configuration for a digital input
 */
/*@{*/
#define IO_DI_PU_10K            IO_PWD_PU_10K    /**< pull up 10kOhm   */
#define IO_DI_PD_10K            IO_PWD_PD_10K    /**< pull down 10kOhm */
#define IO_DI_PD_1K8            IO_PWD_PD_1K8    /**< pull down 1.8kOhm  */
#define IO_DI_PD_110            IO_PWD_PD_110    /**< 110 pull down resistor     */
/*@}*/

/**************************************************************************
 *
 * F U N C T I O N S
 *
 **************************************************************************/

/**********************************************************************//**
 *
 * \brief Setup the Digital Inputs
 *
 * \param di_channel                Digital input:
 *                                      - \c IO_DI_00 .. \c IO_DI_07
 *                                      - \c IO_DI_K15
 *                                      - \c IO_DI_08 .. \c IO_DI_15
 *                                      - \c IO_DI_16 .. \c IO_DI_19
 * \param mode                      Input configuration:
 *                                      - \c IO_DI_PU_10K:   Pull up 10K
 *                                      - \c IO_DI_PD_10K:   Pull down 10K
 *                                      - \c IO_DI_PD_1K8:   Pull down 1.8 kOhm
 *                                      - \c IO_DI_PD_110:   Pull down 110 Ohm
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_INVALID_CHANNEL_ID  the channel id does not exist
 * \retval IO_E_CHANNEL_BUSY        the digital output channel is
 *                                  currently used by another function
 * \retval IO_E_INVALID_PARAMETER   parameter is out of range
 *
 ***************************************************************************
 *
 * \remarks
 *   For these digital inputs, the get-function returns the status of the input at
 *   the point in time the function was called.
 *
 * \remarks
 *   If inputs \c IO_DI_08 .. \c IO_DI_15 and \c IO_DI_K15 are configured, the function
 *   ignores the parameter mode.
 *
 * \remarks
 *   Input mode configuration \c IO_DI_PD_110 is only available for
 *   digital inputs 16..19 (\c IO_DI_16 .. \c IO_DI_19)
 *
 ***************************************************************************/
IO_ErrorType IO_DI_Init( ubyte1 di_channel
                       , ubyte1 mode );

/**********************************************************************//**
 *
 * \brief Setup the Digital Outputs
 *
 * \param do_channel                Digital Output:
 *                                      - \c IO_DO_00 .. \c IO_DO_07
 *                                      - \c IO_DO_08 .. \c IO_DO_11
 *                                      - \c IO_DO_12 .. \c IO_DO_19
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_INVALID_CHANNEL_ID  the channel id does not exist
 * \retval IO_E_CHANNEL_BUSY        the digital output channel is
 *                                  currently used by a other function
 *
 ***************************************************************************
 *
 * \remarks
 *   The digital output channels \c IO_DO_00 .. \c IO_DO_11 are controlled over
 *   SPI shift Registers.
 *   Therefore the output will be set in the Driver Task function at the end
 *   of the cycle. This takes two cycles (two Driver Task end functions calls)
 *   until the output is updated
 *
 * \remarks
 *   The digital output channels \c IO_DO_12 .. \c IO_DO_19 are controlled directly
 *   through CPU port pins.
 *
 * \remarks
 *   For open load/short-circuit detection the digital output channels
 *   \c IO_DO_00 .. \c IO_DO_11 the associated ADC channel will also be configured.
 *
 ***************************************************************************/
IO_ErrorType IO_DO_Init( ubyte1 do_channel );

/**********************************************************************//**
 *
 * \brief Deinitializes a DI channel
 *
 * \param di_channel                Digital Input:
 *                                      - \c IO_DI_00 .. \c IO_DI_07
 *                                      - \c IO_DI_K15
 *                                      - \c IO_DI_08 .. \c IO_DI_15
 *                                      - \c IO_DI_16 .. \c IO_DI_19
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 *
 ***************************************************************************/
IO_ErrorType IO_DI_DeInit( ubyte1 di_channel );

/**********************************************************************//**
 *
 * \brief Deinitializes a DO channel
 *
 * \param do_channel                Digital Output:
 *                                      - \c IO_DO_00 .. \c IO_DO_07
 *                                      - \c IO_DO_08.. \c IO_DO_11
 *                                      - \c IO_DO_12.. \c IO_DO_19
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************/
IO_ErrorType IO_DO_DeInit( ubyte1 do_channel );

/**********************************************************************//**
 *
 * \brief Gets the value of a Digital Input
 *
 * \param di_channel                Digital Input:
 *                                      - \c IO_DI_00 .. \c IO_DI_07
 *                                      - \c IO_DI_K15
 *                                      - \c IO_DI_08 .. \c IO_DI_15
 *                                      - \c IO_DI_16 .. \c IO_DI_19
 * \param di_value                  Input value:
 *                                      - \c TRUE:  High level
 *                                      - \c FALSE: Low level
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the channel id does not exist
 * \retval IO_E_NULL_POINTER
 * \retval IO_E_FET_PROTECTION          FET is disabled, protection is active
 * \retval IO_E_UNKNOWN                 ADC error
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************
 *
 * \remarks
 *   This function returns the current value of the digital input pins at the
 *   time when the function is called.
 *
 ***************************************************************************/
IO_ErrorType IO_DI_Get( ubyte1 di_channel
                      , bool * const di_value );

/**********************************************************************//**
 *
 * \brief Sets the value of a Digital Output
 *
 * \param do_channel                Digital Output:
 *                                      - \c IO_DO_00 .. \c IO_DO_07
 *                                      - \c IO_DO_08 .. \c IO_DO_11
 *                                      - \c IO_DO_12 .. \c IO_DO_19
 * \param do_value                  Input value:
 *                                      - \c TRUE:  High level
 *                                      - \c FALSE: Low level
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 * \retval IO_E_DO_SHORT_CIRCUIT        the output current is too high
 * \retval IO_E_DO_OPEN_LOAD            open load is detected on the output
 * \retval IO_E_DO_DIAG_NOT_FINISHED    output diagnosis could not be finished
 *                                      because \c do_value has been changed
 *                                      too frequently.
 * \retval IO_E_UNKNOWN                 the ADC driver reported an error
 * \retval IO_E_FET_PROTECTION          FET is disabled, protection is active
 *
 ***************************************************************************
 *
 * \remarks
 *   The digital output channels \c IO_DO_00 .. \c IO_DO_11 are controlled over
 *   SPI shift Registers.
 *   Therefore the next output value will be transferred to the shift registers
 *   in the Driver Task function at the beginning of the next cycle. The
 *   transmission of the output values and the activation of the output enable
 *   signal of the shift registers takes two cycles (two Driver Task Begin
 *   functions calls) until the output is updated.
 *
 * \attention
 *   The error code \c IO_E_DO_DIAG_NOT_FINISHED will be returned for the
 *   digital output channels \c IO_DO_00 .. \c IO_DO_11 if the parameter
 *   \c do_value gets changed too frequently (if \c do_value is toggled on 3
 *   or more consecutive calls of IO_DO_Set).
 *
 * \remarks
 *   The digital output channels \c IO_DO_12.. \c IO_DO_19 are controlled directly
 *   through CPU port pins.
 *
 ***************************************************************************/
IO_ErrorType IO_DO_Set( ubyte1 do_channel
                      , bool do_value );

#endif /* _IO_DIO_H */
