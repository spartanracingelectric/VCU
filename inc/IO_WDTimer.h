/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_WDTimer.h
 *
 * \brief IO Driver functions for the CPU's Watchdog timer
 *
 *      The Watchdog Timer (WDT) is a secure mechanism to overcome
 *      life- and dead-locks.
 *      An enabled WDT generates a reset for the system if not
 *      serviced in a configured time frame.
 *
 * \section wdt_note Note
 *      This driver module is not available on \safetycontrollers.
 *      On this target the watchdog timer is configured
 *      and serviced by the internal diagnostic state machine.
 *
 * \section wdt_remark Remark
 *        After the first watchdog timer overrun (ie. when the watchdog
 *        timer is not serviced within the configured timeout),
 *        the CPU will be reset and the application starts again.
 *        After the second overrun the CPU will be held in a reset
 *        state to avoid endles resets.
 *
 * \section wdt_examples Watchdog timer code examples
 *      Example for watchdog timer initialization:
 *      \code
 *      IO_WDTimer_Init(350000); //setup the wdtimer with a timeout of 350ms
 *      \endcode
 *
 *      Example for watchdog timer service:
 *      \code
 *      IO_WDTimer_Service(); //service the watchdog timer
 *      \endcode
 *
 **************************************************************************/
#ifndef SAFETY /* module is not available on \safetycontrollers */

#ifndef _IO_WDTIMER_H
#define _IO_WDTIMER_H

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
#define IO_WDTIMER_TIMEOUT_MAX  13421772
#define IO_WDTIMER_TIMEOUT_MIN  3

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
 * \brief Initialization of the Watchdog Timer
 *
 *   The function
 *      - configures the timeout of the watchdog timer
 *      - enables the timer
 *
 *
 * \param   timeout         timeout for the watchdog timer in us
 *                          the CPU will be resetted if the watchdog
 *                          timer is not serviced within this period
 *                          value range: 3.2us .. 13.4s
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 *
 **************************************************************************/
IO_ErrorType IO_WDTimer_Init( ubyte4 timeout );

/**********************************************************************//**
 *
 * \brief Disable the Watchdog Timer
 *
 *   disables the watchdog timer functionality.
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 *
 **************************************************************************/
IO_ErrorType IO_WDTimer_DeInit( void );

/**********************************************************************//**
 *
 * \brief Service the Watchdog timer
 *
 *   Service routine for the watchdog timer.
 *   If the time between two Service function calls exceeds
 *   the timeout (set with IO_WDTimer_Init) the CPU will be resetted.
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 *
 **************************************************************************/
IO_ErrorType IO_WDTimer_Service( void );

#endif /* _IO_WDTIMER_H */
#endif /* TTC90 */
