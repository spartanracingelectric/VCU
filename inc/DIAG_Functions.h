/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 *
 * \file DIAG_Functions.h
 *
 * \brief Auxiliary functions for the diagnostic state machine
 *
 *      Provides the interface to the diagnostic state machine
 *
 **************************************************************************/
#ifndef _DIAG_FUNCTIONS_H
#define _DIAG_FUNCTIONS_H

/**************************************************************************
 *
 * I N C L U D E S
 *
 **************************************************************************/

#include "IO_Driver.h"
#include "IO_Constants.h"
#include "DIAG_Constants.h"

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
 * \brief status function for diagnostic state machine
 *
 *      Returns the current satus as well as the error codes of
 *      the diagostic state machine and the watchdog CPU.
 *
 *
 * \param   diag_state      current state of the diagnostic state machine
 * \param   watchdog_state  current state of the watchdog CPU
 * \param   diag_error      error codes of the diagnostic state machine
 * \param   watchdog_error  error codes of the watchdog CPU
 *
 * \return IO_ErrorType
 * \retval IO_E_NULL_POINTER        null pointer has been passed
 * \retval IO_E_OK                  everything fine
 *
 ***********************************************************************//**
 *
 * \remarks
 *    If the application cycle time is larger than 25ms, no communication
 *    with the WD CPU is possible. For this reason the variable watchdog_state
 *    as well as the fields of watchdog_error will be 0 in this case.
 *
 **************************************************************************/
 IO_ErrorType DIAG_Status ( ubyte1 * diag_state
                          , ubyte1 * watchdog_state
                          , DIAG_ERRORCODE * diag_error
                          , DIAG_ERRORCODE * watchdog_error );

/**********************************************************************//**
 *
 * \brief allows an application driven safe state
 *
 *      When this function is called the diagnostic state machine
 *      enters the safe state.
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 *
 **************************************************************************/
 IO_ErrorType DIAG_EnterSafestate ( void );

/**********************************************************************//**
 *
 * \brief returns the version number of the watchdog CPU software
 *
 *      The version number of the watchdog CPU software is read
 *      during the startup sequence.
 *      Before the version number has been read, the function
 *      will return the according error value
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_WD_NO_VERSION       the version number has not yet been
 *                                  received from the watchdog cpu.
 * \retval IO_E_WD_TIMEOUT          watchdog communication timeout
 *                                  (version number could not be read)
 *
 **************************************************************************/
 IO_ErrorType DIAG_GetWDVersion ( ubyte1 * const wd_ver_maj
                                , ubyte1 * const wd_ver_min );


#endif /* _DIAG_FUNCTIONS_H */

