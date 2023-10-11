/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_POWER.h
 *
 * \brief IO Driver functions for Power IC control
 *
 *      The Power IC control functions allow switching on and off the
 *      sensor supply voltages of the ECU as well as switching off
 *      the whole unit.
 *
 *      The Power IC driver is a high level SPI device driver and
 *      uses the low level SPI driver.
 *
 *      The communication with the Power IC is handled in a cyclic manner.
 *      This means that changes do not take effect immediately,
 *      but only after the SPI communication has been finished.
 *
 *      Note: The ECU shall only be powered down via IO_POWER_Set(IO_PIN_271, IO_POWER_OFF)
 *      if K15 is low in order to ensure a correct shutdown. It is the responsibility
 *      of the user to check for the right return value of K15 directly before calling
 *      IO_POWER_Set (IO_PIN_271, IO_POWER_OFF).
 *
 *
 *  \section power_examples Power code examples
 *
 *      Power initialization example: \n
 *      The Power driver does not need an explicit initialization.
 *      The Power driver is initialized by IO_Driver_Init().
 *      \code
 *      IO_Driver_Init();
 *      \endcode
 *
 *      Power task example:
 *      \code
 *
 *      // switch of sensor supply 0
 *      IO_POWER_Set (IO_PIN_136, IO_POWER_OFF);
 *
 *      // set variable sensor supply to 14.5 V
 *      IO_POWER_Set (IO_PIN_269, IO_POWER_14_5_V);
 *
 *      // send ECU to sleep mode (switch off K15)
 *      IO_POWER_Set (IO_PIN_271, IO_POWER_OFF);
 *      \endcode
 *
 **************************************************************************/
#ifndef _IO_POWER_H
#define _IO_POWER_H

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
 * \name Power values
 * \anchor power_val_def
 *
 * Selects power configuration.
 */
/*@{*/
#define IO_POWER_OFF      0     /**< switch off */
#define IO_POWER_ON       1     /**< switch on  */
#define IO_POWER_8_5_V    2     /**< for variable sensor supply: set to 8.5V */
#define IO_POWER_10_0_V   3     /**< for variable sensor supply: set to 10.0V */
#define IO_POWER_14_5_V   4     /**< for variable sensor supply: set to 14.5V */
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
 * \brief Initialization of the power module driver
 *
 *   Initialization of power module driver.
 *      - Initializes internal data structure
 *      - Configures SPI driver
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_CHANNEL_BUSY        the module is already initialized
 *
 **************************************************************************
 *
 * \remarks
 *   Module is initialized only once.
 *   To re-initialize the module, the function IO_POWER_DeInit()
 *   needs to be called.
 *
 **************************************************************************/
 IO_ErrorType IO_POWER_Init(void);

/**********************************************************************//**
 *
 * \brief Deinitializes the power module driver
 *
 *   Deinitializes the module.
 *   Allows re-initialization by IO_POWER_Init()
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED module has not been initialized
 *
 **************************************************************************/
 IO_ErrorType IO_POWER_DeInit(void);

/**********************************************************************//**
 *
 * \brief Sets a certain mode of the Power IC
 *
 *   Sets a certain mode for a sensor supply pin or the device power
 *
 *
 * \param   pin     Pin for which the mode shall be set, one of:
 *                      - \c IO_INT_POWERSTAGE_ENABLE
 *                      - \c IO_INT_PWM_POWER  (Power supply for PWM)
 *                      - \c IO_PIN_135        (UGEB1)
 *                      - \c IO_PIN_136        (UGEB0)
 *                      - \c IO_PIN_147        (UGEB1)
 *                      - \c IO_PIN_148        (UGEB0)
 *                      - \c IO_PIN_269        (UGEB_V)
 *                      - \c IO_PIN_271        (K15E, for power down)
 * \param   mode    Sets a certain mode, one of:
 *                      - \c IO_POWER_ON       (not for pin 269)
 *                      - \c IO_POWER_OFF      (not for pin 269)
 *                      - \c IO_POWER_8_5_V    (only for pin 269)
 *                      - \c IO_POWER_10_0_V   (only for pin 269)
 *                      - \c IO_POWER_14_5_V   (only for pin 269)
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                  everything fine
 * \retval IO_E_INVALID_PARAMETER   an invalid parameter has been passed
 * \retval IO_E_CHANNEL_NOT_CONFIGURED module has not been initialized
 *
 **************************************************************************
 *
 * \remarks
 *   \c IO_INT_POWERSTAGE_ENABLE and \c IO_INT_PWM_POWER are internal pins.
 *   - \c IO_INT_POWERSTAGE_ENABLE controls the internal powerstage enable
 *          signal. Without enabling this signal all power outputs remain
 *          low (switched off).
 *   - \c IO_INT_PWM_POWER controls the supply for the PWM outputs.
 *          Without switching this signal to ON the PWM power stages
 *          will not be supplied and the pin therefore will remain low.
 *
 * \remarks
 *   On the safety controllers (\safetycontrollers) the sensor supply
 *   voltages are monitored by the diagnostic state machine also if the sensor
 *   supply is not used by any safety critical input.
 *   The monitoring can be disabled only by switching off the respective
 *   sensor supply.
 *
 * \remarks
 *   If sensor supply 1 (IO_PIN_135 or IO_PIN_147) is disabled on a safety
 *   controller (\safetycontrollers), the safe state will be activated
 *   by the Watchdog CPU.
 *   This is because the Watchdog CPU monitors the sensor supply 1 and the
 *   CPU supply voltage on one pin, therefore this check cannot be disabled.
 *
 **************************************************************************/
 IO_ErrorType IO_POWER_Set(ubyte1 pin, ubyte1 mode);

/**********************************************************************//**
 *
 * \brief Returns the status of the Power IC
 *
 *   Returns whether the Power IC is operating correctly or not.
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK              everything fine
 * \retval IO_E_POWER_IC        Power IC has reported an error
 * \retval IO_E_CHANNEL_NOT_CONFIGURED module has not been initialized
 *
 **************************************************************************/
 IO_ErrorType IO_POWER_GetStatus(void);

#endif /* _IO_POWER_H */
