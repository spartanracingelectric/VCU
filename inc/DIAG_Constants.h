/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file DIAG_Constants.h
 *
 * \brief Global defines for IO Driver diagnostic state machine and WD
 *
 *      This header file defines the Error Codes for diagnostic state machine and WD.
 *
 **************************************************************************/

#ifndef _DIAG_CONSTANTS_H
#define _DIAG_CONSTANTS_H

/**************************************************************************
 *
 * I N C L U D E S
 *
 **************************************************************************/

#include "ptypes_xe167.h"

/**************************************************************************
 *
 * D A T A   S T R U C T U R E
 *
 **************************************************************************/

/**
 *
 * \brief Diagnostic Error code structure
 *
 *  Stores all relevant error parameters returned from the diagnostic
 *  state machine or returned from the WD
 *
 */
typedef struct _diag_errorcode
{
    ubyte1 error_code;          /**< error code                         */
    ubyte1 device_num;          /**< device number which caused error   */
    ubyte2 faulty_value;        /**< value which caused the error       */
} DIAG_ERRORCODE;

/**************************************************************************
 *
 * D E F I N I T I O N S
 *
 **************************************************************************/

/*
 * D I A G N O S T I C   S T A T E   M A C H I N E   E R R O R   V A L U E S
 */

/**
 * \name Diagnostic State Machine Error Values
 * \anchor diag_err_values_doc
 *
 * Errors codes that a function DIAG_Status returns in parameter diag_error.error_code
 */
/*@{*/
#define DIAG_E_NO_ERROR                     0   /**< No error                                               */

#define DIAG_E_ADC_5V_SWITCH                119 /**< ADC 5V input switches error                            */
#define DIAG_E_ADC_VREF                     141 /**< ADC Vref check error                                   */
#define DIAG_E_ADC_5V_SWITCHES_PER          140 /**< ADC 5V periodic switches check error                   */
#define DIAG_E_ADC_VREF_GND_NORMAL_RANGE    121 /**< ADC Vref GDN check normal range error                  */
#define DIAG_E_ADC_VREF_GND_GND_RANGE       122 /**< ADC Vref GDN check ground range error                  */
#define DIAG_E_ADC_RATIO_SHORT_CIRCUIT      123 /**< ADC 5V ratiometric redundant short circuit test error  */
#define DIAG_E_ADC_LIMITS                   125 /**< ADC 5V or ADC 32V limit check error                    */
#define DIAG_E_ADC_CURRENT_LOOP             142 /**< ADC 5V Current Loop check error                        */
#define DIAG_E_ADC_SWITCH                   146 /**< ADC 32V Switch check error                             */
#define DIAG_E_ADC_SENSOR_SUPPLY            147 /**< ADC sensor supply check error                          */

#define DIAG_E_PWD_LIMITS_FREQ              135 /**< PWD frequency limit check error                        */
#define DIAG_E_PWD_LIMITS_PULSE_WIDTH       143 /**< PWD Pulse width limit check error                      */
#define DIAG_E_PWD_SWITCH                   145 /**< PWD Switch check error                                 */
#define DIAG_E_PWD_MUX                      148 /**< PWD Mux check error                                    */

#define DIAG_E_PWM_SHORT_CIRCUIT            130 /**< PWM short circuit check error                          */
#define DIAG_E_PWM_PERIOD_MISMATCH          131 /**< PWM period mismatch                                    */
#define DIAG_E_PWM_LIMITS_RANGE             132 /**< PWM limit range check error                            */
#define DIAG_E_PWM_LIMITS_TOL               133 /**< PWM limit tolerance check error                        */
#define DIAG_E_PWM_CURRENT                  134 /**< PWM current check error                                */
#define DIAG_E_PWM_OVERCURRENT              144 /**< PWM overcurrent check error                            */

#define DIAG_E_STATE_INIT                   183 /**< internal error in init state                           */
#define DIAG_E_STATE_CONFIG                 185 /**< internal error in config state                         */
#define DIAG_E_STATE_MAIN                   186 /**< internal error in main state                           */
#define DIAG_E_STATE_SAFE                   189 /**< internal error in safe state                           */

#define DIAG_E_WATCHDOG_SAFE_STATE          190 /**< WD CPU has entered the safe state                      */
#define DIAG_E_INVALID_DIAG_STATE           191 /**< internal error: invalid state for diag state machine   */
#define DIAG_E_WATCHDOG_TIMEOUT             192 /**< WD CPU has not replied correctly within the timeout    */
#define DIAG_E_APPL_SAFE_STATE              193 /**< Application requested to activate the safe state       */

#define DIAG_E_MEM_REGISTER                 127 /**< MEM register check error                               */
#define DIAG_E_MEM_INT_RAM                  128 /**< internal RAM check error                               */
#define DIAG_E_MEM_EXT_RAM                  129 /**< external RAM check error                               */
#define DIAG_E_MEM_FLASH                    136 /**< Flash check error                                      */
#define DIAG_E_MEM_USER_STACK               126 /**< User Stack check error                                 */
#define DIAG_E_MEM_SYSTEM_STACK             160 /**< System Stack check error                               */
#define DIAG_E_MEM_GENERAL                  161 /**< General CPU error, TRAP 0x28                           */

/*@}*/


/*
 * W A T C H D O G   C P U   E R R O R   V A L U E S
 */

/**
 * \name Watchdog CPU Error Values
 * \anchor diag_wd_err_values_doc
 *
 * Errors codes that a function DIAG_Status returns in parameter watchdog_error.error_code
 */
/*@{*/
#define DIAG_WD_E_NO_ERROR                      0U

#define DIAG_WD_E_INV_SEQ_NO                    1U
#define DIAG_WD_E_INV_OPCODE                    2U
#define DIAG_WD_E_INV_CMD_FRAME_LENGTH          3U
#define DIAG_WD_E_CRC                           4U
#define DIAG_WD_E_VALUE_RANGE                   5U
#define DIAG_WD_E_TIMEOUT                       6U
#define DIAG_WD_E_MISSING_IO_DATA               7U
#define DIAG_WD_E_UNEXPECTED_CMD                8U
#define DIAG_WD_E_WD_CONNECTION_LOST            9U
#define DIAG_WD_E_L_LIMIT_GREATER_H_LIMIT       10U

#define DIAG_WD_E_RPV_SHUT_OFF                  11U
#define DIAG_WD_E_VOLT_MONITOR                  20U
#define DIAG_WD_E_SUPPLY_VOLT                   21U
#define DIAG_WD_E_LOW_VOLTAGE                   22U

#define DIAG_WD_E_STACK_UNDER_OR_OVERFLOW       30U
#define DIAG_WD_E_REGISTER_ERROR                31U
#define DIAG_WD_E_WATCHDOG_RAM                  40U
#define DIAG_WD_E_WATCHDOG_FLASH                41U
#define DIAG_WD_E_PRGM_FLOW                     42U
#define DIAG_WD_E_FLAG_TEST                     43U
#define DIAG_WD_E_FLAG_MONITORING               44U

#define DIAG_WD_E_ADC_SRC                       50U
#define DIAG_WD_E_ADC_SHORT_CIRCUIT             51U
#define DIAG_WD_E_ADC_CR_RC                     52U
#define DIAG_WD_E_ADC_OVERRIDE                  53U
#define DIAG_WD_E_PWM_DUTY_CYCLE_SET            60U
#define DIAG_WD_E_PWM_DUTY_CYCLE_READ           61U
#define DIAG_WD_E_PWM_TOLERANCE                 62U
#define DIAG_WD_E_TPU_LIMIT                     70U
#define DIAG_WD_E_TPU_OVERRIDE                  71U

#define DIAG_WD_E_RELAY                         80U
#define DIAG_WD_E_POWERSTAGES_ENABLE            81U
#define DIAG_WD_E_ADC_SWITCH_SETTINGS           82U

#define DIAG_WD_E_HARMONICS                     90U
#define DIAG_WD_E_MAIN_CPU_SAFE_STATE           91U
#define DIAG_WD_E_APPL_SAFE_STATE               92U

#define DIAG_WD_E_DRV_IO                        100U
#define DIAG_WD_E_DRV_ADC                       101U
#define DIAG_WD_E_DRV_SPI                       102U
#define DIAG_WD_E_DRV_FLASH                     103U
#define DIAG_WD_E_MOD_SPI                       104U
#define DIAG_WD_E_MOD_CHECK_PWM_OUTPUTS         105U
#define DIAG_WD_E_MOD_CHECK_SUPPLY              106U
#define DIAG_WD_E_MOD_CHECK_VOLT_MONITOR        107U
#define DIAG_WD_E_MOD_CHECK_EXT_RELAYS          108U
#define DIAG_WD_E_MOD_CHECK_DIG_TIMER_INPUTS    109U
#define DIAG_WD_E_MOD_CHECK_ANALOG_INPUTS       110U
#define DIAG_WD_E_MOD_CHECK_FLASH               111U
#define DIAG_WD_E_MOD_CHECK_SHORT_CIRCUIT_AI    112U
#define DIAG_WD_E_MOD_CHECK_RPV                 113U
#define DIAG_WD_E_MOD_CHECK_RAM_TEST            114U
#define DIAG_WD_E_MOD_CHECK_REGISTER_TEST       115U
#define DIAG_WD_E_MOD_CHECK_PGM_FLOW            116U
#define DIAG_WD_E_MOD_CHECK_STACK               117U
#define DIAG_WD_E_MOD_CHECK_CONFIG              118U

#define DIAG_WD_E_STATE_INIT                    150U
#define DIAG_WD_E_STATE_START_UP                151U
#define DIAG_WD_E_STATE_RUNNING                 152U
#define DIAG_WD_E_STATE_SAFE_STATE              153U
#define DIAG_WD_E_STATE_SYNC1                   154U
#define DIAG_WD_E_STATE_SYNC2                   155U
#define DIAG_WD_E_STATE_SYNC3                   156U
/*@}*/

/*
 * D I A G N O S T I C   S T A T E   M A C H I N E   S T A T E S
 */

/**
 * \name States of the diagnostic state machine
 * \anchor diag_states
 *
 * State information returned by the function DIAG_Status
 */
/*@{*/
#define DIAG_STATE_INIT                     0x03
#define DIAG_STATE_CONFIG                   0x05
#define DIAG_STATE_MAIN                     0x06
#define DIAG_STATE_SAFE_STATE               0x09
#define DIAG_STATE_DISABLED                 0x00
/*@}*/

/*
 * W A T C H D O G   C P U   S T A T E S
 */

/**
 * \name States of the watchdog cpu
 * \anchor diag_wd_states
 *
 * State information returned by the function DIAG_Status
 */
/*@{*/
#define DIAG_WD_STATE_INIT                  0x03
#define DIAG_WD_STATE_START_UP              0x05
#define DIAG_WD_STATE_RUNNING               0x06
#define DIAG_WD_STATE_SAFE_STATE            0x09
#define DIAG_WD_STATE_UNDEFINED             0x00
/*@}*/


/*
 * D I A G   D E V I C E S
 */

/**
 * \name watchdog and diag devices
 * \anchor diag_wd_devices
 *
 * Additional device defines for watchdog and diagnostic state machine
 */
/*@{*/
#define DIAG_DEV_CONFIG_GLOBAL                 200U
#define DIAG_DEV_CONFIG_ANALOG_5V              201U
#define DIAG_DEV_CONFIG_ANALOG_10V             202U
#define DIAG_DEV_CONFIG_PWM                    203U
#define DIAG_DEV_CONFIG_TPU                    204U

#define DIAG_DEV_IO_ANALOG_5V                  205U
#define DIAG_DEV_IO_ANALOG_10V                 206U
#define DIAG_DEV_IO_PWM                        207U
#define DIAG_DEV_IO_TPU                        208U

#define DIAG_DEV_REF_VOLTAGE_3V                195U
#define DIAG_DEV_REF_VOLTAGE_2P5V              196U
#define DIAG_DEV_REF_VOLTAGE_EXT_POWER         197U

#define DIAG_DEV_RAM_FLASH_WATCHDOG            251U
#define DIAG_DEV_WATCHDOG_CPU                  252U
#define DIAG_DEV_SUPPLY_VOLTAGE                253U
#define DIAG_DEV_TTP_CONTROLLER                254U
#define DIAG_DEV_MAIN_CPU                      255U

#define DIAG_DEV_RELAY1                        110U
#define DIAG_DEV_RELAY2                        111U
#define DIAG_DEV_RELAY3                        112U
/*@}*/

#endif /* _DIAG_CONSTANTS_H */
