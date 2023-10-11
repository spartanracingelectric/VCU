/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_Constants.h
 *
 * \brief Global defines for IO Driver
 *
 *      This header file defines the Error Codes as well as the
 *      IO Pins. Furthermore it provides some aliases.
 *
 **************************************************************************/

#ifndef _IO_CONSTANTS_H
#define _IO_CONSTANTS_H

/**********************************************************************//**
 * \page pin_diag_features Pin and diagnostic features
 *
 * This page provides an overview of the features and diagnostic functions of the ECU pins.
 *
 * \section pin_features_ttc90 Features of \family
 *
 * \verbatim[table]
 * ======================================= ================ ================= ========================================== ==================================================================================== ============== =========================================
 *   PIN GROUP                                                                  FEATURES                                   DIAGNOSTIC FUNCTIONS
 * ========================================================================== ========================================== =============================================================================================================================================
 *   Group                                   Defines          Pin Numbers       I/O Features                                Diagnostics                                                                        Config Type               Error Codes
 * --------------------------------------- ---------------- ----------------- ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *   <b>PWM output groups</b>
 * -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *   \ref IO_PWM.h "PWM Output" \n           #IO_PWM_00\n     #IO_PIN_118\n      \ref IO_PWM.h "PWM Output" \n\n             open load / short to UBAT detection \n                                            non-safety      #IO_E_PWD_OPEN_LOAD
 *   with timer feedback        \n           #IO_PWM_01\n     #IO_PIN_106\n      IO_PWM_SetDuty \n                           (cannot be distinguished)
 * +                                         #IO_PWM_02\n     #IO_PIN_117\n      IO_PWM_DualSetDuty                      ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                           #IO_PWM_03\n     #IO_PIN_105\n                                                  short circuit to GND detection                                                    non-safety      #IO_E_PWD_SHORT_CIRCUIT
 * +                                         #IO_PWM_04\n     #IO_PIN_116\n                                              ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                           #IO_PWM_05\n     #IO_PIN_104\n                                                  measured signal period is too small                                               non-safety      #IO_E_PWD_CAPTURE_ERROR
 * +                                         #IO_PWM_06\n     #IO_PIN_115\n   ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                           #IO_PWM_07\n     #IO_PIN_103\n      \ref IO_DIO.h "Digital Output"\n\n          open load / short to UBAT detection \n                                            non-safety      #IO_E_DO_OPEN_LOAD
 *                                                                               IO_DO_Set                                   (cannot be distinguished)
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                           short circuit to GND detection \n                                                 non-safety      #IO_E_DO_SHORT_CIRCUIT
 * +                                                                          ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                               \ref IO_DIO.h "Digital Input"\n\n           no built-in diagnostic functions                                                  non-safety
 *                                                                               IO_DI_Get
 * +                                                                          ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                               \ref IO_PWD.h "Digital Timer Input" \n\n    open connection / short to UBAT detection \n                                      non-safety      #IO_E_PWD_HIGH_LEVEL
 *                                                                               IO_PWD_FreqGet \n                           (cannot be distinguished)
 * +                                                                             IO_PWD_PulseGet                         ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                           short circuit to GND detection                                                    non-safety      #IO_E_PWD_LOW_LEVEL
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                           measured signal period is too small                                               non-safety      #IO_E_PWD_CAPTURE_ERROR
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                           time measurement not finished yet                                                 non-safety      #IO_E_PWD_NOT_FINISHED
 * --------------------------------------- ---------------- ----------------- ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *   \ref IO_PWM.h "PWM current feedback"\n  #IO_ADC_00\n     #IO_PIN_120\n      \ref IO_PWM.h "PWM current feedback"\n\n    low side switch protection                                                        non-safety      #IO_E_FET_PROTECTION
 *                                                                               IO_PWM_GetCur
 * + PWM current feedback                    #IO_ADC_01\n     #IO_PIN_108\n   ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                           #IO_ADC_02\n     #IO_PIN_119\n      \ref IO_DIO.h "Digital Output" \n\n         open load / short to UBAT detection \n                                            non-safety      #IO_E_DO_OPEN_LOAD
 *                                           #IO_ADC_03\n     #IO_PIN_107\n      IO_DO_Set                                   (cannot be distinguished)
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                           short circuit to GND detection \n                                                 non-safety      #IO_E_DO_SHORT_CIRCUIT
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                           low side switch protection                                                        non-safety      #IO_E_FET_PROTECTION
 * --------------------------------------- ---------------- ----------------- ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *   <b>Digital output groups</b>
 * -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *   \ref IO_DIO.h "Digital Output" \n       #IO_DO_00\n      #IO_PIN_144\n      \ref IO_DIO.h "Digital Output"  \n\n        open load / short to UBAT detection \n                                            non-safety      #IO_E_DO_OPEN_LOAD
 *   with analog feedback                    #IO_DO_01\n      #IO_PIN_132\n      IO_DO_Set                                   (cannot be distinguished)
 * +                                         #IO_DO_02\n      #IO_PIN_143\n                                              ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                           #IO_DO_03\n      #IO_PIN_131\n                                                  short circuit to GND detection \n                                                 non-safety      #IO_E_DO_SHORT_CIRCUIT
 * +                                         #IO_DO_04\n      #IO_PIN_142\n                                              ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                           #IO_DO_05\n      #IO_PIN_130\n                                                  diagnosis not finished \n (output value changed too frequently)\n                 non-safety      #IO_E_DO_DIAG_NOT_FINISHED
 * +                                         #IO_DO_06\n      #IO_PIN_141\n   ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                           #IO_DO_07\n      #IO_PIN_129\n      \ref IO_ADC.h "Analog Input 5V" \n\n        no built-in diagnostic functions                                                  non-safety
 *                                                                               IO_ADC_Get
 * --------------------------------------- ---------------- ----------------- ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *   <b>Analog input groups</b>
 * -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *   \ref IO_ADC.h "ADC Input" \n            #IO_ADC_5V_00\n  #IO_PIN_152\n      \ref IO_ADC.h "Analog Input" \n\n           configuration switch protection \n                                                non-safety      #IO_E_FET_PROTECTION
 *   configurable by SW        \n            #IO_ADC_5V_01\n  #IO_PIN_140\n      IO_ADC_Get  \n                              (only for current sensors)
 *   (3 modes)                               #IO_ADC_5V_02\n  #IO_PIN_151\n
 *                                           #IO_ADC_5V_03\n  #IO_PIN_139\n
 *                                           #IO_ADC_5V_04\n  #IO_PIN_150\n
 *                                           #IO_ADC_5V_05\n  #IO_PIN_138\n
 *                                           #IO_ADC_5V_06\n  #IO_PIN_149\n
 *                                           #IO_ADC_5V_07\n  #IO_PIN_137\n
 * --------------------------------------- ---------------- ----------------- ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *   \ref IO_ADC.h "ADC Input" \n           #IO_ADC_VAR_00\n  #IO_PIN_126\n      \ref IO_ADC.h "Analog Input"  \n\n          no built-in diagnostic functions                                                  non-safety
 *   input range selectable                 #IO_ADC_VAR_01\n  #IO_PIN_114\n      IO_ADC_Get
 *                                          #IO_ADC_VAR_02\n  #IO_PIN_125\n
 *                                          #IO_ADC_VAR_03\n  #IO_PIN_113\n
 *                                          #IO_ADC_VAR_04\n  #IO_PIN_124\n
 *                                          #IO_ADC_VAR_05\n  #IO_PIN_112\n
 *                                          #IO_ADC_VAR_06\n  #IO_PIN_123\n
 *                                          #IO_ADC_VAR_07\n  #IO_PIN_111\n
 * --------------------------------------- ---------------- ----------------- ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *   <b>Digital input groups</b>
 * -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *   \ref IO_DIO.h "Digital Input"           #IO_DI_00\n      #IO_PIN_264\n      \ref IO_DIO.h "Digital Input" \n\n          no built-in diagnostic functions                                                  non-safety
 *                                           #IO_DI_01\n      #IO_PIN_257\n      IO_DI_Get
 *                                           #IO_DI_02\n      #IO_PIN_263\n
 *                                           #IO_DI_03\n      #IO_PIN_256\n
 *                                           #IO_DI_04\n      #IO_PIN_262\n
 *                                           #IO_DI_05\n      #IO_PIN_255\n
 *                                           #IO_DI_06\n      #IO_PIN_261\n
 *                                           #IO_DI_K15\n     #IO_PIN_115\n
 *
 * --------------------------------------- ---------------- ----------------- ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *   \ref IO_PWD.h "Digital Timer Input"     #IO_PWD_08\n     #IO_PIN_275\n      \ref IO_PWD.h "Digital Timer Input" \n\n
 *                                           #IO_PWD_09\n     #IO_PIN_268\n      IO_PWD_ComplexGet \n
 *                                           #IO_PWD_10\n     #IO_PIN_274\n      IO_PWD_CountGet                           configuration switch protection \n (only on current sensors)                        non-safety      #IO_E_FET_PROTECTION
 * +                                         #IO_PWD_11\n     #IO_PIN_267\n                                              ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                         short to UBAT detection \n (only on current sensors)                                non-safety      #IO_E_PWD_CURRENT_THRESHOLD_HIGH
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                         short to GND / open load detection \n (only on current sensors) \n                  non-safety      #IO_E_PWD_CURRENT_THRESHOLD_LOW
 *                                                                                                                         (cannot be distinguished)
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                         timer overflow                                                                      non-safety      #IO_E_PWD_TIMER_OVERFLOW
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                         spikes or high frequency                                                            non-safety      #IO_E_PWD_CAPTURE_ERROR
 * +                                                                                                                     ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                                                                         time measurement not finished yet                                                   non-safety      #IO_E_PWD_NOT_FINISHED
 * +                                                                          ------------------------------------------ ------------------------------------------------------------------------------------ -------------- ----------------------------------------------------------------------------------------------------------------------------- -------------- -----------------------------------------
 *                                                                               \ref IO_DIO.h "Digital Input" \n\n        configuration switch protection \n (only on current sensors)                        non-safety      #IO_E_FET_PROTECTION
 *                                                                               IO_DI_Get
 * +                                                                          ------------------------------------------ ------------------------------------------------------------------------------------ -------------- -----------------------------------------
 *                                                                               \ref IO_ADC.h "Analog Input" \n\n         configuration switch protection \n (only on current sensors)                        non-safety      #IO_E_FET_PROTECTION
 *                                                                               IO_ADC_Get
 * ======================================= ================ ================= ========================================== ==================================================================================== ============== =========================================
 * \endverbatim
 *
 **************************************************************************/

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

typedef ubyte2  IO_ErrorType;

/** Every driver function returns an error code of type IO_ErrorType.
  * Refer to \ref err_values_doc "Error Values" for a documentation of the
  * possible values
  */

/**************************************************************************
 *
 * D E F I N I T I O N S
 *
 **************************************************************************/

/*
 * E R R O R   V A L U E S
 */

/**
 * \name Error Values
 * \anchor err_values_doc
 *
 * Errors codes that a function might return in IO_ErrorType.
 */
/*@{*/
/** everything is fine, no error has occurred. */
#define IO_E_OK                         0
/** Module or function is busy. \n
 *  This error is reported if a function or module has not yet finished its task.
 *  For example the EEPROM write function will return this error code if a previous write command has not been finished yet.
 */
#define IO_E_BUSY                       2
/** General error. No further information can be provided. */
#define IO_E_UNKNOWN                    3
/** A NULL pointer has been passed to the function. \n
 *  This error is reported if a non-optional pointer parameter of the function has been set to NULL.
 */
#define IO_E_NULL_POINTER               30
/** An invalid parameter has been passed to the function. \n
 *  This error is reported if at least one of the parameters which have been passed to the function is outside the allowed range.
 */
#define IO_E_INVALID_PARAMETER          31
/** The IO channel (IO pin) is busy \n
 *  This error is reported if a IO Pin has been initialized before. \n
 *  To change the configuration of the channel during runtime the according DeInit function needs
 *  to be called before the channel can be initialized with a new configuration.
 */
#define IO_E_CHANNEL_BUSY               32
/** The IO channel (IO pin) has not been initialized. \n
 *  This error is reported by an IO driver task function if the channel has not been initialized.
 *  To initialize the channel, the according Init function needs to be called.
 */
#define IO_E_CHANNEL_NOT_CONFIGURED     33
/** The IO channel (IO pin) does not exist. \n
 *  This error is reported if a non-existent channel ID has been passed to the function.
 */
#define IO_E_INVALID_CHANNEL_ID         34
/** A internal switch (FET) protection has been detected.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  PWM current feedback \n      the current on the low side switch is above 2.475A             > 2.475A                    3 * cycletime  \n              n.a.                                         <= 2.475A \n\n                non-safety
 *                                                                                                                          (if function is called each                                                 Reset Time: 1s
 *                                                                                                                          cycle)
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Analog Input \n              input current is higher than 22.5mA                            > 22.5mA                    3 * cycletime \n               n.a.                                         <= 22.5mA \n\n                non-safety
 *  configurable by SW                                                                                                      (if function is called each                                                 Reset Time: 1s
 *  (3 modes)                                                                                                               cycle)
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital timer input \n       input current is higher than 22.5mA                            > 22.5mA                    3 * cycletime \n               n.a.                                         <= 22.5mA \n\n                non-safety
 *                                                                                                                          (if function is called each                                                 Reset Time: 1s
 *                                                                                                                          cycle)
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 * An internal switch (FET) has been disabled to protect it from damage. \n
 *  If the current on an internal FET is too high, the FET will be switched off by software to protect it from damage.
 *  After 1s timeout the driver tries to re-enable the FET. \n
 *  When a FET has been switched off by the protection mechanism, this error code will be returned by the respective
 *  task function.
 *
*/
#define IO_E_FET_PROTECTION             35
/** Global safety configuration is missing \n
 *  This error is reported if an IO is defined as safety critical although no safety configuration has been passed to the IO_Driver_Init() (parameter \c safety_conf) function.
 *  An IO pin is considered as safety critical if a valid safety configuration has been passed to the init function
 *  (see \c safety_conf parameter of the functions IO_ADC_ChannelInit(), IO_PWM_Init(), IO_PWD_IncInit() and IO_PWD_ComplexInit(), IO_PWD_CountInit() )
 */
#define IO_E_DRV_SAFETY_CONF_NOT_CONFIG 36
/** The periodic interrupt timer has not been initialized
 *  This error code is reported if trying to disable the periodic interrupt timer although it has not been setup.
 */
#define IO_E_PERIODIC_NOT_CONFIGURED    37
/** Message object or FIFO buffer overflow. \n
 *  This error is reported if CAN messages have been lost due to a full buffer.
 *  To avoid this error FIFO buffers can be used. If FIFO buffers are already used, try to increase the buffer size.
 */
#define IO_E_CAN_OVERFLOW               40
/** A wrong or invalid handle has been used. \n
 *  This error is reported if:
 *   - a non-existent handle has been used.
 *   - if a write handle has been passed to a read function or vice versa
 *   - if a message object handle has been passed to a FIFO function or vice versa
 */
#define IO_E_CAN_WRONG_HANDLE           41
/** No more message objects are available. \n
 *  The maximum number of available message objects has been reached.
 *  A single message object is needed to setting up a single message object with the function IO_CAN_ConfigMsg().
 *  When configuring a FIFO buffer with the function IO_CAN_ConfigFIFO() the number of needed message objects equals the size of the FIFO buffer.
 *  (single message objects are joined together to a FIFO buffer)
 */
#define IO_E_CAN_MAX_MO_REACHED         42
/** No more message handles are available. \n
 *  The maximum number of message handles has been reached.
 *  A message handle is generated every time the function IO_CAN_ConfigMsg() or IO_CAN_ConfigFIFO() is called without returning an error.
 */
#define IO_E_CAN_MAX_HANDLES_REACHED    43
/** The FIFO buffer is full. \n
 *  When reading: The respective FIFO buffer is full, but no data has been lost. \n
 *  When writing: The data has not been accepted by the driver. The application needs to transmit it when there is enough space in the buffer.
 */
#define IO_E_CAN_FIFO_FULL              44
/** No new data is available. \n
 *  This error is returned if no CAN frame has been received since the last successful read.
 */
#define IO_E_CAN_OLD_DATA               45
/** The CAN node is in error passive state. */
#define IO_E_CAN_ERROR_PASSIVE          46
/** The CAN node is in bus off state. \n
 *  The bus-off recovery sequence is triggered by calling the function IO_CAN_MsgStatus() or IO_CAN_FIFOStatus()
 */
#define IO_E_CAN_BUS_OFF                47
/** The received data is invalid - Read again to get the valid data. */
#define IO_E_CAN_INVALID_DATA           48
/** Power IC has reported an error. */
#define IO_E_POWER_IC                   50
/** Invalid address range. \n
 *  This error is reported if read or write operations are requested for non-existent EEPROM addresses.
 */
#define IO_E_EEPROM_RANGE               60
/** Buffer overrun. \n
 *  An internal SPI buffer has reported an overrun, data was lost.
 */
#define IO_E_EEPROM_BUFFER_FULL         61
/** The UART software buffer is full. \n
 *  When receiving: Too much data has been received since the last successful read operation - data has been lost. \n
 *  When transmitting: The given data does not fit into the buffer, data has been rejected. Try again when there is more space in the buffer.
 */
#define IO_E_UART_BUFFER_FULL           70
/** A UART software buffer is empty. \n
 *  This error code is used internally by the UART driver. It is not reported by any API function.
 */
#define IO_E_UART_BUFFER_EMPTY          71
/** Overflow in the UART hardware buffer. \n
 *  The hardware buffer reported an overflow. This happens if to much data has been received between two consecutive SW cycles. \n
 *  To avoid this problem the application can call the function IO_UART_Task() at any time.
 *  This function copies the data from the hardware buffer to the software buffer.
 */
#define IO_E_UART_OVERFLOW              72
/** UART parity error. \n
 *  The received parity bit doesn't match the calculated one.
 */
#define IO_E_UART_PARITY                73
/** Maximum number of SPI devices reached. \n
 *  This error code is used internally by the SPI driver. It is not reported by any API function.
 */
#define IO_E_SPI_MAX_DEV_REACHED        80
/** SPI hardware buffer is full. \n
 *  This error code is used internally by the SPI driver. It is not reported by any API function.
 */
#define IO_E_SPI_BUFFER_FULL            81
/** ADC conversion has not been finished yet.
 *  Not all ADC channels which have been configured have finished the conversion.
 */
#define IO_E_ADC_NOT_FINISH             90
/** A capture error occurred on a timer channel \n
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================== =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ------------------------------ ------------------------------------------- ------------------------------ ----------------
 *  PWM output with timer \n     measured signal period is too small                             < 1.6us between edges       max. frequency is              n.a.                                        lower signal frequency \n\n    non-safety
 *  feedback Input                                                                                                           10kHz (restricted                                                          Debounce Time: none
 *                                                                                                                           by low pass filter)
 * --------------------------- --------------------------------------------------------------- --------------------------- ------------------------------ ------------------------------------------- ------------------------------ ----------------
 *  Digital Timer Input          required amount of signal edges couldn't be captured            < timer resolution          each cycle                     n.a.                                        wait as long as the signal     non-safety
 *                                                                                               (timer_res)                 (if function is called each                                                period lasts and/or reduce
 *                                                                                                                           cycle)                                                                     value of capture_count \n\n
 *                                                                                                                                                                                                      Debounce Time: none
 * =========================== =============================================================== =========================== ============================== =========================================== ============================== ================
 * \endverbatim
 *
 *  \par PWM output with timer feedback and current feedback in digital timer input mode:
 *  This error is reported if two edges of the measured signal are too close to each other,
 *  and the internal timer cannot measure the time difference anymore (for example spikes caused
 *  by the switching of inductive loads in electric motors).
 *
 *  \par PWM output with timer feedback in digital timer input mode:
 *  This error is reported if two edges of the measured signal are too close to each other,
 *  and the internal timer cannot measure the time difference anymore (for example spikes caused
 *  by the switching of inductive loads in electric motors).
 *
 *  \par Digital Timer input:
 *  This error is reported if two edges of the measured signal are too close to each other,
 *  and the internal timer cannot measure the time difference anymore (for example spikes caused
 *  by the switching of inductive loads in electric motors).
 *
 *  \remarks
 *  timer_res = 0.2us -> max. period of input signal to measure =  3.34s \n
 *  timer_res = 0.4us -> max. period of input signal to measure =  6.68s \n
 *  timer_res = 0.8us -> max. period of input signal to measure = 13.36s \n
 *  timer_res = 1.6us -> max. period of input signal to measure = 26.73s \n
 *  timer_res = 3.2us -> max. period of input signal to measure = 53.47s
 */
#define IO_E_PWD_CAPTURE_ERROR         100
/** The timer channel has not yet finished the measurement. \n
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value        Detection / Debounce Time      Side Conditions                           Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital Timer Input          required amount of signal edges couldn't be captured            parameter mode = 0: min 4      each cycle                     n.a.                                      wait as long as the signal     non-safety
 *                                                                                               edges\n                        (if function is called each                                              period lasts \n\n
 *                                                                                               parameter mode = 1: min 4      cycle)                                                                   Debounce Time: none
 *                                                                                               edges\n
 *                                                                                               parameter mode = 2: min 6
 *                                                                                               edges\n
 *                                                                                               parameter mode = 3: min 8
 *                                                                                               edges\n
 *                                                                                               parameter mode = 4: min 10
 *                                                                                               edges\n
 *                                                                                               parameter mode = 5: min 12
 *                                                                                               edges\n
 *                                                                                               parameter mode = 6: min 14
 *                                                                                               edges\n
 *                                                                                               parameter mode = 7: min 16
 *                                                                                               edges\n
 *                                                                                               parameter mode = 8: min 18
 *                                                                                               edges\n
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital Timer Input \n       required amount of signal edges couldn't be captured            min 2 edges                   104ms                          n.a.                                      a completely captured          non-safety
 *  on PWM outputs                                                                                                                                                                                      period \n\n
 *                                                                                                                                                                                                      Debounce Time: none
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par Digital timer input in digital timer input mode:
 *  Depending on the configuration a certain number of signal edges are required for a timing measurement.
 *  This error is reported if not all edges have been captured.
 *  If the task function is called multiple times during a period of the signal which shall be measured,
 *  the function will return #IO_E_OK only if valid data is available, otherwise it returns this error code.\n
 *  This error is also reported if no edges have been captured at all.
 *
 *  \par Digital timer input on PWM outputs:
 *  This error is reported if not all edges have been captured.
 *  If the task function is called multiple times during a period of the signal which shall be measured,
 *  the function will return #IO_E_OK only if valid data is available, otherwise it returns this error code.\n
 *  This error is also reported if no edges have been captured at all.
 *
 */
#define IO_E_PWD_NOT_FINISHED          101
/** An open load (or short to battery) condition has been detected.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  PWM output \n                high level on feedback channel \n                               4.2V..Ubat                  104ms                        none.                                       a completely captured PWM      non-safety
 *  with timer feedback          (no timing measurement possible)                                                                                                                                     period \n (3 edges) \n\n
 *                                                                                                                                                                                                    Debounce Time: none
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par PWM outputs with timer feedback:
 *  For PWM outputs with timer feedback, this error is reported if the output signal cannot be measured via the timer feedback
 *  (i. e. the level on the feedback channel is high due to a short to battery or due to a open load ).
 *  This means that no edges could be captured on the timer feedback channel within a timeout of 104 ms. \n
 *  The error condition is reset as soon as the timer was able to capture a complete PWM period (3 edges).
 *  This I/O type cannot distinguish between open load and short to UBAT.
 *
 */
#define IO_E_PWD_OPEN_LOAD             102
/** A short circuit condition has been detected.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  PWM output \n                low level on feedback channel \n                                -0.5V..1.8V                 104ms                       PWM power and power stages are enabled.      a completely captured PWM      non-safety
 *  with timer feedback          (no timing measurement possible) \n                                                                                                                                  period \n (3 edges) \n\n
  *                                                                                                                                                                                                   Debounce Time: none
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par PWM outputs with timer feedback:
 *  For PWM outputs with timer feedback, this error is reported if the output signal cannot be measured via the timer feedback
 *  i. e. the level on the feedback channel is low (ground level)
 *  This means that no edges could be captured on the timer feedback channel within a timeout of 104ms. \n
 *  The error condition is reset as soon as the timer was able to capture a complete PWM period (3 edges).
 *
 *  \remarks
 *  No short circuit detection is available if
 *   - the safety switches have been disabled
 *   - the power stages have been disabled
 *
 *  \remarks
 *  Refer to the \ref IO_POWER.h "ECU power functions" for further details.
 *
 */
#define IO_E_PWD_SHORT_CIRCUIT         103
/** A timer overflow occurred.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital Timer Input          signal period to measure is too long                             see Remarks                65535(16bit timer) *          n.a.                                        error code appears only        non-safety
 *                                                                                                                           255(8bit overflow) *                                                      one cycle
 *                                                                                                                           timer_res
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par Digital timer input in digital timer input mode:
 *  This error code appears when the 24bit timer overflows.
 *  The time depends on the timer resolution which can be configured with the respective initialization function.
 *
 *  \remarks
 *  timer_res = 0.2us -> max. period of input signal to measure =  3.34s \n
 *  timer_res = 0.4us -> max. period of input signal to measure =  6.68s \n
 *  timer_res = 0.8us -> max. period of input signal to measure = 13.36s \n
 *  timer_res = 1.6us -> max. period of input signal to measure = 26.73s \n
 *  timer_res = 3.2us -> max. period of input signal to measure = 53.47s
 */
#define IO_E_PWD_TIMER_OVERFLOW        104
/** A short circuit to UBAT or open connection has been detected.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital Timer Input \n       high level \n                                                   4.2V..Ubat                  104ms                          n.a.                                      a completely captured          non-safety
 *  on PWM outputs               (no timing measurement possible) \n                                                                                                                                  period \n (3 edges) \n\n
 *                                                                                                                                                                                                    Debounce Time: none
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par Digital Timer Input on PWM outputs:
 *  This error is reported if no signal can be measured and the level on the feedback channel is high (battery voltage).
 *  This means that no edges could be captured on the timer channel within a timeout of 104ms. \n
 *  The error condition is reset as soon as the timer was able to capture a complete signal period (3 edges).\n
 *  This I/O type cannot distinguish between open connection and short circuit to UBAT.
 *
 */
#define IO_E_PWD_HIGH_LEVEL            105
/** A short circuit to GND has been detected.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital Timer Input \n       low level \n                                                  -0.5V..1.8V                 104ms                          n.a.                                        a completely captured          non-safety
 *  on PWM outputs               (no timing measurement possible) \n                                                                                                                                  period \n (3 edges) \n\n
 *                                                                                                                                                                                                    Debounce Time: none
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par Digital Timer Input on PWM outputs:
 *  This error is reported if no signal can be measured and the level on the feedback channel is low.
 *  This means that no edges could be captured on the timer channel within a timeout of 104ms. \n
 *  The error condition is reset as soon as the timer was able to capture a complete signal period (3 edges).
 *
 */
#define IO_E_PWD_LOW_LEVEL             106
/** A current out of range was detected (too high)
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital Timer Input \n       current level above 20.5mA \n                                   > 20.5mA                    each cycle                     n.a.                                       < 20.5mA \n\n                  non-safety
 *                                                                                                                           (if function is called each                                               Debounce Time: none
 *                                                                                                                           cycle)
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par Digital Timer Input:
 *  A current level above 20.5mA of a current PWD type sensor (7mA/14mA) was too high => short circuit to UBAT
 *
 */
#define IO_E_PWD_CURRENT_THRESHOLD_HIGH 107
/** A current out of range was detected (too low)
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital Timer Input \n       current level below 4.5mA \n                                    < 4.5mA                    each cycle                     n.a.                                       > 4.5mA  \n\n                   non-safety
 *                                                                                                                          (if function is called each                                               Debounce Time: none
 *                                                                                                                          cycle)
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par Digital Timer Input:
 *  A current level below 4.5mA of a current PWD type sensor (7mA/14mA) was too low => short circuit to GND
 *
 */
#define IO_E_PWD_CURRENT_THRESHOLD_LOW 108
/** An open load condition has been detected on a digital output.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  PWM output \n                command value is low and high level                            4.2V..Ubat                  each cycle \n                  command value is low                        low level on feedback        non-safety
 *  with timer feedback          on feedback channel                                                                        (if function is called each                                                channel or command value is
 *                                                                                                                          cycle)                                                                     high \n\n
 *                                                                                                                                                                                                     Debounce Time: none
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital output \n            command value is low and measured feedback                     >= 3V                       3 * cycletime  \n              command value is low                        measured feedback value      non-safety
 *  with analog feedback         value is higher then 3V                                                                    (if function is called each                                                is lower then 3V or
 *                                                                                                                          cycle)                                                                     command value is high \n\n
 *                                                                                                                                                                                                     Debounce Time: none
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital output \n            command value is high and measured feedback value              < 48mV                      3 * cycletime  \n              command value is high                       measured feedback value      non-safety
 *  on current feedback pins     is lower then 48mV                                                                         (if function is called each                                                is higher then 48mV or
 *                                                                                                                          cycle)                                                                     command value is low  \n\n
 *                                                                                                                                                                                                     Debounce Time: none
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par PWM output with timer feedback in digital output mode:
 *  This error code is reported when the output value is low (power stage is switched off)
 *  and the timer feedback signal is high. \n
 *  For PWM outputs with timer feedback, short circuit to battery and open load conditions cannot be distinguished.
 *
 *  \par Digital outputs with analog feedback:
 *  This error code is reported when the output value is low (power stage is switched off)
 *  and the analog feedback signal is higher then 3V. \n
 *  The error condition is reset as soon as the feedback voltage is below 3V or the output is set to high.
 *  For Digital outputs with analog feedback, short circuit to battery and open load conditions cannot be distinguished.
 *
 *  \par Digital outputs on current feedback pins:
 *  This error code is reported when the output value is high (power stage is switched on)
 *  and the analog feedback signal is lower then 48mV. \n
 *  The error condition is reset as soon as the feedback voltage is above 48mV or the output is set to low.
 *
 */
#define IO_E_DO_OPEN_LOAD              110
/** A short circuit condition to GND has been detected on a digital output.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  PWM output \n                command value is high and low level                             -0.5V..1.8V                 each cycle \n                 command value is high                       high level on feedback       non-safety
 *  with timer feedback          on feedback channel                                                                         (if function is called each                                               channel or command value is
 *                                                                                                                           cycle)                                                                    low \n\n
 *                                                                                                                                                                                                     Debounce Time: none
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital output \n            command value is high and the measured feedback                 < 3V                        3 * cycletime \n              command value is high                       measured feedback value      non-safety
 *  with analog feedback         value is lower than 3V                                                                      (if function is called each                                               is higher than 3V or
 *                                                                                                                           cycle)                                                                    command value is low \n\n
 *                                                                                                                                                                                                     Debounce Time: none
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- ------------------------------ ----------------
 *  Digital output \n            command value is high and the measured feedback value           > 4.88V                     3 * cycletime \n              command value is high                       measured feedback value      non-safety
 *  on current feedback pins     is higher than 4.88V                                                                        (if function is called each                                               is lower than 4.88V or
 *                                                                                                                           cycle)                                                                    command value is low \n\n
 *                                                                                                                                                                                                     Debounce Time: none
 * =========================== =============================================================== =========================== ============================= =========================================== ============================== ================
 * \endverbatim
 *
 *  \par PWM output with timer feedback in digital output mode:
 *  This error code is reported when the command value is high (power stage is switched on)
 *  and the timer feedback signal is low. \n
 *
 *  \par Digital outputs with analog feedback:
 *  This error code is reported when the command value is high (power stage is switched on)
 *  and the analog feedback signal is lower then 3V. \n
 *  The error condition is reset as soon as the feedback voltage is upper 3V or the output is set to low.
 *
 *  \par Digital outputs on current feedback pins:
 *  This error code is reported when the command value is high (power stage is switched on)
 *  and the analog feedback signal is higher than 4.88V. \n
 *  The error condition is reset as soon as the feedback voltage is below 4.88V or the output is set to low.
 *
 */
#define IO_E_DO_SHORT_CIRCUIT          111
/** The diagnosis of the digital output could not be finished.
 *
 * \verbatim[table]
 * =========================== =============================================================== =========================== ============================= =========================================== ================================= ================
 *   I/O Type                    Error Condition                                                 Limit / Threshold Value     Detection / Debounce Time      Side Conditions                             Reset Condition                   Config Type
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- --------------------------------- ----------------
 *  PWM output \n                Error code not reported for this type of outputs                -                           -                             -                                           -                               -
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- --------------------------------- ----------------
 *  Digital output \n            The function \c IO_DO_Set has been called too often             Parameter \c do_value       3 * cycletime \n              Result for output diagnosis is not          The parameter \c do_value       non-safety
 *  with analog feedback         with different values for the parameter \c do_value             has been changed too        (if function is called each   available                                   for the function \c IO_DO_Set
 *                               and the diagnosis of the output could not be finished.          frequently                  cycle)                                                                    has to be stable for at least
 *                                                                                                                                                                                                     to consecutive software cycles
 * --------------------------- --------------------------------------------------------------- --------------------------- ----------------------------- ------------------------------------------- --------------------------------- ----------------
 *  Digital output \n            The function \c IO_DO_Set has been called too often             Parameter \c do_value       3 * cycletime \n              Result for output diagnosis is not          The parameter \c do_value       non-safety
 *  on current feedback pins     with different values for the parameter \c do_value             has been changed too        (if function is called each   available                                   for the function \c IO_DO_Set
 *                               and the diagnosis of the output could not be finished.          frequently                  cycle)                                                                    has to be stable for at least
 *                                                                                                                                                                                                     to consecutive software cycles
 * =========================== =============================================================== =========================== ============================= =========================================== ================================= ================
 * \endverbatim
 *
 *  \par PWM output with timer feedback in digital output mode:
 *  This error code is not available for this type of outputs.
 *
 *  \par Digital outputs with analog feedback:
 *  This error code is reported when the parameter \c do_value of the function \c IO_DO_Set
 *  is changed on two consecutive software cycles.
 *
 *  \par Digital outputs on current feedback pins:
 *  This error code is reported when the parameter \c do_value of the function \c IO_DO_Set
 *  is changed on two consecutive software cycles.
 *
 */
#define IO_E_DO_DIAG_NOT_FINISHED      112
/** Timeout in Watchdog communication \n
 *  The Watchdog CPU didn't answer within the expected time.
 */
#define IO_E_WD_TIMEOUT                120
/** An ivalid frame has been received from the WD CPU */
#define IO_E_WD_INVALID_FRAME          121
/** An internal error occurred in the Watchdog communication driver
 *  This error can be caused by a SW or processor malfunction.
 */
#define IO_E_WD_INTERNAL               122
/** Watchdog version information is not available \n
 *  The Watchdog software version number has not yet been read from the Watchdog CPU
 */
#define IO_E_WD_NO_VERSION             123
/** A value range has been violated in the Watchdog communication. \n
 *  A Byte of a received data frame from the Watchdog CPU has exceeded the allowed range
 */
#define IO_E_WDP_VALUE_RANGE           130
/*@}*/


/*
 * P I N   D E F I N I T I O N S
 */

/**
 * \name Connector Pins
 *
 * Defines for the Connector Pins
 */
/*@{*/
#define IO_PIN_101       53      /**< Pin Nr. 101 */
#define IO_PIN_102       69      /**< Pin Nr. 102 */
#define IO_PIN_103       7       /**< Pin Nr. 103 */
#define IO_PIN_104       5       /**< Pin Nr. 104 */
#define IO_PIN_105       3       /**< Pin Nr. 105 */
#define IO_PIN_106       1       /**< Pin Nr. 106 */
#define IO_PIN_107       49      /**< Pin Nr. 107 */
#define IO_PIN_108       47      /**< Pin Nr. 108 */
#define IO_PIN_109       57      /**< Pin Nr. 109 */
#define IO_PIN_110       55      /**< Pin Nr. 110 */
#define IO_PIN_111       23      /**< Pin Nr. 111 */
#define IO_PIN_112       21      /**< Pin Nr. 112 */
#define IO_PIN_113       19      /**< Pin Nr. 113 */
#define IO_PIN_114       17      /**< Pin Nr. 114 */
#define IO_PIN_115       6       /**< Pin Nr. 115 */
#define IO_PIN_116       4       /**< Pin Nr. 116 */
#define IO_PIN_117       2       /**< Pin Nr. 117 */
#define IO_PIN_118       0       /**< Pin Nr. 118 */
#define IO_PIN_119       48      /**< Pin Nr. 119 */
#define IO_PIN_120       46      /**< Pin Nr. 120 */
#define IO_PIN_121       56      /**< Pin Nr. 121 */
#define IO_PIN_122       54      /**< Pin Nr. 122 */
#define IO_PIN_123       22      /**< Pin Nr. 123 */
#define IO_PIN_124       20      /**< Pin Nr. 124 */
#define IO_PIN_125       18      /**< Pin Nr. 125 */
#define IO_PIN_126       16      /**< Pin Nr. 126 */
#define IO_PIN_127       53      /**< Pin Nr. 127 */
#define IO_PIN_128       53      /**< Pin Nr. 128 */
#define IO_PIN_129       31      /**< Pin Nr. 129 */
#define IO_PIN_130       29      /**< Pin Nr. 130 */
#define IO_PIN_131       27      /**< Pin Nr. 131 */
#define IO_PIN_132       25      /**< Pin Nr. 132 */
#define IO_PIN_133       69      /**< Pin Nr. 133 */
#define IO_PIN_134       69      /**< Pin Nr. 134 */
#define IO_PIN_135       37      /**< Pin Nr. 135 */
#define IO_PIN_136       36      /**< Pin Nr. 136 */
#define IO_PIN_137       15      /**< Pin Nr. 137 */
#define IO_PIN_138       13      /**< Pin Nr. 138 */
#define IO_PIN_139       11      /**< Pin Nr. 139 */
#define IO_PIN_140       9       /**< Pin Nr. 140 */
#define IO_PIN_141       30      /**< Pin Nr. 141 */
#define IO_PIN_142       28      /**< Pin Nr. 142 */
#define IO_PIN_143       26      /**< Pin Nr. 143 */
#define IO_PIN_144       24      /**< Pin Nr. 144 */
#define IO_PIN_145       69      /**< Pin Nr. 145 */
#define IO_PIN_146       69      /**< Pin Nr. 146 */
#define IO_PIN_147       37      /**< Pin Nr. 147 */
#define IO_PIN_148       36      /**< Pin Nr. 148 */
#define IO_PIN_149       14      /**< Pin Nr. 149 */
#define IO_PIN_150       12      /**< Pin Nr. 150 */
#define IO_PIN_151       10      /**< Pin Nr. 151 */
#define IO_PIN_152        8      /**< Pin Nr. 152 */

#define IO_PIN_253       45      /**< Pin Nr. 253 */
#define IO_PIN_254       43      /**< Pin Nr. 254 */
#define IO_PIN_255       41      /**< Pin Nr. 255 */
#define IO_PIN_256       39      /**< Pin Nr. 256 */
#define IO_PIN_257       67      /**< Pin Nr. 257 */
#define IO_PIN_258       63      /**< Pin Nr. 258 */
#define IO_PIN_259       65      /**< Pin Nr. 259 */
#define IO_PIN_260       44      /**< Pin Nr. 260 */
#define IO_PIN_261       42      /**< Pin Nr. 261 */
#define IO_PIN_262       40      /**< Pin Nr. 262 */
#define IO_PIN_263       38      /**< Pin Nr. 263 */
#define IO_PIN_264       66      /**< Pin Nr. 264 */
#define IO_PIN_265       64      /**< Pin Nr. 265 */
#define IO_PIN_266       62      /**< Pin Nr. 266 */
#define IO_PIN_267       35      /**< Pin Nr. 267 */
#define IO_PIN_268       33      /**< Pin Nr. 268 */
#define IO_PIN_269       51      /**< Pin Nr. 269 */
#define IO_PIN_270       69      /**< Pin Nr. 270 */
#define IO_PIN_271       52      /**< Pin Nr. 271 */
#define IO_PIN_272       59      /**< Pin Nr. 272 */
#define IO_PIN_273       61      /**< Pin Nr. 273 */
#define IO_PIN_274       34      /**< Pin Nr. 274 */
#define IO_PIN_275       32      /**< Pin Nr. 275 */
#define IO_PIN_276       69      /**< Pin Nr. 276 */
#define IO_PIN_277       69      /**< Pin Nr. 277 */
#define IO_PIN_278       68      /**< Pin Nr. 278 */
#define IO_PIN_279       60      /**< Pin Nr. 279 */
#define IO_PIN_280       58      /**< Pin Nr. 280 */
/*@}*/

/* Should be used for the ADC 5V redundant channel, if a channel should be configured
 * safety critical without a redundant ADC channel
 */
#define IO_ADC_NON_REDUNDANT            0

/**
 * \name Internal Pins
 *
 * These Pins are not available on the ECU Connector.
 * They are needed for internal control or monitoring.
 */
/*@{*/
#define IO_INT_PIN_TEMP             50  /**< Pin for board temperature          */
#define IO_INT_PWM_POWER            70  /**< Pin for PWM power enable           */
#define IO_INT_POWERSTAGE_ENABLE    53  /**< Pin for internal powerstage enable */
#define IO_INT_PIN_REF_2V5          54  /**< internal 2.5V reference voltage    */
#define IO_INT_PIN_UART_CH0         55  /**< internal for UART Channel0 (RS232) */
#define IO_INT_PIN_UART_CH1         56  /**< internal for UART Channel1 (LIN)   */
#define IO_INT_PIN_UART_CH2         57  /**< internal for UART Channel2 (MM)    */
#define IO_INT_PIN_CAN_CH0          58  /**< internal for CAN Channel0          */
#define IO_INT_PIN_CAN_CH1          59  /**< internal for CAN Channel1          */
#define IO_INT_PIN_EEPROM           60  /**< internal for EEPROM                */
#define IO_INT_PIN_POWER            61  /**< internal for Power IC              */
#define IO_INT_PIN_RTC              62  /**< internal for Realtime Clock        */
#define IO_INT_PIN_DRIVER           63  /**< internal for driver                */
#define IO_INT_PIN_PERIODIC         64  /**< internal for periodic timer        */
#define IO_INT_PIN_CAN_CH2          65  /**< internal for CAN Channel0          */
#define IO_INT_PIN_CAN_CH3          66  /**< internal for CAN Channel1          */

/*@}*/

/* Only the relevant pin must be configured (without GND, UB ...) in
 * the table io_pin_tb
 */
#define IO_PIN_MAX                  67

#define IO_UART_TYPE                20
#define IO_CAN_TYPE                 21
#define IO_EEPROM_TYPE              22
#define IO_POWER_TYPE               23
#define IO_RTC_TYPE                 24
#define IO_DRIVER_TYPE              25
#define IO_PERIODIC_TYPE            26


/*
 * D E V I C E S
 */

/* maximum items for every group */
#define IO_PWM_PWD_MAX              8
#define IO_ADC_DO_MAX               12
#define IO_ADC_5V_MAX               8
#define IO_ADC_VAR_MAX              8
#define IO_ADC_MAX                  18
#define IO_ADC_SENSOR_SUPPLY_MAX    3
#define IO_ADC_CUR_MAX              4
#define IO_DI_MAX                   9
#define IO_DO_SHIFT_MAX             12
#define IO_DO_MAX                   8
#define IO_PWD_MAX                  4
#define IO_UART_MAX_CHANNELS        3


/**
 * \name ADC configurable by SW inputs
 * \anchor dev_adc_conf
 *
 * These pins can be used as ADC 5V inputs.
 * They can be configured by software for
 *      - ratiometric (0 .. 5V)
 *      - resistive or
 *      - current measurement (0 .. 20mA)
 *
 * These pins do not have an alternate function.
 *
 */
/*@{*/
#define IO_ADC_5V_00            IO_PIN_152      /**< no alternate function */
#define IO_ADC_5V_01            IO_PIN_140      /**< no alternate function */
#define IO_ADC_5V_02            IO_PIN_151      /**< no alternate function */
#define IO_ADC_5V_03            IO_PIN_139      /**< no alternate function */
#define IO_ADC_5V_04            IO_PIN_150      /**< no alternate function */
#define IO_ADC_5V_05            IO_PIN_138      /**< no alternate function */
#define IO_ADC_5V_06            IO_PIN_149      /**< no alternate function */
#define IO_ADC_5V_07            IO_PIN_137      /**< no alternate function */
/*@}*/
#define IO_ADC_5V_TYPE          1

/**
 * \name ADC variable range inputs
 * \anchor dev_adc_var
 *
 * These pins can be used as ADC inputs.
 * The range can be configured by SW for 2 groups:
 *      - IO_ADC_VAR_00 .. IO_ADC_VAR_03
 *      - IO_ADC_VAR_04 .. IO_ADC_VAR_07
 *
 * The following ranges are possible:
 *      - 0 .. 5V: IO_ADC_RANGE_5V
 *      - 0 .. 10V: IO_ADC_RANGE_10V
 *      - 0 .. 15V: IO_ADC_RANGE_15V
 *      - 0 .. 20V: IO_ADC_RANGE_20V
 *      - 0 .. 25V: IO_ADC_RANGE_25V
 *      - 0 .. 30V: IO_ADC_RANGE_30V
 *      - 0 .. 32V: IO_ADC_RANGE_32V
 *
 * These pins do not have an alternate function.
 *
 */
/*@{*/
#define IO_ADC_VAR_00           IO_PIN_126      /**< no alternate function */
#define IO_ADC_VAR_01           IO_PIN_114      /**< no alternate function */
#define IO_ADC_VAR_02           IO_PIN_125      /**< no alternate function */
#define IO_ADC_VAR_03           IO_PIN_113      /**< no alternate function */
#define IO_ADC_VAR_04           IO_PIN_124      /**< no alternate function */
#define IO_ADC_VAR_05           IO_PIN_112      /**< no alternate function */
#define IO_ADC_VAR_06           IO_PIN_123      /**< no alternate function */
#define IO_ADC_VAR_07           IO_PIN_111      /**< no alternate function */
/*@}*/
#define IO_ADC_VAR_TYPE         2


/**
 * \name ADC inputs
 * \anchor dev_adc_in
 *
 * These pins can be used as ADC inputs.
 * The range is 0 .. 5V.
 *
 * By default these pins are configured for an alternate function.
 */
/*@{*/
#define IO_ADC_00               IO_PIN_144          /**< main function: \ref dev_dig_out_4A "4A digital highside output" */
#define IO_ADC_01               IO_PIN_132          /**< main function: \ref dev_dig_out_4A "4A digital highside output" */
#define IO_ADC_02               IO_PIN_143          /**< main function: \ref dev_dig_out_4A "4A digital highside output" */
#define IO_ADC_03               IO_PIN_131          /**< main function: \ref dev_dig_out_4A "4A digital highside output" */
#define IO_ADC_04               IO_PIN_142          /**< main function: \ref dev_dig_out_4A "4A digital highside output" */
#define IO_ADC_05               IO_PIN_130          /**< main function: \ref dev_dig_out_4A "4A digital highside output" */
#define IO_ADC_06               IO_PIN_141          /**< main function: \ref dev_dig_out_4A "4A digital highside output" */
#define IO_ADC_07               IO_PIN_129          /**< main function: \ref dev_dig_out_4A "4A digital highside output" */
#define IO_ADC_08               IO_PIN_275          /**< main function: \ref dev_cplx_timer_in "digital timer input" \ref dev_dig_in_pwd "digital timer input" */
#define IO_ADC_09               IO_PIN_268          /**< main function: \ref dev_cplx_timer_in "digital timer input" \ref dev_dig_in_pwd "digital timer input" */
#define IO_ADC_10               IO_PIN_274          /**< main function: \ref dev_cplx_timer_in "digital timer input" \ref dev_dig_in_pwd "digital timer input" */
#define IO_ADC_11               IO_PIN_267          /**< main function: \ref dev_cplx_timer_in "digital timer input" \ref dev_dig_in_pwd "digital timer input" */
#define IO_ADC_SENSOR_SUPPLY_0  IO_PIN_136          /**< no main function, feedback input for sensor supply 0 */
#define IO_ADC_SENSOR_SUPPLY_1  IO_PIN_135          /**< no main function, feedback input for sensor supply 1*/
#define IO_BOARD_TEMP           IO_INT_PIN_TEMP     /**< no main function, feedback input for board temperature */
#define IO_SENSOR_SUPPLY_VAR    IO_PIN_269          /**< no main function, feedback input for variable sensor supply */
#define IO_ADC_UBAT             IO_PIN_101          /**< no main function, feedback input for battery voltage */
#define IO_ADC_REF_2V5          IO_INT_PIN_REF_2V5  /**< no main function, feedback input for 2.5V reference voltage */
/*@}*/
#define IO_ADC_TYPE             3
#define IO_ADC_GROUP0_TYPE      40
#define IO_ADC_GROUP1_TYPE      41

/**
 * \name ADC inputs for PWM Current Feedback
 * \anchor dev_adc_cm
 *
 * These pins can be used as ADC current feedback inputs.
 * Therefore the load needs to be connected between a PWM output
 * (IO_PWM_00 .. IO_PWM_07) and one of the following pins.
 *
 * As an alternate function they can be used as \ref dev_dig_out_2A "2A digital lowside output".
 *
 */
/*@{*/
#define IO_ADC_CUR_00           IO_PIN_120      /**< alternate function: \ref dev_dig_out_2A "2A digital lowside output" */
#define IO_ADC_CUR_01           IO_PIN_108      /**< alternate function: \ref dev_dig_out_2A "2A digital lowside output" */
#define IO_ADC_CUR_02           IO_PIN_119      /**< alternate function: \ref dev_dig_out_2A "2A digital lowside output" */
#define IO_ADC_CUR_03           IO_PIN_107      /**< alternate function: \ref dev_dig_out_2A "2A digital lowside output" */
/*@}*/
#define IO_ADC_CUR_TYPE         4


/**
 * \name PWM Outputs
 * \anchor dev_pwm_out
 *
 * These pins can be used to generate a pulse width modulated (PWM) output
 * with a load of max. 2A.
 *
 * As an alternate function they can be used as \ref dev_timer_in "digital timer inputs".
 *
 */
/*@{*/
#define IO_PWM_00               IO_PIN_118      /**< alternate function: \ref dev_timer_in "digital timer input" \ref dev_dig_in "digital input" */
#define IO_PWM_01               IO_PIN_106      /**< alternate function: \ref dev_timer_in "digital timer input" \ref dev_dig_in "digital input" */
#define IO_PWM_02               IO_PIN_117      /**< alternate function: \ref dev_timer_in "digital timer input" \ref dev_dig_in "digital input" */
#define IO_PWM_03               IO_PIN_105      /**< alternate function: \ref dev_timer_in "digital timer input" \ref dev_dig_in "digital input" */
#define IO_PWM_04               IO_PIN_116      /**< alternate function: \ref dev_timer_in "digital timer input" \ref dev_dig_in "digital input" */
#define IO_PWM_05               IO_PIN_104      /**< alternate function: \ref dev_timer_in "digital timer input" \ref dev_dig_in "digital input" */
#define IO_PWM_06               IO_PIN_115      /**< alternate function: \ref dev_timer_in "digital timer input" \ref dev_dig_in "digital input" */
#define IO_PWM_07               IO_PIN_103      /**< alternate function: \ref dev_timer_in "digital timer input" \ref dev_dig_in "digital input" */
/*@}*/
#define IO_PWM_TYPE             5
#define IO_PWM_DUAL_TYPE_PRI    6
#define IO_PWM_DUAL_TYPE_SEC    30

/**
 * \name Digital Timer Inputs
 * \anchor dev_timer_in
 *
 * These pins can be used as digital timer inputs to
 * decode PWM signals.
 * They can measure either the frequency or the pulse duration.
 *
 * The main function of these pins is the usage as \ref dev_pwm_out "PWM Output".
 *
 */
/*@{*/
#define IO_PWD_00               IO_PIN_118      /**< main function: \ref dev_pwm_out "PWM Output" */
#define IO_PWD_01               IO_PIN_106      /**< main function: \ref dev_pwm_out "PWM Output" */
#define IO_PWD_02               IO_PIN_117      /**< main function: \ref dev_pwm_out "PWM Output" */
#define IO_PWD_03               IO_PIN_105      /**< main function: \ref dev_pwm_out "PWM Output" */
#define IO_PWD_04               IO_PIN_116      /**< main function: \ref dev_pwm_out "PWM Output" */
#define IO_PWD_05               IO_PIN_104      /**< main function: \ref dev_pwm_out "PWM Output" */
#define IO_PWD_06               IO_PIN_115      /**< main function: \ref dev_pwm_out "PWM Output" */
#define IO_PWD_07               IO_PIN_103      /**< main function: \ref dev_pwm_out "PWM Output" */
/*@}*/
#define IO_PWD_LB_FREQ_TYPE     7
#define IO_PWD_LB_PULSE_TYPE    50

/**
 * \name Complex Digital Timer Inputs
 * \anchor dev_cplx_timer_in
 *
 * These pins can be used as digital timer inputs to decode
 *      - PWM signals,
 *      - signals from incremental encoders or
 *      - signals from ABS sensors.
 *
 * As PWM decoder the frequency and pulse width can be measured at the same time.
 *
 * As an alternate function they can be used as \ref dev_adc_in "ADC inputs".
 *
 */
/*@{*/
#define IO_PWD_08               IO_PIN_275      /**< alternate function: \ref dev_adc_in "ADC input" \ref dev_dig_in_pwd "digital timer input" */
#define IO_PWD_09               IO_PIN_268      /**< alternate function: \ref dev_adc_in "ADC input" \ref dev_dig_in_pwd "digital timer input" */
#define IO_PWD_10               IO_PIN_274      /**< alternate function: \ref dev_adc_in "ADC input" \ref dev_dig_in_pwd "digital timer input" */
#define IO_PWD_11               IO_PIN_267      /**< alternate function: \ref dev_adc_in "ADC input" \ref dev_dig_in_pwd "digital timer input" */
/*@}*/
#define IO_PWD_INC_TYPE         8
#define IO_PWD_COMP_TYPE        9
#define IO_PWD_COUNT_TYPE       60

/**
 * \name Digital Inputs
 * \anchor dev_dig_in
 *
 * These pins can be used as digital inputs.
 *
 * These pins do not have an alternate function.
 *
 */
/*@{*/
#define IO_DI_00                IO_PIN_263      /**< no alternate function */
#define IO_DI_01                IO_PIN_256      /**< no alternate function */
#define IO_DI_02                IO_PIN_262      /**< no alternate function */
#define IO_DI_03                IO_PIN_255      /**< no alternate function */
#define IO_DI_04                IO_PIN_261      /**< no alternate function */
#define IO_DI_05                IO_PIN_254      /**< no alternate function */
#define IO_DI_06                IO_PIN_260      /**< no alternate function */
#define IO_DI_07                IO_PIN_253      /**< no alternate function */
#define IO_DI_K15               IO_PIN_271      /**< no alternate function */
/*@}*/
#define IO_DI_TYPE              10

/**
 * \name Digital Inputs
 * \anchor dev_dig_in_pwm
 *
 * These pins can be used as digital inputs.
 *
 * The main function of these pins is the usage as \ref dev_pwm_out "PWM Output".
 *
 */
/*@{*/
#define IO_DI_08                IO_PIN_118      /**< main function: \ref dev_pwm_out "PWM Output" \ref dev_timer_in "digital timer input" */
#define IO_DI_09                IO_PIN_106      /**< main function: \ref dev_pwm_out "PWM Output" \ref dev_timer_in "digital timer input" */
#define IO_DI_10                IO_PIN_117      /**< main function: \ref dev_pwm_out "PWM Output" \ref dev_timer_in "digital timer input" */
#define IO_DI_11                IO_PIN_105      /**< main function: \ref dev_pwm_out "PWM Output" \ref dev_timer_in "digital timer input" */
#define IO_DI_12                IO_PIN_116      /**< main function: \ref dev_pwm_out "PWM Output" \ref dev_timer_in "digital timer input" */
#define IO_DI_13                IO_PIN_104      /**< main function: \ref dev_pwm_out "PWM Output" \ref dev_timer_in "digital timer input" */
#define IO_DI_14                IO_PIN_115      /**< main function: \ref dev_pwm_out "PWM Output" \ref dev_timer_in "digital timer input" */
#define IO_DI_15                IO_PIN_103      /**< main function: \ref dev_pwm_out "PWM Output" \ref dev_timer_in "digital timer input" */
/*@}*/
#define IO_DI_PWD_LB_TYPE       11

/**
 * \name Digital Inputs
 * \anchor dev_dig_in_pwd
 *
 * These pins can be used as digital inputs.
 *
 * The main function of these pins is the usage as \ref dev_cplx_timer_in "digital timer input".
 *
 */
/*@{*/
#define IO_DI_16                IO_PIN_275      /**< main function: \ref dev_cplx_timer_in "digital timer input"  */
#define IO_DI_17                IO_PIN_268      /**< main function: \ref dev_cplx_timer_in "digital timer input"  */
#define IO_DI_18                IO_PIN_274      /**< main function: \ref dev_cplx_timer_in "digital timer input"  */
#define IO_DI_19                IO_PIN_267      /**< main function: \ref dev_cplx_timer_in "digital timer input"  */
/*@}*/
#define IO_DI_PWD_TYPE          12

/**
 * \name Digital Outputs
 * \anchor dev_dig_out_4A
 *
 * These pins can be used as digital high-side
 * outputs with a maximum load of 4A
 *
 * As an alternate function they can be used as \ref dev_adc_in "ADC inputs".
 *
 */
/*@{*/
#define IO_DO_00                IO_PIN_144      /**< alternate function: \ref dev_adc_in "ADC input" */
#define IO_DO_01                IO_PIN_132      /**< alternate function: \ref dev_adc_in "ADC input" */
#define IO_DO_02                IO_PIN_143      /**< alternate function: \ref dev_adc_in "ADC input" */
#define IO_DO_03                IO_PIN_131      /**< alternate function: \ref dev_adc_in "ADC input" */
#define IO_DO_04                IO_PIN_142      /**< alternate function: \ref dev_adc_in "ADC input" */
#define IO_DO_05                IO_PIN_130      /**< alternate function: \ref dev_adc_in "ADC input" */
#define IO_DO_06                IO_PIN_141      /**< alternate function: \ref dev_adc_in "ADC input" */
#define IO_DO_07                IO_PIN_129      /**< alternate function: \ref dev_adc_in "ADC input" */
/*@}*/

/**
 * \name Digital Outputs
 * \anchor dev_dig_out_2A
 *
 * These pins can be used as digital low-side
 * outputs with a maximum load of 2A
 *
 * The main function of these pins is the usage as \ref dev_adc_cm "PWM current feedback channels".
 *
 */
/*@{*/
#define IO_DO_08                IO_PIN_120      /**< main function: \ref dev_adc_cm "PWM current feedback channel" */
#define IO_DO_09                IO_PIN_108      /**< main function: \ref dev_adc_cm "PWM current feedback channel" */
#define IO_DO_10                IO_PIN_119      /**< main function: \ref dev_adc_cm "PWM current feedback channel" */
#define IO_DO_11                IO_PIN_107      /**< main function: \ref dev_adc_cm "PWM current feedback channel" */
/*@}*/

/**
 * \name Digital Outputs
 * \anchor dev_dig_out_2A_pwm
 *
 * These pins can be used as digital high-side
 * outputs with a maximum load of 2A
 *
 * The main function of these pins is the usage as \ref dev_pwm_out "PWM Outputs".
 *
 */
/*@{*/
#define IO_DO_12                IO_PIN_118      /**< main function: \ref dev_pwm_out "PWM Outputs" */
#define IO_DO_13                IO_PIN_106      /**< main function: \ref dev_pwm_out "PWM Outputs" */
#define IO_DO_14                IO_PIN_117      /**< main function: \ref dev_pwm_out "PWM Outputs" */
#define IO_DO_15                IO_PIN_105      /**< main function: \ref dev_pwm_out "PWM Outputs" */
#define IO_DO_16                IO_PIN_116      /**< main function: \ref dev_pwm_out "PWM Outputs" */
#define IO_DO_17                IO_PIN_104      /**< main function: \ref dev_pwm_out "PWM Outputs" */
#define IO_DO_18                IO_PIN_115      /**< main function: \ref dev_pwm_out "PWM Outputs" */
#define IO_DO_19                IO_PIN_103      /**< main function: \ref dev_pwm_out "PWM Outputs" */
/*@}*/
#define IO_DO_TYPE              13


#endif /* _IO_CONSTANTS_H */
