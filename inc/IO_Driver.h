/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_Driver.h
 *
 * \brief High level interface to IO Driver
 *
 *      The IO Driver high level interface provides a general
 *      initialization function, a version API and general task functions
 *      which shall wrap the whole user application.
 *
 * \section appl_structure Basic structure of an application
 *
 *      The IO Driver API provides two different types of functions:
 *      - Initialization functions:
 *              These functions are designed to be called once at the
 *              beginning of an application.
 *      - Task functions:
 *              These functions are designed to be called periodically at
 *              runtime.
 *
 *      The function IO_Driver_Init() needs to be the first function, called
 *      during the initialization.
 *
 *      All task functions need to be enclosed by the functions
 *      IO_Driver_TaskBegin() and IO_Driver_TaskEnd()
 *
 *      Example of an application:
 *
 *      \code
 *      void task (void)
 *      {
 *          IO_Driver_TaskBegin();
 *
 *          // User Application
 *          // and calls to driver task functions.
 *
 *          IO_Driver_TaskEnd();
 *      }
 *
 *
 *      void main (void)
 *      {
 *          ubyte4 timestamp
 *
 *          //--------------------------------//
 *          // start of driver initialization //
 *          //--------------------------------//
 *
 *          // IO_Driver_Init() is the first function:
 *          IO_Driver_Init(NULL);  // no safety critical application
 *
 *          //------------------------------//
 *          // end of driver initialization //
 *          //------------------------------//
 *
 *
 *          //--------------------------------------------//
 *          // from now on only task functions are called //
 *          //--------------------------------------------//
 *          while (1){
 *              IO_RTC_StartTime(&timestamp);
 *
 *              task();
 *
 *              while (IO_RTC_GetTimeUS(timestamp) < 5000);
 *          }
 *      }
 *
 *      \endcode
 *
 *      The \c task function is called every 5000us = 5ms. Please
 *      refer to the \ref IO_RTC.h "Real Time Clock" documentation for
 *      details on how to use the RTC functions.
 *
 * \section startup_limits Limitations during startup
 *         During the first cycle the measurement values (eg. ADC, PWD, etc.) are
 *         invalid, which is indicated by the corresponding fresh flag when reading the value.
 *         Starting with the second cycle the actual measurement values are available.
 *
 *         Further limitations for safety controllers:
 *         The safety controllers (\safetycontrollers) perform various tests at startup.
 *         For this reason the measurement values can hold unexpected values during this process.
 *         The application software should wait until the CPU has entered
 *         the Main state before evaluating the input variables.
 *         For details on CPU states and how to retrieve the current state,
 *         refer to \ref DIAG_Functions.h "Functions for ECU diagnostics"
 *
 * \section general_limits General limitations
 *         When the IO Driver is configured as safety critical(safety controllers \safetycontrollers),
 *         all the variables modified by an ISR shall be placed in internal RAM only, otherwise a
 *         safety state could be triggered during the execution of the External RAM Memory check.
 *
 **************************************************************************/

#ifndef _IO_DRIVER_H
#define _IO_DRIVER_H

/**********************************************************************//**
 * \mainpage \family I/O Driver Manual
 *
 * \version 3.6.2
 * \date    16.04.2015
 *
 *
 * This document describes the API of the XC2000 IO Driver.
 *
 * This document applies for all variants of the \family.\n
 * The <b>\ttc50</b> is a family of programmable electronic control units
 * consists of the products
 *  - \ttc50
 *  - \ttc60
 *  - \ttc90
 *  - \ttc94
 *  \if TTC94E
 *  - \ttc94e
 *  \endif
 *
 * Only <em>\safetycontrollers</em> are specifically designed for safety-critical applications.
 * The other variants are subsets of the \ttc94 without the safety features.
 *
 * Throughout this document, the product will be consistently named after
 * the product family \ttc50.
 * Differences between the variants will be explicitly pointed out by referring
 * to the respective product names.
 *
 *
 * The following driver functions are described:
 *      - \ref IO_Driver.h          "General driver items"
 *      - \ref IO_Constants.h       "Driver constants"
 *      - \ref APDB.h               "APDB for boot loader"
 *      - \ref IO_DIO.h             "Driver for digital inputs and outputs (DIO)"
 *      - \ref IO_ADC.h             "Analog to Digital Converter (ADC) driver"
 *      - \ref IO_PWM.h             "Pulse Width Modulation (PWM) driver"
 *      - \ref IO_PWD.h             "Pulse Width Decode (PWD) and digital timer input driver"
 *      - \ref IO_RTC.h             "Real Time Clock (RTC) driver"
 *      - \ref IO_CAN.h             "Controller Area Network (CAN) driver"
 *      - \ref IO_UART.h            "Universal Asynchronous Receiver Transmitter (UART) driver"
 *      - \ref IO_EEPROM.h          "EEPROM driver"
 *      - \ref IO_POWER.h           "Driver for ECU power functions"
 *      - \ref DIAG_Functions.h     "Functions for ECU diagnostics"
 *      - \ref DIAG_Constants.h     "Constants for ECU diagnostics"
 *
 **************************************************************************/

/**********************************************************************//**
 * \page compiler_options Recommended compiler options
 *
 * \section Viper_options Tasking Viper compiler recommended options:
 *
 * - Optimization level = --optimize=3

 * \verbatim[table]
 * ============================= =========================================================================
 *  Optimization levels              Description
 * ----------------------------- -------------------------------------------------------------------------
 *  Level 0 - No optimization     No optimizations are performed. The compiler tries to achieve a 1-to-1
 *                                resemblance between source code and produced code. Expressions are
 *                                evaluated in the order written in the source code, associative and
 *                                commutative properties are not used.
 * ----------------------------- -------------------------------------------------------------------------
 *  Level 1 - Optimize            Enables optimizations that do not affect the debug-ability of the source
 *                                code. Use this level when you encounter problems during debugging your
 *                                source code with optimization level 2.
 * ----------------------------- -------------------------------------------------------------------------
 *  Level 2 - Optimize more       Enables more optimizations to reduce the memory footprint and/or
 *  (default)                     execution time. This is the default optimization level.
 * ----------------------------- -------------------------------------------------------------------------
 *  Level 3 - Optimize most       This is the highest optimization level. Use this level when your
 *                                program/hardware has become too slow to meet your real-time requirements
 * ============================= =========================================================================
 * \endverbatim
 *
 * - Trade-off level = --tradeoff=0
 *
 * Trade-off values range: --tradeoff={0|1|2|3|4}
 *
 * If the compiler uses certain optimizations (option --optimize), you can use this option to specify whether
 * the used optimizations should optimize for more speed (regardless of code size) or for smaller code size
 * (regardless of speed).\n
 * By default the compiler optimizes for code size (--tradeoff=4).\n
 * If you have not specified the option --optimize, the compiler uses the default "Optimize more"
 * optimization. In this case it is still useful to specify a trade-off level.
 *
 **************************************************************************/


 /**********************************************************************//**
 * \page endianness_description Endianness
 *
 * The XC2000 in combination with the Tasking Compiler use the little-endian representation at hardware level.\n
 * The following table shows the different memory organization of the typical 16, 32 or 64-bit words for big-endian and little-endian systems.
 *
 * \verbatim[table]
 * ========= ======================== ==========================
 *  Endian    First byte\n              Last byte\n
 *            (Lowest memory address)  (Highest memory address)
 * --------- ------------------------ --------------------------
    Big       Most significant         Least significant
 * --------- ------------------------ --------------------------
    Little    Least significant        Most significant
 * ========= ======================== ==========================
 * \endverbatim
 *
 **************************************************************************/


 /**********************************************************************//**
 * \page response_times Response times of the \family
 *
 * The <em>response time</em> in this context is the delay between
 * the moment an event occurs on an input of the ECU and the
 * moment the output of the ECU is updated.
 *
 * \section rt_description Description of the response time
 *
 * The response time depends on the following factors:
 *   - Software cycle time
 *   - Type of the input (analog, digital, ...)
 *   - Type of the output (digital, PWM, ...)
 *   - Period and duty cycle (for PWM outputs)
 *
 * The delay between the occurrence of an event on an input and the moment
 * the application SW is able to recognize it is not constant (eg. depends
 * on the relation of the event to the sampling point within the SW cycle).
 * Also the time between an output is set by the application and the output
 * of the ECU is updated can vary.
 *
 * For this reason it is only possible to provide information about the worst
 * and best case response times, giving a window in which the actual
 * response time will be.
 *
 * \section rt_overview Overview of response times
 *
 * The following table gives an overview of the delay times introduced by the different
 * I/O types.
 * \verbatim[table]
 * =============================== ======================== ==========================
 *  IO type                         Best case delay          Worst case delay
 * ------------------------------- ------------------------ --------------------------
 *  Digital input                   no delay                 1 x CT
 * ------------------------------- ------------------------ --------------------------
 *  Analog input                    1 x CT                   2 x CT
 * ------------------------------- ------------------------ --------------------------
 *  Direct digital output \n        no delay                 no delay
 *  (#IO_DO_12 .. #IO_DO_19)
 * ------------------------------- ------------------------ --------------------------
 *  Shift register controlled       2 x CT                   2 x CT
 *  digital output \n
 *  (#IO_DO_00 .. #IO_DO_07,
 *   #IO_DO_08 .. #IO_DO_11)
 * ------------------------------- ------------------------ --------------------------
 *  PWM output                      HT                       PER + HT
 * =============================== ======================== ==========================
 * \endverbatim
 *
 * The following symbols are used in the table above:
 *  - \b CT: cycle time, this is the time of the SW cycle
 *  - \b PER: period time, the period of a PWM output (1/f)
 *  - \b HT: high time, this is the old high time of the PWM output (the one
 *              which has been set before the output is updated)
 *
 * The response time from a specific input to a specific output can be calculated
 * by summing up the values for the respective input and output.\n
 * The following Table lists several examples:
 * \verbatim[table]
 * =============================== =============================== ======================== ==========================
 *  Input type                      Output type                        Best case delay          Worst case delay
 * ------------------------------- ------------------------------- ------------------------ --------------------------
 *  Digital input                   Direct digital output           no delay                 1 x CT
 * ------------------------------- ------------------------------- ------------------------ --------------------------
 *  Digital input                   Shift register controlled       2 x CT                   3 x CT
 *                                  digital output
 * ------------------------------- ------------------------------- ------------------------ --------------------------
 *  Digital input                   PWM output                      HT                       CT + PER + HT
 * ------------------------------- ------------------------------- ------------------------ --------------------------
 *  Analog input                    Direct digital output           1 x CT                   2 x CT
 * ------------------------------- ------------------------------- ------------------------ --------------------------
 *  Analog input                    Shift register controlled       3 x CT                   4 x CT
 *                                  digital output
 * ------------------------------- ------------------------------- ------------------------ --------------------------
 *  Analog input                    PWM output                      CT + HT                  2 x CT + PER + HT
 * =============================== =============================== ======================== ==========================
 * \endverbatim
 *
 * \section rt_justification Justification for the delay times.
 *
 * \par Digital inputs
 *   Digital inputs are usually read (sampled) once every SW cycle. \n
 *   If the event occurs directly before this sampling point, there will be no delay giving the best case. \n
 *   If the event occurs immediately after the sampling point, the SW will see it in the next cycle.
 *   This results in the worst case delay of 1 x CT (one cycle time).
 *
 * \par Analog inputs
 *   The internal ADC triggers a conversion at the end of every SW cycle, so the results are available in the
 *   next SW cycle. This causes an additional delay of one cycle time when compared to the digital inputs. \n
 *   When the event occurs directly before the conversion is triggered, the result will be available in the next round, giving a delay of 1 x CT (one cycle time). \n
 *   If the event occurs directly after the conversion, the event will be taken into account during the following conversion (which is one cycle time later)
 *   and the results will be available in the round after this conversion.
 *   This results in a worst case delay of 2 x CT (two cycle times).
 *
 * \par Direct digital outputs
 *   These outputs are controlled directly by CPU pins, and are therefore updated immediately after the
 *   application requests to do so. This means that for those outputs there is no delay.
 *
 * \par Shift register controlled outputs
 *   These outputs are controlled via a shift register.
 *   The transfer of the new state to the shift register output takes two SW cycles, giving a delay of 2 x CT (two cycle times).
 *
 * \par PWM outputs
 *   The duty cycle of a PWM output is updated on a rising edge of the signal.
 *   The new duty cycle comes to effect after the period of the PWM signal (falling edge).\n
 *   This means if the output is set immediately before the rising edge of the signal,
 *   the output will be set at the end of the PWM period, resulting in a
 *   delay of 1 x HT (one high time of the old duty cycle) \n
 *   If the output is updated immediately after the rising edge of the signal the
 *   duty cycle will be updated one period time later, resulting in the worst case delay of PER + HT (period time + old high time).
 *
 **************************************************************************/

/**************************************************************************
 *
 * I N C L U D E S
 *
 **************************************************************************/

#include "ptypes_xe167.h"
#include "IO_Constants.h"
#include "IO_POWER.h"
#include "IO_UART.h"
#include "IO_EEPROM.h"

/**************************************************************************
 *
 * D E F I N I T I O N S
 *
 **************************************************************************/

/** CPU frequency in MHz */
#if (defined(TTC94E)||defined(TTC60E))
	#define IO_DRIVER_SYSTEM_CLOCK  100  /* MHz */
#else
	#define IO_DRIVER_SYSTEM_CLOCK  80  /* MHz */
#endif
#ifdef SAFETY
extern bool io_driver_use_safety;
#endif

/**************************************************************************
 *
 * D A T A   S T R U C T U R E
 *
 **************************************************************************/

/**
 *
 * \brief Driver Safety Configuration
 *
 * This structure is used to pass the configuration for
 * safety critical application to the IO-Driver.
 *
 */
 typedef struct _io_driver_safety_conf
 {
    ubyte1 glitch_filter_time;      /**<  Only if an error condition persists after
                                          expiration of this time range, an error
                                          reaction is taken. (10..180 [ms])          */
    ubyte2 command_period;          /**<  Time in [us], interval between two
                                          consecutive software cycles.
                                          (1..25000)                                 */
    ubyte1 timeout_count;           /**<  Maximum number of command frame periods
                                          without valid command from the Main CPU:
                                          (1..3)                                     */
    ubyte2 CPU_check_loopcount;     /**<  Number of I/O-state commands after which
                                          the Watchdog CPU verifies the correct function
                                          of the Main CPU diagnosis (1..65535)       */

 } IO_DRIVER_SAFETY_CONF;

/**************************************************************************
 *
 * F U N C T I O N S
 *
 **************************************************************************/

/**********************************************************************//**
 *
 * \brief Global initialization of IO driver.
 *        This function shall be called before any other driver function
 *        (excepting function IO_Driver_GetVersionOfDriver())
 *         - Initializes SPI Devices
 *         - Switches off all power outputs
 *         - Sets the sensor supply
 *         - Initializes the RTC
 *         - Switches on the interrupts of the CPU
 *         - Initializes the measurement of UBAT
 *
 *
 * \param   safety_conf     Configuration for safety critical applications
 *                          Set pointer to \c NULL to disable the safety
 *                          features.
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                    everything fine
 * \retval IO_E_INVALID_PARAMETER     invalid parameter value has been passed
 * \retval IO_E_BUSY                  driver is just initialized
 *
 ***************************************************************************
 *
 * \remarks
 *    If safety_conf != NULL, the application is configured safety critical.
 *    The parameter safety_conf defines the global safety properties.
 *    Then, in other Init functions, like IO_ADC_ChannelInit ... each channel
 *    itself, can be configured safety critical.
 *
 **************************************************************************/
 IO_ErrorType IO_Driver_Init( const IO_DRIVER_SAFETY_CONF * const safety_conf );

/**********************************************************************//**
 *
 * \brief Returns the version number of the driver
 *
 * \param major          Major version
 * \param minor          Minor version
 * \param patchlevel     Patchlevel
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                    everything fine
 * \retval IO_E_NULL_POINTER          null pointer has been passed
 *
 **************************************************************************/
 IO_ErrorType IO_Driver_GetVersionOfDriver( ubyte1 * const major
                                          , ubyte1 * const minor
                                          , ubyte2 * const patchlevel);

/**********************************************************************//**
 *
 * \brief Task function for IO Driver.
 *        This function shall be called at the beginning of the task.
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED
 *
 **************************************************************************/
 IO_ErrorType IO_Driver_TaskBegin(void);

/**********************************************************************//**
 *
 * \brief Task function for IO Driver.
 *        This function shall be called at the end of the task.
 *
 *
 * \return IO_ErrorType
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_ADC_NOT_FINISH          not all ADC channels has can be
 *                                      converted in the last round
 * \retval IO_E_CHANNEL_NOT_CONFIGURED
 *
 **************************************************************************/
 IO_ErrorType IO_Driver_TaskEnd(void);

#endif /* _IO_DRIVER_H */
