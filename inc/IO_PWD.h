/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_PWD.h
 *
 * \brief IO Driver functions for timer input channels.
 *
 *      Contains all service functions for the PWD (Pulse Width Demodulation).
 *      There are two groups of timer inputs available:
 *
 *      - \c IO_PWD_00..\c IO_PWD_07:
 *              Simple timer input. Can be configured to measure
 *              either  frequency or pulse-width. These inputs evaluate only
 *              2 edges (frequency) or 3 edges (pulse-width).
 *      - \c IO_PWD_08..\c IO_PWD_11:
 *              Complex timer inputs. Can be configured to measure
 *              frequency and pulse-width at the same time. Additionally, a voltage
 *              threshold and a pull up/down interface can be configured.
 *              These inputs can accumulate up to 8 pulse samples.
 *              Furthermore these pins can read incremental (relative) encoders.
 *              In this case two inputs are reserved for one incremental encoder
 *              (clock and direction). For current sensors
 *              an additional current range check (7-14mA) is done.
 *
 *  \section pwd_examples PWD code examples
 *
 *      PWD initialization examples:
 *      \code
 *      // setup frequency measurement input
 *      IO_PWD_FreqInit( IO_PWD_00
 *                     , IO_PWD_FALLING_VAR );  // select falling edge as variable
 *
 *      // setup pulse measurement input
 *      IO_PWD_PulseInit( IO_PWD_01
 *                      , IO_PWD_HIGH_TIME );   // measure high time of the signal
 *
 *      // setup complex timer input (frequency and pulse measurement)
 *      IO_PWD_ComplexInit( IO_PWD_08
 *                        , IO_PWD_HIGH_TIME        // measure high time
 *                        , IO_PWD_FALLING_VAR      // select falling edge as variable
 *                        , IO_PWD_RESOLUTION_0_8   // set timer resolution to 0.8us
 *                        , 10                      // set number of accumulations
 *                        , IO_PWD_THRESH_1_25V     // set threshold to 1.25V
 *                        , IO_PWD_PU_10K           // select 10kOhm pull up
 *                        , NULL );                 // not safety critical
 *
 *      // setup incremental input
 *      IO_PWD_IncInit( IO_PWD_11
 *                    , IO_PWD_INC_2_COUNT
 *                    , 0x7FFF                  // set initial value for counter
 *                    , IO_PWD_THRESH_2_5V      // set threshold to 2.5V
 *                    , IO_PWD_PD_1K8           // select 1.8kOhm pull down
 *                    , NULL );                 // not safety critical
 *      \endcode
 *
 *      PWD task examples:
 *      \code
 *      ubyte2 freq_0_val, freq_8_val, inc_11_val;
 *      ubyte4 pulse_1_val, pulse_8_val;
 *
 *      // read frequency value
 *      IO_PWD_FreqGet( IO_PWD_00
 *                    , &freq_0_val );
 *
 *      // read pulse value
 *      IO_PWD_PulseGet( IO_PWD_01
 *                     , &pulse_1_val );
 *
 *      // read complex timer values (frequency and pulse value
 *      IO_PWD_ComplexGet( IO_PWD_08
 *                       , &freq_8_val
 *                       , &pulse_8_val
 *                       , NULL );          // pulse samples not needed
 *
 *      // read incremental counter value
 *      IO_PWD_IncGet ( IO_PWD_11
 *                    , &inc_11_val );
 *      \endcode
 *
 **************************************************************************/

#ifndef _IO_PWD_H
#define _IO_PWD_H
#include "IO_Driver.h"
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
 * \name High- / Low time
 * \anchor pwd_htlt_def
 *
 * Specifies whether the high,low or both(period) time shall be captured
 */
/*@{*/
#define IO_PWD_LOW_TIME                 0   /**< capture the low time of the input signal  */
#define IO_PWD_HIGH_TIME                1   /**< capture the high time of the input signal */
#define IO_PWD_PERIOD_TIME              2   /**< capture the high and low time of the input signal */
/*@}*/


/**
 * \name Variable edge
 * \anchor pwd_var_edge_def
 *
 * Specify the variable edge of the input signal. \n
 * If the rising edge is variable, the frequency is measured between
 * the surrounding falling edges.
 */
/*@{*/
#define IO_PWD_RISING_VAR               2   /**< rising edge of the input signal is the variable one  */
#define IO_PWD_FALLING_VAR              3   /**< falling edge of the input signal is the variable one */
/*@}*/


/**
 * \name Counting mode
 * \anchor pwd_count_mode_def
 *
 * Specify the counting mode of a incremental channel.
 */
/*@{*/
#define IO_PWD_INC_2_COUNT              0x03    /**< count on any edge of the two inputs      */
#define IO_PWD_INC_1_COUNT              0x01    /**< count only on one edge of the two inputs */
/*@}*/

/**
 * \name Threshold voltages
 * \anchor pwd_th_volt_def
 *
 * Specify the input voltage threshold.
 */
/*@{*/
#define IO_PWD_THRESH_1_25V              0   /**< input threshold is 1.25V */
#define IO_PWD_THRESH_2_5V               1   /**< input threshold is 2.5V  */
/*@}*/

/**
 * \name Pull up / down configuration
 * \anchor pwd_th_volt_def
 *
 * Specify the pull up/down resistor for the input
 */
/*@{*/
#define IO_PWD_PU_10K                   0x01    /**< 10kOhm pull up resistor    */
#define IO_PWD_PD_10K                   0x00    /**< 10kOhm pull down resistor  */
#define IO_PWD_PD_1K8                   0x02    /**< 1.8kOhm pull down resistor */
#define IO_PWD_PD_110                   0x04    /**< 100 pull down resistor     */
/*@}*/

/**
 * \name PWD resolution configuration
 * \anchor pwd_res_def
 *
 * Resolution of the timer of a complex timer input.
 * Note: the timing measurement is based on a 16bit timer,
 * therefore the product (65535 * resolution) must be greater than the period
 * that shall be measured
 */
/*@{*/
#define IO_PWD_RESOLUTION_0_2           0x01    /**< resolution is set to 0.2 us */
#define IO_PWD_RESOLUTION_0_4           0x02    /**< resolution is set to 0.4 us */
#define IO_PWD_RESOLUTION_0_8           0x03    /**< resolution is set to 0.8 us */
#define IO_PWD_RESOLUTION_1_6           0x04    /**< resolution is set to 1.6 us */
#define IO_PWD_RESOLUTION_3_2           0x05    /**< resolution is set to 3.2 us */
/*@}*/

/**
 * \name edge count
 * \anchor pwd_count_edge_def
 *
 * Specify on wich edge shall be counted
 */
/*@{*/
#define IO_PWD_RISING_COUNT                1   /**< count on a rising edge  */
#define IO_PWD_FALLING_COUNT               2   /**< count on a falling edge */
#define IO_PWD_BOTH_COUNT                  3   /**< count on both edges     */
/*@}*/

/**
 * \name count direction
 * \anchor pwd_count_dir_def
 *
 * Specify the counting direction
 */
/*@{*/
#define IO_PWD_UP_COUNT                    0   /**< count up  */
#define IO_PWD_DOWN_COUNT                  1   /**< count down */
/*@}*/

/**
 * \name PWD maximum pulse pulse-width samples
 * \anchor pwd_max_pulsewidth_def
 *
 * Maximum pulse-width samples that can be stored in the datastructure
 * \c IO_PWD_PULSE_SAMPLES
 */
/*@{*/
#define IO_PWD_MAX_PULSE_SAMPLES        9
/*@}*/

/**************************************************************************
 *
 * D A T A   S T R U C T U R E
 *
 **************************************************************************/

/**
 *
 * \brief PWD pulse-width data structure
 *
 *  stores each captured pulse-width for one measurement.
 *
 */
typedef struct _io_pwd_pulse_samples
{
    ubyte1 pulse_samples_count;                         /**< number of pulse_samples                              */
    ubyte4 pulse_sample[IO_PWD_MAX_PULSE_SAMPLES];      /**< stores each captured pulse-width for one measurement */
} IO_PWD_PULSE_SAMPLES;

/**
 *
 * \brief Safety configuration for the Incremental or Counter PWD inputs
 *
 *  Stores all relevant safety configuration parameters for the Incremental or counter PWD inputs.
 *  The WD and internal checker modules verify that this inputs contain valid
 *  values
 *
 */
typedef struct _io_pwd_inc_safety_conf
{
    ubyte2 pwd_cnt_val_upper;           /**< Upper PWD counter limit [1..65534]             */
    ubyte2 pwd_cnt_val_lower;           /**< Lower PWD counter limit [1..65534]             */
} IO_PWD_INC_SAFETY_CONF;

/**
 *
 * \brief Safety configuration for the Complex PWD inputs
 *
 *  Stores all relevant safety configuration parameters for the Complex PWD inputs.
 *  The WD and internal checker modules verify that this inputs contain valid
 *  values
 *
 */
typedef struct _io_pwd_cplx_safety_conf
{
    ubyte2 pwd_freq_val_upper;           /**< Upper PWD frequency limit in Hz [1..65534]    */
    ubyte2 pwd_freq_val_lower;           /**< Lower PWD frequency limit in Hz [1..65534]    */
    ubyte4 pwd_pulse_val_upper;          /**< Upper PWD pulse limit in us [1..4294967294]   */
    ubyte4 pwd_pulse_val_lower;          /**< Lower PWD pulse limit in us [1..4294967294]   */
} IO_PWD_CPLX_SAFETY_CONF;

/**************************************************************************
 *
 * F U N C T I O N S
 *
 **************************************************************************/

/***********************************************************************//**
 *
 * \brief Setup single incremental interface
 *
 * \param inc_channel           Incremental channel (\c IO_PWD_08 .. \c IO_PWD_11)
 * \param mode                  Defines the counter behavior
 *                                 - \c IO_PWD_INC_2_COUNT: Counts up/down on any edge
 *                                                     of the two inputs
 *                                 - \c IO_PWD_INC_1_COUNT: Counts up/down on any edge
 *                                                     of one channel (\c IO_PWD_08 or \c IO_PWD_10)
 * \param count_init            Initial value of incremental counter (0..65535)
 * \param threshold             Defines input threshold
 *                                 - \c IO_PWD_THRESH_1_25V: Switch threshold 1.25V
 *                                 - \c IO_PWD_THRESH_2_5V:  Switch threshold 2.5V
 * \param pupd                  Pull up/down interface:
 *                                 - \c IO_PWD_PU_10K:  Pull up    10 kOhm
 *                                 - \c IO_PWD_PD_10K:  Pull down  10 kOhm
 *                                 - \c IO_PWD_PD_1K8:  Pull down 1.8 kOhm
 *                                 - \c IO_PWD_PD_110:  Pull down 110 Ohm
 * \param safety_conf           relevant safety configurations for WD
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_BUSY                the channel is
 *                                          currently used by a other function
 * \retval IO_E_INVALID_CHANNEL_ID          the channel id does not exist
 * \retval IO_E_INVALID_PARAMETER           a parameter is out of range
 * \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing
 *
 ***************************************************************************
 *
 * \remarks
 *   \c IO_PWD_08 or \c IO_PWD_09 defines the first incremental interface
 *   \c IO_PWD_10 or \c IO_PWD_11 defines the second incremental interface.
 *   The voltage threshold and pupd will be set on both channels
 *   e.g. \c IO_PWD_08 and \c IO_PWD_09
 *
 * \remarks
 *   Passing \c IO_PWD_PD_110 selects a sensor as a current sensor (7mA/14mA).
 *   In this case a additional range check will be performed.
 *
 * \remarks
 *    If safety_conf != NULL, the WD and internal checker modules, checks the
 *    given channels against the parameter in safety_conf.
 *    For more detail about each parameter look on the definition of \c IO_PWD_SAFETY_CONF
 *    Note: This function is only available on a ECU with Watchdog CPU
 *
 * \remarks
 *    The given channel is the primary channel. All further operations (step and deinit functions)
 *    must be perform with this channel.
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_IncInit( ubyte1 inc_channel
                           , ubyte1 mode
                           , ubyte2 count_init
                           , ubyte1 threshold
                           , ubyte1 pupd
                           , IO_PWD_INC_SAFETY_CONF const * const safety_conf );

/***********************************************************************//**
 *
 * \brief Get the counter value of a incremental interface
 *
 * \param inc_channel               Incremental channel (\c IO_PWD_08 .. \c IO_PWD_11)
 * \param count                     Value of incremental counter (0..65535)
 *
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED      the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID          the given channel id does not exist
 * \retval IO_E_NULL_POINTER
 * \retval IO_E_UNKNOWN                     the ADC driver reported an error
 *                                          reports a error
 * \retval IO_E_PWD_CURRENT_THRESHOLD_HIGH  last measurement of current threshold
 *                                          was above 20.5mA
 * \retval IO_E_PWD_CURRENT_THRESHOLD_LOW   last measurement of threshold
 *                                          was under 4.5mA
 * \retval IO_E_FET_PROTECTION              FET is disabled, protection is active
 *
 ***************************************************************************
 *
 * \remarks
 *   The return values \c IO_E_PWD_CURRENT_THRESHOLD_HIGH and \c IO_E_PWD_CURRENT_THRESHOLD_LOW
 *   will only be returned if a current sensor (pupd == \c IO_PWD_PD_110) is connected
 *
 * \remarks
 *    Use the primary channel (same channel that is used in the init function).
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_IncGet( ubyte1 inc_channel
                          , ubyte2 * const count );

/***********************************************************************//**
 *
 * \brief Set the counter value of a incremental interface
 *
 * \param inc_channel               Incremental channel (\c IO_PWD_08 .. \c IO_PWD_11)
 * \param count                     New value to set of incremental counter (0..65535)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 *
 ***************************************************************************
 *
 * \remarks
 *    Use the primary channel (same channel that is used in the init function).
 *
 ***************************************************************************/
 IO_ErrorType IO_PWD_IncSet( ubyte1 inc_channel
                           , ubyte2 count );

/***********************************************************************//**
 *
 * \brief Deinitializes a single incremental interface
 *
 * \param inc_channel                   Incremental channel (\c IO_PWD_08 .. \c IO_PWD_11)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************
 *
 * \remarks
 *    Use the primary channel (same channel that is used in the init function).
 *
 ***************************************************************************/
 IO_ErrorType IO_PWD_IncDeInit( ubyte1 inc_channel );

/***********************************************************************//**
 *
 * \brief Setup single timer channel that measures frequency and pulse-width
 *        at the same time
 *
 * \param timer_channel        Timing channel (\c IO_PWD_08 .. \c IO_PWD_11)
 * \param pulse_mode                - \c IO_PWD_HIGH_TIME:   configuration to measure
 *                                                           pulse-high-time
 *                                  - \c IO_PWD_LOW_TIME:    configuration to measure
 *                                                           pulse-low-time
 *                                  - \c IO_PWD_PERIOD_TIME: configuration to measure
 *                                                           pulse-high and low-time (Period)
 * \param freq_mode            Specify the variable edge
 *                                  - \c IO_PWD_RISING_VAR:  rising edge is variable
 *                                                           this means, frequency is measured
 *                                                           on falling edges
 *                                  - \c IO_PWD_FALLING_VAR: falling edge is variable
 *                                                           this means, frequency is measured
 *                                                           on rising edges
 * \param timer_res            Specify the timer resolution
 *                                  - \c IO_PWD_RESOLUTION_0_2:  0.2us
 *                                  - \c IO_PWD_RESOLUTION_0_4:  0.4us
 *                                  - \c IO_PWD_RESOLUTION_0_8:  0.8us
 *                                  - \c IO_PWD_RESOLUTION_1_6:  1.6us
 *                                  - \c IO_PWD_RESOLUTION_3_2:  3.2us
 * \param capture_count        Number of frequency/pulse-width measurements that will be accumulated (0..8)
 * \param threshold            Defines threshold
 *                                  - \c IO_PWD_THRESH_1_25V: Switch threshold 1.25V
 *                                  - \c IO_PWD_THRESH_2_5V:  Switch threshold 2.5V
 * \param pupd                 Pull up/down interface:
 *                                  - \c IO_PWD_PU_10K:  Pull up    10 kOhm
 *                                  - \c IO_PWD_PD_10K:  Pull down  10 kOhm
 *                                  - \c IO_PWD_PD_1K8:  Pull down 1.8 kOhm
 *                                  - \c IO_PWD_PD_110:  Pull down 110 Ohm
 * \param safety_conf          relevant safety configurations for WD
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_BUSY                the channel is
 *                                          currently used by a other function
 * \retval IO_E_INVALID_CHANNEL_ID          the channel id does not exist
 * \retval IO_E_INVALID_PARAMETER           parameter capture_count, timebase or mode is out of range
 * \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing
 *
 ***************************************************************************
 *
 * \remarks
 *   A channel that is initalized with this function can retrieve frequency
 *   and duty cycle by calling the function: IO_PWD_ComplexGet()
 *
 * \remarks
 *   The timing measurement is based on a 16bit timer + 8bit overflow timer,
 *   therefore the product (65535 * 255 * resolution) must be greater than the
 *   period that shall be measured. If this period is greater, the function
 *   return \c IO_E_PWD_TIMER_OVERFLOW.
 *   The maximum frequency that can be measured is around 10kHz
 *   (restricted by low pass filtering)
 *
 * \remarks
 *   Passing \c IO_PWD_PD_110 selects a sensor as a current sensor (7mA/14mA).
 *   In this case a additional range check will be performed.
 *
 * \remarks
 *   If capture_count = 0, the driver accumulates all the measurements that has captured in the last
 *   round. Note: In this mode at least 4 edges are required for one measurement.
 *   If capture_count = 1..8, the driver captures exactly as many measurements are given in
 *   the parameter capture_count. Note: Until not all configured samples are captured, the driver
 *   don't return a value.
 *
 * \remarks
 *    If safety_conf != NULL, the WD and internal checker modules, checks the
 *    given channels against the parameter in safety_conf.
 *    For more detail about each parameter look up on the definition of \c IO_PWD_SAFETY_CONF
 *    Note: This function is only available on ECUs with Watchdog CPU
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_ComplexInit( ubyte1 timer_channel
                               , ubyte1 pulse_mode
                               , ubyte1 freq_mode
                               , ubyte1 timer_res
                               , ubyte1 capture_count
                               , ubyte1 threshold
                               , ubyte1 pupd
                               , IO_PWD_CPLX_SAFETY_CONF const * const safety_conf );

/***********************************************************************//**
 *
 * \brief Get the frequency and the pulse-width from the specified timer channel
 *
 * \param timer_channel            Timer channel (\c IO_PWD_08 .. \c IO_PWD_11)
 * \param frequency                Accumulated frequency in Hz
 * \param pulse_width              Accumulated pulse-width in us
 * \param pulse_samples            contains each pulse-width measure sample
 *
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED      the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID          the given channel id does not exist
 * \retval IO_E_PWD_NOT_FINISHED            not enough edges to accumulate a result
 * \retval IO_E_PWD_CAPTURE_ERROR           the frequency was too high
 * \retval IO_E_NULL_POINTER
 * \retval IO_E_FET_PROTECTION              FET is disabled, protection is active
 * \retval IO_E_PWD_TIMER_OVERFLOW          A timer overflow occured
 * \retval IO_E_PWD_CURRENT_THRESHOLD_HIGH  last measurement of current threshold
 *                                          was above 20.5mA
 * \retval IO_E_PWD_CURRENT_THRESHOLD_LOW   last measurement of threshold
 *                                          was under 4.5mA
 * \retval IO_E_UNKNOWN                     the ADC driver to measure the protection
 *                                          reports a error
 *
 ***************************************************************************
 *
 * \remarks
 *   The timing measurement is based on a 16bit timer + 8bit overflow timer,
 *   therefore the product (65535 * 255 * resolution) must be greater than the
 *   period that shall be measured. If this period is greater, the function
 *   return \c IO_E_PWD_TIMER_OVERFLOW.
 *   The maximum frequency that can be measured is around 10kHz
 *   (restricted by low pass filtering)
 *
 * \remarks
 *   The return values \c IO_E_PWD_CURRENT_THRESHOLD_HIGH and \c IO_E_PWD_CURRENT_THRESHOLD_LOW
 *   will only be returned if a current sensor (pupd == \c IO_PWD_PD_110) is connected
 *
 * \remarks
 *   if each individual measured pulse-width sample is not needed, the parameter
 *   pulse_samples should be set to NULL
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_ComplexGet( ubyte1 timer_channel
                              , ubyte2 * const frequency
                              , ubyte4 * const pulse_width
                              , IO_PWD_PULSE_SAMPLES * const pulse_samples );

/***********************************************************************//**
 *
 * \brief Deinitializes a complex PWD input
 *
 * \param timer_channel                 Timer channel (\c IO_PWD_08 .. \c IO_PWD_11)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_ComplexDeInit( ubyte1 timer_channel );

/***********************************************************************//**
 *
 * \brief Setup single timer channel that measures frequency only
 *
 * \param timer_channel         Timer channel (\c IO_PWD_00 .. \c IO_PWD_07)
 * \param freq_mode             Specify the variable edge
 *                                  - \c IO_PWD_RISING_VAR:  rising edge is variable
 *                                  - \c IO_PWD_FALLING_VAR: falling edge is variable
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                   everything fine
 * \retval IO_E_CHANNEL_BUSY         the timer input channel is
 *                                   currently used by a other function
 * \retval IO_E_INVALID_CHANNEL_ID   the timer input channel id does not exist
 * \retval IO_E_INVALID_PARAMETER    parameter mode is out of range
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_FreqInit( ubyte1 timer_channel
                            , ubyte1 freq_mode );

/***********************************************************************//**
 *
 * \brief Setup single timer channel that measures pulse-width only
 *
 * \param  timer_channel        Timer channel (\c IO_PWD_00 .. \c IO_PWD_07)
 * \param  pulse_mode           Capture mode
 *                                  - \c IO_PWD_HIGH_TIME: configuration to measure
 *                                                    pulse-high-time
 *                                  - \c IO_PWD_LOW_TIME:  configuration to measure
 *                                                    pulse-low-time
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                   everything fine
 * \retval IO_E_CHANNEL_BUSY         the timer input channel is
 *                                   currently used by a other function
 * \retval IO_E_INVALID_CHANNEL_ID   the timer input channel id does not exist
 * \retval IO_E_INVALID_PARAMETER    parameter mode is out of range
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_PulseInit( ubyte1 timer_channel
                             , ubyte1 pulse_mode );

/***********************************************************************//**
 *
 * \brief Get the frequency
 *
 * \param timer_channel            Timer channel (\c IO_PWD_00 .. \c IO_PWD_07)
 * \param frequency                Measured frequency in Hz
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 * \retval IO_E_PWD_CAPTURE_ERROR         frequency too high
 * \retval IO_E_NULL_POINTER
 * \retval IO_E_PWD_HIGH_LEVEL            only a constant high level is detected
 * \retval IO_E_PWD_LOW_LEVEL             only a constant low level is detected
 * \retval IO_E_PWD_NOT_FINISHED          not enough edges to accumulate a result
 *
 ***************************************************************************
 *
 * \remarks
 *   The lowest frequency that can be measured
 *   is 10Hz plus a jitter in the size of the cycle time.
 *   If the signal has a lower frequency,
 *   the function returns a frequency value of 0 and a return a
 *   status of IO_E_PWD_HIGH_LEVEL or IO_E_PWD_LOW_LEVEL after 104ms.
 *   The maximum frequency that can be measured is around 10KHz
 *   (restricted by low pass filtering)
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_FreqGet( ubyte1 timer_channel
                           , ubyte2 * const frequency );

/***********************************************************************//**
 *
 * \brief Get the pulse-width
 *
 * \param timer_channel            Timer channel (\c IO_PWD_00 .. \c IO_PWD_07)
 * \param pulse_width              Measured pulse-width in us
 *
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 * \retval IO_E_PWD_CAPTURE_ERROR         frequency too high
 * \retval IO_E_NULL_POINTER
 * \retval IO_E_PWD_HIGH_LEVEL            only a constant high level is detected
 * \retval IO_E_PWD_LOW_LEVEL             only a constant low level is detected
 * \retval IO_E_PWD_NOT_FINISHED          not enough edges to accumulate a result
 *
 ****************************************************************************
 *
 * \remarks
 *   The largest pulse that can be measured
 *   is 100ms plus a jitter in the size of the cycle time.
 *   If the signal has larger pulses,
 *   the function returns a pulse measurement value of 0 and a return a
 *   status of IO_E_PWD_HIGH_LEVEL or IO_E_PWD_LOW_LEVEL after 104ms.
 *   The maximum frequency that can be measured is around 10KHz
 *   (restricted by low pass filtering)
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_PulseGet( ubyte1 timer_channel
                            , ubyte4 * const pulse_width );

/***********************************************************************//**
 *
 * \brief Deinitializes a PWD input for frequency measurement.
 *        Allows the re-initialization of the input by other functions
 *
 * \param  timer_channel                Timer channel (\c IO_PWD_00 .. \c IO_PWD_07)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_FreqDeInit( ubyte1 timer_channel );

/***********************************************************************//**
 *
 * \brief Deinitializes a PWD input for pulse-width measurement.
 *        Allows the re-initialization of the input by other functions
 *
 * \param  timer_channel                Timer channel (\c IO_PWD_00 .. \c IO_PWD_07)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_PulseDeInit( ubyte1 timer_channel );

/***********************************************************************//**
 *
 * \brief Setup single counter channel
 *
 * \param count_channel        Counter channel (\c IO_PWD_08 or \c IO_PWD_10)
 * \param mode                 Specify on wich edge shall be count
 *                                  - \c IO_PWD_RISING_COUNT:  count on a rising edge
 *                                  - \c IO_PWD_FALLING_COUNT: count on a falling edge
 *                                  - \c IO_PWD_BOTH_COUNT:    count on a both edges
 * \param direction            Specify the counting direction
 *                                  - \c IO_PWD_UP_COUNT:      counts up
 *                                  - \c IO_PWD_DOWN_COUNT:    counts down
 * \param count_init           Init value of counter (0..65535)
 * \param threshold            Defines threshold
 *                                  - \c IO_PWD_THRESH_1_25V: Switch threshold 1.25V
 *                                  - \c IO_PWD_THRESH_2_5V:  Switch threshold 2.5V
 * \param pupd                 Pull up/down interface:
 *                                  - \c IO_PWD_PU_10K:  Pull up    10 kOhm
 *                                  - \c IO_PWD_PD_10K:  Pull down  10 kOhm
 *                                  - \c IO_PWD_PD_1K8:  Pull down 1.8 kOhm
 *                                  - \c IO_PWD_PD_110:  Pull down 110 Ohm
 * \param safety_conf           relevant safety configurations for WD
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_BUSY                the channel is
 *                                          currently used by another function
 * \retval IO_E_INVALID_CHANNEL_ID          the channel id does not exist
 * \retval IO_E_INVALID_PARAMETER           parameter capture_count, timebase or mode is out of range
 * \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing
 *
 ***************************************************************************
 *
 * \remarks
 *   Only channel \c IO_PWD_08 and \c IO_PWD_10 can be used for counting functionality
 *
 * \remarks
 *   Passing IO_PWD_PD_110 selects a sensor as a current sensor (7mA/14mA).
 *   In this case a additional range check will be performed.
 *
 * \remarks
 *    If safety_conf != NULL, the WD and internal checker modules, checks the
 *    given channels against the parameter in safety_conf.
 *    For more detail about each parameter look on the definition of \c IO_PWD_SAFETY_CONF
 *    Note: This function is only available on a ECU with Watchdog CPU
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_CountInit( ubyte1 count_channel
                             , ubyte1 mode
                             , ubyte1 direction
                             , ubyte2 count_init
                             , ubyte1 threshold
                             , ubyte1 pupd
                             , IO_PWD_INC_SAFETY_CONF const * const safety_conf );

/***********************************************************************//**
 *
 * \brief Get the counter value of a single counter channel
 *
 * \param count_channel             Counter channel (\c IO_PWD_08 or \c IO_PWD_10)
 * \param count                     Value of counter (0..65535)
 *
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED      the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID          the given channel id does not exist
 * \retval IO_E_NULL_POINTER
 * \retval IO_E_UNKNOWN                     the ADC driver to measure the protection
 *                                          reports a error
 * \retval IO_E_PWD_CURRENT_THRESHOLD_HIGH  last measurement of current threshold
 *                                          was above 20.5mA
 * \retval IO_E_PWD_CURRENT_THRESHOLD_LOW   last measurement of threshold
 *                                          was under 4.5mA
 * \retval IO_E_FET_PROTECTION              FET is disabled, protection is active
 *
 *
 ***************************************************************************
 *
 * \remarks
 *   Only channel \c IO_PWD_08 and \c IO_PWD_10 can be used for counting functionality
 *
 * \remarks
 *   The return values \c IO_E_PWD_CURRENT_THRESHOLD_HIGH and \c IO_E_PWD_CURRENT_THRESHOLD_LOW
 *   will only be returned if a current sensor (pupd == \c IO_PWD_PD_110) is connected
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_CountGet( ubyte1 count_channel
                            , ubyte2 * const count );

/***********************************************************************//**
 *
 * \brief Set the counter value of a single counter channel
 *
 * \param count_channel             Counter channel (\c IO_PWD_08 or \c IO_PWD_10)
 * \param count                     New value to set of incremental counter (0..65535)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 *
 ***************************************************************************
 *
 * \remarks
 *   Only channel \c IO_PWD_08 and \c IO_PWD_10 can be used for counting functionality
 *
 ***************************************************************************/
 IO_ErrorType IO_PWD_CountSet( ubyte1 count_channel
                             , ubyte2 count );

/***********************************************************************//**
 *
 * \brief Deinitializes a single counter channel
 *
 * \param count_channel        Counter channel (\c IO_PWD_08 or \c IO_PWD_10)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************
 *
 * \remarks
 *   Only channel \c IO_PWD_08 and \c IO_PWD_10 can be used for counting functionality
 *
 ***************************************************************************/
IO_ErrorType IO_PWD_CountDeInit( ubyte1 count_channel );

#endif /* _IO_PWD_H */
