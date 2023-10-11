/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_ADC.h
 *
 * \brief IO Driver functions for ADC.
 *
 *      Contains all service functions for the ADC.
 *      There are three groups of ADC inputs:
 *
 *      - ADC 5V:
 *          Can be configured as resistive, current or ratiometric
 *          inputs.
 *          The used sensor supply is configured when setting up an analog
 *          input channel. The actual value of the sensor supply will be used
 *          by the IO driver to correct the read ADC value and achieve a
 *          higher accuracy.
 *
 *      - ADC Var:
 *          These analog inputs are split into two groups of 4 channels.
 *          The input range can be configured for every group from 5V to 32V.
 *          Additionally a pull up/down resistor can be configured (\c IO_ADC_PU_10K or \c IO_ADC_PD_10K).
 *
 *      - normal ADC:
 *          Various ADC inputs.
 *          Only on channels \c IO_ADC_08 .. \c IO_ADC_11 a pull up/down resistor can be configured.
 *          (\c IO_ADC_PU_10K, \c IO_ADC_PD_10K, \c IO_ADC_PD_1K8 or \c IO_ADC_PD_110)
 *
 *  \section adc_examples ADC Code Examples
 *
 *      Examples for ADC initialization:
 *  \code
 *      // ADC 5V:
 *      IO_ADC_ChannelInit( IO_ADC_5V_00
 *                        , IO_ADC_RATIOMETRIC          // ratiometric configuration
 *                        , 0
 *                        , 0
 *                        , IO_ADC_SENSOR_SUPPLY_0      // sensor supply 0 is used
 *                        , NULL );                     // not safety critical
 *
 *      // ADC Var:
 *      IO_ADC_ChannelInit( IO_ADC_VAR_00
 *                        , IO_ADC_ABSOLUTE
 *                        , IO_ADC_RANGE_10V            // range 0..10V
 *                        , IO_ADC_PU_10K               // 10kOhm pull up
 *                        , 0
 *                        , NULL );                     // not safety critical
 *
 *      // Normal ADC:
 *      IO_ADC_ChannelInit( IO_ADC_00
 *                        , IO_ADC_ABSOLUTE
 *                        , 0
 *                        , 0
 *                        , 0
 *                        , NULL );                     // not safety critical
 *
 *  \endcode
 *
 *      Example for ADC task function call:\n
 *      This function call is identical for every type of ADC inputs.
 *  \code
 *      ubyte2 adc_val_0;
 *      bool adc_fresh_0;
 *
 *      IO_ADC_Get( IO_ADC_5V_00
 *                , &adc_val_0
 *                , &adc_fresh_0 );
 *
 *  \endcode
 *
 **************************************************************************/

#ifndef _IO_ADC_H
#define _IO_ADC_H

/**************************************************************************
 *
 * I N C L U D E S
 *
 **************************************************************************/

#include "IO_PWD.h"
#include "IO_Driver.h"

/**************************************************************************
 *
 * D E F I N I T I O N S
 *
 **************************************************************************/


/**
 * \name Types of Analog 5V or analog 32V inputs
 * \anchor adc_5V_config
 *
 * Input configuration for ADC 5V or analog 32V inputs.
 * These defines can be used for the \c type parameter of the function
 * \c IO_ADC_ChannelInit.
 */
/*@{*/
#define IO_ADC_RATIOMETRIC          0x01    /**< Ratiometric configuration
                                             *   Task function returns voltage in [mV]     */
#define IO_ADC_CURRENT              0x03    /**< Current measurement configuration
                                             *   Task function returns current in [uA]     */
#define IO_ADC_RESISTIVE            0x00    /**< Resistive configuration
                                             *   Task function returns resistance in [Ohm] */
#define IO_ADC_ABSOLUTE             0x02    /**< normal voltage configuration */
/*@}*/

/**
 * \name Pull up / Pull down configuration for ADC inputs
 * \anchor adc_pupd_settings
 *
 * Configuration of the pull up or pull down resistors on the ADC inputs.
 * These defines can be used for the \c pupd parameter of the function
 * \c IO_ADC_ChannelInit.
 */
/*@{*/
#define IO_ADC_PU_10K               0x01            /**< 10 kOhm pull up    */
#define IO_ADC_PD_10K               0x00            /**< 10 kOhm pull down  */
#define IO_ADC_PD_1K8               IO_PWD_PD_1K8   /**< 1.8 kOhm pull down */
#define IO_ADC_PD_110               IO_PWD_PD_110   /**< 110 Ohm pull down  */
/*@}*/

/**
 * \name Range configuration for ADC inputs
 * \anchor adc_pupd_settings
 *
 * Configuration of the ADC input range.
 * These defines can be used for the \c range parameter of the function
 * \c IO_ADC_ChannelInit.
 */
/*@{*/
#define IO_ADC_RANGE_MAX                7
#define IO_ADC_RANGE_5V                 0       /**< ADC range 0 .. 5V  */
#define IO_ADC_RANGE_10V                1       /**< ADC range 0 .. 10V */
#define IO_ADC_RANGE_15V                2       /**< ADC range 0 .. 15V */
#define IO_ADC_RANGE_20V                3       /**< ADC range 0 .. 20V */
#define IO_ADC_RANGE_25V                4       /**< ADC range 0 .. 25V */
#define IO_ADC_RANGE_30V                5       /**< ADC range 0 .. 30V */
#define IO_ADC_RANGE_32V                6       /**< ADC range 0 .. 32V */
/*@}*/

/**************************************************************************
 *
 * D A T A   S T R U C T U R E
 *
 **************************************************************************/

/**
 *
 * \brief Safety configuration for the ADC inputs
 *
 *  Stores all relevant safety configuration parameters for the ADC inputs.
 *  The WD and internal checker modules verify that this inputs contain valid
 *  values
 *
 */
typedef struct _io_adc_safety_conf
{
    ubyte1 adc_val_upper;           /**< Upper ADC limit in % [0..100]                          */
    ubyte1 adc_val_lower;           /**< Lower ADC limit in % [0..100]                          */
    ubyte1 redundant_channel;       /**< Redundant channel [\c IO_ADC_5V_00 .. \c IO_ADC_5V_07]
                                      *  or \c IO_ADC_NON_REDUNDANT
                                      *  Note: redundant channel is optional for all inputs except
                                      *  \c O_ADC_5V_00 .. \c IO_ADC_5V_07
                                      *  In this case the parameter will not be checked
                                      */
} IO_ADC_SAFETY_CONF;

/**************************************************************************
 *
 * F U N C T I O N S
 *
 **************************************************************************/

/***********************************************************************//**
 *
 * \brief Setup one ADC channel
 *
 * \param adc_channel           ADC channel:
 *                                       - (\c IO_ADC_5V_00 .. \c IO_ADC_5V_07)
 *                                       - (\c IO_ADC_VAR_00 .. \c IO_ADC_VAR_07)
 *                                       - (\c IO_ADC_00 .. \c IO_ADC_11)
 *                                       - (\c IO_BOARD_TEMP)
 *                                       - (\c IO_SENSOR_SUPPLY_VAR)
 *                                       - (\c IO_ADC_SENSOR_SUPPLY_0)
 *                                       - (\c IO_ADC_SENSOR_SUPPLY_1)
 *                                       - (\c IO_ADC_UBAT)
 *                                       - (\c IO_ADC_REF_2V5)
 * \param type                  Type of input:
 *                                  - \c IO_ADC_RATIOMETRIC:    voltage input on
 *                                                              sensor supply
 *                                  - \c IO_ADC_CURRENT:        4-20mA input
 *                                  - \c IO_ADC_RESISTIVE:      0-65535Ohm input
 *                                  - \c IO_ADC_ABSOLUTE:       normal voltage input
 * \param range                 Voltage range of input:
 *                                  - \c IO_ADC_RANGE_5V:       0-5V input
 *                                  - \c IO_ADC_RANGE_10V:      0-10V input
 *                                  - \c IO_ADC_RANGE_15V:      0-15V input
 *                                  - \c IO_ADC_RANGE_20V:      0-20V input
 *                                  - \c IO_ADC_RANGE_25V:      0-25V input
 *                                  - \c IO_ADC_RANGE_30V:      0-30V input
 *                                  - \c IO_ADC_RANGE_32V:      0-32V input
 * \param pupd                  Pull up/down interface:
 *                                  - \c IO_ADC_PU_10K:         Pull up    10 KOhm
 *                                  - \c IO_ADC_PD_10K:         Pull down  10 KOhm
 *                                  - \c IO_ADC_PD_1K8:         Pull down 1.8 KOhm
 *                                  - \c IO_ADC_PD_110:         Pull down 100 Ohm
 * \param sensor_supply         Sensor supply:
 *                                  - \c IO_ADC_SENSOR_SUPPLY_0: 5V
 *                                  - \c IO_ADC_SENSOR_SUPPLY_1: 5V
 *                                  - \c IO_SENSOR_SUPPLY_VAR: 8.5V, 10V or 14.5V
 * \param safety_conf           relevant safety configurations for WD
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_INVALID_CHANNEL_ID          the channel id does not exist
 * \retval IO_E_INVALID_PARAMETER           parameter is out of range
 * \retval IO_E_CHANNEL_BUSY                the ADC input channel is
 *                                          currently used by another function
 * \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing
 *
 ***************************************************************************
 *
 * \remarks
 *    Not all channels support all features:
 *        - \c IO_ADC_5V_00..\c IO_ADC_5V_07:
 *             - type:          \c IO_ADC_RATIOMETRIC, \c IO_ADC_CURRENT, IO_ADC_ABSOLUTE or \c IO_ADC_RESISTIVE
 *             - sensor_supply: \c IO_ADC_SENSOR_SUPPLY_0 or \c IO_ADC_SENSOR_SUPPLY_1
 *                              (supported for type \c IO_ADC_RATIOMETRIC)
 *             - safety_conf:   a pointer to valid safety_conf or NULL
 *        - \c IO_ADC_VAR_00..\c IO_ADC_VAR_07:
 *             - range:         all values
 *             - pupd:          \c IO_ADC_PU_10K or \c IO_ADC_PD_10K
 *             - type:          \c IO_ADC_RATIOMETRIC or \c IO_ADC_ABSOLUTE
 *             - sensor_supply: all values (supported for type \c IO_ADC_RATIOMETRIC)
 *             - safety_conf:   a pointer to valid safety_conf or NULL
 *        - \c IO_ADC_00..\c IO_ADC_07:
 *             - type:          \c IO_ADC_RATIOMETRIC or \c IO_ADC_ABSOLUTE
 *             - sensor_supply: \c IO_ADC_SENSOR_SUPPLY_0 or \c IO_ADC_SENSOR_SUPPLY_1
 *                              (supported for type \c IO_ADC_RATIOMETRIC)
 *        - \c IO_ADC_08..\c IO_ADC_11:
 *             - pupd:          \c IO_ADC_PU_10K, \c IO_ADC_PD_10K, \c IO_ADC_PD_1K8 or \c IO_ADC_PD_110
 *        - \c IO_BOARD_TEMP:                   all arguments are ignored and not checked
 *        - \c IO_SENSOR_SUPPLY_VAR:            all arguments are ignored and not checked
 *        - \c IO_ADC_SENSOR_SUPPLY_0:          all arguments are ignored and not checked
 *        - \c IO_ADC_SENSOR_SUPPLY_1:          all arguments are ignored and not checked
 *        - \c IO_ADC_UBAT:                     all arguments are ignored and not checked
 *        - \c IO_ADC_REF_2V5                   all arguments are ignored and not checked
 *    If a channel does not support a function, the value of the associated
 *    parameter will be ignored.
 *
 * \remarks
 *    There are only 2 groups of voltage ranges available:
 *      - \c IO_ADC_VAR_00 .. \c IO_ADC_VAR_03: share the same voltage range
 *      - \c IO_ADC_VAR_04 .. \c IO_ADC_VAR_07: share the same voltage range
 *    Attention: When the voltage range is changed for one channel, the voltage
 *               range of all other channels sharing the same settings are changed,
 *               too.
 *
 * \remarks
 *    If safety_conf != NULL, the WD and internal checker modules, checks the
 *    given channels against the parameter in safety_conf.
 *    For more detail about each parameter look on the definition of \c IO_ADC_SAFETY_CONF
 *    Note: This function is only available on a ECU with Watchdog CPU
 *
 * \remarks
 *    The channel \c IO_ADC_UBAT is just initialized in the function IO_Driver_Init,
 *    therefore the return value is IO_E_CHANNEL_BUSY when IO_Driver_Init was called
 *    before.
 *
 * \remarks
 *    The channel \c IO_ADC_UBAT is for monitoring the battery supply for internal
 *    electronics (ECU pin 101). The battery supply for the power stages
 *    (ECU pins 127 and 128) cannot be monitored.
 *
 * \remarks
 *    The channel \c IO_ADC_REF_2V5 is only available on a ECU with WD CPU
 *
 ***************************************************************************/
IO_ErrorType IO_ADC_ChannelInit( ubyte1 adc_channel
                               , ubyte1 type
                               , ubyte1 range
                               , ubyte1 pupd
                               , ubyte1 sensor_supply
                               , IO_ADC_SAFETY_CONF const * const safety_conf );

/***********************************************************************//**
 *
 * \brief Deinitializes one ADC input
 *
 * \param adc_channel           ADC channel:
 *                                       - (\c IO_ADC_5V_00 .. \c IO_ADC_5V_07)
 *                                       - (\c IO_ADC_VAR_00 .. \c IO_ADC_VAR_07)
 *                                       - (\c IO_ADC_00 .. \c IO_ADC_11)
 *                                       - (\c IO_BOARD_TEMP)
 *                                       - (\c IO_SENSOR_SUPPLY_VAR)
 *                                       - (\c IO_ADC_SENSOR_SUPPLY_0)
 *                                       - (\c IO_ADC_SENSOR_SUPPLY_1)
 *                                       - (\c IO_ADC_UBAT)
 *                                       - (\c IO_ADC_REF_2V5)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************/
IO_ErrorType IO_ADC_ChannelDeInit( ubyte1 adc_channel );

/***********************************************************************//**
 *
 * \brief Returns the value of the given ADC channel
 *
 * \param adc_channel           ADC channel:
 *                                       - (\c IO_ADC_5V_00 .. \c IO_ADC_5V_07)
 *                                       - (\c IO_ADC_VAR_00 .. \c IO_ADC_VAR_07)
 *                                       - (\c IO_ADC_00 .. \c IO_ADC_11)
 *                                       - (\c IO_BOARD_TEMP)
 *                                       - (\c IO_SENSOR_SUPPLY_VAR)
 *                                       - (\c IO_ADC_SENSOR_SUPPLY_0)
 *                                       - (\c IO_ADC_SENSOR_SUPPLY_1)
 *                                       - (\c IO_ADC_UBAT)
 *                                       - (\c IO_ADC_REF_2V5)
 * \param adc_value             ADC value, the range depends on the input group
 *                              and its configuration:
 *                                  - \c IO_ADC_5V_00 .. \c IO_ADC_5V_07
 *                                      - \c IO_ADC_ABSOLUTE:        0..5000   (0V..5.000V)
 *                                      - \c IO_ADC_RATIOMETRIC:     0..5000   (0V..5.000V)
 *                                      - \c IO_ADC_CURRENT:         0..22700   (0mA..22.700mA)
 *                                      - \c IO_ADC_RESISTIVE:       0..65535  (0Ohm..65535Ohm)
 *                                  - \c IO_ADC_VAR_00 .. \c IO_ADC_VAR_07
 *                                      - \c IO_ADC_5V:              0..5000   (0V..5V)
 *                                      - \c IO_ADC_10V:             0..10000  (0V..10V)
 *                                      - \c IO_ADC_15V:             0..15000  (0V..15V)
 *                                      - \c IO_ADC_20V:             0..20000  (0V..20V)
 *                                      - \c IO_ADC_25V:             0..25000  (0V..25V)
 *                                      - \c IO_ADC_30V:             0..30000  (0V..30V)
 *                                      - \c IO_ADC_RANGE_32V:       0..33650  (0V..33V)
 *                                  - \c IO_ADC_00 .. \c IO_ADC_11
 *                                      - none:                   0..5000   (0V..5V)
 *                                  - \c IO_BOARD_TEMP
 *                                      - none:                   3983..24863 (-60.17..148.62 degree)
 *                                  - \c IO_SENSOR_SUPPLY_VAR
 *                                      - none:                   0..15681  (0V..15.681V)
 *                                  - \c IO_ADC_SENSOR_SUPPLY_0
 *                                      - none:                   0..5110   (0V..5.110V)
 *                                  - \c IO_ADC_SENSOR_SUPPLY_1
 *                                      - none:                   0..5110   (0V..5.110V)
 *                                  - \c IO_ADC_UBAT
 *                                      - none:                   0..40106  (0V..40.106V)
 *                                  - \c IO_ADC_REF_2V5
 *                                      - none:                   0..1023   (raw value 0V..5V)
 * \param fresh                 Status of the ADC value
 *                                  - \c TRUE:  ADC value is fresh
 *                                  - \c FALSE: ADC value is old
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                   everything fine
 * \retval IO_E_INVALID_CHANNEL_ID   the given channel id does not exist
 * \retval IO_E_NULL_POINTER
 * \retval IO_E_FET_PROTECTION       FET is disabled, protection is active
 *
 ***************************************************************************
 *
 * \remarks
 *    The temperature value in degree C must recalculated in the following manner, because the value
 *    has an offset of 10000 and is multiplied with 100:
 *    degree_value = ((float) adc_value - 10000) / 100
 *    Please use for this calculation the functions IO_ADC_BoardTempFloat or IO_ADC_BoardTempSbyte
 *
 * \remarks
 *    For the input IO_ADC_5V_00..IO_ADC_5V_07 in IO_ADC_CURRENT mode, the inputs will be switched off
 *    for 1s if the current exceed 22.505mA. In this time the functions returns IO_E_FET_PROTECTION
 *
 * \remarks
 *    The channel \c IO_ADC_REF_2V5 is only available on a ECU with WD CPU
 *
 * \remarks
 *    The channels \c IO_ADC_08 .. \c IO_ADC_11 or \c IO_PWD_08 .. \c IO_PWD_11 can also be
 *    used if initialized with one of the PWD init functions.
 *
 ***************************************************************************/
IO_ErrorType IO_ADC_Get( ubyte1 adc_channel
                       , ubyte2 * const adc_value
                       , bool * const fresh );

/***********************************************************************//**
 *
 * \brief Calculates the board temperature in float
 *
 * \param raw_value                     raw adc board temperature returned
 *                                      from the IO_ADC_Get function
 *
 *
 ***************************************************************************
 *
 * \remarks
 *    Usage: IO_ADC_Get( IO_BOARD_TEMP, &raw_value, &fresh );
 *           temp = IO_ADC_BoardTempFloat( raw_value );
 *
 ***************************************************************************/
float4 IO_ADC_BoardTempFloat( ubyte2 raw_value );

/***********************************************************************//**
 *
 * \brief Calculates the board temperature in sbyte1
 *
 * \param raw_value                     raw adc board temperature returned
 *                                      from the IO_ADC_Get function
 *
 *
 ***************************************************************************
 *
 * \remarks
 *    Usage: IO_ADC_Get( IO_BOARD_TEMP, &raw_value, &fresh );
 *           temp = IO_ADC_BoardTempFloat( raw_value );
 *
 ***************************************************************************/
sbyte1 IO_ADC_BoardTempSbyte( ubyte2 raw_value );

#endif /* _IO_ADC_H */
