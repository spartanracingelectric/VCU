/**************************************************************************
 * Copyright (c) 2010 TTTControl. All rights reserved. Confidential proprietory
 * Schönbrunnerstraße 7, A-1040 Wien, Austria. office@ttcontrol.com
 **************************************************************************/
/**********************************************************************//**
 * \file IO_PWM.h
 *
 * \brief IO Driver functions for PWM channels.
 *
 *      Contains all service functions for the PWM (Pulse Width Modulation) outputs.
 *      Up to 8 channels can be configured: \c IO_PWM_00 .. \c IO_PWM_07.
 *      Not all PWM channels can have their own frequency time base:
 *          - Channel \c IO_PWM_00 .. \c IO_PWM_03 have all their own frequency time base
 *          - Channel \c IO_PWM_04 and \c IO_PWM_05 share one frequency time base
 *          - Channel \c IO_PWM_06 and \c IO_PWM_07 share one frequency time base
 *
 *      Additionally, up to 4 current feedback measurement channels can be configured.
 *      The current value can be retrieved with the function IO_PWM_GetCur()
 *
 *  \section pwm_examples PWM code examples
 *
 *      PWM initialization examples:
 *      \code
 *      // Setup a PWM output with current measurement
 *      IO_PWM_Init( IO_PWM_00
 *                 , 100            // frequency is 100 Hz
 *                 , TRUE           // positive polarity
 *                 , TRUE           // enable current measurement
 *                 , IO_ADC_CUR_00
 *                 , FALSE          // no diag margin
 *                 , NULL );        // not safety critical
 *
 *      // Setup a PWM output without current measurement
 *      IO_PWM_Init( IO_PWM_01
 *                 , 150            // frequency is 150 Hz
 *                 , TRUE           // positive polarity
 *                 , FALSE          // disable current measurement
 *                 , 0
 *                 , FALSE
 *                 , NULL );        // not safety critical
 *      \endcode
 *
 *      PWM task examples:
 *      \code
 *      ubyte2 curr;
 *
 *      IO_PWM_SetDuty( IO_PWM_00   // set duty cycle to 50%
 *                    , 0x8000
 *                    , NULL );     // feedback measurement ignored
 *
 *      IO_PWM_GetCur( IO_PWM_00    // read current value of PWM output
 *                   , &curr );
 *
 *      \endcode
 *
 **************************************************************************/

#ifndef _IO_PWM_H
#define _IO_PWM_H

/**************************************************************************
 *
 * I N C L U D E S
 *
 **************************************************************************/

#include "IO_Driver.h"
#include "IO_RTC.h"

/**************************************************************************
 *
 * D A T A   S T R U C T U R E
 *
 **************************************************************************/

/**
 *
 * \brief Safety configuration for the PWM outputs
 *
 *  Stores all relevant safety configuration parameters for the PWM outputs.
 *  The WD and internal checker modules verify that this inputs contain valid
 *  values
 *
 * \remarks
 *   Info for PWMs with current measurement:
 *   For the diagnostic of the PWM outputs a minimum duty cycle will be applied by the driver software.
 *   This minimum duty cycle is 100us.
 *   Additionally, for diagnostics of the current feedback, for PWMs with current feedback a minimum current
 *   of 50mA has to be measured when the minimum duty cycle is applied.
 *   Therefore a minimum duty cycle greater than 100us might be required.
 *   The driver calculates this minimum duty cycle according to the following formula:
 *   \image html pwm_value.png
 *   R has to be set by the application dependent on the used actuator.
 *   However, actuator coils are not ideal (e.g. reduced effective current due to losses in the magnetic iron, temperature dependence
 *   and the influence of the free wheeling diode).
 *   Therefore it is necessary to tune the R value (parameter resistance) for the used
 *   actuator so that at least 50mA are reached under worst case conditions (considering e.g. temperature effects).
 *   The resistance value determined by this tuning procedure may differ significantly from the nominal resistance of the used coil.
 *   For a rough estimation of the resistance value the following formula below can be used. R_coil has to be set to the ohmic
 *   resistance of the actuator coil:
 *   \image html Rmod.png
 *   By setting the values I_set = 0.05A, U_diode = 0.7V, the formula calculates to:
 *   \image html Rmod_specific.png
 *   But keep in mind that this formula does not take the inductance of the coil and its interaction with the free-wheeling diode
 *   into account. The developer of the application software still has to calculate the value by considering the inductance and
 *   magnetic losses or by determining the value under worst case conditions by trial and error. As the electric current through
 *   coils is temperature dependent, the best advice for the application developer is to estimate the resistor value and regulate
 *   the electric current to a value greater than 50mA to be sure it never drops below this limit.
 *
 */
typedef struct _io_pwm_safety_conf
{
    ubyte1 resistance;             /**< Resistence of the coil [Ohm]  */
} IO_PWM_SAFETY_CONF;

/**************************************************************************
 *
 * F U N C T I O N S
 *
 **************************************************************************/

/***********************************************************************//**
 *
 * \brief Setup single PWM output
 *
 * \param pwm_channel        PWM channel (\c IO_PWM_00 .. \c IO_PWM_07)
 * \param frequency          PWM frequency (50Hz .. 1000Hz)
 * \param polarity           Polarity of output signal
 *                              - FALSE: Low output signal is variable
 *                              - TRUE:  High output signal is variable
 * \param cur_measurement    Specifies whether this PWM channel has a current measurement
 *                              - TRUE:   current measurement on
 *                              - FALSE:  current measurement off
 * \param cur_channel        Current loopback channel (\c IO_ADC_CUR_00 .. \c IO_ADC_CUR_03)
 * \param diag_margin        Indicate if a margin should be applied or not
 *                              - TRUE:  margin is on
 *                              - FALSE: no margin will be applied
 * \param safety_conf        relevant safety configurations for WD
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_BUSY                the PWM output channel or the timer input channel is
 *                                          currently used by another function
 * \retval IO_E_INVALID_CHANNEL_ID          the PWM output channel or the timer input channel
 *                                          id don't exist
 * \retval IO_E_INVALID_PARAMETER           a given parameter is out of range
 * \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing
 *
 ***********************************************************************//**
 *
 * \remarks
 *   The associated timer loopback channel will also be configured for open load
 *   and short circuit detection
 *
 * \remarks
 *   If a current measurement is configured for the given PWM channel,
 *   the PWM channel will be switched off if the threshold 2.48A is reached.
 *
 * \remarks
 *   The duty cycle cannot exceed the margin of 100us(lower boundary) and 250us(upper boundary) used for
 *   diagnostic
 *   if the parameter diag_margin is TRUE. This mode is important for hydraulic coils
 *   If the parameter diag_margin is FALSE, no duty cycle range margin will be applied
 *
 * \remarks
 *   Not all PWM channels can have their own frequency time base.
 *     - Channel \c IO_PWM_00..\c IO_PWM_03 have their own frequency time base
 *     - Channel \c IO_PWM_04 and \c IO_PWM_05 share one frequency time base
 *     - Channel \c IO_PWM_06 and \c IO_PWM_07 share one frequency time base
 *
 *   Attention: When the frequency is changed on one of the channels that
 *   share their frequency time base, the associated channels will change
 *   their frequency!
 *
 * \remarks
 *    If safety_conf != NULL, the WD and internal checker modules, checks the
 *    given channels against the parameter in safety_conf.
 *    For more detail about each parameter look on the definition of \c IO_PWM_SAFETY_CONF
 *    Note: This function is only available on a ECU with Watchdog CPU
 *
 * \remarks
 *    If safety_conf != NULL, the parameter diag_margin is forced to TRUE to
 *    allow diagnostics
 *
 * \remarks
 *    The max frequency is 1000Hz, but be careful, the power switches works only
 *    correct with max. 200Hz at full load. If the frequency is higher than 200Hz
 *    and the full load is applied on the power output, this can be damage the power stages!
 *
 * \remark
 *    Static friction and stiction can cause a hysteresis and make the control of a
 *    hydraulic valve erratic and unpredictable. In order to counteract these hysteretic effects,
 *    small vibrations about the desired position shall be created in the valve.
 *    This constantly breaks the static friction ensuring that it will move even with small input changes,
 *    and the effects of hysteresis are average out. A proper setting of PWM frequency according to
 *    the resonance frequency of the actuator allows to adjust this desired small vibration,
 *    low enough in amplitude to prevent noticeable oscillations on the hydraulic output but sufficient
 *    high to prevent friction.
 *    The PWM frequency can be set in the range of 10 .. 200Hz, a typical range for hydraulic valves
 *    to operate without friction is 90 .. 160Hz.
 *
 ***************************************************************************/
IO_ErrorType IO_PWM_Init( ubyte1 pwm_channel
                        , ubyte2 frequency
                        , bool polarity
                        , bool cur_measurement
                        , ubyte1 cur_channel
                        , bool diag_margin
                        , IO_PWM_SAFETY_CONF const * const safety_conf );

/***********************************************************************//**
 *
 * \brief Setup two PWM outputs for alternating driving
 *
 * \param pwm_channel        PWM channel (\c IO_PWM_00..\c IO_PWM_07)
 * \param frequency          PWM frequency (50Hz .. 1000Hz)
 * \param polarity           Polarity of output signal
 *                              - FALSE: Low output signal is variable
 *                              - TRUE:  High output signal is variable
 * \param cur_channel        Current loopback channel (\c IO_ADC_CUR_00..\c IO_ADC_CUR_03)
 * \param safety_conf        relevant safety configurations for WD
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                          everything fine
 * \retval IO_E_CHANNEL_BUSY                the PWM output channel or the timer input channel is
 *                                          currently used by another function
 * \retval IO_E_INVALID_CHANNEL_ID          the PWM output channel or the timer input channel
 *                                          id don't exist
 * \retval IO_E_INVALID_PARAMETER           a given parameter is out of range
 * \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing
 *
 ***********************************************************************//**
 *
 * \remarks
 *   The associated timer loopback channels will also be configured for open load
 *   and short circuit detection
 *
 * \remarks
 *   The duty cycle cannot exceed the margin of 100us(lower boundary) and 250us(upper boundary)
 *   used for diagnostic
 *
 * \remarks
 *   The PWM channels will be switched off if a current measurement
 *   threshold of 2.48A is reached.
 *
 * \remarks
 *   The function setup also the neighbor PWM channel for alternating driving
 *   e.g. \c IO_PWM_00 has \c IO_PWM_01 as neighbor
 *        \c IO_PWM_07 has \c IO_PWM_06 as neighbor ...
 *
 * \remarks
 *    If safety_conf != NULL, the WD and internal checker modules, checks the
 *    given channels against the parameter in safety_conf.
 *    For more detail about each parameter look on the definition of \c IO_PWM_SAFETY_CONF
 *    Note: This function is only available on a ECU with Watchdog CPU
 *
 * \remarks
 *    The max frequency is 1000Hz, but be careful, the power switches works only
 *    correct with max. 200Hz at full load. If the frequency is higher than 200Hz
 *    and the full load is applied on the power output, this can be damage the power stages!
 *
 * \remarks
 *    The given channel is the primary channel. All further operations (step and deinit functions)
 *    must be perform with this channel.
 *
 * \remarks
 *    Alternating driving driver means, that only on one channel the given duty cycle will be
 *    applied (active channel) and on the other channel (passive channel) a small diagnostic
 *    signal will be outputed.
 *    With the parameter direction in function IO_PWM_DualSetDuty(), the active/passive state
 *    of the two channels can be switched.
 *
 * \remark
 *    Static friction and stiction can cause a hysteresis and make the control of a
 *    hydraulic valve erratic and unpredictable. In order to counteract these hysteretic effects,
 *    small vibrations about the desired position shall be created in the valve.
 *    This constantly breaks the static friction ensuring that it will move even with small input changes,
 *    and the effects of hysteresis are average out. A proper setting of PWM frequency according to
 *    the resonance frequency of the actuator allows to adjust this desired small vibration,
 *    low enough in amplitude to prevent noticeable oscillations on the hydraulic output but sufficient
 *    high to prevent friction.
 *    The PWM frequency can be set in the range of 10 .. 200Hz, a typical range for hydraulic valves
 *    to operate without friction is 90 .. 160Hz.
 *
 ***************************************************************************/
IO_ErrorType IO_PWM_DualInit( ubyte1 pwm_channel
                            , ubyte2 frequency
                            , bool polarity
                            , ubyte1 cur_channel
                            , IO_PWM_SAFETY_CONF const * const safety_conf );

/***********************************************************************//**
 *
 * \brief Deinitializes a PWM output.
 *   Allows the re-initialization of the output by other functions
 *
 * \param pwm_channel    PWM channel (\c IO_PWM_00..\c IO_PWM_07)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************
 *
 * \remarks
 *   If a current measurement is configured for the given PWM channel,
 *   the current measurement channel will also be deinitialized.
 *
 ***************************************************************************/
IO_ErrorType IO_PWM_DeInit( ubyte1 pwm_channel );

/***********************************************************************//**
 *
 * \brief Deinitializes two PWM outputs for alternating driving
 *   Allows the re-initialization of the output by other functions
 *
 * \param pwm_channel    PWM channel (\c IO_PWM_00..\c IO_PWM_07)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                      everything fine
 * \retval IO_E_INVALID_CHANNEL_ID      the given channel id does not exist
 * \retval IO_E_CHANNEL_NOT_CONFIGURED  the given channel is not configured
 *
 ***************************************************************************
 *
 * \remarks
 *   The current measurement channel will also be deinitialized.
 *
 ***************************************************************************/
IO_ErrorType IO_PWM_DualDeInit( ubyte1 pwm_channel );

/***********************************************************************//**
 *
 * \brief Set the duty cycle for a PWM channel.
 *
 * \param pwm_channel        PWM channel (\c IO_PWM_00..\c IO_PWM_07)
 * \param duty_cycle         Duty cycle for the channel (0..65535)
 * \param duty_cycle_fb      Duty cycle feedback for the channel in us
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 * \retval IO_E_PWD_OPEN_LOAD             open load has been detected
 * \retval IO_E_PWD_SHORT_CIRCUIT         short circuit has been detected
 * \retval IO_E_PWD_CAPTURE_ERROR         a capture error occured on the loopback channel
 *
 ***************************************************************************
 *
 * \remarks
 *   The duty cycle cannot exceed a margin of 100us(lower boundary) and 250us(upper boundary)
 *   used for diagnostic
 *   if the parameter diag_margin is TRUE. This mode is important for hydraulic coils
 *   If the parameter diag_margin is FALSE, no duty cycle range margin will be applied
 *
 * \remarks
 *   If the parameter duty_cycle_fb is NULL, the parameter is ignored.
 *   The parameter duty_cycle_fb returns the measured pulse-width of the PWM signal
 *   in the last round in us.
 *
 ***************************************************************************/
IO_ErrorType IO_PWM_SetDuty( ubyte1 pwm_channel
                           , ubyte2 duty_cycle
                           , ubyte4 * const duty_cycle_fb );

/***********************************************************************//**
 *
 * \brief Set the duty cycle two PWM outputs for alternating driving
 *
 * \param pwm_channel        PWM channel (\c IO_PWM_00..\c IO_PWM_07)
 * \param duty_cycle         Duty cycle for the channel (0..65535)
 * \param direction          select a PWM channel
 *                              TRUE:  controls the first PWM channel
 *                              FALSE: controls the second PWM channel
 * \param duty_cycle_fb_0    Duty cycle feedback for the first channel in us
 * \param duty_cycle_fb_1    Duty cycle feedback for the second channel in us
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 * \retval IO_E_PWD_OPEN_LOAD             open load has been detected
 * \retval IO_E_PWD_SHORT_CIRCUIT         short circuit has been detected
 * \retval IO_E_PWD_CAPTURE_ERROR         a capture error occured on the loopback channel
 *
 ***************************************************************************
 *
 * \remarks
 *   The duty cycle cannot exceed a margin of 100us(lower boundary) and 250us(upper boundary)
 *   used for diagnostic
 *
 * \remarks
 *   If the parameter duty_cycle_fb_0 or duty_cycle_fb_1 is NULL, the parameter is ignored.
 *   The parameter duty_cycle_fb returns the measured pulse-width of the PWM signal
 *   in the last round in us.
 *
 * \remarks
 *    Alternating driving driver means, that only on one channel the given duty cycle will be
 *    applied (active channel) and on the other channel (passive channel) a small diagnostic
 *    signal will be outputed.
 *    With the parameter direction, the active/passive state
 *    of the two channels can be switched.
 *
 ***************************************************************************/
IO_ErrorType IO_PWM_DualSetDuty( ubyte1 pwm_channel
                               , ubyte2 duty_cycle
                               , bool direction
                               , ubyte4 * const duty_cycle_fb_0
                               , ubyte4 * const duty_cycle_fb_1 );

/***********************************************************************//**
 *
 * \brief Returns the measured current of the given channel
 *
 * \param pwm_channel        PWM channel (\c IO_PWM_00..\c IO_PWM_07)
 * \param current            Measured current 0..2500 (0mA..2500mA)
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 * \retval IO_E_NULL_POINTER
 * \retval IO_E_UNKNOWN                   an ADC error occured
 * \retval IO_E_FET_PROTECTION            FET is disabled, protection is active
 *
 ***************************************************************************
 *
 * \remarks
 *   If the function is called more than one time in a cycle
 *   (between IO_Driver_TaskBegin and IO_Driver_TaskEnd), the second call return
 *   the error code \c IO_E_UNKNOWN, because the ADC value is old.
 *
 ***************************************************************************/
IO_ErrorType IO_PWM_GetCur( ubyte1 pwm_channel
                          , ubyte2 * const current );

/***********************************************************************//**
 *
 * \brief Set the frequency for a PWM channel.
 *
 * \param pwm_channel        PWM channel (\c IO_PWM_00..\c IO_PWM_03)
 * \param frequency          PWM frequency in 1/10Hz (0 (1/10Hz) .. 10000 (1/10Hz))
 *
 * \return IO_ErrorType:
 * \retval IO_E_OK                        everything fine
 * \retval IO_E_CHANNEL_NOT_CONFIGURED    the given channel is not configured
 * \retval IO_E_INVALID_CHANNEL_ID        the given channel id does not exist
 * \retval IO_E_INVALID_PARAMETER         a given parameter is out of range
 *
 ***************************************************************************
 *
 * \remarks
 *    The max frequency is 1000Hz, but be careful, the power switches works only
 *    correct with max. 200Hz at full load. If the frequency is higher than 200Hz
 *    and the full load is applied on the power output, this can be damage the power stages!
 *
 * \remarks
 *    frequency = 0 switches off the output
 *
 * \remarks
 *    The duty cycle remains the same as set by the function IO_PWM_SetDuty()
 *    The high/low time will only be adapted for the new frequency.
 *
 ***************************************************************************/
IO_ErrorType IO_PWM_SetFreq( ubyte1 pwm_channel
                           , ubyte2 frequency );

#endif /* _IO_PWM_H */
