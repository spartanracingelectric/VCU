/*****************************************************************************
 * pid.c - Proportional-Integral-Derivative (PID) controller
 * Initial Author: Harleen Sandhu / Mehul Williams
 * Additional Author: Shaun Gilmore
 ******************************************************************************
 * General purpose PID controller, initially designed for Torque Vectoring.
 *
 * The PID controller works by using three terms to calculate an output value
 *that is used to control a system. The three terms are: Proportional: This term
 *is proportional to the error between the target and current values. It is
 *multiplied by a constant gain value (Kp) that determines how much the
 *controller responds to changes in the error. Integral: This term is
 *proportional to the running sum of errors over time. It is multiplied by a
 *constant gain value (Ki) that determines how much the controller responds to
 *steady-state errors. Derivative: This term is proportional to the rate of
 *change of the error. It is multiplied by a constant gain value (Kd) that
 *determines how much the controller responds to changes in the rate of change
 *of the error. By adjusting the values of the three gain constants (Kp, Ki, and
 *Kd), the controller can be tuned to respond differently to changes in the
 *error, steady-state errors, and changes in the rate of change of the error.
 * Generally, higher values of Kp will lead to faster response to changes in the
 *error, while higher values of Ki will lead to faster response to steady-state
 *errors, and higher values of Kd will lead to faster response to changes in the
 *rate of change of the error. Conversion between SlipR and Torque -> Kp
 * Proportional test first with other output 0, get midway with target and then
 *tune other items. There are many factors of noise. Kp will give you the
 *difference between 0.1 current vs 0.2 target -> if you want to apply 50nm if
 *your error is 0.1 then you need 500 for Kp to get target
 ****************************************************************************/

#include "PID.h"

#include <stdlib.h>

/**
 * @brief Create a PID object on the heap with specified gains and saturation
 */
PID *PID_new(sbyte1 Kp, sbyte1 Ki, sbyte1 Kd, sbyte4 saturationValue) {
    PID *pid = (PID *)malloc(sizeof(PID));

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->setpoint = 0;
    pid->previousError = 0;
    pid->totalError = 0;

    // 100 means 100 Hz => loop dt=0.01 s if you want to interpret it that way
    pid->dH = 100;
    pid->output = 0;
    pid->proportional = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->saturationValue = saturationValue;
    pid->antiWindupFlag = FALSE;

    return pid;
}

/** COMPUTATIONS **/
void PID_computeOutput(PID *pid, sbyte4 sensorValue) {
    sbyte4 currentError = (sbyte4)(pid->setpoint - sensorValue);

    // Proportional term
    pid->proportional = (sbyte4)(pid->Kp * currentError / 10);

    // Integral term
    sbyte4 newTotalError = pid->totalError + currentError;
    pid->integral = (sbyte4)((pid->Ki * newTotalError) / pid->dH / 10);

    // Derivative term (with low-pass filtering)
    sbyte4 diff = (sbyte4)(currentError - pid->previousError);
    double alpha = 0.1;  // Low-pass filter coefficient
    pid->derivative = (sbyte4)(alpha * (pid->Kd * diff * pid->dH) / 10 +
                               (1 - alpha) * pid->derivative);

    pid->previousError = currentError;
    pid->totalError = newTotalError;

    pid->output = (pid->proportional + pid->integral + pid->derivative);

    // Apply saturation and anti-windup
    if (pid->saturationValue > 0) {
        if (pid->output > pid->saturationValue) {
            pid->output = pid->saturationValue;
            pid->antiWindupFlag = TRUE;
        } else {
            pid->antiWindupFlag = FALSE;
        }
    }
}

/** SETTER FUNCTIONS **/
void PID_updateGainValues(PID *pid, sbyte1 Kp, sbyte1 Ki, sbyte1 Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_setTotalError(PID *pid, sbyte4 error) { pid->totalError = error; }

void PID_setSaturationPoint(PID *pid, sbyte4 saturationValue) {
    pid->saturationValue = saturationValue;
}

void PID_updateSetpoint(PID *pid, sbyte4 setpoint) {
    // If saturationValue > 0, clamp the setpoint
    if (pid->saturationValue > 0 && setpoint > pid->saturationValue)
        pid->setpoint = pid->saturationValue;
    else
        pid->setpoint = setpoint;
}

/** GETTER FUNCTIONS **/
sbyte1 PID_getKp(PID *pid) { return pid->Kp; }
sbyte1 PID_getKi(PID *pid) { return pid->Ki; }
sbyte1 PID_getKd(PID *pid) { return pid->Kd; }
sbyte4 PID_getSetpoint(PID *pid) { return pid->setpoint; }
sbyte4 PID_getPreviousError(PID *pid) { return pid->previousError; }
sbyte4 PID_getTotalError(PID *pid) { return pid->totalError; }
sbyte4 PID_getOutput(PID *pid) { return pid->output; }
sbyte4 PID_getProportional(PID *pid) { return pid->proportional; }
sbyte4 PID_getIntegral(PID *pid) { return pid->integral; }
sbyte4 PID_getDerivative(PID *pid) { return pid->derivative; }
sbyte4 PID_getSaturationValue(PID *pid) { return pid->saturationValue; }
bool PID_getAntiWindupFlag(PID *pid) { return pid->antiWindupFlag; }
