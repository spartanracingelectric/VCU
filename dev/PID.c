/*****************************************************************************
 * pid.c - Proportional-Integral-Derivative (PID) controller
 * Initial Author: Harleen Sandhu / Mehul Williams
 * Additional Author: Shaun Gilmore
 *****************************************************************************
 * General purpose PID controller, initially designed for Torque Vectoring.
 * 
 * The PID controller works by using three terms to calculate an output value that is used to control a system. The three terms are:
 * Proportional: This term is proportional to the error between the target and current values. It is multiplied by a constant gain value (Kp) that determines how much the controller responds to changes in the error.
 * Integral: This term is proportional to the running sum of errors over time. It is multiplied by a constant gain value (Ki) that determines how much the controller responds to steady-state errors.
 * Derivative: This term is proportional to the rate of change of the error. It is multiplied by a constant gain value (Kd) that determines how much the controller responds to changes in the rate of change of the error.
 * By adjusting the values of the three gain constants (Kp, Ki, and Kd), the controller can be tuned to respond differently to changes in the error, steady-state errors, and changes in the rate of change of the error.
 * Generally, changing Kp values will affect the response to changes in the "current error", while changing Ki values will affect response to steady-state error, and changing Kd values will affect the response to changes in the rate-of-change of the error.
 ****************************************************************************/
#include "PID.h"
/** PID gain values are allowed to be negative for the use-case of a "Reverse Acting PID." 
 *  The author of this comment cannot imagine a currently viable use-case of this, 
 *  but nonetheless the option remains for those that choose to dabble in such magic
*/
PID* PID_new(sbyte2 Kp, sbyte2 Ki, sbyte2 Kd, sbyte2 saturationValue) {
    PID* pid = (PID*)malloc(sizeof(PID));
    /** malloc returns NULL if it fails to allocate memory. Ideally, this trips a flag & outputs on CAN, 
     *  but such a thing is beyond the current scope of this commit
    */
    if (pid == NULL)
        return NULL;
    //max range of PID gain values [-32,768 to 32,767] -> effectively [-3,276.8 to 3,276.7] : see last line of PID_computeOutput() for reason why
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint      = 0; 
    pid->previousError = 0;
    pid->totalError    = 0; //max range of values [-32,768 to 32,767]
    pid->dH            = VCU_CYCLE_TIME_HZ; // 100 Hz aka 10 ms cycle time. view as inverese of 0.01 seconds, being done to avoid fpu usage
    pid->output        = 0;
    pid->proportional  = 0;
    pid->integral      = 0;
    pid->derivative    = 0;
    pid->saturationValue = saturationValue;
    pid->antiWindupFlag = FALSE;
    return pid;
}

/** SETTER FUNCTIONS  **/

void PID_updateGainValues(PID* pid, sbyte2 Kp, sbyte2 Ki, sbyte2 Kd){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_setTotalError(PID* pid, sbyte2 error){
    pid->totalError = error;
}

void PID_setSaturationPoint(PID *pid, sbyte2 saturationValue){
    pid->saturationValue = saturationValue;
}

void PID_updateSetpoint(PID *pid, sbyte2 setpoint) {
    if(pid->saturationValue > setpoint)
        pid->setpoint = setpoint;
    else
        pid->setpoint = pid->saturationValue;
        
    //this if statement exists for any uncapped pid that has no saturation point.
    if(pid->saturationValue == 0)
        pid->setpoint = setpoint;
}

/** COMPUTATIONS **/

void PID_computeOutput(PID *pid, sbyte2 sensorValue) {
    sbyte2 currentError = pid->setpoint - sensorValue;
    pid->proportional   = (sbyte2) pid->Kp * currentError;
    pid->integral       = (sbyte2) pid->Ki * (pid->totalError + currentError) / pid->dH ;
    pid->derivative     = (sbyte2) pid->Kd * (currentError - pid->previousError) * pid->dH ;
    pid->previousError  = currentError;
    /** Some pid implementations will increase totalError by "currentError * integral time" but we will not do this, and just add it instead,
     *  removing one mul/div per function call (its going to happen anyways in the integral component). Additionally, we could reduce lines
     *  by changing totalError prior to pid->integral calculations, but for - readability/understandability purposes - , we will take the 
     *  single additional clock cycle penalty associated with this lack of "optimization"
    */

    // At minimum, a P(ID) Controller will always use Proportional Control
    pid->output = pid->proportional;

    //Check to see if motor is saturated at max torque request already, if so, clamp the output to the saturation value
    if(pid->saturationValue > sensorValue){
        pid->antiWindupFlag = FALSE;
        pid->output += pid->integral;
        pid->output += pid->derivative;
        pid->totalError    += (sbyte4)currentError;
    }
    else{
        pid->antiWindupFlag = TRUE;
        /** Back Calculation here -> this is the old way of doing it with the previous "pid" method in LaunchControl.c of SR-15 main branch
         *  Simulink recomends a "Kb" or separate tuning value for unwinding a controller, and to use either clamping or back-calculation
         *  Both methods of anti-windup can be used simultaneously, but the complexity of using both will likely cause some unintendeed consequences.
         *  If experiencing oscillations at saturation limits, I advise to try tuning the Ki gain value first before trying to use back-Calculation 
         *  (and if doing so, its likely best to write a whole new function, and a switch case between clamping & backCalcs, rather than amending the line below)
        */
        pid->totalError -= pid->previousError;
    }
    // Divide by 10 is used to convert the error from deci-units to normal units (gain values are in deci-units)
    pid->output = pid->output / 10;
}

/** GETTER FUNCTIONS **/

sbyte2 PID_getKp(PID *pid){
    return pid->Kp;
}

sbyte2 PID_getKi(PID *pid){
    return pid->Ki;
}

sbyte2 PID_getKd(PID *pid){
    return pid->Kd;
}

sbyte2 PID_getSetpoint(PID *pid){
    return pid->setpoint;
}

sbyte2 PID_getPreviousError(PID *pid){
    return pid->previousError;
}

sbyte2 PID_getTotalError(PID* pid){
    return pid->totalError;
}

sbyte2 PID_getOutput(PID *pid){
    return pid->output;
}

sbyte2 PID_getProportional(PID *pid){
    return pid->proportional;
}

sbyte2 PID_getIntegral(PID *pid){
    return pid->integral;
}

sbyte2 PID_getDerivative(PID *pid){
    return pid->derivative;
}

sbyte2 PID_getSaturationValue(PID *pid){
    return pid->saturationValue;
}

bool PID_getAntiWindupFlag(PID *pid){
    return pid->antiWindupFlag;
}
