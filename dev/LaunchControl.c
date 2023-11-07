#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "LaunchControl.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"
#include "main.h"

extern Button LC_Button;
extern Sensor Sensor_DRSKnob;

void initPIDController(PIDController* controller, float4 p, float4 i, float4 d, float4 initialTorque) {
    controller->kp = p;
    controller->ki = i;
    controller->kd = d;
    controller->errorSum = initialTorque; //Will be initial torque command close to 240 (need tuning for initial torque)
    controller->lastError = 0;
}
float calculatePIDController(PIDController* controller, float4 target, float4 current, sbyte2 maxTorque) {
    // Calculate the error between the target and current values
    float4 error = target - current;
    float4 propError = controller->kp * error;
    // Add the current error to the running sum of errors for the integral term
    // Time constant variance w/ System response (CYCLE_TIME)
    controller->errorSum += controller->ki * error * CYCLE_TIME;
    // Calculate the derivative of the error
    float4 dError = ((error * controller->kd) - controller->lastError) / CYCLE_TIME;
    controller->lastError = error * controller->kd;
    // Calculate the output of the PID controller using the three terms (proportional, integral, and derivative)
    float4 output = propError + controller->errorSum + dError;
    //Anti-Windup Calculation (needs to be done on integral controllers)
    if (output > (float4)maxTorque && (controller->ki * error * CYCLE_TIME) > 0) {
        output = (float4)maxTorque;
        controller->errorSum -= controller->ki * error * CYCLE_TIME;
    }
    if (output < 0 && (controller->ki * error * CYCLE_TIME) < 0) { //Torque can't go negative in Launch Control (only reduced from Torque Max)
        output = 0;
        controller->errorSum = 0.0; // Something is wrong lets reset the integral term
    }
    return output;
}
/* The PID controller works by using three terms to calculate an output value that is used to control a system. The three terms are:
Proportional: This term is proportional to the error between the target and current values. It is multiplied by a constant gain value (kp) that determines how much the controller responds to changes in the error.
Integral: This term is proportional to the running sum of errors over time. It is multiplied by a constant gain value (ki) that determines how much the controller responds to steady-state errors.
Derivative: This term is proportional to the rate of change of the error. It is multiplied by a constant gain value (kd) that determines how much the controller responds to changes in the rate of change of the error.
By adjusting the values of the three gain constants (kp, ki, and kd), the controller can be tuned to respond differently to changes in the error, steady-state errors, and changes in the rate of change of the error.
Generally, higher values of kp will lead to faster response to changes in the error, while higher values of ki will lead to faster response to steady-state errors, and higher values of kd will lead to faster response to changes in the rate of change of the error.
Conversion between SlipR and Torque -> kp
Proportional test first with other output 0, get midway with target and then tune other items. There are many factors of noise.
Kp will give you the difference between 0.1 current vs 0.2 target -> if you want to apply 50nm if your error is 0.1 then you need 500 for kp to get target
*/
/* Start of Launch Control */
LaunchControl *LaunchControl_new(ubyte1 potLC){
    LaunchControl* me = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    me->slipRatio = 0;
    me->lcTorque = -1;
    me->LCReady = FALSE;
    me->LCStatus = FALSE;
    me->potLC = potLC;
    me->sr_valid = FALSE;
    return me;
}
void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *me){
    me->slipRatio = ((wss->speed_FL_RPM + wss->speed_FR_RPM) / (wss->speed_RL_RPM + wss->speed_RR_RPM)) - 1;
    // Limit to the range of -1 to 1
    if (me->slipRatio > 1){
        me->slipRatio = 1;
    }
    if (me->slipRatio < -1){
        me->slipRatio = -1;
    }
    me->sr_valid = wss_above_min_speed(wss, 1.5);
}

bool wss_above_min_speed(WheelSpeeds *wss, float4 minSpeed){
    if (wss->speed_FL_RPM_S > minSpeed && wss->speed_FR_RPM_S > minSpeed && wss->speed_RL_RPM_S > minSpeed && wss->speed_RR_RPM_S > minSpeed) {
        return TRUE;
    }
    else {
        return FALSE;
    }
}

void launchControlTorqueCalculation(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm) {
    sbyte2 steeringAngle = (sbyte2)steering_degrees();
    PIDController *controller = (PIDController *)malloc(sizeof(PIDController));
     if (LC_Button.sensorValue == TRUE && MCM_getGroundSpeedKPH(mcm) < 5 && steeringAngle > -LC_STEERING_THRESHOLD && steeringAngle < LC_STEERING_THRESHOLD) {
        me->LCReady = TRUE;
        me->lcTorque = 0; // On the motor controller side, this torque should stay this way regardless of the values by the pedals while LC is ready
        initPIDController(controller, -1.0, 0, 0, 170); // Set your PID values here to change various setpoints /* Setting to 0 for off */ Kp, Ki, Kd
        // Because acceleration is in the negative regime of slip ratio and we want to increase torque to make it more negative
     }
     if (me->LCReady == TRUE && LC_Button.sensorValue == FALSE && tps->travelPercent > .90 && bps->percent < .05) {
        me->LCStatus = TRUE;
        me->lcTorque = controller->errorSum; // Set to the initial torque
        if (me->sr_valid) {
            me->lcTorque = calculatePIDController(controller, -0.2, me->slipRatio, mcm->commands_torqueLimit/10.0); // Set your target, current, dt
        }
    }
    if (bps->percent > .05 || steeringAngle > LC_STEERING_THRESHOLD || steeringAngle < -LC_STEERING_THRESHOLD || (tps->travelPercent < 0.90 && me->LCStatus == TRUE) || (!me->sr_valid && mcm->motorRPM > 1000)) {
        me->LCStatus = FALSE;
        me->LCReady = FALSE;
        me->lcTorque = -1;
    }
    // Update launch control state and torque limit
    mcm->LCState = me->LCStatus;
    mcm->LaunchControl_Torque = me->lcTorque * 10;
    if (mcm->LaunchControl_Torque < 0) {
        mcm->LaunchControl_Torque = 0;
    }
}
