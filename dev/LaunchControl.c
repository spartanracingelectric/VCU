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
extern Sensor DRSKnob;
extern TorqueEncoder *tps;
extern BrakePressureSensor *bps;
extern MotorController *mcm;

void initPIDController(PIDController *controller, float4 p, float4 i, float4 d, float4 initialTorque)
{
    controller->kp = p;
    controller->ki = i;
    controller->kd = d;
    controller->errorSum = initialTorque; // Will be initial torque command close to 240 (need tuning for initial torque)
    controller->lastError = 0;
}
float calculatePIDController(PIDController *controller, float4 target, float4 current, sbyte2 maxTorque)
{
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
    // Anti-Windup Calculation (needs to be done on integral controllers)
    if (output > (float4)maxTorque && (controller->ki * error * CYCLE_TIME) > 0)
    {
        output = (float4)maxTorque;
        controller->errorSum -= controller->ki * error * CYCLE_TIME;
    }
    if (output < 0 && (controller->ki * error * CYCLE_TIME) < 0)
    { // Torque can't go negative in Launch Control (only reduced from Torque Max)
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
void LaunchControl_new(LaunchControl *me, ubyte1 potLC)
{
    me->slipRatio = 0;
    me->lcTorque = -1;
    me->LCReady = FALSE;
    me->LCState = FALSE;
    me->potLC = potLC;
    me->sr_valid = FALSE;
    me->pidController = (PIDController *)malloc(sizeof(PIDController));
    initPIDController(me->pidController, -1.0, 0, 0, 170); // Set your PID values here to change various setpoints /* Setting to 0 for off */ Kp, Ki, Kd
}

void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *me)
{
    me->slipRatio = ((wss->speed_FL_RPM + wss->speed_FR_RPM) / (wss->speed_RL_RPM + wss->speed_RR_RPM)) - 1;
    // Limit to the range of -1 to 1
    if (me->slipRatio > 1)
    {
        me->slipRatio = 1;
    }
    if (me->slipRatio < -1)
    {
        me->slipRatio = -1;
    }
    me->sr_valid = wss_above_min_speed(wss, 1.5);
}

bool wss_above_min_speed(WheelSpeeds *wss, float4 minSpeed)
{
    return (wss->speed_FL_RPM_S > minSpeed && wss->speed_FR_RPM_S > minSpeed && wss->speed_RL_RPM_S > minSpeed && wss->speed_RR_RPM_S > minSpeed);
}

void launchControlTorqueCalculation(LaunchControl *me)
{
    sbyte2 steeringAngle = (sbyte2)steering_degrees();
    if (LC_Button.sensorValue && MCM_getGroundSpeedKPH(mcm) < 5 && abs(steeringAngle) < LC_STEERING_THRESHOLD)
    {
        me->LCReady = TRUE;
        me->lcTorque = 0;                                      // On the motor controller side, this torque should stay this way regardless of the values by the pedals while LC is ready
        initPIDController(me->pidController, -1.0, 0, 0, 170); // Set your PID values here to change various setpoints /* Setting to 0 for off */ Kp, Ki, Kd
                                                               // Because acceleration is in the negative regime of slip ratio and we want to increase torque to make it more negative
    }
    // When LC is ready and the button is released, and TPS and BPS are within bounds then enable LC
    if (me->LCReady && !LC_Button.sensorValue && tps->travelPercent > .90 && bps->percent < .05)
    {
        me->LCState = TRUE;
        me->lcTorque = abs(me->pidController->errorSum); // Set to the initial torque, this is abs in case the error becomes negative at the same time as the SR becomes invalid
        if (me->sr_valid)
        {
            me->lcTorque = calculatePIDController(me->pidController, -0.2, me->slipRatio, mcm->commands_torqueLimit / 10.0); // Set your target, current, dt
        }
    }
    // Turn off launch control if:
    // 1. The brakes are applied more than 5%
    // 2. The steering angle is more than the threshold
    // 3. The TPS goes below 90% while LC is active
    // 4. There is no currently valid slip ratio and the motor exceeds 1000 RPM (so LC can be tested on stands)
    if (bps->percent > .05 || abs(steeringAngle) > LC_STEERING_THRESHOLD || (tps->travelPercent < 0.90 && me->LCState) || (!me->sr_valid && mcm->motorRPM > 1000))
    {
        me->LCState = FALSE;
        me->LCReady = FALSE;
        me->lcTorque = 0;
    }
}
