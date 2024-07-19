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
#include "sensorCalculations.h"
#include "PID.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

extern Sensor Sensor_LCButton;
extern Sensor Sensor_DRSKnob;
/*
 Start of PID Controller 

void initPIDController(PIDController* controller, float p, float i, float d, float initialTorque) {
    controller->kp = p;
    controller->ki = i;
    controller->kd = d;
    controller->errorSum = initialTorque; //Will be initial torque command close to 240 (need tuning for initial torque)
    controller->lastError = 0;
}
float calculatePIDController(PIDController* controller, float target, float current, float dt, sbyte2 maxTorque) {
    // Calculate the error between the target and current values
    float error = target - current;
    float propError = controller->kp * error;
    // Add the current error to the running sum of errors for the integral term
    // Time constant variance w/ System response (dt)
    controller->errorSum += controller->ki * error * dt;
    // Calculate the derivative of the error
    float dError = ((error * controller->kd) - controller->lastError) / dt;
    controller->lastError = error * controller->kd;
    // Calculate the output of the PID controller using the three terms (proportional, integral, and derivative)
    float output = propError + controller->errorSum + dError;
    //Anti-Windup Calculation (needs to be done on integral controllers)
    if (error > 0) {
        controller->errorSum -= controller->ki * error * dt;
    }
    if (output > (float)maxTorque){
        output = (float)maxTorque;
    }
    if (output < 0){ //Torque can't go negative in Launch Control (only reduced from Torque Max)
        output = 0;
        //controller->errorSum = controller->errorSum - error * dt; Is this needed?
    }
    return output;
}

 The PID controller works by using three terms to calculate an output value that is used to control a system. The three terms are:
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
LaunchControl *LaunchControl_new(){// this goes outside the while loop
    LaunchControl* me = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    me->slipRatio = 0;
    me->lcTorque = -1;
    me->LCReady = FALSE;
    me->LCStatus = FALSE;
    me->pidController = PID_new(20, 0, 0, 0.2); 
    me->pidController->total_error = 0; // setting PID total error @ init torque = 170 Nm?
    me->buttonDebug = 0;
    return me;
}
void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *me){
    float4 unfilt_speed = (WheelSpeeds_getSlowestFront(wss) / (WheelSpeeds_getFastestRear(wss))) - 1;
    float4 filt_speed = unfilt_speed;
    if (unfilt_speed > 1.0) {
        filt_speed = 1.0;
    }
    if (unfilt_speed < -1.0) {
        filt_speed = -1.0;
    }
    me->slipRatio = filt_speed;
    //me->slipRatio = (WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) / WheelSpeeds_getWheelSpeedRPM(wss, RR, TRUE)) - 1; //Delete if doesn't work
}
void launchControlTorqueCalculation(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm)
{
    sbyte2 speedKph = MCM_getGroundSpeedKPH(mcm);
    sbyte2 steeringAngle = steering_degrees();
    sbyte2 mcm_Torque_max = (MCM_commands_getTorqueLimit(mcm) / 10.0); //Do we need to divide by 10? Or does that automatically happen elsewhere?
    
    
    // SENSOR_LCBUTTON values are reversed: FALSE = TRUE and TRUE = FALSE, due to the VCU internal Pull-Up for the button and the button's Pull-Down on Vehicle
     if(Sensor_LCButton.sensorValue == TRUE && speedKph < 5 && bps->percent < .35) {
        me->LCReady = TRUE;
     }
     if(me->LCReady == TRUE && Sensor_LCButton.sensorValue == TRUE){
        me->lcTorque = 0; // On the motorcontroller side, this torque should stay this way regardless of the values by the pedals while LC is ready
        //initPIDController(me->pidController, 20, 0, 0, 170); // Set your PID values here to change various setpoints /* Setting to 0 for off */ Kp, Ki, Kd // Set your delta time long enough for system response to previous change
     }
     if(me->LCReady == TRUE && Sensor_LCButton.sensorValue == FALSE && tps->travelPercent > .90){
        me->LCStatus = TRUE;
        me->lcTorque = me->pidController->total_error; // Set to the initial torque
        if(speedKph > 3)
        {

            PID_setpointUpdate(me->pidController, 0.2);
            //PID_dtUpdate(me->pidController, 0.01);// updates the dt 
            //float Calctorque = calculatePIDController(me->pidController, 0.2, me->slipRatio, 0.01, mcm_Torque_max); // Set your target, current, dt
            float4 PIDtorque= (float4)PID_compute(me->pidController,me->slipRatio);// we erased the saturation checks for now we just want the basic calculation
            float4 appsTqPercent;
            TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
            ubyte2 torque= MCM_getMaxTorqueDNm(mcm);
            me->lcTorque =(ubyte2)(torque * appsTqPercent)+PIDtorque; // adds the ajusted value from the pid to the torqueval         }
    }
     }
    if(bps->percent > .05 || steeringAngle > 35 || steeringAngle < -35 || (tps->travelPercent < 0.90 && me->LCStatus == TRUE)){
        me->LCStatus = FALSE;
        me->LCReady = FALSE;
        me->lcTorque = -1;
    }
    // Update launch control state and torque limit
    MCM_update_LaunchControl_State(mcm, me->LCStatus);
    MCM_update_LaunchControl_TorqueLimit(mcm, me->lcTorque * 10);
     }
bool getLaunchControlStatus(LaunchControl *me){
    return me->LCStatus;
}
sbyte2 getCalculatedTorque(LaunchControl *me){
    return me->lcTorque;
}
ubyte1 getButtonDebug(LaunchControl *me) {
    return me->buttonDebug;
}


