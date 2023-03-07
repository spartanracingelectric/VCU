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

extern Sensor Sensor_LCButton;
float Calctorque;

/* Start of PID Controller */
typedef struct {
    float kp;         // Proportional gain
    float ki;         // Integral gain
    float kd;         // Derivative gain
    float errorSum;   // Running sum of errors for the integral term
    float lastError;  // Previous error for the derivative term
} PIDController;

void initPIDController(PIDController* controller, float p, float i, float d) {
    controller->kp = p;
    controller->ki = i;
    controller->kd = d;
    controller->errorSum = 0;
    controller->lastError = 0;
}

float calculatePIDController(PIDController* controller, float target, float current, float dt) {
    // Calculate the error between the target and current values
    float error = target - current;

    // Add the current error to the running sum of errors for the integral term
    // Time constant variance w/ System response (dt)
    controller->errorSum += error * dt;

    // Calculate the derivative of the error
    float dError = (error - controller->lastError) / dt;
    controller->lastError = error;

    // Calculate the output of the PID controller using the three terms (proportional, integral, and derivative)
    float output = controller->kp * error + controller->ki * controller->errorSum + controller->kd * dError;
    return output;
}

/* The PID controller works by using three terms to calculate an output value that is used to control a system. The three terms are:
Proportional: This term is proportional to the error between the target and current values. It is multiplied by a constant gain value (kp) that determines how much the controller responds to changes in the error.
Integral: This term is proportional to the running sum of errors over time. It is multiplied by a constant gain value (ki) that determines how much the controller responds to steady-state errors.
Derivative: This term is proportional to the rate of change of the error. It is multiplied by a constant gain value (kd) that determines how much the controller responds to changes in the rate of change of the error.
By adjusting the values of the three gain constants (kp, ki, and kd), the controller can be tuned to respond differently to changes in the error, steady-state errors, and changes in the rate of change of the error. 
Generally, higher values of kp will lead to faster response to changes in the error, while higher values of ki will lead to faster response to steady-state errors, and higher values of kd will lead to faster response to changes in the rate of change of the error.
Conversion between SlipR and Torque -> kp
*/

/* Start of Launch Control */

LaunchControl *LaunchControl_new(){

    LaunchControl* me = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    me->slipRatio = 0;
    me->lcTorque = -1;
    me->LCReady = FALSE;
    me->LCStatus = FALSE; 

    return me;
}

void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *me){
    me->slipRatio = (WheelSpeeds_getSlowestFront(wss) / (WheelSpeeds_getFastestRear(wss))) - 1;
}

void launchControlTorqueCalculation(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm){

    PIDController controller;
    initPIDController(&controller, 0.1, 0.01, 0.001); // Set your PID values here to change various setpoints /* Setting to 0 for off */
    float targetSlipRatio = 0.2; // Set your target slip ratio here
    float dt = 0.01; // Set your delta time long enough for system response to previous change

    sbyte2 mcm_Torque_max = (MCM_commands_getTorqueLimit(mcm) / 10.0);

    sbyte2 speedKph = MCM_getGroundSpeedKPH(mcm);

    sbyte2 steeringAngle = steering_degrees();
    
     if(Sensor_LCButton.sensorValue == FALSE && speedKph < 5 && bps->percent < .35 && steeringAngle > -35 && steeringAngle < 35) {
        me->LCReady = TRUE;
     }
     
     if(me->LCReady == TRUE && Sensor_LCButton.sensorValue == FALSE){
        me->lcTorque = 0; // On the motorcontroller side, this torque should stay this way regardless of the values by the pedals while LC is ready
     }

     if(me->LCReady == TRUE && Sensor_LCButton.sensorValue == TRUE && tps->travelPercent > .95){
        me->LCStatus = TRUE;
        me->lcTorque = 30; 
        if(speedKph > 3){
            Calctorque = calculatePIDController(&controller, targetSlipRatio, me->slipRatio, dt);
            if(Calctorque < mcm_Torque_max){
                //me->lcTorque = Calctorque; // Test PID Controller before uncommenting
            }
        } 
     }

    if(bps->percent > 0.10 || steeringAngle > 35 || steeringAngle < -35){ //tps->travelPercent < 0.80
            me->LCStatus = FALSE;
            me->LCReady = FALSE;
            me->lcTorque = -1;
    }

    // Update launch control state and torque limit
    MCM_update_LaunchControl_State(mcm, me->LCStatus);
    MCM_update_LaunchControl_TorqueLimit(mcm, me->lcTorque);
}

bool getLaunchControlStatus(LaunchControl *me){
    return me->LCStatus;
}

float getCalculatedTorque(){
    return Calctorque;
}