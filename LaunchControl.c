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
extern Sensor Sensor_DRSKnob;
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
    controller->errorSum = 230; //Will be initial torque command close to 240 (need tuning for initial torque)
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
    if (output > maxTorque && (controller->ki * error * dt) > 0){
        output = maxTorque;
        controller->errorSum -= controller->ki * error * dt;
    }

    if (output < 0 && (controller->ki * error * dt) < 0){ //Torque can't go negative in Launch Control (only reduced from Torque Max)
        output = 0;
        //controller->errorSum = controller->errorSum - error * dt; Is this needed?
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
- Proportional test first with other output 0, get midway with target and then tune other items. There are many factors of noise. 
- Kp will give you the difference between 0.1 current vs 0.2 target -> if you want to apply 50nm if your error is 0.1 then you need 500 for kp to get target
*/

/* Start of Launch Control */

LaunchControl *LaunchControl_new(ubyte1 potLC){

    LaunchControl* me = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    me->slipRatio = 0;
    me->lcTorque = -1;
    me->LCReady = FALSE;
    me->LCStatus = FALSE; 
    me->potLC = potLC; 

    return me;
}

void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *me){
    me->slipRatio = (WheelSpeeds_getSlowestFront(wss) / (WheelSpeeds_getFastestRear(wss))) - 1;
    me->slipRatio = (WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) / WheelSpeeds_getWheelSpeedRPM(wss, RR, TRUE)) - 1; //Delete if doesn't work
}

void launchControlTorqueCalculation(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm){

    sbyte2 speedKph = MCM_getGroundSpeedKPH(mcm);

    sbyte2 steeringAngle = steering_degrees();

    sbyte2 mcm_Torque_max = (MCM_commands_getTorqueLimit(mcm) / 10.0); //Do we need to divide by 10? Or does that automatically happen elsewhere?

    PIDController controller;
    
    // SENSOR_LCBUTTON values are reversed: FALSE = TRUE and TRUE = FALSE, due to the VCU internal Pull-Up for the button and the button's Pull-Down on Vehicle
     if(Sensor_LCButton.sensorValue == FALSE && speedKph < 5 && bps->percent < .35 && steeringAngle > -35 && steeringAngle < 35) {
        me->LCReady = TRUE;
     } 
     
     if(me->LCReady == TRUE && Sensor_LCButton.sensorValue == FALSE){
        me->lcTorque = 0; // On the motorcontroller side, this torque should stay this way regardless of the values by the pedals while LC is ready
        initPIDController(&controller, 0, 0, 0); // Set your PID values here to change various setpoints /* Setting to 0 for off */ Kp, Ki, Kd // Set your delta time long enough for system response to previous change 
     }

     if(me->LCReady == TRUE && Sensor_LCButton.sensorValue == TRUE && tps->travelPercent > .90){

        me->LCStatus = TRUE;

        if(me->potLC == 1){    
            if (Sensor_DRSKnob.sensorValue == 0)
            {    me->lcTorque = 0; }
            else if (Sensor_DRSKnob.sensorValue <= 1.1)
            {    me->lcTorque = 1; }
            else if (Sensor_DRSKnob.sensorValue <= 2.2)
            {    me->lcTorque = 2; }
            else if (Sensor_DRSKnob.sensorValue <= 3.3)
            {    me->lcTorque = 3; }
            else if (Sensor_DRSKnob.sensorValue <= 4.4)
            {    me->lcTorque = 4; }
        }  else {
            me->lcTorque = 5; 
        }  

        if(speedKph > 3){
            Calctorque = calculatePIDController(&controller, 0.2, me->slipRatio, 0.10, mcm_Torque_max); // Set your target, current, dt
            //me->lcTorque = Calctorque; // Test PID Controller before uncommenting
        }
        
    }  

    if(bps->percent > .05 || steeringAngle > 35 || steeringAngle < -35 || (tps->travelPercent < 0.90 && me->LCStatus == TRUE)){ 
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

sbyte2 getCalculatedTorque(){
    return Calctorque;
}
