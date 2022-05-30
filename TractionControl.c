#include <stdlib.h> //Needed for malloc
#include <math.h>
#include "IO_RTC.h" //wat is?
#include "IO_DIO.h" //wat is?

#include "motorcontroller.h"
#include "TractionControl.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"

#include "sensors.h"
#include "PID.h"

/*****************************************************************************
* Traction Control
******************************************************************************
* This function reduces requested torque demand when wheel spin is detected
* Also functions as launch control
****************************************************************************/

/*

****** Unnecessary due to #include initializations.h"
extern Sensor Sensor_TCSSwitchUp;   // used currently for regen 
extern Sensor Sensor_TCSSwitchDown; // used currently for regen
extern Sensor Sensor_TCSKnob;       // used currently for regen

*/

//Check Data Types

struct _TractionControl
{
    float slipRatio;
    float slipAngle; //Need steering angle measurement fed through CAN to VCU
    float slipAim; 
    sbyte2 tcTorque;
    bool tcStatus;
};


//Initialize the values of each element of struct tc
TractionControl *TractionControl_new(void)
{
    TractionControl *tc = (TractionControl *)malloc(sizeof(struct _TractionControl));
    tc->slipRatio = 0;
    tc->slipAngle = 0;
    tc->slipAim = 0;
    tc->tcTorque = 0;
    tc->tcStatus = FALSE;

    return tc;
}
/************************
 PID SLIP CONTROL 
 ************************
 *Sets proportional, integral, and derivative gains for controller

 *T and tau are disrete terms                                                                                                                                   
 *tau should be 5-7x faster than sampling r?
 */ 



void setTCPIDControlgains_slip(PIDController *pid)
{
    /* Controller Gains */

	pid->Kp = 180.0f
	pid->Ki = 0.0f
	pid->Kd = 0.3f

    pid->T = .033f   //Sampling Time of discrete controller in seconds //HY-TTC-60 10kHz Timer In *Is this the correct sampling time?
                     // main.c loop is running on 33ms cycle time (30hz). How fast are wheel speed sensors? 
                     // Can only control Nyquist frequency for controller to 15hz?
    pid->tau = 0.0f
}

/**********************************************************************************
*Slip Ratio Caluclation
***********************************************************************************
*Slip ratio defined per SAE J670, see page 39 of RCVD - Milliken
    *SlipRatio = (Ω*R_e)/(V*cos(α))-1
    *Where
    *  Ω = Wheel Angular Velocity (rad/s)
    *  R_e = Effective Tire Radius
    *  V = Vehicle Speed (or belt surface speed)
    *  α = Slip Angle
    // If Slip angle is set to 0, Slip ratio will be pure longitudinal slip. Slip Angle is not currently calculated
    // Tire sees peak tractive forces from .1 to .15 slip ratio (RCVD pg37), and peak lateral forces at ~3-7 deg slipAngle RCVD(pg30)
    // Read paper Slip Angle Estimation: Development and Experimental Evaluation Seung-Hi Lee   
* Compares SlipActual calculations between motor speed, front wheel speeds, Rear Wheel Speeds and GPS speed(TBD) in the event a sensor may be in error
**********************************************************************************************************************/
void slipRatio_calc(WheelSpeeds *me, TractionControl *tc) //Sensor_GPS?
{ 
        //Free Wheel vs Driven Wheel **(AWD will require some independent form of vehicle speed)** 
        
        tc->slipRatio = (WheelSpeeds_getSlowestFront(me) / (WheelSpeeds_getFastestRear(me))*cos(tc->slipAngle)) - 1 ; 
            //Is there an instance where you would want to choose the faster front? i.e. when there is steering angle
            //slipRatio will turn negative when slip occurs under braking
            
        
        /*
        //Front Wheel Speed vs Groundspeed (through MotorSpeed) 
        SlipActual = (WheelSpeeds_getSlowestFront(WheelSpeeds *me / MCM_getGroundSpeedKPH(MotorController *me)) - 1; //GroundSpeedKPH is in KPH here

        //GPS vs Rear Wheel Speed
        SlipActual = ((GPSSpeed() / WheelSpeeds_getFastestRear(WheelSpeeds *me)) /  - 1;

        //GPS vs GroundSpeed through MotorSpeed
        SlipActual = ((GPSSpeed() / MCM_getGroundSpeedKPH(MotorController *me)) /  - 1;
        */

        //Needs if statements for checks in case the sensors are broken
}

// If wheel speed differences between left and right exceed X amount, swap calculation method. Kalman Filter?
// If front wheels are turned (i.e. SteeringAngle != 0), Front and Rear wheel speed differences will be non-zero 
// If rear wheels are independently driven (inhub motors), wheel speed differences vs average?



/*********************************************************************************
 * Traction Control Mode Settings
 *********************************************************************************
 * Select Traction Control Mode
 **Sets slipAim target and modifies torque reduction target based on chosen setting
 ** Slip generally does not exceed -.2 and +.2 within grip according to telemetry data

*Slip Aim
    //Allowable Slip Ratio Target (changes at different velocities due to downforce) and slip angle see RCVD(pg42)
    //Can be made into a map/multidimensional array of velocity, slip angle, against slip target,
    //Value between between needs to be within the range of -slipAim < X < slipAim for both braking and acceleration
    //Slipratio of 1 is complete slippage

 **********************************************************************************/
void TC_setTCMode(TCSMode) 
{
    switch (TCSMode) 
    {   //Adjust slipAim targets for when traction control engages
        case TC1: 
            tc->slipAim = .2;
            break;

        case TC2: //Traction control activates sooner
            tc->slipAim = .15;
            break;

        case TC3:
            tc->slipAim = .1;
            break;

        case TC0: //Traction control OFF
        default:
            //tc->slipAim = 9999999999.0;
            tc->tcTorque = 0;
            break;
        
    }
}

/*********************************************************************************
Torque Reduction
********************************************************************************** 
*Triggered when an error exists between slipRatio and slipAim target chosen by TCmode
    tcTorque is the controller output
*********************************************************************************/

void tcTorqueReduction(PIDController *pid, TractionControl *tc)
{       
    if (TCSMode != TC0) //If traction control setting is not set to OFF
    {
        if (tc->slipRatio >= tc->slipAim)
        {
            tc->tcStatus = TRUE; //TC active
            tc->tcTorque = PIDController_Update(PIDController *pid, tc->slipAim, tc->slipRatio); //**tcTorque is an sbyte2 but PIDController_Update is a float??
            //Apply traction control when slipRatio exceeds slipAim under Acceleration
        }
        else if (tc->slipRatio <= -1*tc->slipAim)
        {
            tc->tcStatus = TRUE; //TC active
            tc->tcTorque = PIDController_Update(PIDController *pid, -1*tc->slipAim, tc->slipRatio)
            //Apply traction control when slipRatio exceeds slipAim under Braking
        }
        else
        {
            tc->tcStatus = FALSE; //TC off
            tc->tcTorque = 0;
            //If slipRatio is within slipAim bounds, do not apply torque reduction
        }
    }
    else if(TCSMode = TC0) //If traction control setting is set to OFF
    {
        tc->tcStatus = FALSE; //TC off
        tc->tcTorque = 0;
    }
}

//Display Traction Control Light active if tcStatus = TRUE
//CAN message should be added to send 1 if tcStatus = TRUE and 0 if FALSE for DAQ logging

//maps of traction/torque against vehicle speed/slip ratio/slip angle

