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
                           
//Do these need to be called here? isn't used here atm. Needs to be integrated into TCSMode switch

****** Unnecessary due to #include initializations.h"
extern Sensor Sensor_TCSSwitchUp;   // used currently for regen 
extern Sensor Sensor_TCSSwitchDown; // used currently for regen
extern Sensor Sensor_TCSKnob;       // used currently for regen

*/

//Check Data Types

float slipRatio;
float slipAngle; //Need steering angle measurement fed through CAN to VCU
float slipAim; 
sbyte2 tcTorque;

/************************
 PID SLIP CONTROL 
 ************************
 *Sets proportional, integral, and derivative gains for controller

 *Also sets time and tau?                                                                                                                                      
 *tau should be 5-7x faster than sampling r?

 */ 



void setPIDControlgains_slip(PIDController *pid)
{

    /* Controller Gains */
	pid->Kp = 0.0f
	pid->Ki = 0.0f
	pid->Kd = 0.0f

    pid->T = 0.0f
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
    // If Slip angle is set to 0, Slip ratio will be pure longitudinal slip    
* Compares SlipActual calculations between motor speed, front wheel speeds, Rear Wheel Speeds and GPS speed(TBD) in the event a sensor may be in error
**********************************************************************************************************************/
void slipRatio_calc(MotorController *me, WheelSpeeds *me) //Sensor_GPS?
{ 
        //Free Wheel vs Driven Wheel **(AWD will require some independent form of vehicle speed)** 
        
        slipRatio = (WheelSpeeds_getSlowestFront(WheelSpeeds *me) / WheelSpeeds_getFastestRear(WheelSpeeds *me)) - 1 ; 
            //CHECK UNITS
            //Is there an instance where you would want to choose the faster front? i.e. when there is steering angle
        
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
   //Allowable Slip Ratio Target (changes at different velocities due to downforce)
    //Can be made into a map/multidimensional array of velocity, slip angle, against slip target,
    //Value between between needs to be within the range of -slipAim < X < slipAim for both braking and acceleration
    //Slipratio of 1 is complete slippage

 **********************************************************************************/


void TC_setTCMode(TCSMode) //**HEADER FILE FIX**
{
    switch (TCSMode) 
    {
        case TC1: 
            slipAim = .2;
            break;

        case TC2: //Traction control activates sooner
            slipAim = .15;
            break;

        case TC3:
            slipAim = .1;
            break;

        case TC0: //Traction control OFF
        default:
            slipAim = 1.0;
            break;
        
    }
}

/*********************************************************************************
Torque Reduction
********************************************************************************** 
*Triggered when an error exists between slipRatio and slipAim target chosen by TCmode
*********************************************************************************/

void tcTorqueReduction(PIDController *pid, MotorController *me, WheelSpeeds *me, slipAim, slipRatio)
{ 
        if (slipRatio > slipAim)
        {
            tcTorque = PIDController_Update(PIDController *pid, slipAim, slipRatio); //**tcTorque is an sbyte2 but PIDController_Update is a float??
            //Apply traction control when slipRatio exceed slipAim under Acceleration
        }
        else if (slipRatio < -1*slipAim)
        {
            tcTorque = PIDController_Update(PIDController *pid, -1*slipAim, slipRatio)
            //Apply traction control when slipRatio exceed slipAim under Braking
        }
        else
        {
            tcTorque = 0;
            //If slipRatio is within slipAim bounds, do not apply torque reduction
        }
}

//NEEDS TO BE APPLIED TO MAIN LOOP, slipRatio_calc and tcTorque Reduction should be re-evaluated in every step



/************************
 *POOR MAN's PID Control
 **********************

void torqueTrim(MotorController *me, WheelSpeeds *me) { //need to call this out in motorcontroller (CODE WAS hanging without method)
 if (abs(SlipRatio(MotorController *me, WheelSpeeds *me)) > slipAim && ThrottlePos > 5); //Compares SlipActual value against SlipAim, is this right way to call out SlipActual? Checks if Throttle is actuated at 5%
    {
        for (i=0, abs(SlipRatio(MotorController *me, WheelSpeeds *me)) > slipAim ,i++)
            me->TCTorque = TCMultiplier*i; //reduces TCMultiplier Nm of Toruqe for every instance it sees that SlipRatio > Slip Aim
                                            //careful so that TorqueOutput does not become negative
                                            //Does this keep recalculating SlipRatio through the for loop?
    }

else
    me->tcTorque = 0; //How does this pointer get stored?    
}


*****************************/


//TractionControl Function torqueTrim needs to be called out in motorController.c

//need to add this TCTorque pointer element into motorcontroller

//Currently poorman's control system, does not reduce torque relative to error 
//Actual Control system will reduce torque until slip aim target is achieved
//**For ABS, BPS needs to be triggered instead**

//Display Traction Control Light active




//if slip ratio exceed this amount, reduce max torque? or getTorquedemand, 
//reduce by 5Nm, and then return back original value

//how come the regen modes arent written as methods of an object i.e. regenmode.TorqueLimitDNm

//maps of traction/torque against vehicle speed/slip ratio/steering angle


//add include "TractionControl.h" to motorController for TCTorque

