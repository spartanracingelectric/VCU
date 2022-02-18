#include <stdlib.h> //Needed for malloc
#include <math.h>
#include "IO_RTC.h" //wat is?
#include "IO_DIO.h" //wat is?

#include "motorcontroller.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"

#include "sensors.h"
#include "PID.h"

#define PI 3.141592654

/*****************************************************************************
* Traction Control
******************************************************************************
* This function reduces requested torque demand when wheel spin is detected
* Also functions as launch control
****************************************************************************/

extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;

//Do these need to be called here? isn't used here atm. Needs to be integrated into TCSMode switch
extern Sensor Sensor_TCSSwitchUp;   // used currently for regen
extern Sensor Sensor_TCSSwitchDown; // used currently for regen
extern Sensor Sensor_TCSKnob;       // used currently for regen


//does this have to be sbyte2? should always be positive and shouldnt need 2 bytes of values
//Check Data Types

sbyte1 SlipAngle; //Need steering angle measurement fed through CAN to VCU
sbyte1 SlipActual; //does this need to be initialized as 0? can be a fractional #
sbyte1 SlipAim; // value can be a decimal # 
float4 ThrottlePos = 100 * getPercent(appsOutputPercent, me->regen_percentAPPSForCoasting, 1, TRUE); //this needs to be entered correctly, throttle must be detected to indicate driver is intending to accelerate
sbyte1 TCMultiplier;
ubyte2 TCTorque;



/***************************************************************
*SlipAngle
*Steering angle needs to be sent through CAN to VCU***
***************************************************************/
SlipAngle = 0 * PI / 180; //set to 0 until slip angle measurement is incorprated from sensors. (*PI/180 is a conversion to radians)

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
void SlipRatio(MotorController *me, WheelSpeeds *me) //Sensor_GPS?
{
        //Free Wheel vs Driven Wheel **(AWD will require some independent form of vehicle speed)** 
        SlipActual = (WheelSpeeds_getSlowestFront / WheelSpeeds_getFastestRear(WheelSpeeds *me)*cos(SlipAngle)) - 1 ; 
            //CHECK UNITS
            //Is there an instance where you would want to choose the faster front? i.e. when there is steering angle

        //Front Wheel Speed vs Groundspeed (through MotorSpeed) 
        SlipActual = (WheelSpeeds_getSlowestFront(WheelSpeeds *me / MCM_getGroundSpeedKPH(MotorController *me)*cos(SlipAngle)) - 1; //GroundSpeedKPH is in KPH here

        //GPS vs Rear Wheel Speed
        SlipActual = ((GPSSpeed() / WheelSpeeds_getFastestRear(WheelSpeeds *me)*cos(SlipAngle)) /  - 1;

        //GPS vs GroundSpeed through MotorSpeed
        SlipActual = ((GPSSpeed() / MCM_getGroundSpeedKPH(MotorController *me)*cos(SlipAngle)) /  - 1;

        //Needs if statements for checks in case the sensors are broken
        return SlipActual
}
//should SlipRatio function be defined at a higher level? would also be used in braking 
//written in its own .c file for vehicle dynamics?


// If wheel speed differences between left and right exceed X amount, swap calculation method. Kalman Filter?
// If front wheels are turned (i.e. SteeringAngle != 0), Front and Rear wheel speed differences will be non-zero 
// If rear wheels are independently driven (inhub motors), wheel speed di


/***********************************************************************************************************
Slip Aim
*************************************************************************************************************
    //Allowable Slip Ratio Target (changes at different velocities due to downforce)
    //Can be made into a map/multidimensional array of velocity, slip angle, against slip target,
*************************************************************************************************************/
//void SlipAim 

            //some value between between +.20 and -.20
            //Slipratio of 1 is complete slippage


/*********************************************************************************
 * Traction Control Mode Settings
 *********************************************************************************
 * Selections Traction Control Mode
 **Sets SlipAim target and torque reduction multipliers based on TC setting
 ** Slip generally does not exceed -.2 and +.2 within grip according to data
 **********************************************************************************/
void TC_setMode(TractionControl *TCSMode) //**HEADER FILE FIX**
{
    switch (TCSMode) //
    {
        case TC1: 
            SlipAim = .2;
            me->TCMultiplier = 50; //5 Nm, does the pointer work like this? how do you start a reference to *me, TCmode is not an object yet
            
            
        case TC2:
            SlipAim = .15;
            me->TCMultiplier = 200; //20 Nm

        case TC3:
            SlipAim = .1;
            me->TCMultiplier = 500; //50 Nm

        default:
        
        //USE PID CONTROLLER CALLED FROM PID.C
    }

}

//Does the header file need to contain pointer to TCMode* me sbyte1

/*********************************************************************************
Torque Reduction
********************************************************************************** 
*Triggered when SlipRatio exceeds SlipAim target chosen by TCmode
*********************************************************************************/

if (abs(SlipRatio(MotorController *me, WheelSpeeds *me)) > SlipAim && ThrottlePos > 5); //Compares SlipActual value against SlipAim, is this right way to call out SlipActual? Checks if Throttle is actuated at 5%
    {
        for (i=0, abs(SlipRatio(MotorController *me, WheelSpeeds *me)) > SlipAim ,i++)
            me->TCTorque = me->TCMultiplier * i; //reduces TCMultiplier Nm of Toruqe for every instance it sees that SlipRatio > Slip Aim
                                            //careful so that TorqueOutput does not become negative
                                            //Does this keep recalculating SlipRatio through the for loop?
                                            //Start i=1 so first iteration reduces torque?
                                            //if driver locks up braking but is also at 5% throttle, abs(SlipRatio) may exceed SlipAim and reduce torque******
    }

else
    me->TCTorque = 0; //How does this pointer get stored? 




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