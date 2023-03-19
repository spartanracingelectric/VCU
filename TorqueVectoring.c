//Inhub Motor Psuedo Code

//Torque Vectoring Flow Chart
//https://lucid.app/lucidchart/05a9f8e2-389b-4abe-9cab-97e58586c4b7/edit?page=0_0&invitationId=inv_d7d89c7f-f9fd-4ebf-9aac-497cd0f05dea#


#include <stdlib.h>
#include <math.h>
#include "sensors.h"
#include "motorcontroller.h"
#include "TorqueVectoring.h"


/****************************
 * Torque Vectoring
 * Torque Vectoring reads available sensors on the vehicle, uses a look up table to choose torque setpoint
 * based on available friction & driver torque command and yaw rate based on driver steering and velocity
*****************************/

float Mz; //Yaw Moment Target
float lookupValue;

struct _TorqueVectoring
{
    float friction;
    ubyte2 Fz
    float trim
    //check data types
};

//Initialize Tire values
void TorqueVectoring *Tire_new(ubyte1 corner, float friction, ubyte2 Fz, float trim)
{
    TorqueVectoring *tireFL = (TorqueVectoring *)malloc(sizeof(struct _TorqueVectoring));
        tireFL->friction = 0;
        tireFL->Fz = 0;
        tireFL->trim = 0;

    TorqueVectoring *tireFR = (TorqueVectoring *)malloc(sizeof(struct _TorqueVectoring));
        tireFR->friction = 0;
        tireFR->Fz = 0;
        tireFR->trim = 0;

    TorqueVectoring *tireFR = (TorqueVectoring *)malloc(sizeof(struct _TorqueVectoring));
        tireRL->friction = 0;
        tireRL->Fz = 0;
        tireRL->trim = 0;
    
    TorqueVectoring *tireRR = (TorqueVectoring *)malloc(sizeof(struct _TorqueVectoring));
        tireRR->friction = 0;
        tireRR->Fz = 0;
        tireRR->trim = 0;
}



/**************
Calculate Commands
Retrieve Sensor Data and driver requested inputs
Steering Angle, Load cells, Torque Request, Wheel Speeds (already evaluated in motor controller)
Slip Ratio
    //Eventually IMU, Tire Pressures, Tire temp
    //if no load sensors, can estimate loads by using accelarometer 
**********************/


void TVcalculateCommads()
{
                    //Call inverters 1,2,3,4 to get torques

                     //TODO:Driver Throttle (Torque demand actually)

                     //TODO:Add steering position angle
    sbyte2 steeringAngle = steering_degrees();
                     //TODO:slipRatio calculation
    sbyte2 slipRatio = 

                     //TODO:add slip angle calculation, needs to calculate both front and rear

                     //needs to call the functions that use the calculations as inputs
    
    tireLearn()

    torqueDistribution()
}


/***************************
 * Tire Learn*
 * *************************/
//Function applied to each tire individually
//Tire Learn should automatically adjust to what 
       //(can be when slip ratio exceeds threshold or TC is triggered), This will drop friction for every sampling time tc is triggered, only want 1 instance 
        //TireLearn adjustment should be dependent on the slipRatio magnitude the tire is experiencing
        //Dial to change TireLearn learn rate
        //Record TireLearn status
        //Output to CAN estimated Friction


//TODO: Make function to add to trim and subtract trim; tire corner as input

//
void tireLearn(tire) //TODO: have input as a generic tire, not specific corner
{
    if tc->tcStatus = TRUE 
    {
        if tc->tcStatus != tc->prev_tcStatus { // if tc_Status changes to FALSE //FIXME: 
                                                //Add prev_tcStatus to tc struct 
            tire->trim = tire->trim -.05        //current friction needs to be the friction at the current parameters of the vehicle (i.e. normalLoad, yawRate)
                                                //while main loop runs
                                                //if tc->tcStatus changes from TRUE to FALSE -> is there a method to do this?
            tc->prev_tcStatus = tc->tcStatus
                                                //tireLearn should not be separate from tc status, slipRatio function should be separated
        }
        trim = trim + .001 //Relearns friction Limit - This is only a scalar value, how would you decrease a certain portion of the torque map
    }
}

/*********************************************************
LookupTable - Reads look up table
    Must specify the sensors available for use
    Table must be X by Y columns and rows
    Ideally would be able to upload a lookup table instead of hard coding, but cannot upload CSV to VCU build.
        Could potentially send table through CAN but less than ideal 
*********************************************************/

float lookupTable(table, x, y)  //Function to read lookupTable
   {
    //TODO: create 2D array Lookup Table for Long Friction
    //TODO: create 2D array Lookup Table for Yaw Rate Request

    return lookupValue 
    }

/*
    SlipAngleFz_Friction
    SlipRatioFz_Friction
    .csv filename
    .csv needs an identifier if axes are in slipRatio, slipAngle, Fz, etc..
*/



/********************
Assign Friction 
thru Lookup Tables
*********************/

//Assign available friction
//data types need to be assigned
// Provide requested torque as a percentage of peak motor torque
//as long as friction limit is not exceeded with given parameters
    
assignFriction(*tireFL, *tireFR, *tireRL, *tireRR) //TODO fix inputs to have inputs
{   
    tireFL->Fric = lookupTable(SlipRatioFz_Friction,slipRatio, tireFL->Fz) + tireFL->trim//assign peak static friction available to each tire with trim applied
    tireFR->LongFric = lookupTable(SlipRatioFz_Friction,slipRatio, tireFR->Fz)
    tireRL->LongFric = lookupTable(SlipRatioFz_Friction,slipRatio, tireRL->Fz)
    tireRR->LongFric = lookupTable(SlipRatioFz_Friction,slipRatio, tireRR->Fz)


    //TODO make Tire struct where Tire corner is indicated by 1,2,3,4

    FzTotal = tireFL->Fz + tireFR->Fz + tireRL->Fz + tireRR->Fz; //Evaluate total weight on all 4 tires
    lambda = (tireFL->Fz + tireFR->Fz)/(FzTotal); //Front to Rear Bias %, lambda = 1 means full Front bias
    delta = (tireFL->Fz + tireRL->Fz)/(FzTotal);  //Left to Right Bias %, delta = 1 means full Left bias

} 
/*
    tireLF->LatFric = lookupTable(slipAngle, Fz)            //ignorning Lateral Friction due to lack of SA sensor
    tireLF->currentFriction = ((LatFric^2 + LongFric^2)^.5)   //long+lat friction magnitude
    tireLF->TireLoadVector = tan^-1(LatFric/LongFric)         //Load vector direction 
    tireL->pressureNormal = lookupTable(pressure,);
    tireL->cornerstiffNormal = lookupTable(CorneringStiffness,);
*/



/*********************
Assign Yaw Moment target based on yaw rate request from driver
Uses PID controller to determine the Yaw Moment required to generate requested Yaw Rate
**********************/
assignMz(yawRateReq )
{
if yawdd > setpoint value
}

alpha = 2*rw/tw; //Constants: rw = Effective tire radius, tw = track width (track width is assumed to be equivalent at front and rear axle)

/********************
Torque Distribution 
*********************/

//as a function of Torque request (APPS, BPS),Yaw Moment(from Steering angle, yaw rate map)
//Yaw moment must come form yaw controller 
//Slip control must feed tREF as a function of peak torque allowable and torque demand
//Torque distribution without normal load sensors (but need normal load sensors for friction maps)
//

torqueDistribution(Mz,tREF,lambda,delta)                                  
{
        tqFL = tREF - Mz*alpha*(2*lambda*delta-3*delta+1);
        tqFR = tREF - Mz*alpha*(lambda*delta-lambda-2*delta+1);
        tqRL = tREF - Mz*alpha*(-2*delta+1);
        tqRR = tREF - Mz*alpha*(lambda-2*delta);
}



/*********************************
 * Tokyo Drift Mode
 *********************************/

//Drift mode applies torque such that it exceeds the available static friction at the rear tires so that they stay spinning
//Throttle must correspond to the amount of Kinetic friction the driver is requesting
//Throttle can also be managed by yaw rate request
//Placed in Torque_distribution


//if InitialD->status = true