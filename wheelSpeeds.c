#include <stdlib.h> //Needed for malloc
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"

#include "wheelSpeeds.h"
#include "mathFunctions.h"

#include "sensors.h"
//extern Sensor Sensor_BPS0;
//extern Sensor Sensor_BenchTPS1;

/*****************************************************************************
* Wheel Speed object
******************************************************************************
* This object converts raw wheel speed sensor readings to usable formats
* for i.e. traction control
****************************************************************************/

struct _WheelSpeeds
{
    float4 tireCircumferenceMeters_F; //calculated
    float4 tireCircumferenceMeters_R; //calculated
    float4 pulsesPerRotation_F;
    float4 pulsesPerRotation_R;
    float4 speed_FL;
    float4 speed_FR;
    float4 speed_RL;
    float4 speed_RR;
};

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
WheelSpeeds *WheelSpeeds_new(float4 tireDiameterInches_F, float4 tireDiameterInches_R, ubyte1 pulsesPerRotation_F, ubyte1 pulsesPerRotation_R)
{
    WheelSpeeds *me = (WheelSpeeds *)malloc(sizeof(struct _WheelSpeeds));

    //1 inch = .0254 m
    me->tireCircumferenceMeters_F = 3.14159 * (.0254 * tireDiameterInches_F);
    me->tireCircumferenceMeters_R = 3.14159 * (.0254 * tireDiameterInches_R);
    me->pulsesPerRotation_F = pulsesPerRotation_F;
    me->pulsesPerRotation_R = pulsesPerRotation_R;
    me->speed_FL = 0;
    me->speed_FR = 0;
    me->speed_RL = 0;
    me->speed_RR = 0;

    //Turn on WSS power pins
    IO_DO_Set(IO_DO_07, TRUE); // WSS x4

    return me;
}

void WheelSpeeds_update(WheelSpeeds *me)
{
    //speed (m/s) = m * pulses/sec / pulses
    me->speed_FL = me->tireCircumferenceMeters_F * Sensor_WSS_FL.sensorValue / me->pulsesPerRotation_F;
    me->speed_FR = me->tireCircumferenceMeters_F * Sensor_WSS_FR.sensorValue / me->pulsesPerRotation_F;
    me->speed_RL = me->tireCircumferenceMeters_R * Sensor_WSS_RL.sensorValue / me->pulsesPerRotation_R;
    me->speed_RR = me->tireCircumferenceMeters_R * Sensor_WSS_RR.sensorValue / me->pulsesPerRotation_R;
}

float4 WheelSpeeds_getWheelSpeed(WheelSpeeds *me, Wheel corner)
{
    float4 speed;
    switch (corner)
    {
    case FL:
        speed = me->speed_FL;
        break;
    case FR:
        speed = me->speed_FR;
        break;
    case RL:
        speed = me->speed_RL;
        break;
    case RR:
        speed = me->speed_RR;
        break;
    default:
        speed = 0;
    }

    return speed;
}

float4 WheelSpeeds_getWheelSpeedRPM(WheelSpeeds *me, Wheel corner, bool interpolate)
{
    float4 speed;
    if (interpolate) {
        switch (corner)
        {
            case FL:
                speed = Sensor_WSS_FL.heldSensorValue;
                break;
            case FR:
                speed = Sensor_WSS_FR.heldSensorValue;
                break;
            case RL:
                speed = Sensor_WSS_RL.heldSensorValue;
                break;
            case RR:
                speed = Sensor_WSS_RR.heldSensorValue;
                break;
            default:
                speed = 0;
        }
    }
    else {
        switch (corner)
        {
            case FL:
                speed = me->speed_FL;
                break;
            case FR:
                speed = me->speed_FR;
                break;
            case RL:
                speed = me->speed_RL;
                break;
            case RR:
                speed = me->speed_RR;
                break;
            default:
                speed = 0;
        }
    }

    return speed*60.0;
}

float4 WheelSpeeds_getSlowestFront(WheelSpeeds *me)
{
    return (me->speed_FL < me->speed_FR) ? me->speed_FL : me->speed_FR;
}

float4 WheelSpeeds_getFastestRear(WheelSpeeds *me)
{
    return (me->speed_RL > me->speed_RR) ? me->speed_RL : me->speed_RR;
}

float4 WheelSpeeds_getGroundSpeed(WheelSpeeds *me)
{
    return (me->speed_FL + me->speed_FR) / 2;
}
