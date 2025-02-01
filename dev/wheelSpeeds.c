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
    float4 tireCircumferenceMillimeters_F; //calculated
    float4 tireCircumferenceMillimeters_R; //calculated
    ubyte1 rotationsperPulse_F;
    ubyte1 rotationsperPulse_R;
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
WheelSpeeds *WheelSpeeds_new(ubyte4 tireDiameterInches_F, ubyte4 tireDiameterInches_R, ubyte1 pulsesPerRotation_F, ubyte1 pulsesPerRotation_R)
{
    WheelSpeeds *me = (WheelSpeeds *)malloc(sizeof(struct _WheelSpeeds));

    //1 inch = .0254 m
    me->tireCircumferenceMillimeters_F = 0.79796386 * tireDiameterInches_F; // FORMER CODE me->tireCircumferenceMillimeters_F = 3.14159 * .0254 * tireDiameterInches_F;
    me->tireCircumferenceMillimeters_R = 0.79796386 * tireDiameterInches_R; // FORMER CODE me->tireCircumferenceMillimeters_R = 3.14159 * .0254 * tireDiameterInches_R;
    me->rotationsperPulse_F = 1 / pulsesPerRotation_F; // This flip exists to speed up future computations to avoid the float division that waws being used in WheelSpeeds_update()
    me->rotationsperPulse_R = 1 / pulsesPerRotation_R;
    me->speed_FL = 0;
    me->speed_FR = 0;
    me->speed_RL = 0;
    me->speed_RR = 0;

    //Turn on WSS power pins
    IO_DO_Set(IO_DO_07, TRUE); // WSS x4

    return me;
}

void WheelSpeeds_update(WheelSpeeds *me, bool interpolate)
{
    //speed (m/s) = m * pulses/sec / pulses
    if (interpolate)
    {
        me->speed_FL = me->tireCircumferenceMillimeters_F * Sensor_WSS_FL.heldSensorValue * me->rotationsperPulse_F;
        me->speed_FR = me->tireCircumferenceMillimeters_F * Sensor_WSS_FR.heldSensorValue * me->rotationsperPulse_F;
        me->speed_RL = me->tireCircumferenceMillimeters_R * Sensor_WSS_RL.heldSensorValue * me->rotationsperPulse_R;
        me->speed_RR = me->tireCircumferenceMillimeters_R * Sensor_WSS_RR.heldSensorValue * me->rotationsperPulse_R;
    }
    else
    {
        me->speed_FL = me->tireCircumferenceMillimeters_F * Sensor_WSS_FL.sensorValue * me->rotationsperPulse_F;
        me->speed_FR = me->tireCircumferenceMillimeters_F * Sensor_WSS_FR.sensorValue * me->rotationsperPulse_F;
        me->speed_RL = me->tireCircumferenceMillimeters_R * Sensor_WSS_RL.sensorValue * me->rotationsperPulse_R;
        me->speed_RR = me->tireCircumferenceMillimeters_R * Sensor_WSS_RR.sensorValue * me->rotationsperPulse_R;
    }
}

//Trash code
//void WheelSpeed_UnitCorrection(WheelSpeeds *me)
//{
//    me->speed_FL = me->speed_FL * 3.6;
//    me->speed_FR = me->speed_FR * 3.6;
//    me->speed_RL = me->speed_RL * 3.6;
//    me->speed_RR = me->speed_RR * 3.6;
//}

ubyte4 WheelSpeeds_getWheelSpeed(WheelSpeeds *me, Wheel corner)
{
    ubyte4 speed;
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

ubyte4 WheelSpeeds_getWheelSpeedRPM(WheelSpeeds *me, Wheel corner, bool interpolate)
{
    ubyte4 speed;
    if (interpolate)
    {
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
    else
    {
        switch (corner)
        {
            case FL:
                speed = Sensor_WSS_FL.sensorValue;
                break;
            case FR:
                speed = Sensor_WSS_FR.sensorValue;
                break;
            case RL:
                speed = Sensor_WSS_RL.sensorValue;
                break;
            case RR:
                speed = Sensor_WSS_RR.sensorValue;
                break;
            default:
                speed = 0;
        }
    }

    //Multiply sensorValue by 60 seconds to get RPM (1 Hz per bump)
    return speed*60.0f/NUM_BUMPS;
}

//UNUSED, NEEDS ADJUSTMENT TO INTERPOLATED SPEEDS
ubyte4 WheelSpeeds_getSlowestFront(WheelSpeeds *me)
{
    return (me->speed_FL < me->speed_FR) ? me->speed_FL : me->speed_FR;
}

//UNUSED, NEEDS ADJUSTMENT TO INTERPOLATED SPEEDS
ubyte4 WheelSpeeds_getFastestRear(WheelSpeeds *me)
{
    return (me->speed_RL > me->speed_RR) ? me->speed_RL : me->speed_RR;
}


ubyte4 WheelSpeeds_getGroundSpeed(WheelSpeeds *me, ubyte1 tire_config)
{
    //Grab interpolated Wheel Speed
    switch (tire_config)
    {
        //Use both
        case 0:
            return (me->speed_FL + me->speed_FR) / 2;

        //Use only FL
        case 1:
            return me->speed_FL;

        //Use only FR
        case 2:
            return me->speed_FR;
    }

    return 0;
}

ubyte4 WheelSpeeds_getGroundSpeedKPH(WheelSpeeds *me, ubyte1 tire_config)
{
    return (WheelSpeeds_getGroundSpeed(me, tire_config) * 3.6); //m/s to kph
}
