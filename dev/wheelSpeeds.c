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
    ubyte4 frontTireCircumferenceMillimeters; //calculated
    ubyte4 rearTireCircumferenceMillimeters; //calculated
    ubyte1 pulsesPerRelvolutionFront;
    ubyte1 pulsesPerRelvolutionRear;
    ubyte4 frontLeftMillimetersPerSecond;
    ubyte4 frontRightMillimetersPerSecond;
    ubyte4 rearLeftMillimetersPerSecond;
    ubyte4 rearRightMillimetersPerSecond;
};

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
WheelSpeeds *WheelSpeeds_new(ubyte4 frontTireDiameterInches, ubyte4 rearTireDiameterInches, ubyte1 pulsesPerRelvolutionFront, ubyte1 pulsesPerRelvolutionRear)
{
    WheelSpeeds *me = (WheelSpeeds *)malloc(sizeof(struct _WheelSpeeds));

    //1 inch = .0254 m
    me->frontTireCircumferenceMillimeters   = PI_SIX_DIGITS_UBYTE * 254 * frontTireDiameterInches / 1000000; // Dividing by 1,000,000 for correct units in millimeters // FORMER CODE me->frontTireCircumferenceMillimeters = 3.14159 * .0254 * tireDiameterInches_F;
    me->rearTireCircumferenceMillimeters    = PI_SIX_DIGITS_UBYTE * 254 * rearTireDiameterInches / 1000000; // Dividing by 1,000,000 for correct units in millimeters // FORMER CODE me->rearTireCircumferenceMillimeters = 3.14159 * .0254 * tireDiameterInches_R;
    me->pulsesPerRelvolutionFront       = pulsesPerRelvolutionFront;
    me->pulsesPerRelvolutionRear        = pulsesPerRelvolutionRear;
    me->frontLeftMillimetersPerSecond   = 0;
    me->frontRightMillimetersPerSecond  = 0;
    me->rearLeftMillimetersPerSecond    = 0;
    me->rearRightMillimetersPerSecond   = 0;

    //Turn on WSS power pins
    IO_DO_Set(IO_DO_07, TRUE); // WSS x4

    return me;
}

void WheelSpeeds_update(WheelSpeeds *me, bool interpolate)
{
    //speed (m/s) = m * pulses/sec / pulses
    if (interpolate)
    {
        me->frontLeftMillimetersPerSecond   = me->frontTireCircumferenceMillimeters * Sensor_WSS_FL.heldSensorValue / me->pulsesPerRelvolutionFront;
        me->frontRightMillimetersPerSecond  = me->frontTireCircumferenceMillimeters * Sensor_WSS_FR.heldSensorValue / me->pulsesPerRelvolutionFront;
        me->rearLeftMillimetersPerSecond    = me->rearTireCircumferenceMillimeters * Sensor_WSS_RL.heldSensorValue / me->pulsesPerRelvolutionRear;
        me->rearRightMillimetersPerSecond   = me->rearTireCircumferenceMillimeters * Sensor_WSS_RR.heldSensorValue / me->pulsesPerRelvolutionRear;
    }
    else
    {
        me->frontLeftMillimetersPerSecond   = me->frontTireCircumferenceMillimeters * Sensor_WSS_FL.sensorValue / me->pulsesPerRelvolutionFront;
        me->frontRightMillimetersPerSecond  = me->frontTireCircumferenceMillimeters * Sensor_WSS_FR.sensorValue / me->pulsesPerRelvolutionFront;
        me->rearLeftMillimetersPerSecond    = me->rearTireCircumferenceMillimeters * Sensor_WSS_RL.sensorValue / me->pulsesPerRelvolutionRear;
        me->rearRightMillimetersPerSecond   = me->rearTireCircumferenceMillimeters * Sensor_WSS_RR.sensorValue / me->pulsesPerRelvolutionRear;
    }
}

ubyte4 WheelSpeeds_getWheelSpeedMMPS(WheelSpeeds *me, Wheel corner)
{
    ubyte4 speed;
    switch (corner)
    {
        case FL:
            speed = me->frontLeftMillimetersPerSecond;
            break;
        case FR:
            speed = me->frontRightMillimetersPerSecond;
            break;
        case RL:
            speed = me->rearLeftMillimetersPerSecond;
            break;
        case RR:
            speed = me->rearRightMillimetersPerSecond;
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
    return (me->frontLeftMillimetersPerSecond < me->frontRightMillimetersPerSecond) ? me->frontLeftMillimetersPerSecond : me->frontRightMillimetersPerSecond;
}

//UNUSED, NEEDS ADJUSTMENT TO INTERPOLATED SPEEDS
ubyte4 WheelSpeeds_getFastestRear(WheelSpeeds *me)
{
    return (me->rearLeftMillimetersPerSecond > me->rearRightMillimetersPerSecond) ? me->rearLeftMillimetersPerSecond : me->rearRightMillimetersPerSecond;
}


ubyte4 WheelSpeeds_getGroundSpeedMMPS(WheelSpeeds *me, ubyte1 tire_config)
{
    //Grab interpolated Wheel Speed
    switch (tire_config)
    {
        //Use both
        case 0:
            return (me->frontLeftMillimetersPerSecond + me->frontRightMillimetersPerSecond) / 2;

        //Use only FL
        case 1:
            return me->frontLeftMillimetersPerSecond;

        //Use only FR
        case 2:
            return me->frontRightMillimetersPerSecond;
    }

    return 0;
}

ubyte4 WheelSpeeds_getGroundSpeedKPH(WheelSpeeds *me, ubyte1 tire_config)
{
    return (WheelSpeeds_getGroundSpeed(me, tire_config) * 36 / 10000); //mm/s to kph
}
