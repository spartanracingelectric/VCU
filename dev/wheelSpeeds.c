#include <stdlib.h> //Needed for malloc
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"

#include "wheelSpeeds.h"
#include "mathFunctions.h"

#include "sensors.h"

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

void WheelSpeeds_update(WheelSpeeds *me, bool interpolate)
{
    me->speed_FL_RPM = Sensor_WSS_FL.sensorValue / me->pulsesPerRotation_F;
    me->speed_FR_RPM = Sensor_WSS_FR.sensorValue / me->pulsesPerRotation_F;
    me->speed_RL_RPM = Sensor_WSS_RL.sensorValue / me->pulsesPerRotation_R;
    me->speed_RR_RPM = Sensor_WSS_RR.sensorValue / me->pulsesPerRotation_R;
    me->speed_FL_RPM_S = Sensor_WSS_FL.heldSensorValue / me->pulsesPerRotation_F;
    me->speed_FR_RPM_S = Sensor_WSS_FR.heldSensorValue / me->pulsesPerRotation_F;
    me->speed_RL_RPM_S = Sensor_WSS_RL.heldSensorValue / me->pulsesPerRotation_R;
    me->speed_RR_RPM_S = Sensor_WSS_RR.heldSensorValue / me->pulsesPerRotation_R;
    //speed (m/s) = m * pulses/sec / pulses
    if (interpolate)
    {
        me->speed_FL = me->tireCircumferenceMeters_F * me->speed_FL_RPM_S;
        me->speed_FR = me->tireCircumferenceMeters_F * me->speed_FR_RPM_S;
        me->speed_RL = me->tireCircumferenceMeters_R * me->speed_RL_RPM_S;
        me->speed_RR = me->tireCircumferenceMeters_R * me->speed_RR_RPM_S;
    }
    else
    {
        me->speed_FL = me->tireCircumferenceMeters_F * me->speed_FL_RPM;
        me->speed_FR = me->tireCircumferenceMeters_F * me->speed_FR_RPM;
        me->speed_RL = me->tireCircumferenceMeters_R * me->speed_RL_RPM;
        me->speed_RR = me->tireCircumferenceMeters_R * me->speed_RR_RPM;
    }
}
