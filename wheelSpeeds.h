#ifndef _WHEELSPEEDS_H
#define _WHEELSPEEDS_H

#include "IO_Driver.h"
#include "sensors.h"
#include "sensorCalculations.h"

typedef enum { FL,FR,RL,RR } Wheel;


/*****************************************************************************
* Wheel Speed object
******************************************************************************
* This object converts raw wheel speed sensor readings to usable formats
* for i.e. traction control
****************************************************************************/

typedef struct _WheelSpeeds
{
    float4 tireCircumferenceMeters_F; //calculated
    float4 tireCircumferenceMeters_R; //calculated
    float4 pulsesPerRotation_F;
    float4 pulsesPerRotation_R;
    float4 speed_FL;
    float4 speed_FR;
    float4 speed_RL;
    float4 speed_RR;
} WheelSpeeds;
//After update(), access to tps Sensor objects should no longer be necessary.
//In other words, only updateFromSensors itself should use the tps Sensor objects
//Also, all values in the TorqueEncoder object are from 

WheelSpeeds* WheelSpeeds_new(float4 tireDiameterInches_F, float4 tireDiameterInches_R, ubyte1 pulsesPerRotation_F, ubyte1 pulsesPerRotation_R);
void WheelSpeeds_update(WheelSpeeds* me, bool interpolate);
float4 WheelSpeeds_getWheelSpeed(WheelSpeeds* me, Wheel corner);
float4 WheelSpeeds_getWheelSpeedRPM(WheelSpeeds* me, Wheel corner, bool interpolate);
float4 WheelSpeeds_getSlowestFront(WheelSpeeds* me);
float4 WheelSpeeds_getFastestRear(WheelSpeeds* me);
float4 WheelSpeeds_getGroundSpeed(WheelSpeeds* me, ubyte1 tire_config);
float4 WheelSpeeds_getGroundSpeedKPH(WheelSpeeds *me, ubyte1 tire_config);

#endif //  _WHEELSPEEDS_H