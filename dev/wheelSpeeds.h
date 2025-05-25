#ifndef _WHEELSPEEDS_H
#define _WHEELSPEEDS_H

#include "IO_Driver.h"
#include "sensors.h"
#include "sensorCalculations.h"

typedef enum { FL,FR,RL,RR } Wheel;

//After update(), access to tps Sensor objects should no longer be necessary.
//In other words, only updateFromSensors itself should use the tps Sensor objects
//Also, all values in the TorqueEncoder object are from 
typedef struct _WheelSpeeds WheelSpeeds;

WheelSpeeds* WheelSpeeds_new(float4 tireDiameterInches_F, float4 tireDiameterInches_R, ubyte1 pulsesPerRotation_F, ubyte1 pulsesPerRotation_R);
void WheelSpeeds_update(WheelSpeeds* me, bool interpolate);
float4 WheelSpeeds_getWheelSpeed(WheelSpeeds* me, Wheel corner);
float4 WheelSpeeds_getWheelSpeedRPM(WheelSpeeds* me, Wheel corner, bool interpolate);
float4 WheelSpeeds_getSlowestFront(WheelSpeeds* me);
float4 WheelSpeeds_getSlowestFrontRPM(WheelSpeeds *me);
float4 WheelSpeeds_getFastestRear(WheelSpeeds* me);
float4 WheelSpeeds_getRearAverage(WheelSpeeds *me);
float4 WheelSpeeds_getGroundSpeed(WheelSpeeds* me, ubyte1 tire_config);
float4 WheelSpeeds_getGroundSpeedKPH(WheelSpeeds *me, ubyte1 tire_config);

#endif //  _WHEELSPEEDS_H