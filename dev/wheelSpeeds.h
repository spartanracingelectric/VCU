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

WheelSpeeds* WheelSpeeds_new(ubyte4 tireDiameterInches_F, ubyte4 tireDiameterInches_R, ubyte1 pulsesPerRotation_F, ubyte1 pulsesPerRotation_R);
void WheelSpeeds_update(WheelSpeeds* me, bool interpolate);
ubyte4 WheelSpeeds_getWheelSpeedMMPS(WheelSpeeds* me, Wheel corner);
ubyte4 WheelSpeeds_getWheelSpeedRPM(WheelSpeeds* me, Wheel corner, bool interpolate);
ubyte4 WheelSpeeds_getSlowestFront(WheelSpeeds* me);
ubyte4 WheelSpeeds_getFastestRear(WheelSpeeds* me);
ubyte4 WheelSpeeds_getGroundSpeedMMPS(WheelSpeeds* me, ubyte1 tire_config);
ubyte4 WheelSpeeds_getGroundSpeedKPH(WheelSpeeds *me, ubyte1 tire_config);

#endif //  _WHEELSPEEDS_H