// Torque Vectoring header
#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

#include "wheelspeeds.h"
#include "sensors.h"

typedef struct TorqueVectoring
{
    float friction;
    ubyte2 Fz;
    float trim;
    ubyte2 speed;
    float slipRatio;
    //check data types
} TorqueVectoring;

TorqueVectoring* TorqueVectoring_init(float friction, ubyte2 Fz, float trim, ubyte2 speed, float slipRatio);

#endif