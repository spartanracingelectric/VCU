// Torque Vectoring header

#include "wheelspeeds.h"
#include "sensors.h"

typedef struct _TorqueVectoring TorqueVectoring;

TorqueVectoring* TorqueVectoring_new(float friction, ubyte2 Fz, float trim, ubyte2 speed, float slipRatio); //Creates struct of TractionControl into a function TorqueVectoring_new