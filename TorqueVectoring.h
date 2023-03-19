// Torque Vectoring header

#include "wheelspeeds.h"
#include "sensors.h"

typedef struct _TorqueVectoring TorqueVectoring;

TorqueVectoring* TorqueVectoring_new(ubyte1 corner, float friction, ubyte2 Fz); //Creates struct of TractionControl into a function TorqueVectoring_new