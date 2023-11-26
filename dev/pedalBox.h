#ifndef _PEDALBOX_H
#define _PEDALBOX_H

#include "IO_Driver.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "sensors.h"

typedef struct _PedalBox
{
    TorqueEncoder *tps;
    BrakePressureSensor *bps;

    bool calibrated;

} PedalBox;