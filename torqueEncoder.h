#ifndef _TORQUEENCODER_H
#define _TORQUEENCODER_H

#include "IO_Driver.h"
#include "sensors.h"

//After updateFromSensors, access to tps Sensor objects should no longer be necessary.
//In other words, only updateFromSensors itself should use the tps Sensor objects
//Also, all values in the TorqueEncoder object are from
typedef struct _TorqueEncoder
{
    bool bench;

    Sensor *tps0;
    Sensor *tps1;

    ubyte4 tps0_calibMin; //Must be 4 bytes to support PWM (digital/timer) sensor
    ubyte4 tps0_calibMax;
    bool tps0_reverse;
    ubyte4 tps0_value;
    float4 tps0_percent;

    ubyte4 tps1_calibMin;
    ubyte4 tps1_calibMax;
    bool tps1_reverse;
    ubyte4 tps1_value;
    float4 tps1_percent;

    bool runCalibration;
    ubyte4 timestamp_calibrationStart;
    ubyte1 calibrationRunTime;

    float4 outputCurveExponent;

    bool calibrated;
    float4 travelPercent;
    bool implausibility;
} TorqueEncoder;

TorqueEncoder *TorqueEncoder_new(bool benchMode);
void TorqueEncoder_update(TorqueEncoder *me);
void TorqueEncoder_getIndividualSensorPercent(TorqueEncoder *me, ubyte1 sensorNumber, float4 *percent);
void TorqueEncoder_resetCalibration(TorqueEncoder *me);
void TorqueEncoder_saveCalibrationToEEPROM(TorqueEncoder *me);
void TorqueEncoder_loadCalibrationFromEEPROM(TorqueEncoder *me);
void TorqueEncoder_startCalibration(TorqueEncoder *me, ubyte1 secondsToRun);
void TorqueEncoder_calibrationCycle(TorqueEncoder *me, ubyte1 *errorCount);
//void TorqueEncoder_plausibilityCheck(TorqueEncoder* me, ubyte1* errorCount, bool* isPlausible);
void TorqueEncoder_getPedalTravel(TorqueEncoder *me, ubyte1 *errorCount, float4 *pedalPercent);
void TorqueEncoder_getOutputPercent(TorqueEncoder *me, float4 *outputPercent);

#endif //  _TORQUEENCODER_H
