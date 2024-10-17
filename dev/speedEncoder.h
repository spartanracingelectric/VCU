/*******************************************************************************
 * Same Formatting as torqueEncoder.h file
 ******************************************************************************/

#ifndef _SPEEDENCODER_H
#define _SPEEDENCODER_H

#include "IO_Driver.h"
#include "sensors.h"

//After updateFromSensors, access to tps Sensor objects should no longer be necessary.
//In other words, only updateFromSensors itself should use the tps Sensor objects
//Also, all values in the SpeedEncoder object are from
typedef struct _SpeedEncoder
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
} SpeedEncoder;

SpeedEncoder *SpeedEncoder_new(bool benchMode);
void SpeedEncoder_update(SpeedEncoder *me);
void SpeedEncoder_getIndividualSensorPercent(SpeedEncoder *me, ubyte1 sensorNumber, float4 *percent);
void SpeedEncoder_resetCalibration(SpeedEncoder *me);
void SpeedEncoder_saveCalibrationToEEPROM(SpeedEncoder *me);
void SpeedEncoder_loadCalibrationFromEEPROM(SpeedEncoder *me);
void SpeedEncoder_startCalibration(SpeedEncoder *me, ubyte1 secondsToRun);
void SpeedEncoder_calibrationCycle(SpeedEncoder *me, ubyte1 *errorCount);
//void SpeedEncoder_plausibilityCheck(SpeedEncoder* me, ubyte1* errorCount, bool* isPlausible);
void SpeedEncoder_getPedalTravel(SpeedEncoder *me, ubyte1 *errorCount, float4 *pedalPercent);
void SpeedEncoder_getOutputPercent(SpeedEncoder *me, float4 *outputPercent);

#endif //  _SPEEDENCODER_H