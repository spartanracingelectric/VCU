#ifndef _BRAKEPRESSURESENSOR_H
#define _BRAKEPRESSURESENSOR_H

#include "IO_Driver.h"
#include "sensors.h"

//After update(), access to tps Sensor objects should no longer be necessary.
//In other words, only updateFromSensors itself should use the tps Sensor objects
//Also, all values in the TorqueEncoder object are from
typedef struct _BrakePressureSensor
{
    bool bench;

    Sensor *bps0;
    //Sensor* bps1;

    ubyte2 bps0_calibMin; //2 bytes for ADC, 4 bytes if switch to digital/timer/PWM
    ubyte2 bps0_calibMax;
    bool bps0_reverse;
    ubyte2 bps0_value;
    float4 bps0_percent;

    /*ubyte4 bps1_calibMin;
    ubyte4 bps1_calibMax;
    bool bps1_reverse; 
    ubyte4 bps1_value;
    float4 bps1_percent;
    */

    bool runCalibration;
    ubyte4 timestamp_calibrationStart;
    ubyte1 calibrationRunTime;

    bool calibrated;
    float4 percent;
    bool brakesAreOn;
    bool implausibility;
} BrakePressureSensor;

BrakePressureSensor *BrakePressureSensor_new(void);
void BrakePressureSensor_update(BrakePressureSensor *me, bool bench);
void BrakePressureSensor_getIndividualSensorPercent(BrakePressureSensor *me, ubyte1 sensorNumber, float4 *percent);
void BrakePressureSensor_resetCalibration(BrakePressureSensor *me);
void BrakePressureSensor_saveCalibrationToEEPROM(BrakePressureSensor *me);
void BrakePressureSensor_loadCalibrationFromEEPROM(BrakePressureSensor *me);
void BrakePressureSensor_startCalibration(BrakePressureSensor *me, ubyte1 secondsToRun);
void BrakePressureSensor_calibrationCycle(BrakePressureSensor *me, ubyte1 *errorCount);
void BrakePressureSensor_getPedalTravel(BrakePressureSensor *me, ubyte1 *errorCount, float4 *pedalPercent);

#endif //  _BRAKEPRESSURESENSOR_H
