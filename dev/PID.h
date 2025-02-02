#ifndef _PID_H
#define _PID_H

#include <stdlib.h>  // for malloc
// --------------------------------------------------------------------
// Define custom data types once here
// --------------------------------------------------------------------
typedef signed   char  sbyte1;
typedef signed   short sbyte2;
typedef signed   int   sbyte4;
typedef signed   int   sbyte4;
typedef unsigned char  ubyte1;
typedef unsigned short ubyte2;
typedef unsigned int   ubyte4;

typedef enum {
    FALSE = 0,
    TRUE = 1
} bool;

// --------------------------------------------------------------------
// PID structure
// --------------------------------------------------------------------
typedef struct _PID {
    sbyte1 Kp;               // Proportional gain
    sbyte1 Ki;               // Integral gain
    sbyte1 Kd;               // Derivative gain

    sbyte4 setpoint;         // Target value
    sbyte4 previousError;
    sbyte4 totalError;
    sbyte4 dH;               // “Scaled” delta-time factor for discrete PID
    sbyte4 output;           // The final PID output
    sbyte4 proportional;
    sbyte4 integral;
    sbyte4 derivative;
    sbyte4 saturationValue;
    bool   antiWindupFlag;
} PID;

/**
 * @brief Create a new PID controller instance.
 * @param Kp   Proportional gain   (in deci-units)
 * @param Ki   Integral gain       (in deci-units)
 * @param Kd   Derivative gain     (in deci-units)
 * @param saturationValue  If > 0, maximum allowable setpoint or clamp
 */
PID *PID_new(sbyte1 Kp, sbyte1 Ki, sbyte1 Kd, sbyte4 saturationValue);

// ------------------ Computation -------------------
void PID_computeOutput(PID *pid, sbyte4 sensorValue);

// ------------------ Setter Functions -------------------
void PID_updateGainValues(PID* pid, sbyte1 Kp, sbyte1 Ki, sbyte1 Kd);
void PID_setTotalError(PID* pid, sbyte4 error);
void PID_setSaturationPoint(PID *pid, sbyte4 saturationValue);
void PID_updateSetpoint(PID *pid, sbyte4 setpoint);

// ------------------ Getter Functions -------------------
sbyte1 PID_getKp(PID *pid);
sbyte1 PID_getKi(PID *pid);
sbyte1 PID_getKd(PID *pid);
sbyte4 PID_getSetpoint(PID *pid);
sbyte4 PID_getPreviousError(PID *pid);
sbyte4 PID_getTotalError(PID* pid);
sbyte4 PID_getOutput(PID *pid);
sbyte4 PID_getProportional(PID *pid);
sbyte4 PID_getIntegral(PID *pid);
sbyte4 PID_getDerivative(PID *pid);
sbyte4 PID_getSaturationValue(PID *pid);
bool   PID_getAntiWindupFlag(PID *pid);

#endif //_PID_H

