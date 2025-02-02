/*****************************************************************************
 * powerLimit.c - Power Limiting using a PID controller & LUT
 * Initial Author(s): Harleen Sandhu / Shaun Gilmore
 *****************************************************************************/

#include "powerLimit.h"

#include <stdlib.h>  // for malloc

#include "motorController.h"

/** Small inline helpers to replace the calls from mathFunctions.h **/
static inline ubyte4 ubyte4_lowerStepInterval(ubyte4 val, ubyte4 step) {
    return (val / step) * step;
}
static inline ubyte4 ubyte4_upperStepInterval(ubyte4 val, ubyte4 step) {
    return ((val + step - 1) / step) * step;
}

// Constants
#ifndef POWERLIMITCONSTANTS
#define POWERLIMITCONSTANTS

#define VOLTAGE_STEP 5
#define RPM_STEP 160

#endif

/*****************************************************************************
 * Constructor
 ****************************************************************************/
PowerLimit* POWERLIMIT_new() {
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));

    // Create the internal PID with example Kp=40, Ki=20, Kd=0, saturation=231
    me->pid = PID_new(40, 20, 0, 231);

    // Default mode
    me->plMode = TQ_ONLY_MODE;

    me->plStatus = FALSE;
    me->plTorqueCommand = 0;
    me->plTargetPower = 80;  // kW
    me->plInitializationThreshold = me->plTargetPower - 15;
    me->plExit = TRUE;  
    me->plOn = FALSE;   // helper variable for use of plExit

    // LUT corners
    me->vFloorRFloor = 0;
    me->vFloorRCeiling = 0;
    me->vCeilingRFloor = 0;
    me->vCeilingRCeiling = 0;

    return me;
}

void POWERLIMIT_setLimpModeOverride(PowerLimit* me) {
    // Example placeholder
    // if (some_button_press) {
    //     me->plMode = 5;
    //     me->plTargetPower = 20;
    //     me->plInitializationThreshold = 0;
    // }
}

void PowerLimit_calculateCommand(PowerLimit* me, MotorController* mcm,
                                 bool fieldWeakening) {
    // ensure no negative threshold
    me->plInitializationThreshold =
        (me->plTargetPower > 15) ? (me->plTargetPower - 15) : 0;

    switch (me->plMode) {
        case TQ_ONLY_MODE:
            POWERLIMIT_calculateTorqueCommandTorqueEquation(me, mcm);
            break;
        case PID_ONLY_MODE:
            POWERLIMIT_calculateTorqueCommandPowerPID(me, mcm);
            break;
        case LUT_ONLY_MODE:
            // write the saftey checks for these make sure that if the
            // lut is out of range it uses tq equation
            POWERLIMIT_calculateTorqueCommand(me, mcm);
            break;
        case TQ_LUT_MODE:
            // if there is no field weakening, use TQ equation; if there is field weakening, use LUT
            if (fieldWeakening)
                POWERLIMIT_calculateTorqueCommand(me, mcm);
            else
                POWERLIMIT_calculateTorqueCommandTorqueEquation(me, mcm);
            break;
        default:
            me->plStatus = FALSE;
            break;
    }
}
/*****************************************************************************
 *  LUT-based approach:
 *  - once power >= threshold, we do a LUT-based torque limit w/ simple PID
 ****************************************************************************/
void POWERLIMIT_calculateTorqueCommand(PowerLimit* me, MotorController* mcm) {
    // if PL does not end on exit condition and PL is currently on OR PL entry condition is met
    if ((!me->plExit && me->plOn) || (MCM_getPower(mcm) / 1000) >= 
          me->plInitializationThreshold) {
        if (!me->plExit && !me->plStatus) {
            me->plOn = TRUE;
        }
        me->plStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model
        // is 0.027 ohms
        sbyte4 noLoadVoltage = (mcmCurrent * 27 / 1000) + mcmVoltage;
        // 27 / 100 (0.027) is the estimated IR. Should attempt
        // to revalidate on with new powerpack.

        // sbyte4 pidSetpoint = (sbyte4)POWERLIMIT_retrieveTorqueFromLUT(me,
        // &me->hashtable[me->plMode], noLoadVoltage, motorRPM); sbyte2
        // pidSetpoint = (sbyte2)POWERLIMIT_retrieveTorqueFromLUT(me,
        // me->hashtable, noLoadVoltage, motorRPM);

        // issue here
        sbyte2 pidSetpoint =
            POWERLIMIT_retrieveTorqueFromLUT(me, noLoadVoltage, motorRPM);

        // TQ equation. uncomment to run this instead
        // pidSetpoint = (sbyte2)(me->plTargetPower * 9549 /
        // MCM_getMotorRPM(mcm));

        // If the LUT gives a bad value this is our catch all
        if (pidSetpoint < 0 | pidSetpoint > 231) {
            pidSetpoint =
                (sbyte2)(me->plTargetPower * 9549 / MCM_getMotorRPM(mcm));
        }

        sbyte2 commandedTorque = (sbyte2)MCM_getCommandedTorque(mcm);

        PID_updateSetpoint(me->pid, pidSetpoint);
        PID_computeOutput(me->pid, commandedTorque);
        me->plTorqueCommand = (commandedTorque + PID_getOutput(me->pid)) *
                              10;  // deciNewton-meters
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    } else {
        if (me->plExit) {
            me->plStatus = FALSE;
            MCM_update_PL_setTorqueCommand(mcm, -1);
            MCM_set_PL_updateStatus(mcm, me->plStatus);
        }
    }
}

/*****************************************************************************
 * Retrieve torque from LUT + interpolation
 ****************************************************************************/
sbyte4 POWERLIMIT_retrieveTorqueFromLUT(PowerLimit* me, sbyte4 voltage,
                                        sbyte4 rpm) {
    // LUT Lower Bounds
    ubyte4 VOLTAGE_MIN = 280;
    ubyte4 RPM_MIN = 2000;

    // Convert to “indices”
    ubyte4 rpmInput = (rpm > RPM_MIN) ? (rpm - RPM_MIN) : 0;
    ubyte4 voltageInput = (voltage > VOLTAGE_MIN) ? (voltage - VOLTAGE_MIN) : 0;

    ubyte4 voltageFloor =
        ubyte4_lowerStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 voltageCeiling =
        ubyte4_upperStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 rpmFloor = ubyte4_lowerStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    ubyte4 rpmCeiling = ubyte4_upperStepInterval(rpmInput, RPM_STEP) + RPM_MIN;

    // For interpolation
    ubyte4 voltageLowerDiff =
        (voltage > voltageFloor) ? (voltage - voltageFloor) : 0;
    ubyte4 voltageUpperDiff =
        (voltageCeiling > voltage) ? (voltageCeiling - voltage) : 0;
    ubyte4 rpmLowerDiff = (rpm > rpmFloor) ? (rpm - rpmFloor) : 0;
    ubyte4 rpmUpperDiff = (rpmCeiling > rpm) ? (rpmCeiling - rpm) : 0;

    // Grab corners
    me->vFloorRFloor = POWERLIMIT_getTorqueFromArray(voltageFloor, rpmFloor);
    me->vFloorRCeiling =
        POWERLIMIT_getTorqueFromArray(voltageFloor, rpmCeiling);
    me->vCeilingRFloor =
        POWERLIMIT_getTorqueFromArray(voltageCeiling, rpmFloor);
    me->vCeilingRCeiling =
        POWERLIMIT_getTorqueFromArray(voltageCeiling, rpmCeiling);

    // If voltageFloor == voltageCeiling => no need 2D interpolation
    // If rpmFloor == rpmCeiling => same
    // We’ll do a simplified interpolation approach:

    // Edge case: same voltage “bin”
    if (voltageFloor == voltageCeiling) {
        // Just interpolate along rpm dimension
        sbyte4 ret =
            (sbyte4)(me->vFloorRFloor +
                     (rpmLowerDiff * (me->vFloorRCeiling - me->vFloorRFloor) /
                      RPM_STEP));
        return ret;
    }

    // Edge case: same rpm “bin”
    if (rpmFloor == rpmCeiling) {
        // Interpolate along voltage dimension
        sbyte4 ret =
            (sbyte4)(me->vFloorRFloor +
                     (voltageLowerDiff *
                      (me->vCeilingRFloor - me->vFloorRFloor) / VOLTAGE_STEP));
        return ret;
    }

    // Full bilinear interpolation
    ubyte4 stepDivider = (ubyte4)VOLTAGE_STEP * RPM_STEP;
    ubyte4 torqueFloorFloor =
        (ubyte4)me->vFloorRFloor * voltageUpperDiff * rpmUpperDiff;
    ubyte4 torqueFloorCeiling =
        (ubyte4)me->vFloorRCeiling * voltageUpperDiff * rpmLowerDiff;
    ubyte4 torqueCeilingFloor =
        (ubyte4)me->vCeilingRFloor * voltageLowerDiff * rpmUpperDiff;
    ubyte4 torqueCeilingCeiling =
        (ubyte4)me->vCeilingRCeiling * voltageLowerDiff * rpmLowerDiff;

    return (sbyte4)((torqueFloorFloor + torqueFloorCeiling +
                     torqueCeilingFloor + torqueCeilingCeiling) /
                    stepDivider);
}

/*****************************************************************************
 * TQ Equation method
 ****************************************************************************/
void POWERLIMIT_calculateTorqueCommandTorqueEquation(PowerLimit* me,
                                                     MotorController* mcm) {
    PID_setSaturationPoint(me->pid, 8000);  // example bigger clamp

    // if PL does not end on exit condition and PL is currently on OR PL entry condition is met
    if ((!me->plExit && me->plOn) || (MCM_getPower(mcm) / 1000) >= 
          me->plInitializationThreshold) {
        if (!me->plExit && !me->plStatus) {
            me->plOn = TRUE;
        }
        me->plStatus = TRUE;

        sbyte4 motorRPM = MCM_getMotorRPM(mcm);
        if (motorRPM == 0) motorRPM = 1;  // avoid divide by zero

        // TQ eqn => P(kW)*9549 / rpm
        sbyte4 pidSetpoint =
            (sbyte4)((sbyte4)me->plTargetPower * 9549 / motorRPM);

        sbyte4 commandedTorque = MCM_getCommandedTorque(mcm);

        PID_updateSetpoint(me->pid, pidSetpoint);
        PID_computeOutput(me->pid, commandedTorque);

        me->plTorqueCommand =
            (sbyte4)((commandedTorque + PID_getOutput(me->pid)) * 10);
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    } else {
        if (me->plExit) {
            me->plStatus = FALSE;
            MCM_update_PL_setTorqueCommand(mcm, -1);
            MCM_set_PL_updateStatus(mcm, me->plStatus);
        }
    }
}

/*****************************************************************************
 * Power-based PID approach
 ****************************************************************************/
void POWERLIMIT_calculateTorqueCommandPowerPID(PowerLimit* me,
                                               MotorController* mcm) {
    PID_setSaturationPoint(me->pid, 80000);
    me->plMode = PID_ONLY_MODE;  // not strictly needed, but clarifies

    // if PL does not end on exit condition and PL is currently on OR PL entry condition is met
    if ((!me->plExit && me->plOn) || (MCM_getPower(mcm) / 1000) >= 
          me->plInitializationThreshold) {
        if (!me->plExit && !me->plStatus) {
            me->plOn = TRUE;
        }
        me->plStatus = TRUE;

        // Suppose MCM_getPower(mcm) is in W => convert to “PID scale”
        sbyte4 pidTargetValue = (sbyte4)(me->plTargetPower * 1000);
        sbyte4 pidCurrentValue = (sbyte4)(MCM_getPower(mcm) / 10);

        sbyte4 commandedTorque = MCM_getCommandedTorque(mcm);

        PID_updateSetpoint(me->pid, pidTargetValue);
        PID_computeOutput(me->pid, pidCurrentValue);

        // Simple ratio to adjust commandedTorque
        // In practice, you may want a safer formula
        // to avoid divide-by-zero if pidCurrentValue=0
        if (pidCurrentValue == 0) pidCurrentValue = 1;

        sbyte4 newTorque = (sbyte4)(commandedTorque +
                                    (commandedTorque * PID_getOutput(me->pid) /
                                     pidCurrentValue));

        // saturate
        if (newTorque > 231) newTorque = 231;

        me->plTorqueCommand = newTorque * 10;
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    } else {
        if (me->plExit) {
            me->plStatus = FALSE;
            MCM_update_PL_setTorqueCommand(mcm, -1);
            MCM_set_PL_updateStatus(mcm, me->plStatus);
        }
    }
}

/*****************************************************************************
 * The array-based LUT retrieval
 ****************************************************************************/
ubyte1 POWERLIMIT_getTorqueFromArray(ubyte4 noLoadVoltage, ubyte4 rpm) {
    // Example boundaries
    ubyte2 VOLTAGE_MIN = 280;
    ubyte2 VOLTAGE_MAX = 405;
    ubyte2 RPM_MIN = 2000;
    ubyte2 RPM_MAX = 6000;

    // Example LUT size
    const ubyte1 NUM_V = 26;
    const ubyte1 NUM_S = 26;

    // Hard-coded LUT for “80 kW” limiting example
    const ubyte1 POWER_LIM_LUT_80[26][26] = {
        {231, 231, 199, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231,
         231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {222, 229, 180, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231,
         231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {205, 214, 161, 228, 231, 231, 231, 231, 231, 231, 231, 231, 231,
         231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {187, 198, 146, 214, 221, 227, 231, 231, 231, 231, 231, 231, 231,
         231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {172, 180, 130, 198, 205, 214, 221, 226, 231, 231, 231, 231, 231,
         231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {157, 166, 117, 182, 189, 198, 205, 213, 218, 226, 231, 231, 231,
         231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {144, 152, 103, 168, 175, 183, 190, 199, 205, 213, 218, 231, 231,
         231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {131, 138, 90,  154, 161, 170, 177, 184, 193, 199, 207, 221, 231,
         231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {118, 126, 77,  141, 150, 157, 164, 171, 179, 185, 192, 208, 221,
         226, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227},
        {106, 114, 63,  131, 138, 145, 153, 159, 167, 173, 180, 198, 209,
         216, 216, 217, 217, 217, 217, 216, 217, 217, 217, 217, 217, 217},
        {95,  104, 48,  120, 127, 133, 140, 148, 155, 162, 169, 185, 199,
         205, 207, 207, 207, 208, 208, 208, 208, 208, 208, 208, 208, 208},
        {84,  93,  32,  109, 116, 123, 131, 137, 144, 151, 157, 174, 186,
         197, 198, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199},
        {73,  81,  0,   98,  106, 113, 121, 127, 134, 139, 147, 164, 176,
         186, 190, 191, 191, 191, 191, 191, 191, 192, 191, 191, 192, 192},
        {61,  71,  0,   88,  95,  103, 110, 117, 124, 131, 138, 153, 166,
         177, 182, 183, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184},
        {48,  59,  0,   77,  86,  94,  101, 108, 115, 120, 128, 144, 157,
         166, 175, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177},
        {34,  47,  0,   67,  76,  84,  91,  99,  105, 111, 119, 135, 147,
         157, 170, 170, 170, 171, 171, 171, 171, 171, 171, 171, 171, 171},
        {12,  33,  0,   56,  66,  74,  82,  89,  96,  103, 110, 126, 138,
         150, 159, 164, 164, 165, 165, 165, 165, 165, 165, 165, 165, 165},
        {0,   10,  0,   44,  55,  64,  72,  80,  88,  94,  102, 117, 129,
         139, 150, 156, 159, 159, 159, 159, 159, 160, 160, 160, 160, 160},
        {0,   0,   0,   30,  43,  54,  63,  71,  78,  86,  93,  109, 120,
         132, 141, 151, 153, 154, 154, 154, 154, 154, 154, 154, 155, 155},
        {0,   0,   0,   6,   29,  42,  52,  61,  69,  77,  85,  101, 113,
         125, 134, 142, 148, 149, 149, 150, 149, 150, 150, 150, 150, 150},
        {0,   0,   0,   0,   4,   28,  41,  51,  60,  68,  76,  93,  105,
         117, 126, 135, 142, 144, 145, 145, 145, 145, 145, 145, 145, 142},
        {0,   0,   0,   0,   0,   0,   27,  40,  50,  59,  68,  85,  98,
         109, 119, 127, 136, 139, 140, 140, 141, 141, 141, 141, 141, 141},
        {0,   0,   0,   0,   0,   0,   0,   26,  39,  49,  59,  77,  90,
         101, 111, 120, 128, 134, 136, 136, 137, 137, 137, 137, 137, 137},
        {0,  0,   0,   0,   0,   0,   0,   0,   25,  38,  49,  68,  83,
         94, 104, 113, 121, 129, 132, 132, 133, 133, 133, 133, 133, 133},
        {0,  0,  0,   0,   0,   0,   0,   0,   0,   24,  38,  60,  75,
         87, 97, 106, 115, 121, 127, 129, 129, 129, 129, 129, 129, 129},
        {0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   25,  51,  67,
         80, 90, 99, 107, 115, 122, 125, 126, 126, 126, 126, 126, 126}};

    // Basic clamping
    if (noLoadVoltage < VOLTAGE_MIN) noLoadVoltage = VOLTAGE_MIN;
    if (noLoadVoltage > VOLTAGE_MAX) noLoadVoltage = VOLTAGE_MAX;
    if (rpm < RPM_MIN) rpm = RPM_MIN;
    if (rpm > RPM_MAX) rpm = RPM_MAX;

    // Convert to LUT row/col
    ubyte2 col = (ubyte2)((noLoadVoltage - VOLTAGE_MIN) / VOLTAGE_STEP);
    ubyte2 row = (ubyte2)((rpm - RPM_MIN) / RPM_STEP);

    // Return the cell
    return POWER_LIM_LUT_80[row][col];
}

// ------------------------------------------------------------------------
//                          Getter Functions
// ------------------------------------------------------------------------
ubyte1 POWERLIMIT_getStatusCodeBlock(PowerLimit* me) { return me->plMode; }
bool POWERLIMIT_getStatus(PowerLimit* me) { return me->plStatus; }
ubyte1 POWERLIMIT_getMode(PowerLimit* me) { return me->plMode; }
sbyte4 POWERLIMIT_getTorqueCommand(PowerLimit* me) {
    return me->plTorqueCommand;
}
ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me) { return me->plTargetPower; }
ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me) {
    return me->plInitializationThreshold;
}
ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner) {
    // corner: 1->vFloorRFloor, 2->vFloorRCeiling, 3->vCeilingRFloor,
    // 4->vCeilingRCeiling
    switch (corner) {
        case 1:
            return me->vFloorRFloor;
        case 2:
            return me->vFloorRCeiling;
        case 3:
            return me->vCeilingRFloor;
        case 4:
            return me->vCeilingRCeiling;
        default:
            return 0xFF;
    }
}
