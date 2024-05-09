//"Include guard" - prevents this file from being #included more than once
#ifndef _MOTORCONTROLLER_H
#define _MOTORCONTROLLER_H

#include "IO_CAN.h"
#include "IO_Driver.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "readyToDriveSound.h"

#define POWER_LIM_UPPER_POWER_THRESH 80 * 1000
#define POWER_LIM_LOWER_POWER_THRESH 70 * 1000
#define POWER_LIM_LOWER_TORQUE_THRESH 170 * 10
#define POWER_LIM_UPPER_TORQUE_THRESH 240 * 10
#define POWER_LIM_TAKEAWAY_SCALAR ((POWER_LIM_UPPER_TORQUE_THRESH - POWER_LIM_LOWER_TORQUE_THRESH)/10) / ((POWER_LIM_UPPER_POWER_THRESH - POWER_LIM_LOWER_POWER_THRESH)/1000)

#include "serial.h"

typedef enum { ENABLED, DISABLED, UNKNOWN } Status;

//Rotation direction as viewed from shaft end of motor
//0 = CW = REVERSE (for our car)
//1 = CCW = FORWARD (for our car)
typedef enum { CLOCKWISE, COUNTERCLOCKWISE, FORWARD, REVERSE, _0, _1 } Direction;

// Regen mode
typedef enum { REGENMODE_OFF = 0, REGENMODE_FORMULAE, REGENMODE_HYBRID, REGENMODE_TESLA, REGENMODE_FIXED } RegenMode;

typedef struct _MotorController MotorController;

MotorController* MotorController_new(SerialManager* sm, ubyte2 canMessageBaseID, Direction initialDirection, sbyte2 torqueMaxInDNm, sbyte1 minRegenSpeedKPH, sbyte1 regenRampdownStartSpeed);

//----------------------------------------------------------------------------
// Command Functions
//----------------------------------------------------------------------------
//CAN Message Parameters
//Note: Speed Command (angular velocity) not used when in torque mode
void MCM_commands_setTorqueDNm(MotorController* me, sbyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
void MCM_commands_setDirection(MotorController* me, Direction rotation);
void MCM_commands_setInverter(MotorController* me, Status inverterState);
void MCM_commands_setDischarge(MotorController* me, Status dischargeState);
void MCM_commands_setTorqueLimit(MotorController* me, sbyte2 torqueLimit);

void MCM_commands_resetUpdateCountAndTime(MotorController* me);
ubyte4 MCM_commands_getTimeSinceLastCommandSent(MotorController* me);

//----------------------------------------------------------------------------
// Mutator Functions
//----------------------------------------------------------------------------
//Allow other object access to the private struct
//Note: only added as needed, not necessarily comprehensive
void MCM_setRegen_TorqueLimitDNm(MotorController* mcm, ubyte2 torqueLimit);
void MCM_setRegen_TorqueAtZeroPedalDNm(MotorController* mcm, ubyte2 torqueZero);
void MCM_setRegen_PercentBPSForMaxRegen(MotorController* mcm, float4 percentBPS);
void MCM_setRegen_PercentAPPSForCoasting(MotorController* mcm, float4 percentAPPS);

sbyte4 MCM_getPower(MotorController* me);

sbyte4 MCM_getGroundSpeedKPH(MotorController* me);

sbyte2 MCM_getRegenBPSForMaxRegenZeroToFF(MotorController* me);
sbyte2 MCM_getRegenAPPSForMaxCoastingZeroToFF(MotorController* me);
ubyte2 MCM_get_max_torque_power_limit(MotorController *me);

//----------------------------------------------------------------------------
//Inter-object functions
//----------------------------------------------------------------------------
// void MCM_readTCSSettings(MotorController* me, Sensor* TCSSwitchUp, Sensor* TCSSwitchDown, Sensor* TCSPot);
void MCM_setRegenMode(MotorController *me, RegenMode regenMode);
void MCM_calculateCommands(MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps);

void MCM_relayControl(MotorController* mcm);
void MCM_inverterControl(MotorController* mcm, TorqueEncoder* tps, BrakePressureSensor* bps, ReadyToDriveSound* rtds);

#endif // _MOTORCONTROLLER_H
