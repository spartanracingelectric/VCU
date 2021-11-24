//"Include guard" - prevents this file from being #included more than once
#ifndef _MOTORCONTROLLER_H
#define _MOTORCONTROLLER_H

#include "IO_CAN.h"
#include "IO_Driver.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "readyToDriveSound.h"
//#include "safety.h"
#include "serial.h"

//typedef enum { TORQUE, DIRECTION, INVERTER, DISCHARGE, TORQUELIMIT} MCMCommand;
typedef enum { ENABLED, DISABLED, UNKNOWN } Status;

//Rotation direction as viewed from shaft end of motor
//0 = CW = REVERSE (for our car)
//1 = CCW = FORWARD (for our car)
typedef enum { CLOCKWISE, COUNTERCLOCKWISE, FORWARD, REVERSE, _0, _1 } Direction;

// Regen mode
typedef enum { REGENMODE_OFF = 0, REGENMODE_FORMULAE, REGENMODE_HYBRID, REGENMODE_TESLA } RegenMode;

struct _MotorController
{
    SerialManager *serialMan;
    //----------------------------------------------------------------------------
    // Controller statuses/properties
    //----------------------------------------------------------------------------
    // These represent the state of the controller (set at run time, not compile
    // time.)  These are updated by canInput.c
    //----------------------------------------------------------------------------
    ubyte2 canMessageBaseId; //Starting message ID for messages that will come in from this controller
    ubyte4 timeStamp_inverterEnabled;

    //Motor controller torque units are in 10ths (500 = 50.0 Nm)
    //Positive = accel, negative = regen
    //Reverse not allowed
    ubyte2 torqueMaximumDNm; //Max torque that can be commanded in deciNewton*meters ("100" = 10.0 Nm)

    //Regen torque calculations in whole Nm..?
    RegenMode regen_mode;                //Software reading of regen knob position.  Each mode has different regen behavior (variables below).
    ubyte2 regen_torqueLimitDNm;         //Tuneable value.  Regen torque (in Nm) at full regen.  Positive value.
    ubyte2 regen_torqueAtZeroPedalDNm;   //Tuneable value.  Amount of regen torque (in Nm) to apply when both pedals at 0% travel.  Positive value.
    float4 regen_percentBPSForMaxRegen;  //Tuneable value.  Amount of brake pedal required for full regen. Value between zero and one.
    float4 regen_percentAPPSForCoasting; //Tuneable value.  Amount of accel pedal required to exit regen.  Value between zero and one.
    sbyte1 regen_minimumSpeedKPH;        //Assigned by main
    sbyte1 regen_SpeedRampStart;

    bool relayState;
    bool previousHVILState;
    ubyte4 timeStamp_HVILLost;

    ubyte4 timeStamp_HVILOverrideCommandReceived;
    bool HVILOverride;

    ubyte4 timeStamp_InverterEnableOverrideCommandReceived;
    ubyte4 timeStamp_InverterDisableOverrideCommandReceived;
    Status InverterOverride;

    ubyte1 startupStage;
    Status lockoutStatus;
    Status inverterStatus;
    bool startRTDS;
    /*ubyte4 vsmStatus0;      //0xAA Byte 0,1
    ubyte4 vsmStatus1;      //0xAA Byte 0,1
    ubyte4 vsmStatus2;      //0xAA Byte 0,1
    ubyte4 vsmStatus3;      //0xAA Byte 0,1
    ubyte4 faultCodesPOST; //0xAB Byte 0-3
    ubyte4 faultCodesRUN;  //0xAB Byte 4-7*/

    ubyte1 faultHistory[8];

    sbyte2 motor_temp;
    sbyte4 DC_Voltage;
    sbyte4 DC_Current;

    sbyte2 commandedTorque;
    ubyte4 currentPower;

    sbyte2 motorRPM;
    //----------------------------------------------------------------------------
    // Control parameters
    //----------------------------------------------------------------------------
    // These are updated by ??? and will be sent to the MCM over CAN
    //----------------------------------------------------------------------------
    //struct _commands {
    ubyte4 timeStamp_lastCommandSent; //from IO_RTC_StartTime(&)
    ubyte2 updateCount;               //Number of updates since lastCommandSent

    sbyte2 commands_torque;
    sbyte2 commands_torqueLimit;
    ubyte1 commands_direction;
    //unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
    Status commands_discharge;
    Status commands_inverter;
    //ubyte1 controlSwitches; // example: 0b00000001 = inverter is enabled, discharge is disabled

    /*
    //----------------------------------------------------------------------------
    // Control functions - for functions nested within struct
    //----------------------------------------------------------------------------
    void(*setTorque)(MotorController* me, ubyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
    void(*setDirection)(MotorController* me, Direction rotation);
    void(*setInverter)(MotorController* me, Status inverterState);
    void(*setDischarge)(MotorController* me, Status dischargeState);
    void(*setTorqueLimit)(MotorController* me, ubyte2 torqueLimit);
    //        void(*motorController_setTorque)(MotorController* me, ubyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
    //        void(*motorController_setDirection)(MotorController* me, Direction rotation);
    //        void(*motorController_setInverter)(MotorController* me, Status inverterState);
    //        void(*motorController_setDischarge)(MotorController* me, Status dischargeState);
    //        void(*motorController_setTorqueLimit)(MotorController* me, ubyte2 torqueLimit);
    void(*updateLockoutStatus)(MotorController* me, Status newState);
    void(*updateInverterStatus)(MotorController* me, Status newState);
    void(*getLockoutStatus)(MotorController* me);
    void(*getInverterStatus)(MotorController* me);
    }
    */
    //_commands commands;
    //};
};

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
//void setCommand(MotorController* me, MCMCommand command, void* setting);


sbyte2 MCM_commands_getTorque(MotorController* me); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
Direction MCM_commands_getDirection(MotorController* me);
Status MCM_commands_getInverter(MotorController* me);
Status MCM_commands_getDischarge(MotorController* me);
sbyte2 MCM_commands_getTorqueLimit(MotorController* me); 

ubyte2 MCM_commands_getUpdateCount(MotorController* me);
void MCM_commands_resetUpdateCountAndTime(MotorController* me);
ubyte4 MCM_commands_getTimeSinceLastCommandSent(MotorController* me);

//----------------------------------------------------------------------------
// Update Functions (CAN Inputs)
//----------------------------------------------------------------------------
//void updatefromCAN(MotorController* me, CANFRAME or MOVE THIS EXTERNAL);
void MCM_updateLockoutStatus(MotorController* me, Status newState);
void MCM_updateInverterStatus(MotorController* me, Status newState);

//----------------------------------------------------------------------------
// Status Functions (CAN Inputs)
//----------------------------------------------------------------------------
Status MCM_getLockoutStatus(MotorController* me);
Status MCM_getInverterStatus(MotorController* me);

sbyte4 MCM_getPower(MotorController* me);
ubyte2 MCM_getCommandedTorque(MotorController* me);

bool MCM_getHvilOverrideStatus(MotorController* me);
bool MCM_getHvilOverrideStatus(MotorController* me);

void MCM_setRTDSFlag(MotorController* me, bool start);
bool MCM_getRTDSFlag(MotorController* me);
//MOVE ALL COUNT UPDATES INTO THESE FUNCTIONS

//void motorController_UpdateFromCan(IO_CAN_DATA_FRAME *canMessage); //Update the MCU object from its CAN messages
//void motorController_SendControlMessage(IO_CAN_DATA_FRAME *canMessage); //This is an alias for canOutput_sendMcuControl
//void motorController_setAllCommands(ReadyToDriveSound* rtds);

sbyte2 MCM_getTemp(MotorController* me);
sbyte2 MCM_getMotorTemp(MotorController* me);

sbyte2 MCM_getGroundSpeedKPH(MotorController* me);
sbyte1 MCM_getRegenMinSpeed(MotorController* me);
sbyte1 MCM_getRegenRampdownStartSpeed(MotorController* me);

ubyte1 MCM_getRegenMode(MotorController* me);
sbyte2 MCM_getRegenTorqueLimitDNm(MotorController* me);
sbyte2 MCM_getRegenTorqueAtZeroPedalDNm(MotorController* me);
sbyte2 MCM_getRegenBPSForMaxRegenZeroToFF(MotorController* me);
sbyte2 MCM_getRegenAPPSForMaxCoastingZeroToFF(MotorController* me);

//----------------------------------------------------------------------------
//Inter-object functions
//----------------------------------------------------------------------------
// void MCM_readTCSSettings(MotorController* me, Sensor* TCSSwitchUp, Sensor* TCSSwitchDown, Sensor* TCSPot);
void MCM_setRegenMode(MotorController *me, RegenMode regenMode);
void MCM_calculateCommands(MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps);

void MCM_relayControl(MotorController* mcm, Sensor* HVILTermSense);
void MCM_inverterControl(MotorController* mcm, TorqueEncoder* tps, BrakePressureSensor* bps, ReadyToDriveSound* rtds);

void MCM_parseCanMessage(MotorController* mcm, IO_CAN_DATA_FRAME* mcmCanMessage);

ubyte1 MCM_getStartupStage(MotorController* me);
void MCM_setStartupStage(MotorController* me, ubyte1 stage);

#endif // _MOTORCONTROLLER_H
