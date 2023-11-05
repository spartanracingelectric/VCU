#include <stdlib.h> //Needed for malloc
#include "IO_Driver.h"
#include "IO_DIO.h" //TEMPORARY - until MCM relay control  / ADC stuff gets its own object
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "motorController.h"
#include "mathFunctions.h"
#include "sensors.h"
#include "sensorCalculations.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "readyToDriveSound.h"
#include "serial.h"
#include "main.h"

#include "canManager.h"

extern Button Sensor_RTDButton;
extern Button Sensor_EcoButton;
extern Sensor Sensor_TCSKnob;       // used currently for regen
extern Button Sensor_HVILTerminationSense;

#define CELL_RESISTANCE 0.025f //Ohms resistance of the cell+ fuselink
#define CELLS_IN_PARALLEL 7
#define CELLS_IN_SERIES 96
const float4 PACK_RESISTANCE = (1/(CELLS_IN_PARALLEL/CELL_RESISTANCE))*CELLS_IN_SERIES; // pack resistance

/*****************************************************************************
 * Motor Controller (MCM)
 ****************************************************************************/

MotorController *MotorController_new(SerialManager *sm, ubyte2 canMessageBaseID, Direction initialDirection, sbyte2 torqueMaxInDNm, sbyte1 minRegenSpeedKPH, sbyte1 regenRampdownStartSpeed)
{
    MotorController *me = (MotorController *)malloc(sizeof(struct _MotorController));
    me->serialMan = sm;

    me->canMessageBaseId = canMessageBaseID;
    //Dummy timestamp for last MCU message
    MCM_commands_resetUpdateCountAndTime(me);

    me->lockoutStatus = UNKNOWN;
    me->inverterStatus = UNKNOWN;

    me->motorRPM = 0;
    me->DC_Voltage = 0;
    me->DC_Current = 0;
    me->power_torque_lim = 0;

    me->commands_direction = initialDirection;
    me->commands_torqueLimit = me->torqueMaximumDNm = torqueMaxInDNm;

    me->regen_mode = REGENMODE_OFF;
    me->regen_torqueLimitDNm = 0;
    me->regen_torqueAtZeroPedalDNm = 0;
    me->regen_percentBPSForMaxRegen = 1; //zero to one.. 1 = 100%
    me->regen_percentAPPSForCoasting = 0;
    me->regen_minimumSpeedKPH = minRegenSpeedKPH;       //Assigned by main
    me->regen_SpeedRampStart = regenRampdownStartSpeed; //Assigned by main

    me->startupStage = 0; //Off

    me->relayState = FALSE; //Low

    me->motor_temp = 99;

    me->LaunchControl_TorqueLimit = 0;

    me->LCState = FALSE;
    
    return me;
}

void MCM_setRegenMode(MotorController *me, RegenMode regenMode)
{
    switch (regenMode)
    {
    case REGENMODE_FORMULAE: //Position 1 = Coasting mode (Formula E mode)
        me->regen_mode = 1;
        me->regen_torqueLimitDNm = me->torqueMaximumDNm * 0.5;
        me->regen_torqueAtZeroPedalDNm = 0;
        me->regen_percentAPPSForCoasting = 0;
        me->regen_percentBPSForMaxRegen = .3; //zero to one.. 1 = 100%
        break;

    case REGENMODE_HYBRID: //Position 2 = light "engine braking" (Hybrid mode)
        me->regen_mode = 2;
        me->regen_torqueLimitDNm = me->torqueMaximumDNm * 0.5;
        me->regen_torqueAtZeroPedalDNm = me->regen_torqueLimitDNm * 0.3;
        me->regen_percentAPPSForCoasting = .2;
        me->regen_percentBPSForMaxRegen = .3; //zero to one.. 1 = 100%
        break;

    case REGENMODE_TESLA: //Position 3 = One pedal driving (Tesla mode)
        me->regen_mode = 3;
        me->regen_torqueLimitDNm = me->torqueMaximumDNm * .5;
        me->regen_torqueAtZeroPedalDNm = me->regen_torqueLimitDNm;
        me->regen_percentAPPSForCoasting = .1;
        me->regen_percentBPSForMaxRegen = 0;
        break;

    case REGENMODE_FIXED: //Position 4 = Fixed Regen
        me->regen_mode = 4;
        me->regen_torqueLimitDNm = 250; //1000 = 100Nm
        me->regen_torqueAtZeroPedalDNm = me->regen_torqueLimitDNm;
        me->regen_percentAPPSForCoasting = .05;
        me->regen_percentBPSForMaxRegen = 0;
        break;

        // TODO:  User customizable regen settings - Issue #97
        // case REGENMONDE_CUSTOM:
        //     me->regen_mode = 4;
        //     me->regen_torqueLimitDNm = 0;
        //     me->regen_torqueAtZeroPedalDNm = 0;
        //     me->regen_percentBPSForMaxRegen = 0; //zero to one.. 1 = 100%
        //     me->regen_percentAPPSForCoasting = 0;
        //     break;

    case REGENMODE_OFF:
    default:
        me->regen_mode = REGENMODE_OFF;
        me->regen_torqueLimitDNm = 0;
        me->regen_torqueAtZeroPedalDNm = 0;
        me->regen_percentAPPSForCoasting = 0;
        me->regen_percentBPSForMaxRegen = 0; //zero to one.. 1 = 100%
        break;
    }
}

/*****************************************************************************
* Motor Control Functions
* Reads sensor objects and sets MCM control object values, which will be picked up
* later by CAN function
* > Direction
* > Torque
*   - Delay command after inverter enable (temporary until noise fix)
*   - Calculate Nm to request based on pedal position
*   - DO NOT limit Nm based on external system limits - that is handled by safety.c,
*     and we still want to know the driver's requested torque
*   - DO limit Nm based on driver inputs (dash settings for regen)
*   - Keep track of update count to prevent CANbus overload
* > Torque limit
*   - Get from TCS function
* Manages the different phases startup/ready-to-drive procedure
* > Turn on MCM relay?  Should this be done elsewhere?
* > Disable inverter lockout
* > Listen for driver to complete startup sequence
* > Enable inverter
* > Play RTDS
****************************************************************************/
void MCM_calculateCommands(MotorController *me, TorqueEncoder *tps, BrakePressureSensor *bps)
{
    //----------------------------------------------------------------------------
    // Control commands
    //Note: Safety checks (torque command limiting) are done EXTERNALLY.  This is a preliminary calculation
    //which should return the intended torque based on pedals
    //Note: All stored torque values should be positive / unsigned
    //----------------------------------------------------------------------------
    MCM_commands_setDischarge(me, DISABLED);
    MCM_commands_setDirection(me, FORWARD); //1 = forwards for our car, 0 = reverse

    sbyte2 torqueOutput = 0;
    sbyte2 appsTorque = 0;
    sbyte2 bpsTorque = 0;

    float4 appsOutputPercent;

    TorqueEncoder_getOutputPercent(tps, &appsOutputPercent);
    me->power_torque_lim = MCM_get_max_torque_power_limit(me);
    appsTorque = me->torqueMaximumDNm * getPercent(appsOutputPercent, me->regen_percentAPPSForCoasting, 1, TRUE) - me->regen_torqueAtZeroPedalDNm * getPercent(appsOutputPercent, me->regen_percentAPPSForCoasting, 0, TRUE);
    bpsTorque = 0 - (me->regen_torqueLimitDNm - me->regen_torqueAtZeroPedalDNm) * getPercent(bps->percent, 0, me->regen_percentBPSForMaxRegen, TRUE);

    if(me->LCState == TRUE){
        torqueOutput = me->LaunchControl_TorqueLimit;
    } else if (me->LaunchControl_TorqueLimit == 0){
        torqueOutput = me->LaunchControl_TorqueLimit;
    } else {
        torqueOutput = appsTorque + bpsTorque;
        //torqueOutput = me->torqueMaximumDNm * tps->percent;  //REMOVE THIS LINE TO ENABLE REGEN
    }

    if ((torqueOutput > me->power_torque_lim * 10) && POWER_LIMIT && !me->LCState) { // The requested torque is greater than the power limit, we have the power limit enables, and were not trying to launch
        torqueOutput = me->power_torque_lim * 10; // it is in DNm
    }
    
    MCM_commands_setTorqueDNm(me, torqueOutput);

    //Causes MCM relay to be driven after 30 seconds with TTC60?
    me->HVILOverride = (IO_RTC_GetTimeUS(me->timeStamp_HVILOverrideCommandReceived) < 1000000);

    //Temporarily disable MCM relay control via HVILOverride
    //me->HVILOverride = FALSE;

    // Inverter override
    if (IO_RTC_GetTimeUS(me->timeStamp_InverterDisableOverrideCommandReceived) < 1000000)
        me->InverterOverride = DISABLED;
    else if (IO_RTC_GetTimeUS(me->timeStamp_InverterDisableOverrideCommandReceived) < 1000000)
        me->InverterOverride = ENABLED;
    else
        me->InverterOverride = UNKNOWN;
}

void MCM_relayControl(MotorController *me)
{
    //If HVIL Term Sense is low (HV is down)
    if (Sensor_HVILTerminationSense.sensorValue == FALSE && me->HVILOverride == FALSE)
    {
        //If we just noticed the HVIL went low
        if (me->previousHVILState == TRUE)
        {
            SerialManager_send(me->serialMan, "Term sense went low\n");
            IO_RTC_StartTime(&me->timeStamp_HVILLost);
        }

        //If the MCM is on (and we lost HV)
        if (me->relayState == TRUE)
        {
            //Okay to turn MCM off once 0 torque is commanded, or after 2 sec
            //TODO: SIMILAR CODE SHOULD BE EMPLOYED AT HVIL SHUTDOWN CONTROL PIN
            if (me->commandedTorque == 0 || IO_RTC_GetTimeUS(me->timeStamp_HVILLost) > 2000000)
            {
                IO_DO_Set(IO_DO_00, FALSE); //Need MCM relay object
                me->relayState = FALSE;
            }
            else
            {
                //Safety.c will command zero torque
                //For now do nothing
            }
        }
        me->startupStage = 0;
        me->inverterStatus = UNKNOWN;
        me->lockoutStatus = UNKNOWN;
        me->previousHVILState = FALSE;
    }
    else // HVILTermSense->sensorValue == TRUE || me->HVILOverride == TRUE
    {
        //If HVIL just changed, send a message
        if (me->previousHVILState == FALSE)
        {
            SerialManager_send(me->serialMan, "Term sense went high\n");
            if (me->startupStage == 0)
            {
                me->startupStage = 1;
            } //Reset the startup procedure because HV just went high and we are now turning on the MCM
        }
        me->previousHVILState = TRUE;

        //Turn on the MCM relay
        IO_DO_Set(IO_DO_00, TRUE);
        me->relayState = TRUE;
    }
}

//See diagram at https://onedrive.live.com/redir?resid=F9BB8F0F8FDB5CF8!30410&authkey=!ABSF-uVH-VxQRAs&ithint=file%2chtml
void MCM_inverterControl(MotorController *me, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds)
{
    float4 RTDPercent;
    RTDPercent = (Sensor_RTDButton.sensorValue == TRUE ? 1 : 0);
    //----------------------------------------------------------------------------
    // Determine inverter state
    //----------------------------------------------------------------------------
    //New Handshake NOTE: Switches connected to ground.. TRUE = high = off = disconnected = open circuit, FALSE = low = grounded = on = connected = closed circuit
    switch (me->startupStage)
    {
    case 0: //MCM relay is off --> stay until lockout is enabled;
        //It shouldn't be necessary to check mcm relayState or inverter status <> UNKNOWN
        //but if we have trouble we can add that here.

    case 1: //MCM relay is on, lockout=enabled, inverter=disabled --> stay until lockout is disabled
        //Actions to perform upon entering this state ------------------------------------------------
        MCM_commands_setInverter(me, DISABLED);
        //Light_set(Light_dashRTD, 0);

        //How to transition to next state ------------------------------------------------
        if (me->lockoutStatus == DISABLED)
        {
            SerialManager_send(me->serialMan, "MCM lockout has been disabled.\n");
            me->startupStage = 2;
        }
        break;

    case 2: //MCM on, lockout=disabled, inverter=disabled --> stay until RTD button pressed
        //Actions to perform upon entering this state ------------------------------------------------
        //Nothing: wait for RTD button

        //How to transition to next state ------------------------------------------------
        if (Sensor_RTDButton.sensorValue == TRUE && tps->calibrated == TRUE && bps->calibrated == TRUE && tps->travelPercent < .05  && bps->percent > .25  // Should be high enough to ensure driver is on the brakes reasonably hard
        )
        {
            MCM_commands_setInverter(me, ENABLED); //Change the inverter command to enable
            SerialManager_send(me->serialMan, "Changed MCM inverter command to ENABLE.\n");
            me->startupStage = 3;
        }
        break;

    case 3: //inverted=disabled, rtd=pressed, waiting for inverter to be enabled
        //Actions to perform upon entering this state ------------------------------------------------
        //Nothing: wait for mcm to say it's enabled

        //How to transition to next state ------------------------------------------------
        if (me->inverterStatus == ENABLED)
        {
            RTDPercent = 1; //Doesn't matter if button is no longer pressed - RTD light should be on if car is drivable
            SerialManager_send(me->serialMan, "Inverter has been enabled.  S                                                    tarting RTDS.  Car is ready to drive.\n");
            RTDS_setVolume(rtds, 1, 1500000);
            me->startupStage = 4; //leave this stage since we've already kicked off the RTDS
        }
        break;

    case 4: //inverter=disabled, rtds=already started
        //Actions to perform upon entering this state ------------------------------------------------
        SerialManager_send(me->serialMan, "RTD procedure complete.\n"); //Just send a message
        RTDPercent = 1;                                                 //This line is redundant

        //How to transition to next state ------------------------------------------------
        //Always do, since we sent a message.
        me->startupStage = 5;
        break;

    case 5: //inverter=enabled, rtds=not_started
        //What happens in this state ------------------------------------------------
        RTDPercent = 1; //This line is redundant
        //This case is here so we don't send a message anymore

        //How to transition to next state ------------------------------------------------
        //Don't. //MCM_setStartupStage(me, MCM_getStartupStage(me) + 1);
        break;

    default:
        SerialManager_send(me->serialMan, "ERROR: Lost track of MCM startup status.\n");
        break;
    }

    //----------------------------------------------------------
    //Inverter Command Overrides
    //----------------------------------------------------------
    switch (me->InverterOverride)
    {
    case DISABLED:
        MCM_commands_setInverter(me, DISABLED);
        break;
    case ENABLED:
        MCM_commands_setInverter(me, DISABLED);
        break;
    case UNKNOWN:
        break;
    }

    //After all that, we can turn the RTD light on/off
    Light_set(Light_dashRTD, RTDPercent);
}


/*****************************************************************************
* Accessors / Mutators (Set/Get)
****************************************************************************/
//Will be divided by 10 e.g. pass in 100 for 10.0 Nm
void MCM_commands_setTorqueDNm(MotorController *me, sbyte2 newTorque)
{
    me->updateCount += (me->commands_torque == newTorque) ? 0 : 1;
    me->commands_torque = newTorque;
}

void MCM_commands_setDirection(MotorController *me, Direction newDirection)
{
    switch (newDirection)
    {
    case _0:
    case CLOCKWISE:
    case REVERSE:
        me->updateCount += (me->commands_direction == 0) ? 0 : 1;
        me->commands_direction = 0;
        break;

    case _1:
    case COUNTERCLOCKWISE:
    case FORWARD:
        me->updateCount += (me->commands_direction == 1) ? 0 : 1;
        me->commands_direction = 1;
        break;

    default:
        //Invalid direction?
        break;
    }
}
void MCM_commands_setInverter(MotorController *me, Status newInverterState)
{
    me->updateCount += (me->commands_inverter == newInverterState) ? 0 : 1;
    me->commands_inverter = newInverterState;
}
void MCM_commands_setDischarge(MotorController *me, Status setDischargeTo)
{
    me->updateCount += (me->commands_discharge == setDischargeTo) ? 0 : 1;
    me->commands_discharge = setDischargeTo;
}
void MCM_commands_setTorqueLimit(MotorController *me, sbyte2 newTorqueLimit)
{
    me->updateCount += (me->commands_torqueLimit == newTorqueLimit) ? 0 : 1;
    me->commands_torqueLimit = newTorqueLimit;
}

void MCM_commands_resetUpdateCountAndTime(MotorController *me)
{
    me->updateCount = 0;
    IO_RTC_StartTime(&(me->timeStamp_lastCommandSent));
}

ubyte4 MCM_commands_getTimeSinceLastCommandSent(MotorController *me)
{
    return IO_RTC_GetTimeUS(me->timeStamp_lastCommandSent);
}

sbyte4 MCM_getPower(MotorController *me)
{
    return (me->DC_Voltage * me->DC_Current);
}

sbyte4 MCM_getGroundSpeedKPH(MotorController *me)
{   
    sbyte4 FD_Ratio = 3.55; //divide # of rear teeth by number of front teeth
    sbyte4 Revolutions = 60; //this converts the rpm to rotations per hour
    //tireCirc does PI * Diameter_Tire because otherwise it doesn't work
    //for 16s set tireCirc to 1.295 for 18s set tireCirc to 1.395 
    //sbyte4 PI = 3.141592653589; 
    //sbyte4 Diameter_Tire = 0.4;
    sbyte4 tireCirc = 1.395; //the actual average tire circumference in meters
    sbyte4 KPH_Unit_Conversion = 1000.0;
    sbyte4 groundKPH = ((me->motorRPM/FD_Ratio) * Revolutions * tireCirc) / KPH_Unit_Conversion; 

    return groundKPH;
    
}

sbyte2 MCM_getRegenBPSForMaxRegenZeroToFF(MotorController *me)
{
    return 0xFF * me->regen_percentBPSForMaxRegen;
}
sbyte2 MCM_getRegenAPPSForMaxCoastingZeroToFF(MotorController *me)
{
    return 0xFF * me->regen_percentAPPSForCoasting;
}

void MCM_setRegen_TorqueLimitDNm(MotorController* me, ubyte2 newTorqueLimit)
{
    if (newTorqueLimit >= 0)
        me->regen_torqueLimitDNm = newTorqueLimit;
}
void MCM_setRegen_TorqueAtZeroPedalDNm(MotorController* me, ubyte2 newTorqueZero)
{
    if(newTorqueZero >= 0)
        me->regen_torqueAtZeroPedalDNm = newTorqueZero;
}
void MCM_setRegen_PercentBPSForMaxRegen(MotorController* me, float4 percentBPS)
{
    if(percentBPS >=0 || percentBPS <= 1)
        me->regen_percentBPSForMaxRegen = percentBPS;
}
void MCM_setRegen_PercentAPPSForCoasting(MotorController* me, float4 percentAPPS)
{
    if(percentAPPS >=0 || percentAPPS <= 1)
        me->regen_percentAPPSForCoasting = percentAPPS;
}

ubyte2 MCM_pack_no_load_voltage(MotorController *me)
{

    return me->DC_Voltage + (me->DC_Current * PACK_RESISTANCE);
}

const float4 POWER_LIM_LUT[25][25] = {
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{231.74, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{218.42, 227.51, 230.85, 230.85, 230.85, 230.85, 227.73, 222.03, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81, 217.81},
	{206.97, 215.54, 222.86, 221.34, 225.38, 223.81, 221.47, 218.16, 214.28, 209.40, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27, 205.27},
	{196.98, 205.14, 212.11, 218.33, 211.03, 214.14, 212.98, 211.27, 209.07, 206.14, 202.49, 198.59, 194.00, 194.00, 194.00, 194.00, 194.00, 194.00, 194.00, 194.00, 194.00, 194.00, 194.00, 194.00, 194.00},
	{188.06, 195.95, 202.61, 208.54, 214.01, 200.72, 203.93, 203.15, 201.93, 200.20, 198.22, 195.56, 192.56, 189.13, 185.17, 184.20, 184.20, 184.20, 184.20, 184.20, 184.20, 184.20, 184.20, 184.20, 184.20},
	{179.99, 187.66, 194.11, 199.85, 205.07, 209.95, 190.07, 194.65, 194.03, 193.13, 191.90, 190.44, 188.49, 186.25, 183.65, 180.71, 177.36, 175.27, 175.27, 175.27, 175.27, 175.27, 175.27, 175.27, 175.27},
	{172.51, 180.12, 186.42, 191.97, 197.05, 201.76, 206.18, 181.88, 186.05, 185.67, 185.00, 184.09, 182.93, 181.51, 179.83, 177.83, 175.66, 173.12, 170.16, 167.15, 167.15, 167.15, 167.15, 167.15, 167.15},
	{165.50, 173.11, 179.35, 184.79, 189.73, 194.29, 198.60, 202.67, 175.41, 178.14, 177.83, 177.34, 176.67, 175.79, 174.84, 172.99, 171.95, 170.32, 168.29, 166.09, 163.70, 160.87, 159.81, 159.81, 159.81},
	{158.76, 166.49, 172.72, 178.15, 183.00, 187.47, 191.65, 195.62, 199.39, 167.75, 170.79, 170.58, 170.27, 169.77, 169.12, 168.32, 167.25, 166.12, 164.89, 163.31, 161.69, 159.76, 157.63, 155.26, 152.92},
	{152.10, 160.15, 166.48, 171.88, 176.73, 181.14, 185.24, 189.11, 192.79, 196.33, 159.95, 163.92, 163.73, 163.55, 163.19, 162.62, 162.09, 161.35, 160.51, 159.24, 158.28, 156.92, 155.52, 153.87, 152.02},
	{145.25, 153.91, 160.46, 165.95, 170.79, 175.21, 179.27, 183.08, 186.69, 190.14, 193.45, 153.52, 157.46, 157.30, 157.06, 156.91, 156.55, 156.10, 155.55, 154.89, 154.12, 153.25, 152.23, 151.03, 149.70},
	{137.71, 147.59, 154.55, 160.23, 165.14, 169.56, 173.64, 177.42, 180.99, 184.38, 187.63, 190.76, 147.26, 154.72, 151.28, 151.17, 150.93, 150.74, 150.40, 150.01, 149.51, 148.94, 148.25, 147.50, 146.60},
	{127.93, 140.83, 148.59, 154.59, 159.66, 164.16, 168.24, 172.03, 175.60, 178.97, 182.18, 185.26, 134.72, 142.62, 148.62, 145.50, 145.47, 145.43, 145.20, 144.96, 144.73, 144.30, 143.88, 143.38, 143.44},
	{0.00, 132.97, 142.31, 148.90, 154.26, 158.90, 163.07, 166.89, 170.45, 173.84, 177.03, 180.08, 183.01, 130.22, 135.25, 143.78, 140.08, 139.85, 139.99, 139.88, 139.74, 139.54, 139.30, 138.97, 138.62},
	{0.00, 118.64, 135.22, 142.95, 148.80, 153.70, 158.02, 161.92, 165.53, 168.90, 172.09, 175.16, 178.07, 180.88, 125.24, 130.78, 135.53, 134.91, 134.87, 134.82, 134.76, 134.77, 134.63, 135.30, 134.26}};
const float4 v_min = 283.200;
const float4 v_max = 403.200;
const sbyte4 s_min = 0;
const sbyte4 s_max = 6000;
const ubyte1 num_v = 25;
const ubyte1 num_s = 25;

float4 interpolate(float4 x0, float4 x1, float4 y0, float4 y1, float4 x, float4 y, float4 q00, float4 q01, float4 q10, float4 q11) {
    float4 x0y0 = (x1 - x) * (y1 - y);
    float4 x1y0 = (x - x0) * (y1 - y);
    float4 x0y1 = (x1 - x) * (y - y0);
    float4 x1y1 = (x - x0) * (y - y0);
    return (q00 * x0y0 + q10 * x1y0 + q01 * x0y1 + q11 * x1y1) / ((x1 - x0) * (y1 - y0));
}

float4 solve_lut(float4 v, float4 s) {
    const float4 v_step = (v_max - v_min) / (num_v - 1);
    const float4 s_step = (s_max - s_min) / (num_s - 1);

    ubyte1 v_index = (v - v_min) / v_step;
    ubyte1 s_index = (s - s_min) / s_step;

    // Clamping indices to be within the table bounds
    v_index = v_index < 0 ? 0 : (v_index > num_v - 2 ? num_v - 2 : v_index);
    s_index = s_index < 0 ? 0 : (s_index > num_s - 2 ? num_s - 2 : s_index);

    float4 v0 = v_min + v_index * v_step;
    float4 v1 = v0 + v_step;
    float4 s0 = s_min + s_index * s_step;
    float4 s1 = s0 + s_step;

    // Get the four surrounding points
    float4 q00 = POWER_LIM_LUT[s_index][v_index];
    float4 q01 = POWER_LIM_LUT[s_index][v_index + 1];
    float4 q10 = POWER_LIM_LUT[s_index + 1][v_index];
    float4 q11 = POWER_LIM_LUT[s_index + 1][v_index + 1];

    // Interpolate the table value at (v, s)
    return interpolate(v0, v1, s0, s1, v, s, q00, q01, q10, q11);
}


ubyte2 MCM_get_max_torque_power_limit(MotorController *me)
{
    return solve_lut(MCM_pack_no_load_voltage(me), me->motorRPM);
}
