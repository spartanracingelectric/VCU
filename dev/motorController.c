#include <stdlib.h> //Needed for malloc
#include "IO_Driver.h"
#include "IO_DIO.h" //TEMPORARY - until MCM relay control  / ADC stuff gets its own object
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "motorController.h"
#include "mathFunctions.h"
#include "sensors.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "readyToDriveSound.h"
#include "serial.h"
#include "main.h"

#include "canManager.h"

#include "power_lim.h"

extern Button RTD_Button;
extern Button Cal_Button;
extern Sensor Sensor_TCSKnob;       // used currently for regen
extern Button Sensor_HVILTerminationSense;
extern DigitalOutput RTD_Light;
extern DigitalOutput MCM_Power;

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

    me->LaunchControl_Torque = 0;

    me->LCState = FALSE;
    me->LCReady = FALSE;
    
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
        torqueOutput = me->LaunchControl_Torque;
    } else if(me->LCReady == TRUE) {
        torqueOutput = 0;
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
            IO_RTC_StartTime(&me->timeStamp_HVILLost);
        }

        //If the MCM is on (and we lost HV)
        if (me->relayState == TRUE)
        {
            //Okay to turn MCM off once 0 torque is commanded, or after 2 sec
            //TODO: SIMILAR CODE SHOULD BE EMPLOYED AT HVIL SHUTDOWN CONTROL PIN
            if (me->commandedTorque == 0 || IO_RTC_GetTimeUS(me->timeStamp_HVILLost) > 2000000)
            {
                DigitalOutput_set(&MCM_Power, FALSE);
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
        DigitalOutput_set(&MCM_Power, TRUE);
        me->relayState = TRUE;
    }
}

//See diagram at https://onedrive.live.com/redir?resid=F9BB8F0F8FDB5CF8!30410&authkey=!ABSF-uVH-VxQRAs&ithint=file%2chtml
void MCM_inverterControl(MotorController *me, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds)
{
    float4 RTD_En;
    RTD_En = (RTD_Button.sensorValue == TRUE ? 1 : 0);
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
        if (RTD_Button.sensorValue == TRUE && tps->calibrated == TRUE && bps->calibrated == TRUE && tps->travelPercent < .05  && bps->percent > .25  // Should be high enough to ensure driver is on the brakes reasonably hard
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
            RTD_En = 1; //Doesn't matter if button is no longer pressed - RTD light should be on if car is drivable
            SerialManager_send(me->serialMan, "Inverter has been enabled.  Starting RTDS.  Car is ready to drive.\n");
            RTDS_play_sound(rtds);
            me->startupStage = 4; //leave this stage since we've already kicked off the RTDS
        }
        break;

    case 4: //inverter=disabled, rtds=already started
        //Actions to perform upon entering this state ------------------------------------------------
        SerialManager_send(me->serialMan, "RTD procedure complete.\n"); //Just send a message
        RTD_En = 1;                                                 //This line is redundant

        //How to transition to next state ------------------------------------------------
        //Always do, since we sent a message.
        me->startupStage = 5;
        break;

    case 5: //inverter=enabled, rtds=not_started
        //What happens in this state ------------------------------------------------
        RTD_En = 1; //This line is redundant
        //This case is here so we don't send a message anymore

        //How to transition to next state ------------------------------------------------
        //Don't. //MCM_setStartupStage(me, MCM_getStartupStage(me) + 1);
        break;

    default:
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
    DigitalOutput_set(&RTD_Light, RTD_En);
}


/*****************************************************************************
* Accessors / Mutators (Set/Get)
****************************************************************************/
//Will be divided by 10 e.g. pass in 100 for 10.0 Nm


void MCM_commands_setTorqueDNm(MotorController *me, sbyte2 newTorque)
{   
    // newTorque = 2400;
    // sbyte4 dummyPower = 72321; //watts
    
    sbyte2 takeaway = 0;
    if (MCM_getPower(me) > POWER_LIM_LOWER_POWER_THRESH) {
        takeaway = (sbyte2)((MCM_getPower(me) - POWER_LIM_LOWER_POWER_THRESH) / 100);
        // takeaway = (sbyte2)(dummyPower - POWER_LIM_LOWER_POWER_THRESH)/100;
        
        if ( newTorque > POWER_LIM_UPPER_TORQUE_THRESH - (takeaway * POWER_LIM_TAKEAWAY_SCALAR) )  {  //if newTorque is greater than powerlim adjust max torque
            newTorque = POWER_LIM_UPPER_TORQUE_THRESH - (takeaway * POWER_LIM_TAKEAWAY_SCALAR);       //set it to powerlim adjust max torque    
        }
    }
    me->takeaway = takeaway;

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
    return (me->DC_Voltage * me->DC_Current/100);
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

float4 MCM_pack_no_load_voltage(MotorController *me)
{

    return ((float4)me->DC_Voltage / 10) + ((float4)me->DC_Current * PACK_RESISTANCE / 10);
}



float4 interpolate(float4 x0, float4 x1, float4 y0, float4 y1, float4 x, float4 y, float4 q00, float4 q01, float4 q10, float4 q11) {
    float4 x0y0 = (x1 - x) * (y1 - y);
    float4 x1y0 = (x - x0) * (y1 - y);
    float4 x0y1 = (x1 - x) * (y - y0);
    float4 x1y1 = (x - x0) * (y - y0);
    return (q00 * x0y0 + q10 * x1y0 + q01 * x0y1 + q11 * x1y1) / ((x1 - x0) * (y1 - y0));
}

float4 solve_lut(float4 v, float4 s) {
    const float4 v_step = (V_MAX - V_MIN) / (NUM_V - 1);
    const float4 s_step = (S_MAX - S_MIN) / (NUM_S - 1);
    if (v > V_MAX) {
        v = V_MAX;
    }
    if (v < V_MIN) {
        v = V_MIN;
    }
    if (s < S_MIN) {
        s = S_MIN;
    }
    if (s > S_MAX) {
        s = S_MAX;
    }

    ubyte1 v_index = (v - V_MIN) / v_step;
    ubyte1 s_index = (s - S_MIN) / s_step;

    // Clamping indices to be within the table bounds
    v_index = v_index < 0 ? 0 : (v_index > NUM_V - 2 ? NUM_V - 2 : v_index);
    s_index = s_index < 0 ? 0 : (s_index > NUM_S - 2 ? NUM_S - 2 : s_index);

    float4 v0 = V_MIN + v_index * v_step;
    float4 v1 = v0 + v_step;
    float4 s0 = S_MIN + s_index * s_step;
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
    me->nl_voltage = MCM_pack_no_load_voltage(me);
    ubyte2 power_lim = solve_lut(me->nl_voltage, (float4)me->motorRPM);
    if (power_lim < 0) {
        power_lim = 0;
    }
    return power_lim;
}
