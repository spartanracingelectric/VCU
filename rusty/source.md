# Source Code Submission
Note: This file can be viewed with proper formatting by visiting https://github.com/spartanracingelectric/VCU/blob/user/rusty/regen/rusty/source.md

Rather than providing raw source code/files or point to the entire SRE-2 repo, I thought it would be better to explain code snippets in a markdown file.  I have pared down the source for simplicity, but full source is available in the user branch at https://github.com/spartanracingelectric/VCU/commits/user/rusty/regen

## Main loop - main.c
All VCU code runs in this loop, which currently is set to a 33ms cycle time.
Code is divided broadly into four blocks:
1. Read inputs: CAN messages, analog and digital inputs (from sensors/switches/etc)
2. Calculate outputs needed (update internal states without performing anything)
3. Run safety check: override calculated out values
4. Enact outputs - send CAN messages, set analog/digital/PWM pins, etc
```c
while (1)
{
    IO_RTC_StartTime(&timestamp_mainLoopStart);

    // Inputs
    sensors_updateSensors();
    CanManager_read(canMan, CAN0_HIPRI, mcm0, bms, sc);
    TorqueEncoder_update(tps);
    BrakePressureSensor_update(bps, bench);
    WheelSpeeds_update(wss);
    CoolingSystem_calculations(cs, MCM_getTemp(mcm0), MCM_getMotorTemp(mcm0), BMS_getMaxTemp(bms));

    // Calculations
    MCM_setRegenMode(mcm0, REGENMODE_TESLA);
    MCM_calculateCommands(mcm0, tps, bps);

    // Safety checks/overrides
    SafetyChecker_update(sc, mcm0, bms, tps, bps, &Sensor_HVILTerminationSense, &Sensor_LVBattery);
    SafetyChecker_reduceTorque(sc, mcm0, bms);

    // Outputs
    CoolingSystem_enactCooling(cs);
    MCM_relayControl(mcm0, &Sensor_HVILTerminationSense);
    MCM_inverterControl(mcm0, tps, bps, rtds);
    canOutput_sendDebugMessage(canMan, tps, bps, mcm0, wss, sc);

    while (IO_RTC_GetTimeUS(timestamp_mainLoopStart) < 33000) // 1000 = 1ms
    {
    }

} //end of main loop
```

## MCM_setRegenMode - motorcontroller.c
Sets parameters that control regen behavior.  This function will be called when a CAN message is received from the dashboard indicating that the driver / race engineer wants to make adjustments.  Originally, the driver was able to select from a handful of presets, represented by `regen_mode`.  In the future, each value will be independently adjustable via a new text UI on the car's new dashboard.

Important variables:
- `regen_torqueLimitDNm`         - max regen torque (positive value of 1234 means 123.4 Nm of regen)
- `regen_torqueAtZeroPedalDNm`   - how much regen torque to request when no pedals are pressed
- `regen_percentAPPSForCoasting` - how far you have to press APPS to start requesting forward torque instead of regen
- `regen_percentBPSForMaxRegen`  - how far you have to press the brake to get 100% regen

```c
void MCM_setRegenMode(MotorController *me, RegenMode regenMode)
{
    me->regen_torqueLimitDNm = 1000;
    switch (regenMode)
    {
    case REGENMODE_FORMULAE: //Position 1 = Coasting mode (Formula E mode)
        me->regen_mode = REGENMODE_FORMULAE;
        me->regen_torqueAtZeroPedalDNm = 0;
        me->regen_percentAPPSForCoasting = 0;
        me->regen_percentBPSForMaxRegen = .3; //zero to one.. 1 = 100%
        break;

    case REGENMODE_HYBRID: //Position 2 = light "engine braking" (Hybrid mode)
        me->regen_mode = REGENMODE_HYBRID;
        me->regen_torqueAtZeroPedalDNm = me->regen_torqueLimitDNm * 0.3;
        me->regen_percentAPPSForCoasting = .2;
        me->regen_percentBPSForMaxRegen = .3; //zero to one.. 1 = 100%
        break;

    case REGENMODE_TESLA: //Position 3 = One pedal driving (Tesla mode)
        me->regen_mode = REGENMODE_TESLA;
        me->regen_torqueAtZeroPedalDNm = me->regen_torqueLimitDNm;
        me->regen_percentAPPSForCoasting = .05;
        me->regen_percentBPSForMaxRegen = 0;
        break;

    case REGENMODE_OFF:
    default:
        me->regen_mode = REGENMODE_OFF;

        me->regen_torqueLimitDNm = 0;  // override regen_torqueLimitDNm - set to zero
        me->regen_torqueAtZeroPedalDNm = 0;
        me->regen_percentAPPSForCoasting = 0;
        me->regen_percentBPSForMaxRegen = 0; //zero to one.. 1 = 100%
        break;
    }
}
```

## MCM_calculateCommands - motorcontroller.c
Maps driver pedal inputs to torque values to request from the motor controller.  The algorithm is explained in the project report.  Also determines whether the drive inverter should be enables/disabled, among other things.

```c
void MCM_calculateCommands(MotorController *me, TorqueEncoder *tps, BrakePressureSensor *bps)
{
    //----------------------------------------------------------------------------
    // Control commands
    //Note: Safety checks (torque command limiting) are done EXTERNALLY.  This is a preliminary calculation
    //which should return the intended torque based on pedals
    //Note: All stored torque values should be positive / unsigned
    //----------------------------------------------------------------------------
    MCM_commands_setDischarge(me, DISABLED);
    MCM_commands_setDirection(me, REVERSE); //1 = forwards for our car, 0 = reverse

    sbyte2 torqueOutput = 0;
    sbyte2 appsTorque = 0;
    sbyte2 bpsTorque = 0;

    float4 appsOutputPercent;

    TorqueEncoder_getOutputPercent(tps, &appsOutputPercent);

    appsTorque = me->torqueMaximumDNm * getPercent(appsOutputPercent, me->regen_percentAPPSForCoasting, 1, TRUE) - me->regen_torqueAtZeroPedalDNm * getPercent(appsOutputPercent, me->regen_percentAPPSForCoasting, 0, TRUE);
    bpsTorque = 0 - (me->regen_torqueLimitDNm - me->regen_torqueAtZeroPedalDNm) * getPercent(bps->percent, 0, me->regen_percentBPSForMaxRegen, TRUE);

    torqueOutput = appsTorque + bpsTorque;
    //torqueOutput = me->torqueMaximumDNm * tps->percent;  //REMOVE THIS LINE TO ENABLE REGEN
    MCM_commands_setTorqueDNm(me, torqueOutput);

    me->HVILOverride = (IO_RTC_GetTimeUS(me->timeStamp_HVILOverrideCommandReceived) < 1000000);

    // Inverter override
    if (IO_RTC_GetTimeUS(me->timeStamp_InverterDisableOverrideCommandReceived) < 1000000)
        me->InverterOverride = DISABLED;
    else if (IO_RTC_GetTimeUS(me->timeStamp_InverterDisableOverrideCommandReceived) < 1000000)
        me->InverterOverride = ENABLED;
    else
        me->InverterOverride = UNKNOWN;
}
```

## Excepts from SafetyChecker_reduceTorque - safety.c
The block of code below reduces regen torque as the vehicle speed reduces below a set speed (`MCM_getRegenMinSpeed`)
```c
    if ( groundSpeedKPH < MCM_getRegenRampdownStartSpeed(mcm))
    {
        float4 regenMultiplier = 1 - getPercent(groundSpeedKPH, MCM_getRegenMinSpeed(mcm), MCM_getRegenRampdownStartSpeed(mcm), TRUE);
        if (regenMultiplier < multiplier ) { multiplier = regenMultiplier; } // Use regenMultiplier if it is lower
    }
```

At FSAE Michigan 2021, this code block saved the from disassembling the HV system during during parc fermé, which would have required hours of work and re-submitting the battery for inspection.  It preventing an overactive hardware system from shutting down the high voltage system by disabling torque momentarily (0.5 sec) if the driver accidentally pushed both brake and accel pedals at the same time.
```c
me->softBSPD_bpsHigh = bps->bps0->sensorValue > 2500;
me->softBSPD_kwHigh = MCM_getPower(mcm) > 4000;

if (me->softBSPD_bpsHigh && me->softBSPD_kwHigh)
{
    IO_RTC_StartTime(&timestamp_SoftBSPD);
    me->softBSPD_fault = FALSE;
}
else if (IO_RTC_GetTimeUS(timestamp_SoftBSPD) >= 500000 || IO_RTC_GetTimeUS(timestamp_SoftBSPD) == 0)
{
    timestamp_SoftBSPD = 0;
    me->softBSPD_fault = FALSE;
    me->faults &= ~F_softBSPDFault;
}
```

At the end of the day, if any fault has been detected, the torque command will be set to zero prior to the command message being sent to the motor controller via CAN.
```c
if (me->faults > 0) //Any VCU fault exists
{
    multiplier = 0;
}

...

MCM_commands_setTorqueDNm(mcm, MCM_commands_getTorque(mcm) * multiplier);
```

These bits of code are the most relevant to this project.  I have omitted CAN messaging, ADC input/output, etc, as those low-level functions are irrelevant to this project so long as they do their job.  As stated previously, the VCU source code in its entirety is available at https://github.com/spartanracingelectric/VCU/commits/user/rusty/regen
