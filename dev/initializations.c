// VCU initializations
// Object (sensors, controllers, etc) instantiations
// ONLY THIS FILE should have "true" version of object variables
// Everything else should have "extern" declarations of variables

#include "IO_ADC.h"
#include "IO_CAN.h"
#include "IO_DIO.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - probably should be included in every c file
#include "IO_PWM.h"

#include "initializations.h"
#include "sensors.h"
// #include "can.h"

/*****************************************************************************
 * ADC
 ****************************************************************************/
// Turns on the VCU's ADC channels and power supplies.
void vcu_initializeADC(bool benchMode)
{
    //----------------------------------------------------------------------------
    // Power supplies/outputs
    //----------------------------------------------------------------------------
    // Analog sensor supplies
    Sensor_TPS0.ioErr_powerSet = Sensor_BPS0.ioErr_powerSet =
        IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON); // Pin 148 and 136
    Sensor_TPS1.ioErr_powerSet = Sensor_BPS1.ioErr_powerSet =
        IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON); // Pin 147

    // Variable power supply
    IO_POWER_Set(IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V); // IO_POWER_Set(IO_PIN_269, IO_POWER_8_5_V);

    // Digital/power outputs ---------------------------------------------------
    // Relay power outputs
    IO_DO_Init(IO_DO_00);
    IO_DO_Set(IO_DO_00, FALSE); // mcm0 Relay
    IO_DO_Init(IO_DO_01);
    IO_DO_Set(IO_DO_01, FALSE); // VCU-BMS Shutdown Relay
    IO_DO_Init(IO_DO_02);
    IO_DO_Set(IO_DO_02, FALSE); // Water pump signal (No longer using PWM signal for the Water Pump)
    IO_DO_Init(IO_DO_03);
    IO_DO_Set(IO_DO_03, FALSE); // Fan relay - motor fan and radiator fan are on same circuit
    IO_DO_Init(IO_DO_04);
    IO_DO_Set(IO_DO_04, FALSE); // NOT USED
    IO_DO_Init(IO_DO_05);
    IO_DO_Set(IO_DO_05, benchMode); // power output for switches - only used on bench
    IO_DO_Init(IO_DO_06);
    IO_DO_Set(IO_DO_06, FALSE); // DRS Open
    IO_DO_Init(IO_DO_07);
    IO_DO_Set(IO_DO_07, FALSE); // DRS Close

    // Lowside outputs (connects to ground when on)
    IO_DO_Init(IO_ADC_CUR_00);
    IO_DO_Set(IO_ADC_CUR_00, FALSE); // Brake light
    IO_DO_Init(IO_ADC_CUR_01);
    IO_DO_Set(IO_ADC_CUR_01, FALSE); // Eco
    IO_DO_Init(IO_ADC_CUR_02);
    IO_DO_Set(IO_ADC_CUR_02, FALSE); // Err
    IO_DO_Init(IO_ADC_CUR_03);
    IO_DO_Set(IO_ADC_CUR_03, FALSE); // RTD

    // Wheel Speed Sensor supplies
    // Sensor_WSS_FL.ioErr_powerInit = Sensor_WSS_FR.ioErr_powerInit = Sensor_WSS_RL.ioErr_powerInit =
    // Sensor_WSS_RR.ioErr_powerInit = IO_DO_Init(IO_DO_07); // WSS power
    //  IO_POWER_Set (IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);

    // Digital PWM outputs ---------------------------------------------------
    //  RTD Sound
    IO_PWM_Init(IO_PWM_01, 750, TRUE, FALSE, 0, FALSE, NULL);
    IO_PWM_SetDuty(IO_PWM_01, 0, NULL);

    // Bench LED 12V source
    IO_PWM_Init(IO_PWM_03, 500, TRUE, FALSE, 0, FALSE, NULL);
    IO_PWM_SetDuty(IO_PWM_03, benchMode == TRUE ? 0xFFFF : 0, NULL);

    // Rad Fans (SR-14 and above)
    IO_PWM_Init(IO_PWM_02, 100, TRUE, FALSE, 0, FALSE,
                NULL); // Pin, Frequency Hz, Boolean for Pos polarity, Current measurement enabled bool, Weird other pin
                       // (current), No diag margin, Not safety Critical
    IO_PWM_SetDuty(IO_PWM_02, .90 * 0xFFFF, NULL); // Pin, 0 - 65535, Feedback Measurement

    // Accum fan signal
    IO_PWM_Init(IO_PWM_04, 100, TRUE, FALSE, 0, FALSE, NULL);
    IO_PWM_SetDuty(IO_PWM_04, 1.00 * 0xFFFF, NULL); // Default 100%, build up momentum in fans or whatever? Lol

    //----------------------------------------------------------------------------
    // ADC channels
    //----------------------------------------------------------------------------
    // TPS+BPS
    extern Sensor Sensor_BenchTPS0; // wtf where are these even defined?
    extern Sensor Sensor_BenchTPS1;

    // IO_ADC_ChannelInit(IO_ADC_5V_00, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
    // IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);

    // TPS/BPS
    // Sensor_BPS0.ioErr_init = IO_ADC_ChannelInit(IO_ADC_5V_02, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0,
    // NULL);
    if (benchMode == TRUE)
    {
        // Redo BPS ratiometric
        Sensor_TPS0.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_00, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
        Sensor_TPS1.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
        Sensor_BPS0.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_02, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
        Sensor_BPS1.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    }
    else // Not bench mode
    {
        // In the future, production TPS will be digital instead of analog (see PWD section, below)
        // Sensor_TPS0.ioErr_signalInit = IO_PWD_PulseInit(IO_PWM_00, IO_PWD_HIGH_TIME);
        // Sensor_TPS1.ioErr_signalInit = IO_PWD_PulseInit(IO_PWM_01, IO_PWD_HIGH_TIME);
        // Redo BPS ratiometric
        Sensor_TPS0.ioErr_signalInit =
            IO_ADC_ChannelInit(IO_ADC_5V_00, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
        Sensor_TPS1.ioErr_signalInit =
            IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);
        Sensor_BPS0.ioErr_signalInit =
            IO_ADC_ChannelInit(IO_ADC_5V_02, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
        Sensor_BPS1.ioErr_signalInit =
            IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);
    }

    // Unused
    // IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);

    // SAS (Steering Angle Sensor)
    Sensor_SAS.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_ABSOLUTE, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);
    // Using absolute due to the external 5V supply

    // DRS
    // Sensor_DRSRotary.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_32V_00, IO_ADC_ABSOLUTE, 0, 0,
    // IO_ADC_SENSOR_SUPPLY_1, NULL); // IO_ADC_ABSOLUTE / IO_ADC_RATIO..
    Sensor_DRSKnob.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_VAR_00, IO_ADC_ABSOLUTE, IO_ADC_RANGE_25V, 0, 0, NULL);

    // TCS Pot
    // IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_RESISTIVE, 0, 0, 0, NULL);

    // Unused
    // IO_ADC_ChannelInit(IO_ADC_5V_05, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    // IO_ADC_ChannelInit(IO_ADC_5V_06, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    // IO_ADC_ChannelInit(IO_ADC_5V_07, IO_ADC_RESISTIVE, 0, 0, 0, NULL);

    //----------------------------------------------------------------------------
    // PWD channels
    //----------------------------------------------------------------------------
    // TPS
    // MOVED TO TPS/BPS BLOCK ABOVE

    // Wheel Speed Sensors (Pulse Width Detection)

    IO_RTC_StartTime(&Sensor_WSS_FL.timestamp);
    IO_RTC_StartTime(&Sensor_WSS_FR.timestamp);
    IO_RTC_StartTime(&Sensor_WSS_RL.timestamp);
    IO_RTC_StartTime(&Sensor_WSS_RR.timestamp);

    Sensor_WSS_FL.heldSensorValue = Sensor_WSS_FR.heldSensorValue = Sensor_WSS_RL.heldSensorValue =
        Sensor_WSS_RR.heldSensorValue                             = 0;

    Sensor_WSS_FL.ioErr_signalInit =
        IO_PWD_ComplexInit(IO_PWD_10, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_0_8, 4,
                           IO_PWD_THRESH_1_25V, NULL, NULL); // P274
    Sensor_WSS_FR.ioErr_signalInit =
        IO_PWD_ComplexInit(IO_PWD_08, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_0_8, 4,
                           IO_PWD_THRESH_1_25V, NULL, NULL); // P275
    Sensor_WSS_RL.ioErr_signalInit =
        IO_PWD_ComplexInit(IO_PWD_09, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_0_8, 4,
                           IO_PWD_THRESH_1_25V, NULL, NULL); // P268
    Sensor_WSS_RR.ioErr_signalInit =
        IO_PWD_ComplexInit(IO_PWD_11, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_0_8, 4,
                           IO_PWD_THRESH_1_25V, NULL, NULL); // P267
    // Maybe look for falling edge because we're using NPN/sinking WSS?

    //----------------------------------------------------------------------------
    // Switches
    //----------------------------------------------------------------------------
    Sensor_RTDButton.ioErr_signalInit = IO_DI_Init(IO_DI_00, IO_DI_PU_10K); // RTD Button
    Sensor_EcoButton.ioErr_signalInit = IO_DI_Init(IO_DI_01, IO_DI_PD_10K); // Eco Button
    // Sensor_TCSSwitchUp.ioErr_signalInit = IO_DI_Init(IO_DI_02, IO_DI_PU_10K);   //TCS Switch A
    Sensor_LCButton.ioErr_signalInit  = IO_DI_Init(IO_DI_03, IO_DI_PD_10K); // Launch Control Enable Button
    Sensor_DRSButton.ioErr_signalInit = IO_DI_Init(IO_DI_04, IO_DI_PD_10K); // DRS Button
    // TODO unoccupied I/O on VCU
    Sensor_TestButton.ioErr_signalInit = IO_DI_Init(IO_DI_02, IO_DI_PD_10K); // Test Button

    // Sensor_IO_DI_06.ioErr_signalInit = IO_DI_Init(IO_DI_06, IO_DI_PD_10K); //Unused
    Sensor_HVILTerminationSense.ioErr_signalInit =
        IO_DI_Init(IO_DI_07, IO_DI_PD_10K); // HVIL Term sense, high = HV present
}

//----------------------------------------------------------------------------
// Waste CPU cycles until we have valid data
//----------------------------------------------------------------------------
void vcu_ADCWasteLoop(void)
{
    bool   tempFresh = FALSE;
    ubyte2 tempData;
    ubyte4 timestamp_sensorpoll = 0;
    IO_RTC_StartTime(&timestamp_sensorpoll);
    while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 1000000)
    {
        IO_Driver_TaskBegin();

        IO_PWM_SetDuty(IO_PWM_01, 0, NULL);

        IO_DO_Set(IO_DO_00, FALSE); // False = low
        IO_DO_Set(IO_DO_01, FALSE); // HVIL shutdown relay

        // IO_DI (digital inputs) supposed to take 2 cycles before they return valid data
        IO_DI_Get(IO_DI_04, &tempData);
        IO_DI_Get(IO_DI_05, &tempData);
        IO_ADC_Get(IO_ADC_5V_00, &tempData, &tempFresh);
        IO_ADC_Get(IO_ADC_5V_01, &tempData, &tempFresh);

        IO_Driver_TaskEnd();
        // TODO: Find out if EACH pin needs 2 cycles or just the entire DIO unit
        while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 12500)
            ; // wait until 1/8/10s (125ms) have passed
    }
}

/*****************************************************************************
 * Sensors
 ****************************************************************************/
Sensor Sensor_TPS0; // = { 0, 0.5, 4.5 };
Sensor Sensor_TPS1; // = { 0, 4.5, 0.5 };
Sensor Sensor_BPS0; // = { 1, 0.5, 4.5 };  //Brake system pressure (or front only in the future)
Sensor
    Sensor_BPS1; // = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)
Sensor Sensor_WSS_FL; // = { 2 };
Sensor Sensor_WSS_FR; // = { 2 };
Sensor Sensor_WSS_RL; // = { 2 };
Sensor Sensor_WSS_RR; // = { 2 };
Sensor Sensor_WPS_FL; // = { 3 };
Sensor Sensor_WPS_FR; // = { 3 };
Sensor Sensor_WPS_RL; // = { 3 };
Sensor Sensor_WPS_RR; // = { 3 };
Sensor Sensor_SAS;    // = { 4 };
Sensor Sensor_LVBattery;

Sensor Sensor_TCSKnob;
Sensor Sensor_RTDButton;
Sensor Sensor_TestButton;
Sensor Sensor_EcoButton;
Sensor Sensor_TCSSwitchUp;
Sensor Sensor_LCButton;
Sensor Sensor_HVILTerminationSense;

Sensor Sensor_DRSButton;
Sensor Sensor_DRSKnob;
// Switches
// precharge failure

// Other
extern Sensor Sensor_LVBattery; // = { 0xA };  //Note: There will be no init for this "sensor"
