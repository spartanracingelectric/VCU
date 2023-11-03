//VCU initializations
//Object (sensors, controllers, etc) instantiations
//ONLY THIS FILE should have "true" version of object variables
//Everything else should have "extern" declarations of variables

#include "IO_Driver.h" //Includes datatypes, constants, etc - probably should be included in every c file
#include "IO_ADC.h"
#include "IO_PWM.h"
#include "IO_CAN.h"
#include "IO_DIO.h"

#include "sensors.h"
#include "initializations.h"
#include "lut.h"


LUT* LV_BATT_SOC_LUT;

/*****************************************************************************
* ADC
****************************************************************************/
//Turns on the VCU's ADC channels and power supplies.
void vcu_initializeADC(bool benchMode)
{
    //----------------------------------------------------------------------------
    //Power supplies/outputs
    //----------------------------------------------------------------------------
    //Analog sensor supplies
    Sensor_TPS0.ioErr_powerSet = Sensor_BPS0.ioErr_powerSet = IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);  // Pin 148 and 136
    Sensor_TPS1.ioErr_powerSet = Sensor_BPS1.ioErr_powerSet = IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);  // Pin 147

    //Variable power supply
    IO_POWER_Set(IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);    //IO_POWER_Set(IO_PIN_269, IO_POWER_8_5_V);

    //Digital/power outputs ---------------------------------------------------
    //Relay power outputs
    IO_DO_Init(IO_DO_00);    IO_DO_Set(IO_DO_00, FALSE); //mcm0 Relay
    IO_DO_Init(IO_DO_01);    IO_DO_Set(IO_DO_01, FALSE); //VCU-BMS Shutdown Relay
    IO_DO_Init(IO_DO_02);    IO_DO_Set(IO_DO_02, FALSE); //Water pump signal (No longer using PWM signal for the Water Pump)
    IO_DO_Init(IO_DO_03);    IO_DO_Set(IO_DO_03, FALSE); //Fan relay - motor fan and radiator fan are on same circuit
    IO_DO_Init(IO_DO_04);    IO_DO_Set(IO_DO_04, FALSE); //Battery fan relay - not used on SRE-4
    IO_DO_Init(IO_DO_05);    IO_DO_Set(IO_DO_05, benchMode); //power output for switches - only used on bench
    IO_DO_Init(IO_DO_06);    IO_DO_Set(IO_DO_06, FALSE); //DRS Open
    IO_DO_Init(IO_DO_07);    IO_DO_Set(IO_DO_07, FALSE); //DRS Close

    //Lowside outputs (connects to ground when on)
    IO_DO_Init(IO_ADC_CUR_00);    IO_DO_Set(IO_ADC_CUR_00, FALSE); //Brake light
    IO_DO_Init(IO_ADC_CUR_01);    IO_DO_Set(IO_ADC_CUR_01, FALSE); //Eco
    IO_DO_Init(IO_ADC_CUR_02);    IO_DO_Set(IO_ADC_CUR_02, FALSE); //Err
    IO_DO_Init(IO_ADC_CUR_03);    IO_DO_Set(IO_ADC_CUR_03, FALSE); //RTD

    //Digital PWM outputs ---------------------------------------------------
    // RTD Sound
    IO_PWM_Init(IO_PWM_01, 750, TRUE, FALSE, 0, FALSE, NULL);
    IO_PWM_SetDuty(IO_PWM_01, 0, NULL);

    //Bench LED 12V source
    IO_PWM_Init(IO_PWM_03, 500, TRUE, FALSE, 0, FALSE, NULL);
    IO_PWM_SetDuty(IO_PWM_03, benchMode == TRUE ? 0xFFFF : 0, NULL);
    
    // Rad Fans (SR-14 and above)
    IO_PWM_Init(IO_PWM_02, 100, TRUE, FALSE, 0, FALSE, NULL); //Pin, Frequency Hz, Boolean for Pos polarity, Current measurement enabled bool, Weird other pin (current), No diag margin, Not safety Critical
    IO_PWM_SetDuty(IO_PWM_02, .90 * 0xFFFF, NULL); //Pin, 0 - 65535, Feedback Measurement

    //Accum fan signal
    IO_PWM_Init(IO_PWM_04, 100, TRUE, FALSE, 0, FALSE, NULL);
    IO_PWM_SetDuty(IO_PWM_04, 1.00 * 0xFFFF, NULL);    //Default 100%, build up momentum in fans or whatever? Lol

    //----------------------------------------------------------------------------
    //ADC channels
    //----------------------------------------------------------------------------
    //TPS/BPS
    
    Sensor_TPS0.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_00, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
    Sensor_TPS1.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);
    Sensor_BPS0.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_02, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
    Sensor_BPS1.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);

    // SAS (Steering Angle Sensor)
    Sensor_SAS.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_ABSOLUTE, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);
    // Using absolute due to the external 5V supply

    // DRS
    Sensor_DRSKnob.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_VAR_00 , IO_ADC_ABSOLUTE , IO_ADC_RANGE_25V, 0, 0, NULL );

    //TCS Pot
    //IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_RESISTIVE, 0, 0, 0, NULL);


    //----------------------------------------------------------------------------
    //PWD channels
    //----------------------------------------------------------------------------
    //TPS

    //Wheel Speed Sensors (Pulse Width Detection)

    Sensor_WSS_FL = *PWDSensor_new(IO_PWD_10);
    Sensor_WSS_FR = *PWDSensor_new(IO_PWD_08);
    Sensor_WSS_RL = *PWDSensor_new(IO_PWD_09);
    Sensor_WSS_RR = *PWDSensor_new(IO_PWD_11);
    
    //----------------------------------------------------------------------------
    //Switches
    //----------------------------------------------------------------------------
    Sensor_RTDButton = *Button_new(IO_DI_00, TRUE); //RTD Button
    Sensor_EcoButton = *Button_new(IO_DI_01, TRUE); //Eco Button
    Sensor_LCButton  = *Button_new(IO_DI_03, TRUE); // Launch Control Enable Button
    Sensor_DRSButton = *Button_new(IO_DI_04, TRUE); // DRS Button

    //----------------------------------------------------------------------------
    Sensor_HVILTerminationSense = *Button_new(IO_DI_07, FALSE); //HVIL Term sense, high = HV present
}

//----------------------------------------------------------------------------
// Waste CPU cycles until we have valid data
//----------------------------------------------------------------------------
void vcu_ADCWasteLoop(void)
{
    bool tempFresh = FALSE;
    ubyte2 tempData;
    ubyte4 timestamp_sensor_poll = 0;
    IO_RTC_StartTime(&timestamp_sensor_poll);
    while (IO_RTC_GetTimeUS(timestamp_sensor_poll) < 1000000)
    {
        IO_Driver_TaskBegin();

        IO_PWM_SetDuty(IO_PWM_01, 0, NULL);

        IO_DO_Set(IO_DO_00, FALSE); //False = low
        IO_DO_Set(IO_DO_01, FALSE); //HVIL shutdown relay

        //IO_DI (digital inputs) supposed to take 2 cycles before they return valid data
        IO_DI_Get(IO_DI_05, &tempData);
        IO_ADC_Get(IO_ADC_5V_00, &tempData, &tempFresh);
        IO_ADC_Get(IO_ADC_5V_01, &tempData, &tempFresh);

        IO_Driver_TaskEnd();
        //TODO: Find out if EACH pin needs 2 cycles or just the entire DIO unit
        while (IO_RTC_GetTimeUS(timestamp_sensor_poll) < 12500)
            ; // wait until 1/8/10s (125ms) have passed
    }
    IO_DI_DeInit(IO_DI_05);
}

void init_lv_battery_lut(void)
{
    float4 initialValues[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    LV_BATT_SOC_LUT = createLUT(2.8, 4.2, 10, initialValues);
}

/*****************************************************************************
* Sensors
****************************************************************************/
Sensor Sensor_TPS0; // = { 0, 0.5, 4.5 };
Sensor Sensor_TPS1; // = { 0, 4.5, 0.5 };
Sensor Sensor_BPS0; // = { 1, 0.5, 4.5 };  //Brake system pressure (or front only in the future)
Sensor Sensor_BPS1;  // = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)
PWDSensor Sensor_WSS_FL; // = { 2 };
PWDSensor Sensor_WSS_FR; // = { 2 };
PWDSensor Sensor_WSS_RL; // = { 2 };
PWDSensor Sensor_WSS_RR; // = { 2 };
Sensor Sensor_SAS;    // = { 4 };
Sensor Sensor_LVBattery;

Sensor Sensor_TCSKnob;
Button Sensor_RTDButton;
Button Sensor_EcoButton;
Button Sensor_LCButton;
Button Sensor_HVILTerminationSense;

Button Sensor_DRSButton;
Sensor Sensor_DRSKnob;
//Switches
//precharge failure

//Other
extern Sensor Sensor_LVBattery; // = { 0xA };  //Note: There will be no init for this "sensor"
