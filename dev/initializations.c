// VCU initializations
// Object (sensors, controllers, etc) instantiations
// ONLY THIS FILE should have "true" version of object variables
// Everything else should have "extern" declarations of variables

#include "IO_Driver.h" //Includes datatypes, constants, etc - probably should be included in every c file
#include "IO_ADC.h"
#include "IO_PWM.h"
#include "IO_CAN.h"
#include "IO_DIO.h"

#include "sensors.h"
#include "initializations.h"
#include "lut.h"
#include "watch_dog.h"
#include "canManager.h"
#include "motorController.h"
#include "instrumentCluster.h"
#include "readyToDriveSound.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "wheelSpeeds.h"
#include "safety.h"
#include "serial.h"
#include "cooling.h"
#include "bms.h"
#include "LaunchControl.h"
#include "drs.h"

LUT* LV_BATT_SOC_LUT;

/*****************************************************************************
* ADC
****************************************************************************/
//Turns on the VCU's ADC channels and power supplies.
void vcu_initializeADC(void)
{
    //Variable power supply
    IO_POWER_Set(IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);    //IO_POWER_Set(IO_PIN_269, IO_POWER_8_5_V);

    //Digital/power outputs ---------------------------------------------------
    //Relay power outputs
    MCM_Power = *DigitalOutput_new(IO_DO_00, FALSE); //mcm0 Relay
    VCU_BMS_Power = *DigitalOutput_new(IO_DO_01, FALSE); //VCU-BMS Shutdown Relay
    Water_Pump = *DigitalOutput_new(IO_DO_02, FALSE); //Water pump signal (No longer using PWM signal for the Water Pump)
    Other_Fans = *DigitalOutput_new(IO_DO_03, FALSE); //Fan relay - motor fan and radiator fan are on same circuit
    Accum_Fans = *DigitalOutput_new(IO_DO_04, FALSE); //Battery fan relay - not used on SRE-4
    Bullshit = *DigitalOutput_new(IO_DO_05, FALSE); //power output for switches - only used on bench
    DRS_Open = *DigitalOutput_new(IO_DO_06, FALSE); //DRS Open
    DRS_Close = *DigitalOutput_new(IO_DO_07, FALSE); //DRS Close

    //Lowside outputs (connects to ground when on)
    Brake_Light = *DigitalOutput_new(IO_ADC_CUR_00, FALSE); //Brake light
    Eco_Light = *DigitalOutput_new(IO_ADC_CUR_01, FALSE); //Eco
    Err_Light = *DigitalOutput_new(IO_ADC_CUR_02, FALSE); //Err
    RTD_Light = *DigitalOutput_new(IO_ADC_CUR_03, FALSE); //RTD

    //Digital PWM outputs ---------------------------------------------------
    // RTD Sound
    RTD_Sound = *PWMOutput_new(IO_PWM_01, 750, 0.0);
    
    // Rad Fans (SR-14 and above)
    Rad_Fans = *PWMOutput_new(IO_PWM_02, 100, 0.9);

    //Accum fan signal
    Accum_Fan = *PWMOutput_new(IO_PWM_03, 100, 1.0);

    //----------------------------------------------------------------------------
    // ADC channels
    //----------------------------------------------------------------------------
    //TPS/BPS
    
    TPS0 = *Sensor_new(IO_ADC_5V_00, IO_ADC_SENSOR_SUPPLY_0);
    TPS1 = *Sensor_new(IO_ADC_5V_01, IO_ADC_SENSOR_SUPPLY_1);
    BPS0 = *Sensor_new(IO_ADC_5V_02, IO_ADC_SENSOR_SUPPLY_0);
    BPS1 = *Sensor_new(IO_ADC_5V_03, IO_ADC_SENSOR_SUPPLY_1);
    Sensor_power_set(&TPS0);
    Sensor_power_set(&TPS1);
    Sensor_power_set(&BPS0);
    Sensor_power_set(&BPS1);

    // SAS (Steering Angle Sensor)
    SAS = *Sensor_new(IO_ADC_5V_04, IO_ADC_SENSOR_SUPPLY_1);
    // Using absolute due to the external 5V supply

    // DRS
    DRSKnob = *Sensor_new(IO_ADC_VAR_00, NULL);

    LVBattery = *Sensor_new(IO_ADC_UBAT, NULL);
    //----------------------------------------------------------------------------
    // PWD channels
    //----------------------------------------------------------------------------
    //Wheel Speed Sensors (Pulse Width Detection)

    WSS_FL = *PWDSensor_new(IO_PWD_10);
    WSS_FR = *PWDSensor_new(IO_PWD_08);
    WSS_RL = *PWDSensor_new(IO_PWD_09);
    WSS_RR = *PWDSensor_new(IO_PWD_11);
    
    //----------------------------------------------------------------------------
    // Switches
    //----------------------------------------------------------------------------
    RTD_Button = *Button_new(IO_DI_00, TRUE); //RTD Button
    Cal_Button = *Button_new(IO_DI_01, TRUE); //Eco Button
    LC_Button  = *Button_new(IO_DI_03, TRUE); // Launch Control Enable Button
    DRS_Button = *Button_new(IO_DI_04, TRUE); // DRS Button

    //----------------------------------------------------------------------------
    HVILTerminationSense = *Button_new(IO_DI_07, FALSE); //HVIL Term sense, high = HV present
}

//----------------------------------------------------------------------------
// Waste CPU cycles until we have valid data
//----------------------------------------------------------------------------
void vcu_ADCWasteLoop(void)
{
    bool tempFresh = FALSE;
    ubyte2 tempData;
    bool tempBool;
    ubyte4 timestamp_sensor_poll = 0;
    IO_RTC_StartTime(&timestamp_sensor_poll);
    while (IO_RTC_GetTimeUS(timestamp_sensor_poll) < 1000000)
    {
        IO_Driver_TaskBegin();

        IO_PWM_SetDuty(IO_PWM_01, 0, NULL);

        IO_DO_Set(IO_DO_00, FALSE); // False = low
        IO_DO_Set(IO_DO_01, FALSE); // HVIL shutdown relay

        //IO_DI (digital inputs) supposed to take 2 cycles before they return valid data
        IO_DI_Get(IO_DI_05, &tempBool);
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
Sensor TPS0; // = { 0, 0.5, 4.5 };
Sensor TPS1; // = { 0, 4.5, 0.5 };
Sensor BPS0; // = { 1, 0.5, 4.5 };  //Brake system pressure (or front only in the future)
Sensor BPS1;  // = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)
PWDSensor WSS_FL; // = { 2 };
PWDSensor WSS_FR; // = { 2 };
PWDSensor WSS_RL; // = { 2 };
PWDSensor WSS_RR; // = { 2 };
Sensor SAS;    // = { 4 };
Sensor LVBattery;

Sensor TCSKnob;
Button RTD_Button;
Button Cal_Button;
Button LC_Button;
Button HVILTerminationSense;

Button DRS_Button;
Sensor DRSKnob;

DigitalOutput Brake_Light;
DigitalOutput TCS_Light;
DigitalOutput Eco_Light;
DigitalOutput Err_Light;
DigitalOutput RTD_Light;
DigitalOutput MCM_Power;
DigitalOutput VCU_BMS_Power; // I have no idea what this is for
DigitalOutput Water_Pump;
DigitalOutput Other_Fans;
DigitalOutput Accum_Fans;
DigitalOutput Bullshit;
DigitalOutput DRS_Open;
DigitalOutput DRS_Close;

PWMOutput RTD_Sound;
PWMOutput Rad_Fans;
PWMOutput Accum_Fan;

WatchDog wd;

CanManager canMan;
ReadyToDriveSound rtds;
BatteryManagementSystem bms;
MotorController mcm;
InstrumentCluster ic;
TorqueEncoder tps;
BrakePressureSensor bps;
WheelSpeeds wss;
SafetyChecker sc;
CoolingSystem cs;
LaunchControl lc;
DRS drs;
TimerDebug td;