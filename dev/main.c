/*****************************************************************************
* SRE-2 Vehicle Control Firmware for the TTTech HY-TTC 50 Controller (VCU)
******************************************************************************
* For project info and history, see https://github.com/spartanracingelectric/SRE-2
* For software/development questions, email rusty@pedrosatech.com
******************************************************************************
* Files
* The Git repository does not contain the complete firmware for SRE-2.  Modules
* provided by TTTech can be found on the CD that accompanied the VCU. These 
* files can be identified by our naming convention: TTTech files start with a
* prefix in all caps (such as IO_Driver.h), except for ptypes_xe167.h which
* they also provided.
* For instructions on setting up a build environment, see the SRE-2 getting-
* started document, Programming for the HY-TTC 50, at http://1drv.ms/1NQUppu
*****************************************************************************/

//-------------------------------------------------------------------
//VCU Initialization Stuff
//-------------------------------------------------------------------

//VCU/C headers
#include <stdio.h>
#include <string.h>
#include "APDB.h"
#include "IO_DIO.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
#include "IO_UART.h"

//Our code
#include "main.h"
#include "initializations.h"
#include "sensors.h"
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
#include "watch_dog.h"

//Application Database, needed for TTC-Downloader
APDB appl_db =
    {
        0, /* ubyte4 versionAPDB        */
        {0}, /* BL_T_DATE flashDate       */
        {(ubyte4)(((((ubyte4)RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26))}, /* BL_T_DATE buildDate                   */
        0, /* ubyte4 nodeType           */
        0, /* ubyte4 startAddress       */
        0, /* ubyte4 codeSize           */
        0, /* ubyte4 legacyAppCRC       */
        0, /* ubyte4 appCRC             */
        1, /* ubyte1 nodeNr             */
        0, /* ubyte4 CRCInit            */
        0, /* ubyte4 flags              */
        0, /* ubyte4 hook1              */
        0, /* ubyte4 hook2              */
        0, /* ubyte4 hook3              */
        APPL_START, /* ubyte4 mainAddress        */
        {0, 1}, /* BL_T_CAN_ID canDownloadID */
        {0, 2}, /* BL_T_CAN_ID canUploadID   */
        0, /* ubyte4 legacyHeaderCRC    */
        0, /* ubyte4 version            */
        500, /* ubyte2 canBaudrate        */
        0, /* ubyte1 canChannel         */
        {0}, /* ubyte1 reserved[8*4]      */
        0 /* ubyte4 headerCRC          */
};

extern Button Cal_Button;
extern DigitalOutput Eco_Light;
extern DigitalOutput Err_Light;
extern WatchDog wd;
extern CanManager canMan;
extern ReadyToDriveSound rtds;
extern BatteryManagementSystem bms;
extern MotorController mcm;
extern InstrumentCluster ic;
extern TorqueEncoder tps;
extern BrakePressureSensor bps;
extern WheelSpeeds wss;
extern SafetyChecker sc;
extern CoolingSystem cs;
extern LaunchControl lc;
extern DRS drs;
extern TimerDebug td;

void main(void)
{
    ubyte4 timestamp_startTime = 0;
    ubyte4 timestamp_EcoButton = 0;
    ubyte1 calibrationErrors; //NOT USED

    /*******************************************/
    /*        Low Level Initializations        */
    /*******************************************/
    IO_Driver_Init(NULL); //Handles basic startup for all VCU subsystems
    serial_init();
    IO_RTC_StartTime(&timestamp_startTime);
    serial_send("\n\n\n\n\n\n\n\n\n\n----------------------------------------------------\n");
    serial_send("VCU serial is online.\n");

    //----------------------------------------------------------------------------
    // VCU Subsystem Initializations
    // Eventually, all of these functions should be made obsolete by creating
    // objects instead, like the RTDS/MCM/TPS objects below
    //----------------------------------------------------------------------------
    serial_send("VCU objects/subsystems initializing.\n");
    vcu_initializeADC(); //Configure and activate all I/O pins on the VCU

    //Do some loops until the ADC stops outputting garbage values
    vcu_ADCWasteLoop();

    //vcu_init functions may have to be performed BEFORE creating CAN Manager object
    CanManager_new(&canMan, 200000);

    WatchDog_new(&wd, 50000); //50 ms 

    //----------------------------------------------------------------------------
    // Object representations of external devices
    // Most default values for things should be specified here
    //----------------------------------------------------------------------------
    ubyte1 pot_DRS_LC = 1; // 0 is for DRS and 1 is for launch control/Auto DRS - CHANGE HERE FOR POT MODE

    RTDS_new(&rtds, 1, 1500000);
    BMS_new(&bms, BMS_BASE_ADDRESS);
    MotorController_new(&mcm, 0xA0, REVERSE, 2400, 5, 10);
    InstrumentCluster_new(&ic, 0x702);
    TorqueEncoder_new(&tps);
    BrakePressureSensor_new(&bps);
    WheelSpeeds_new(&wss, WHEEL_DIAMETER, WHEEL_DIAMETER, F_WSS_TICKS, R_WSS_TICKS);
    SafetyChecker_new(&sc, 320, 32); //Must match amp limits
    CoolingSystem_new(&cs);
    LaunchControl_new(&lc, pot_DRS_LC);
    DRS_new(&drs);
    TimerDebug_new(&td);
    init_lv_battery_lut();

    ubyte4 timestamp_mainLoopStart = 0;
    serial_send("VCU initializations complete.  Entering main loop.\n");

    while (1)
    {
        TimerDebug_startTimer(&td);
        IO_RTC_StartTime(&timestamp_mainLoopStart);
        IO_Driver_TaskBegin();

        /*******************************************/
        /*              Read Inputs                */
        /*******************************************/
        sensors_updateSensors();
        CanManager_read(&canMan, CAN0);
        CanManager_read(&canMan, CAN1);

        //No regen below 5kph
        sbyte4 groundSpeedKPH = MCM_getGroundSpeedKPH(&mcm);
        if (groundSpeedKPH < 15) {
            MCM_setRegenMode(&mcm, REGENMODE_OFF);
        } else {
            // Regen mode is now set based on battery voltage to preserve overvoltage fault 
            // if(BMS_getPackVoltage(bms) >= 38500 * 10){ 
            //     MCM_setRegenMode(mcm0, REGENMODE_FORMULAE); 
            // } else {
            //     MCM_setRegenMode(mcm0, REGENMODE_FIXED);
            // } 
        }

        if (Cal_Button.sensorValue) {
            WatchDog_reset(&wd); // tapping eco will reset the watchdog for now
            if (timestamp_EcoButton == 0) {
                serial_send("Eco button detected\n");
                IO_RTC_StartTime(&timestamp_EcoButton);
            }
            else if (IO_RTC_GetTimeUS(timestamp_EcoButton) >= 3000000) {
                serial_send("Eco button held 3s - starting calibrations\n");
                //calibrateTPS(TRUE, 5);
                TorqueEncoder_startCalibration(&tps, 5);
                BrakePressureSensor_startCalibration(&bps, 5);
                DigitalOutput_set(&Eco_Light, TRUE);
                //DIGITAL OUTPUT 4 for STATUS LED ???? I dont believe this -Ian
            }
        }
        else {
            if (IO_RTC_GetTimeUS(timestamp_EcoButton) > 10000 && IO_RTC_GetTimeUS(timestamp_EcoButton) < 1000000) {
                serial_send("Eco mode requested\n");
            }
            timestamp_EcoButton = 0;
        }
        TorqueEncoder_update(&tps);
        TorqueEncoder_calibrationCycle(&tps, &calibrationErrors); //Todo: deal with calibration errors
        BrakePressureSensor_update(&bps);
        BrakePressureSensor_calibrationCycle(&bps, &calibrationErrors);

        //Update WheelSpeed and interpolate
        WheelSpeeds_update(&wss, TRUE);
        slipRatioCalculation(&wss, &lc);

        //Cool DRS things
        DRS_update(&drs, pot_DRS_LC, lc.LCReady || lc.LCStatus);

        CoolingSystem_calculations(&cs, mcm.motor_temp/*This was just mcm temp but it was really just getting motor temp*/, mcm.motor_temp, bms.highestCellTemperature/BMS_TEMPERATURE_SCALE, &HVILTerminationSense);
        
        CoolingSystem_enactCooling(&cs); //This belongs under outputs but it doesn't really matter for cooling

        //Assign motor controls to MCM command message
        //DOES NOT set inverter command or rtds flag
        launchControlTorqueCalculation(&lc);
        MCM_calculateCommands(&mcm);

        SafetyChecker_update(&sc);

        /*******************************************/
        /*  Output Adjustments by Safety Checker   */
        /*******************************************/
        SafetyChecker_reduceTorque(&sc);

        /*******************************************/
        /*              Enact Outputs              */
        /*******************************************/
        //MOVE INTO SAFETYCHECKER
        DigitalOutput_set(&Err_Light, (sc.faults == 0) ? FALSE : TRUE);
        //Handle motor controller startup procedures
        MCM_relayControl(&mcm);
        MCM_inverterControl(&mcm);

        IO_ErrorType err = 0;
        //Comment out to disable shutdown board control
        err = BMS_relayControl(&bms);

        canOutput_sendDebugMessage(&canMan);
        
        RTDS_shutdownHelper(&rtds); 
        IO_Driver_TaskEnd();
        
        TimerDebug_stopTimer(&td);
        
        while (IO_RTC_GetTimeUS(timestamp_mainLoopStart) < CYCLE_TIME_US) // 1000 = 1ms
        {
            IO_UART_Task(); //The task function shall be called every SW cycle.
        }

    } //end of main loop
}
