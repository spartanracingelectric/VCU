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

void main(void)
{
    ubyte4 timestamp_startTime = 0;
    ubyte4 timestamp_EcoButton = 0;
    ubyte1 calibrationErrors; //NOT USED

    /*******************************************/
    /*        Low Level Initializations        */
    /*******************************************/
    IO_Driver_Init(NULL); //Handles basic startup for all VCU subsystems

    //Initialize serial first so we can use it to debug init of other subsystems
    SerialManager *serialMan = SerialManager_new();
    IO_RTC_StartTime(&timestamp_startTime);
    SerialManager_send(serialMan, "\n\n\n\n\n\n\n\n\n\n----------------------------------------------------\n");
    SerialManager_send(serialMan, "VCU serial is online.\n");

    //----------------------------------------------------------------------------
    // VCU Subsystem Initializations
    // Eventually, all of these functions should be made obsolete by creating
    // objects instead, like the RTDS/MCM/TPS objects below
    //----------------------------------------------------------------------------
    SerialManager_send(serialMan, "VCU objects/subsystems initializing.\n");
    vcu_initializeADC(); //Configure and activate all I/O pins on the VCU

    //Do some loops until the ADC stops outputting garbage values
    vcu_ADCWasteLoop();

    //vcu_init functions may have to be performed BEFORE creating CAN Manager object
    CanManager *canMan = CanManager_new(CAN_0_BAUD, 50, 50, CAN_1_BAUD, 10, 10, 200000, serialMan); //3rd param = messages per node (can0/can1; read/write)
    //can0_busSpeed -------------------------^       ^   ^       ^       ^   ^     ^         ^
    //can0_read_messageLimit ------------------------|   |       |       |   |     |         |
    //can0_write_messageLimit----------------------------+       |       |   |     |         |
    //can1_busSpeed----------------------------------------------+       |   |     |         |
    //can1_read_messageLimit---------------------------------------------+   |     |         |
    //can1_write_messageLimit------------------------------------------------+     |         |
    //defaultSendDelayus-----------------------------------------------------------+         |
    //SerialManager* sm----------------------------------------------------------------------+

    //----------------------------------------------------------------------------
    // Object representations of external devices
    // Most default values for things should be specified here
    //----------------------------------------------------------------------------
    ubyte1 pot_DRS_LC = 1; // 0 is for DRS and 1 is for launch control/Auto DRS - CHANGE HERE FOR POT MODE

    ReadyToDriveSound *rtds = RTDS_new();
    BatteryManagementSystem *bms = BMS_new(serialMan, BMS_BASE_ADDRESS);
    MotorController *mcm0 = MotorController_new(serialMan, 0xA0, FORWARD, 2400, 5, 10);
    InstrumentCluster *ic0 = InstrumentCluster_new(serialMan, 0x702);
    TorqueEncoder *tps = TorqueEncoder_new();
    BrakePressureSensor *bps = BrakePressureSensor_new();
    WheelSpeeds *wss = WheelSpeeds_new(WHEEL_DIAMETER, WHEEL_DIAMETER, F_WSS_TICKS, R_WSS_TICKS);
    SafetyChecker *sc = SafetyChecker_new(serialMan, 320, 32); //Must match amp limits
    CoolingSystem *cs = CoolingSystem_new(serialMan);
    LaunchControl *lc = LaunchControl_new(pot_DRS_LC);
    DRS *drs = DRS_new();
    init_lv_battery_lut();

    ubyte4 timestamp_mainLoopStart = 0;
    SerialManager_send(serialMan, "VCU initializations complete.  Entering main loop.\n");

    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff (start)
        //----------------------------------------------------------------------------
        IO_RTC_StartTime(&timestamp_mainLoopStart);
        IO_Driver_TaskBegin();

        /*******************************************/
        /*              Read Inputs                */
        /*******************************************/
        sensors_updateSensors();

        //Pull messages from CAN FIFO and update our object representations.
        CanManager_read(canMan, CAN0_HIPRI, mcm0, ic0, bms, sc);

        //No regen below 5kph
        sbyte4 groundSpeedKPH = MCM_getGroundSpeedKPH(mcm0);
        if (groundSpeedKPH < 15) {
            MCM_setRegenMode(mcm0, REGENMODE_OFF);
        } else {
            // Regen mode is now set based on battery voltage to preserve overvoltage fault 
            // if(BMS_getPackVoltage(bms) >= 38500 * 10){ 
            //     MCM_setRegenMode(mcm0, REGENMODE_FORMULAE); 
            // } else {
            //     MCM_setRegenMode(mcm0, REGENMODE_FIXED);
            // } 
        }

        if (Cal_Button.sensorValue) {
            if (timestamp_EcoButton == 0) {
                SerialManager_send(serialMan, "Eco button detected\n");
                IO_RTC_StartTime(&timestamp_EcoButton);
            }
            else if (IO_RTC_GetTimeUS(timestamp_EcoButton) >= 3000000) {
                SerialManager_send(serialMan, "Eco button held 3s - starting calibrations\n");
                //calibrateTPS(TRUE, 5);
                TorqueEncoder_startCalibration(tps, 5);
                BrakePressureSensor_startCalibration(bps, 5);
                Light_set(Light_dashEco, 1);
                //DIGITAL OUTPUT 4 for STATUS LED
            }
        }
        else {
            if (IO_RTC_GetTimeUS(timestamp_EcoButton) > 10000 && IO_RTC_GetTimeUS(timestamp_EcoButton) < 1000000) {
                SerialManager_send(serialMan, "Eco mode requested\n");
            }
            timestamp_EcoButton = 0;
        }
        TorqueEncoder_update(tps);
        //Every cycle: if the calibration was started and hasn't finished, check the values again
        TorqueEncoder_calibrationCycle(tps, &calibrationErrors); //Todo: deal with calibration errors
        BrakePressureSensor_update(bps);
        BrakePressureSensor_calibrationCycle(bps, &calibrationErrors);

        //Update WheelSpeed and interpolate
        WheelSpeeds_update(wss, TRUE);
        slipRatioCalculation(wss, lc);

        //Cool DRS things
        DRS_update(drs, mcm0, tps, bps, pot_DRS_LC, lc->LCReady || lc->LCStatus);

        CoolingSystem_calculations(cs, mcm0->motor_temp/*This was just mcm temp but it was really just getting motor temp*/, mcm0->motor_temp, bms->highestCellTemperature/BMS_TEMPERATURE_SCALE, &Sensor_HVILTerminationSense);
        
        CoolingSystem_enactCooling(cs); //This belongs under outputs but it doesn't really matter for cooling

        //Assign motor controls to MCM command message
        //DOES NOT set inverter command or rtds flag
        launchControlTorqueCalculation(lc, tps, bps, mcm0);
        MCM_calculateCommands(mcm0, tps, bps);

        SafetyChecker_update(sc, mcm0, bms, tps, bps);

        /*******************************************/
        /*  Output Adjustments by Safety Checker   */
        /*******************************************/
        SafetyChecker_reduceTorque(sc, mcm0, bms, wss);

        /*******************************************/
        /*              Enact Outputs              */
        /*******************************************/
        //MOVE INTO SAFETYCHECKER
        Light_set(Light_dashError, (sc->faults == 0) ? 0 : 1);
        //Handle motor controller startup procedures
        MCM_relayControl(mcm0);
        MCM_inverterControl(mcm0, tps, bps, rtds);

        IO_ErrorType err = 0;
        //Comment out to disable shutdown board control
        err = BMS_relayControl(bms);

        //Send debug data
        canOutput_sendDebugMessage(canMan, tps, bps, mcm0, ic0, bms, wss, sc, lc, drs);

        //----------------------------------------------------------------------------
        // Task management stuff (end)
        //----------------------------------------------------------------------------
        RTDS_shutdownHelper(rtds); //Stops the RTDS from playing if the set time has elapsed

        //Task end function for IO Driver - This function needs to be called at the end of every SW cycle
        IO_Driver_TaskEnd();
        //wait until the cycle time is over
        while (IO_RTC_GetTimeUS(timestamp_mainLoopStart) < CYCLE_TIME_US) // 1000 = 1ms
        {
            IO_UART_Task(); //The task function shall be called every SW cycle.
        }

    } //end of main loop
}
