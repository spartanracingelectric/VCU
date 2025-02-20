/*****************************************************************************
 * SRE-2 Vehicle Control Firmware for the TTTech HY-TTC 50 Controller (VCU)
 ******************************************************************************
 * For project info and history, see https://github.com/spartanracingelectric/SRE-2
 * For software/development questions, email rusty@pedrosatech.com
 ******************************************************************************
 * Files
 * The Git repository does not contain the complete firmware for SRE-2.  Modules
 * provided by TTTech can be found on the CD that accompanied the VCU. These
 * files can be identified by our naming convetion: TTTech files start with a
 * prefix in all caps (such as IO_Driver.h), except for ptypes_xe167.h which
 * they also provided.
 * For instructions on setting up a build environment, see the SRE-2 getting-
 * started document, Programming for the HY-TTC 50, at http://1drv.ms/1NQUppu
 ******************************************************************************
 * Organization
 * Our code is laid out in the following manner:
 *
 *****************************************************************************/

//-------------------------------------------------------------------
// VCU Initialization Stuff
//-------------------------------------------------------------------

// VCU/C headers
#include "APDB.h"
#include "IO_DIO.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
#include "IO_UART.h"
#include <stdio.h>
#include <string.h>
// #include "IO_CAN.h"
// #include "IO_PWM.h"

// Our code
#include "bms.h"
#include "brakePressureSensor.h"
#include "canManager.h"
#include "cooling.h"
#include "drs.h"
#include "initializations.h"
#include "instrumentCluster.h"
#include "LaunchControl.h"
#include "motorController.h"
#include "readyToDriveSound.h"
#include "safety.h"
#include "sensorCalculations.h"
#include "sensors.h"
#include "serial.h"
#include "torqueEncoder.h"
#include "wheelSpeeds.h"

// Application Database, needed for TTC-Downloader
APDB appl_db = {
    0 /* ubyte4 versionAPDB        */
    ,
    {0} /* BL_T_DATE flashDate       */
    /* BL_T_DATE buildDate                   */
    ,
    {(ubyte4) (((((ubyte4) RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
               ((((ubyte4) RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
               ((((ubyte4) RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
               ((((ubyte4) RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
               ((((ubyte4) RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26))},
    0 /* ubyte4 nodeType           */
    ,
    0 /* ubyte4 startAddress       */
    ,
    0 /* ubyte4 codeSize           */
    ,
    0 /* ubyte4 legacyAppCRC       */
    ,
    0 /* ubyte4 appCRC             */
    ,
    1 /* ubyte1 nodeNr             */
    ,
    0 /* ubyte4 CRCInit            */
    ,
    0 /* ubyte4 flags              */
    ,
    0 /* ubyte4 hook1              */
    ,
    0 /* ubyte4 hook2              */
    ,
    0 /* ubyte4 hook3              */
    ,
    APPL_START /* ubyte4 mainAddress        */
    ,
    {0, 1} /* BL_T_CAN_ID canDownloadID */
    ,
    {0, 2} /* BL_T_CAN_ID canUploadID   */
    ,
    0 /* ubyte4 legacyHeaderCRC    */
    ,
    0 /* ubyte4 version            */
    ,
    500 /* ubyte2 canBaudrate        */
    ,
    0 /* ubyte1 canChannel         */
    ,
    {0} /* ubyte1 reserved[8*4]      */
    ,
    0 /* ubyte4 headerCRC          */
};

extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
extern Sensor Sensor_BPS1;
extern Sensor Sensor_WSS_FL;
extern Sensor Sensor_WSS_FR;
extern Sensor Sensor_WSS_RL;
extern Sensor Sensor_WSS_RR;
extern Sensor Sensor_WPS_FL;
extern Sensor Sensor_WPS_FR;
extern Sensor Sensor_WPS_RL;
extern Sensor Sensor_WPS_RR;
extern Sensor Sensor_SAS;
extern Sensor Sensor_TCSKnob;

extern Sensor Sensor_RTDButton;
extern Sensor Sensor_TestButton;
extern Sensor Sensor_TEMP_BrakingSwitch;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_DRSButton;

/*****************************************************************************
 * Main!
 * Initializes I/O
 * Contains sensor polling loop (always running)
 ****************************************************************************/
void main(void)
{
    ubyte4 timestamp_startTime = 0;
    ubyte4 timestamp_EcoButton = 0;
    ubyte1 calibrationErrors; // NOT USED

    /*******************************************/
    /*        Low Level Initializations        */
    /*******************************************/
    IO_Driver_Init(NULL); // Handles basic startup for all VCU subsystems

    // Initialize serial first so we can use it to debug init of other subsystems
    SerialManager *serialMan = SerialManager_new();
    IO_RTC_StartTime(&timestamp_startTime);
    SerialManager_send(serialMan, "\n\n\n\n\n\n\n\n\n\n----------------------------------------------------\n");
    SerialManager_send(serialMan, "VCU serial is online.\n");

    // Read initial values from EEPROM
    // EEPROMManager* EEPROMManager_new();

    /*******************************************/
    /*      System Level Initializations       */
    /*******************************************/

    //----------------------------------------------------------------------------
    // Check if we're on the bench or not
    //----------------------------------------------------------------------------
    bool bench;
    IO_DI_Init(IO_DI_06, IO_DI_PD_10K);
    IO_RTC_StartTime(&timestamp_startTime);
    while (IO_RTC_GetTimeUS(timestamp_startTime) < 55555)
    {
        IO_Driver_TaskBegin();

        // IO_DI (digital inputs) supposed to take 2 cycles before they return valid data
        IO_DI_Get(IO_DI_06, &bench);

        IO_Driver_TaskEnd();
        // TODO: Find out if EACH pin needs 2 cycles or just the entire DIO unit
        while (IO_RTC_GetTimeUS(timestamp_startTime) < 10000)
            ; // wait until 10ms have passed
    }
    IO_DI_DeInit(IO_DI_06);
    SerialManager_send(serialMan, bench == TRUE ? "VCU is in bench mode.\n" : "VCU is NOT in bench mode.\n");

    //----------------------------------------------------------------------------
    // VCU Subsystem Initializations
    // Eventually, all of these functions should be made obsolete by creating
    // objects instead, like the RTDS/MCM/TPS objects below
    //----------------------------------------------------------------------------
    SerialManager_send(serialMan, "VCU objects/subsystems initializing.\n");
    vcu_initializeADC(bench); // Configure and activate all I/O pins on the VCU
    // vcu_initializeCAN();
    // vcu_initializeMCU();

    // Do some loops until the ADC stops outputting garbage values
    vcu_ADCWasteLoop();

    // vcu_init functions may have to be performed BEFORE creating CAN Manager object
    CanManager *canMan = CanManager_new(500, 50, 50, 500, 10, 10, 200000,
                                        serialMan); // 3rd param = messages per node (can0/can1; read/write)
    // can0_busSpeed ---------------------^    ^   ^   ^    ^   ^     ^         ^
    // can0_read_messageLimit -----------------|   |   |    |   |     |         |
    // can0_write_messageLimit---------------------+   |    |   |     |         |
    // can1_busSpeed-----------------------------------+    |   |     |         |
    // can1_read_messageLimit-------------------------------+   |     |         |
    // can1_write_messageLimit----------------------------------+     |         |
    // defaultSendDelayus---------------------------------------------+         |
    // SerialManager* sm--------------------------------------------------------+

    //----------------------------------------------------------------------------
    // Object representations of external devices
    // Most default values for things should be specified here
    //----------------------------------------------------------------------------
    // ! to remove -- retired functionality
    // ubyte1 pot_DRS_LC = 0; // 0 is for DRS and 1 is for launch control/Auto DRS - CHANGE HERE FOR POT MODE

    ReadyToDriveSound       *rtds = RTDS_new();
    BatteryManagementSystem *bms  = BMS_new(serialMan, BMS_BASE_ADDRESS);

    // 231 Nm
    MotorController *mcm0 = MotorController_new(serialMan, 0xA0, FORWARD, 2310, 5,
                                                10); // CAN addr, direction, torque limit x10 (100 = 10Nm)
    // To change direction, also edit line 276 in motorcontroller.c
    InstrumentCluster   *ic0 = InstrumentCluster_new(serialMan, 0x702);
    TorqueEncoder       *tps = TorqueEncoder_new(bench);
    BrakePressureSensor *bps = BrakePressureSensor_new();
    WheelSpeeds         *wss = WheelSpeeds_new(WHEEL_DIAMETER, WHEEL_DIAMETER, NUM_BUMPS, NUM_BUMPS);
    SafetyChecker       *sc  = SafetyChecker_new(serialMan, 320, 32); // Must match amp limits
    CoolingSystem       *cs  = CoolingSystem_new(serialMan);
    LaunchControl       *lc  = LaunchControl_new();
    DRS                 *drs = DRS_new();

    //----------------------------------------------------------------------------
    // TODO: Additional Initial Power-up functions
    // //----------------------------------------------------------------------------
    // ubyte2 tps0_calibMin = 0xABCD;  //me->tps0->sensorValue;
    // ubyte2 tps0_calibMax = 0x9876;  //me->tps0->sensorValue;
    // ubyte2 tps1_calibMin = 0x5432;  //me->tps1->sensorValue;
    // ubyte2 tps1_calibMax = 0xCDEF;  //me->tps1->sensorValue;
    ubyte2 tps0_calibMin = 700;  // me->tps0->sensorValue;
    ubyte2 tps0_calibMax = 2000; // me->tps0->sensorValue;
    ubyte2 tps1_calibMin = 2600; // me->tps1->sensorValue;
    ubyte2 tps1_calibMax = 5000; // me->tps1->sensorValue;
    // TODO: Read calibration data from EEPROM?
    // TODO: Run calibration functions?
    // TODO: Power-on error checking?

    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/
    /* main loop, executed periodically with a defined cycle time (here: 5 ms) */
    ubyte4 timestamp_mainLoopStart = 0;
    ubyte4 coolingOnTimer          = 0;
    ubyte1 coolingOn               = 0;
    // IO_RTC_StartTime(&timestamp_calibStart);
    SerialManager_send(serialMan, "VCU initializations complete.  Entering main loop.\n");
    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff (start)
        //----------------------------------------------------------------------------
        // Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_mainLoopStart);
        // Mark the beginning of a task - what does this actually do?
        IO_Driver_TaskBegin();

        // SerialManager_send(serialMan, "VCU has entered main loop.");

        /*******************************************/
        /*              Read Inputs                */
        /*******************************************/
        //----------------------------------------------------------------------------
        // Handle data input streams
        //----------------------------------------------------------------------------
        // Get readings from our sensors and other local devices (buttons, 12v battery, etc)
        sensors_updateSensors();

        // Pull messages from CAN FIFO and update our object representations.
        // Also echoes can0 messages to can1 for DAQ.
        CanManager_read(canMan, CAN0_HIPRI, mcm0, ic0, bms, sc);

        if (Sensor_TestButton.sensorValue == TRUE)
        {
            // TODO rewire Sensor_TestButton
            lc->buttonDebug |= 0x02;
        }
        else
        {
            lc->buttonDebug &= ~0x02;
        }
        if (Sensor_DRSButton.sensorValue == TRUE)
        { // mark gives 02
            lc->buttonDebug |= 0x01;
        }
        else
        {
            lc->buttonDebug &= ~0x01;
        }
        if (Sensor_EcoButton.sensorValue == TRUE)
        { // cal gives 04
            lc->buttonDebug |= 0x04;
        }
        else
        {
            lc->buttonDebug &= ~0x04;
        }
        if (Sensor_LCButton.sensorValue == TRUE)
        { // drs gives 08
            lc->buttonDebug |= 0x08;
        }
        else
        {
            lc->buttonDebug &= ~0x08;
        }

        /*switch (CanManager_getReadStatus(canMan, CAN0_HIPRI))
        {
            case IO_E_OK: SerialManager_send(serialMan, "IO_E_OK: everything fine\n"); break;
            case IO_E_NULL_POINTER: SerialManager_send(serialMan, "IO_E_NULL_POINTER: null pointer has been passed to
        function\n"); break; case IO_E_CAN_FIFO_FULL: SerialManager_send(serialMan, "IO_E_CAN_FIFO_FULL: overflow of
        FIFO buffer\n"); break; case IO_E_CAN_WRONG_HANDLE: SerialManager_send(serialMan, "IO_E_CAN_WRONG_HANDLE:
        invalid handle has been passed\n"); break; case IO_E_CHANNEL_NOT_CONFIGURED: SerialManager_send(serialMan,
        "IO_E_CHANNEL_NOT_CONFIGURED: the given handle has not been configured\n"); break; case IO_E_CAN_OLD_DATA:
        SerialManager_send(serialMan, "IO_E_CAN_OLD_DATA: no data has been received\n"); break; default:
        SerialManager_send(serialMan, "Warning: Unknown CAN read status\n"); break;
        }*/

        /*******************************************/
        /*          Perform Calculations           */
        /*******************************************/
        // calculations - Now that we have local sensor data and external data from CAN, we can
        // do actual processing work, from pedal travel calcs to traction control
        // calculations_calculateStuff();

        // Run calibration if commanded
        // if (IO_RTC_GetTimeUS(timestamp_calibStart) < (ubyte4)5000000)

        // SensorValue TRUE and FALSE are reversed due to Pull Up Resistor

        // No regen below 5kph
        sbyte4 groundSpeedKPH = MCM_getGroundSpeedKPH(mcm0);
        if (groundSpeedKPH < 15)
        {
            MCM_setRegenMode(mcm0, REGENMODE_OFF);
        }
        else
        {
            // Regen mode is now set based on battery voltage to preserve overvoltage fault
            // if(BMS_getPackVoltage(bms) >= 38500 * 10){
            //     MCM_setRegenMode(mcm0, REGENMODE_FORMULAE);
            // } else {
            //     MCM_setRegenMode(mcm0, REGENMODE_FIXED);
            // }
        }

        if (Sensor_EcoButton.sensorValue == TRUE ||
            (Sensor_RTDButton.sensorValue == FALSE &&
             Sensor_HVILTerminationSense.sensorValue == FALSE)) // temp make rtd button rtd button in lv
        {
            if (timestamp_EcoButton == 0)
            {
                SerialManager_send(serialMan, "Eco button detected\n");
                IO_RTC_StartTime(&timestamp_EcoButton);
            }
            else if (IO_RTC_GetTimeUS(timestamp_EcoButton) >= 3000000) // Hold Calibration button for 3 seconds
            {
                SerialManager_send(serialMan, "Eco button held 3s - starting calibrations\n");
                // calibrateTPS(TRUE, 5);
                TorqueEncoder_startCalibration(tps, 5);
                BrakePressureSensor_startCalibration(bps, 5);
                Light_set(Light_dashEco, 1);
                // DIGITAL OUTPUT 4 for STATUS LED
            }
        }
        else
        {
            if (IO_RTC_GetTimeUS(timestamp_EcoButton) > 10000 && IO_RTC_GetTimeUS(timestamp_EcoButton) < 1000000)
            {
                SerialManager_send(serialMan, "Eco mode requested\n");
            }
            timestamp_EcoButton = 0;
        }
        TorqueEncoder_update(tps);
        // Every cycle: if the calibration was started and hasn't finished, check the values again
        TorqueEncoder_calibrationCycle(tps, &calibrationErrors); // Todo: deal with calibration errors
        BrakePressureSensor_update(bps, bench);
        BrakePressureSensor_calibrationCycle(bps, &calibrationErrors);

        // TractionControl_update(tps, mcm0, wss, daq);

        // Update WheelSpeed and interpolate
        WheelSpeeds_update(wss, TRUE);
        slipRatioCalculation(wss, lc);

        // Cool DRS things
        DRS_update(drs, mcm0, tps, bps);

        // DataAquisition_update(); //includes accelerometer
        // TireModel_update()
        // ControlLaw_update();
        /*
        ControlLaw //Tq command
            TireModel //used by control law -> read from WSS, accelerometer
            StateObserver //choose driver command or ctrl law
        */

        // CoolingSystem_calculations(cs, MCM_getTemp(mcm0), MCM_getMotorTemp(mcm0), BMS_getHighestCellTemp_degC(bms),
        // &Sensor_HVILTerminationSense); CoolingSystem_enactCooling(cs); //This belongs under outputs but it doesn't
        // really matter for cooling

        // New Code: Pump, ALWAYS ON
        if (coolingOnTimer == 0)
        {
            if (Sensor_LCButton.sensorValue == TRUE && Sensor_HVILTerminationSense.sensorValue == FALSE)
            {
                IO_RTC_StartTime(&coolingOnTimer);
                coolingOn = ~coolingOn;
            }
        }
        else
        {
            if (IO_RTC_GetTimeUS(coolingOnTimer) > 1000000)
            {
                coolingOnTimer = 0;
            }
        }

        if (Sensor_HVILTerminationSense.sensorValue == FALSE)
        {
            if (coolingOn == 0)
            {
                IO_DO_Set(IO_DO_02, FALSE);
                IO_DO_Set(IO_DO_03, FALSE);
            }
            else
            {
                IO_DO_Set(IO_DO_02, TRUE);
                IO_DO_Set(IO_DO_03, TRUE);
            }
        }
        else
        {
            // cooling on in hv
            IO_DO_Set(IO_DO_02, TRUE);
            IO_DO_Set(IO_DO_03, TRUE);
        }

        // Assign motor controls to MCM command message
        // motorController_setCommands(rtds);
        // DOES NOT set inverter command or rtds flag
        // MCM_setRegenMode(mcm0, REGENMODE_FORMULAE); // TODO: Read regen mode from DCU CAN message - Issue #96
        //  MCM_readTCSSettings(mcm0, &Sensor_TCSSwitchUp, &Sensor_TCSSwitchDown, &Sensor_TCSKnob);
        launchControlTorqueCalculation(lc, tps, bps, mcm0);
        MCM_calculateCommands(mcm0, tps, bps);

        SafetyChecker_update(sc, mcm0, bms, tps, bps, &Sensor_HVILTerminationSense, &Sensor_LVBattery);

        /*******************************************/
        /*  Output Adjustments by Safety Checker   */
        /*******************************************/
        SafetyChecker_reduceTorque(sc, mcm0, bms, wss);

        /*******************************************/
        /*              Enact Outputs              */
        /*******************************************/
        // MOVE INTO SAFETYCHECKER
        // SafetyChecker_setErrorLight(sc);
        Light_set(Light_dashError, (SafetyChecker_getFaults(sc) == 0) ? 0 : 1);
        // Handle motor controller startup procedures
        MCM_relayControl(mcm0, &Sensor_HVILTerminationSense);
        MCM_inverterControl(mcm0, tps, bps, rtds);

        IO_ErrorType err = 0;
        // Comment out to disable shutdown board control
        err = BMS_relayControl(bms);

        // CanManager_sendMCMCommandMessage(mcm0, canMan, FALSE);

        // Drop the sensor readings into CAN (just raw data, not calculated stuff)
        // canOutput_sendMCUControl(mcm0, FALSE);

        // Send debug data
        canOutput_sendDebugMessage(canMan, tps, bps, mcm0, ic0, bms, wss, sc, lc, drs);
        canOutput_sendDebugMessage1(canMan, mcm0, tps);
        // canOutput_sendSensorMessages();
        // canOutput_sendStatusMessages(mcm0);

        //----------------------------------------------------------------------------
        // Task management stuff (end)
        //----------------------------------------------------------------------------
        RTDS_shutdownHelper(rtds); // Stops the RTDS from playing if the set time has elapsed

        // Task end function for IO Driver - This function needs to be called at the end of every SW cycle
        IO_Driver_TaskEnd();
        // wait until the cycle time is over
        while (IO_RTC_GetTimeUS(timestamp_mainLoopStart) < 10000) // 1000 = 1ms
        {
            IO_UART_Task(); // The task function shall be called every SW cycle.
        }

    } // end of main loop

    //----------------------------------------------------------------------------
    // VCU Subsystem Deinitializations
    //----------------------------------------------------------------------------
    // IO_ADC_ChannelDeInit(IO_ADC_5V_00);
    // Free memory if object won't be used anymore
}
