#include "main.h"

//Application Database, needed for TTC-Downloader
APDB appl_db =
    {
        0 /* ubyte4 versionAPDB        */
        ,
        {0} /* BL_T_DATE flashDate       */
            /* BL_T_DATE buildDate                   */
        ,
        {(ubyte4)(((((ubyte4)RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26))},
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
    ubyte1 calibrationErrors; //NOT USED

    /*******************************************/
    /*        Low Level Initializations        */
    /*******************************************/
    IO_Driver_Init(NULL); //Handles basic startup for all VCU subsystems
    IO_RTC_StartTime(&timestamp_startTime);
    
    IO_RTC_StartTime(&timestamp_startTime);
    while (IO_RTC_GetTimeUS(timestamp_startTime) < 55555)
    {
        IO_Driver_TaskBegin();
        IO_Driver_TaskEnd();
        while (IO_RTC_GetTimeUS(timestamp_startTime) < 10000)
            ; // wait until 10ms have passed
    }
    //----------------------------------------------------------------------------
    // VCU Subsystem Initializations
    // Eventually, all of these functions should be made obsolete by creating
    // objects instead, like the RTDS/MCM/TPS objects below
    //----------------------------------------------------------------------------
    vcu_initializeADC(); //Configure and activate all I/O pins on the VCU
    vcu_ADCWasteLoop();

    //vcu_init functions may have to be performed BEFORE creating CAN Manager object
    CanManager *canMan = CanManager_new(500, 50, 50, 500, 10, 10, 200000); //3rd param = messages per node (CAN1/CAN2; read/write)   


    ubyte1 pot_DRS_LC = 1; // 0 is for DRS and 1 is for launch control/Auto DRS - CHANGE HERE FOR POT MODE

    ReadyToDriveSound *rtds = RTDS_new();
    BatteryManagementSystem *bms = BMS_new(BMS_BASE_ADDRESS);
    MotorController *mcm0 = MotorController_new(0xA0, FORWARD, 600, 5, 10); //CAN addr, direction, torque limit x10 (100 = 10Nm)
    InstrumentCluster *ic0 = InstrumentCluster_new(0x702);
    TorqueEncoder *tps = TorqueEncoder_new();
    BrakePressureSensor *bps = BrakePressureSensor_new();
    WheelSpeeds *wss = WheelSpeeds_new(WHEEL_DIAMETER, WHEEL_DIAMETER, NUM_BUMPS, NUM_BUMPS);
    SafetyChecker *sc = SafetyChecker_new(320, 32); //Must match amp limits
    CoolingSystem *cs = CoolingSystem_new();
    LaunchControl *lc = LaunchControl_new(pot_DRS_LC);
    DRS *drs = DRS_new();
    ubyte4 timestamp_mainLoopStart = 0;
    while (1)
    {
        IO_RTC_StartTime(&timestamp_mainLoopStart);
        IO_Driver_TaskBegin();
        sensors_updateSensors();
        CanManager_read(canMan, CAN1_HIPRI, mcm0, ic0, bms, sc); // read CAN1
        CanManager_read(canMan, CAN2_LOPRI, mcm0, ic0,bms, sc); // read CAN2

        if (Sensor_EcoButton.sensorValue == FALSE)
        {
            if (timestamp_EcoButton == 0)
            {
                
                IO_RTC_StartTime(&timestamp_EcoButton);
            }
            else if (IO_RTC_GetTimeUS(timestamp_EcoButton) >= 3000000)
            {
                //calibrateTPS(TRUE, 5);
                TorqueEncoder_startCalibration(tps, 5);
                BrakePressureSensor_startCalibration(bps, 5);
                Light_set(Light_dashEco, 1);
                //DIGITAL OUTPUT 4 for STATUS LED
            }
        }

        TorqueEncoder_update(tps);
        TorqueEncoder_calibrationCycle(tps, &calibrationErrors); //Todo: deal with calibration errors
        BrakePressureSensor_update(bps);
        BrakePressureSensor_calibrationCycle(bps, &calibrationErrors);
        WheelSpeeds_update(wss, TRUE);
        slipRatioCalculation(wss, lc);
        DRS_update(drs, mcm0, tps, bps, pot_DRS_LC);
        CoolingSystem_calculations(cs, MCM_getTemp(mcm0), MCM_getMotorTemp(mcm0), BMS_getHighestCellTemp_degC(bms), &Sensor_HVILTerminationSense);
        
        CoolingSystem_enactCooling(cs); 
        launchControlTorqueCalculation(lc, tps, bps, mcm0);
        MCM_calculateCommands(mcm0, tps, bps);

        SafetyChecker_update(sc, mcm0, bms, tps, bps, &Sensor_HVILTerminationSense, &Sensor_LVBattery);
        SafetyChecker_reduceTorque(sc, mcm0, bms, wss);

        /*******************************************/
        /*              Enact Outputs              */
        /*******************************************/
        //MOVE INTO SAFETYCHECKER
        //SafetyChecker_setErrorLight(sc);
        Light_set(Light_dashError, (SafetyChecker_getFaults(sc) == 0) ? 0 : 1);
        //Handle motor controller startup procedures
        MCM_relayControl(mcm0, &Sensor_HVILTerminationSense);
        MCM_inverterControl(mcm0, tps, bps, rtds);

        IO_ErrorType err = 0;
        //Comment out to disable shutdown board control
        err = BMS_relayControl(bms);

        canOutput_sendDebugMessage(canMan, tps, bps, mcm0, ic0, bms, wss, sc, lc, drs);
        RTDS_shutdownHelper(rtds); 
        IO_Driver_TaskEnd();
        while (IO_RTC_GetTimeUS(timestamp_mainLoopStart) < 10000) // 1000 = 1ms
        {
            IO_UART_Task(); //The task function shall be called every SW cycle.
        }

    } //end of main loop

}
