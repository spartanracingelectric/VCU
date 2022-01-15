
#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "serial.h"
#include "mathFunctions.h"

/**************************************************************************
 *     REVISION HISTORY:
 *    2016-5-11 - Rabeel Elahi - Added bms_commands_getPower();
 *                             - Added bms_commands_getPackTemp();
 *                             
 *  2016-4-20 - Rabeel Elahi - Added bms_parseCANMessage()
 *                             - Moved cases back to bms.c
 *                             - Added #includes
 *                             - Leaving endian conversion functions until new CAN manager complete
 *
 *     2016-4-8 - Rabeel Elahi - Moved cases to canInput.c
 *                            - Moved endian conversion functions to canInput.c
 *                            - Changed uints to ubytes
 *
 *    2016-4-7 - Rabeel Elahi - Created this file.
 *                            - Defined cases for BMS can messages
 *                            - TODO: Rename variable types to VCU types
 *                            - TODO: Move canInput_readMesagges to caninput.c
 *                            - TODO: Move ENDIAN conversion helper functions to caninput.c?
 *
 *
 **************************************************************************/

/**********************************************************
 *               *********** CAUTION ***********              *
 * MULTI-BYTE VALUES FOR THE ELITHION BMS ARE BIG-ENDIAN  *
 *                                                          *
 **********************************************************/

struct _BatteryManagementSystem
{

    ubyte2 canMessageBaseId;

    SerialManager *sm;

    // 0x302 //

    ubyte1 faultCode;     // fault code, stored
    ubyte1 levelFaults;   // Level fault flags (e.g. over voltage, under voltage, etc) 

    // 0x304 //

    ubyte1 warnings;    // warning flags

    // 0x310 //

    ubyte1 state;   // current BMS state
    ubyte1 flags1;   // status flags 1 (pre charge relay closed, main contactor (+ and -) closed, and HVIL is present)
    ubyte1 flags2;   // status flags 2 (BMS monitor power active, cell balancing active) 

    // 0x311 //

    ubyte2 chargeLimit;    // Maximum current acceptable (charge) NOTE: Not sure about chargeLimit and dischargeLimit, please look over them
    ubyte2 dischargeLimit; // Maximum current available (discharge)

    // 0x320 //

    ubyte4 packVoltage; // pack voltage (unit: KV) 
    sbyte4 packCurrent; // pack current (unit: A)

    // 0x321 //

    ubyte2 SOC; // state of charge
    ubyte2 SOH; // pack health

    // 0x322 //

    ubyte1 minVtg;     // Voltage of least charged cell
    ubyte1 minVtgCell; // Position of cell with lowest voltage
    ubyte1 maxVtg;     // Voltage of most charged cell
    ubyte1 maxVtgCell; // Position of cell with highest voltage

    // 0x323 //

    sbyte1 minTempCell; // ID of cell with lowest temperature
    sbyte1 maxTempCell; // ID of cell with highest temperature

    // Variables that I couldn't figure out (sorry)

    ubyte2 timer;       // power up time
    ubyte4 batteryEnergyIn;  // Total energy into battery
    ubyte4 batteryEnergyOut; // Total energy out of battery
    ubyte2 DOD;      // depth of discharge
    ubyte2 capacity; // actual capacity of pack
    sbyte1 packTemp;    // average pack temperature
    sbyte1 minTemp;     // Temperature of coldest sensor
    ubyte2 packRes;    // resistance of entire pack
    ubyte1 minRes;     // resistance of lowest resistance cells
    ubyte1 minResCell; // ID of cell with lowest resistance
    ubyte1 maxRes;     // resistance of highest resistance cells
    ubyte1 maxResCell; // ID of cell with highest resistance
    sbyte1 maxTemp;     //Max Temp[104]
    sbyte1 avgTemp;     //Avg Temp[096]

    //ubyte1 SOC;          //SOC(%)[112]
    //ubyte1 SOC;          //SOC(%)[112]
    ubyte1 CCL; //DO NOT USE
    ubyte1 DCL; //DO NOT USE

    // signed = 2's complement: 0XfFF = -1, 0x00 = 0, 0x01 = 1
};

BatteryManagementSystem *BMS_new(SerialManager *serialMan, ubyte2 canMessageBaseID)
{

    BatteryManagementSystem *me = (BatteryManagementSystem *)malloc(sizeof(struct _BatteryManagementSystem));

    me->canMessageBaseId = canMessageBaseID;
    me->sm = serialMan;
    me->maxTemp = 99;

    me->packCurrent = 0;
    me->packVoltage = 0;

    me->CCL = 0;
    me->DCL = 0;
    me->chargeLimit = 0;
    me->dischargeLimit = 0;

    return me;
}

void BMS_parseCanMessage(BatteryManagementSystem *bms, IO_CAN_DATA_FRAME *bmsCanMessage)
{
    ubyte2 utemp16;
    //    sbyte1  temp16;
    ubyte4 utemp32;

    switch (bmsCanMessage->id)
    {

    case 0x302:

        bms->faultCode = bmsCanMessage->data[0];
        bms->levelFaults = bmsCanMessage->data[1];

        break;

    case 0x304:

        bms->warnings = bmsCanMessage->data[0];

        break;

    case 0x310:

        bms->state = bmsCanMessage->data[6];
        bms->flags1 = bmsCanMessage->data[3];
        bms->flags2 = bmsCanMessage->data[2];

        break;

    case 0x311:

        utemp16 = ((bmsCanMessage->data[2] << 8) | (bmsCanMessage->data[3]));
        bms->chargeLimit = swap_uint16(utemp16);

        utemp16 = ((bmsCanMessage->data[4] << 8) | (bmsCanMessage->data[5]));
        bms->dischargeLimit = swap_uint16(utemp16);

        break;

    case 0x320:

        bms->packVoltage = (((bmsCanMessage->data[7] << 8) | (bmsCanMessage->data[4])) / 1000); //V
        bms->packCurrent = (((bmsCanMessage->data[3] << 8) | (bmsCanMessage->data[0])) / 1000); //V

        break;

    case 0x321:

        bms->SOC = ((bmsCanMessage->data[7] << 8) | (bmsCanMessage->data[6])); //% The code for this and state of health probably aren't right
        bms->SOH = ((bmsCanMessage->data[5] << 8) | (bmsCanMessage->data[4])); //%

        break;

    case 0x322:

        bms->minVtg = (((bmsCanMessage->data[5] << 8) | (bmsCanMessage->data[4])) / 1000); 
        bms->minVtgCell = bmsCanMessage->data[3];    //probably wrong
        bms->maxVtg = (((bmsCanMessage->data[7] << 8) | (bmsCanMessage->data[6])) / 1000); 
        bms->maxVtgCell = bmsCanMessage->data[5];    //also probably wrong

        break;

    case 0x323:

        //The system for identifying cells confuses the hell out of me so idk

    }
}

sbyte1 BMS_getAvgTemp(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "AvgPackTemp: %i\n", me->avgTemp);
    return (me->avgTemp);
}
sbyte1 BMS_getMaxTemp(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "MaxTemp: %i\n", me->maxTemp);
    return (me->maxTemp);
}

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower(BatteryManagementSystem *me)
{
    //char buffer[32];
    //sprintf(buffer, "packVoltage: %f\n", me->packVoltage);
    return (me->packCurrent * me->packVoltage);
}

ubyte2 BMS_getPackTemp(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "PackTemp: %i\n", me->packTemp);
    return (me->packTemp);
}

ubyte1 BMS_getCCL(BatteryManagementSystem *me)
{
    //return me->CCL;
    return me->chargeLimit;
}

ubyte1 BMS_getDCL(BatteryManagementSystem *me)
{
    //return me->DCL;
    return me->dischargeLimit;
}

// ELITHION BMS OPTIONS //

//ubyte1  updateState(BMS*);
//ubyte2 updateTimer();
//ubyte1  updateFlags();
//ubyte1  updateFaultCode();
//ubyte1  updateLevelFaults();
//
//// PACK //
//
//ubyte2 updatePackVoltage();     // volts
//ubyte1  updateMinVtg();         // volts; individual cell voltage
//ubyte1  updateMaxVtg();
//ubyte1  updateMinVtgCell();
//ubyte1  updateMaxVtgCell();
//
//// CURRENT //
//
//sbyte2  updatePackCurrent();                  // amps
//ubyte2 updateChargeLimit();                // 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//ubyte2 updateDischargeLimit();            // 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//
//ubyte1  updateSOC();            // Returns a value from 0-100
//ubyte2 updateDOD();            // (Ah)
//ubyte2 updateCapacity();
//ubyte1  updateSOH();
//
//// TEMP //
//
//sbyte1  updatePackTemp();            // average pack temperature
//sbyte1  updateMinTemp();                // Temperature of coldest sensor
//sbyte1  updateMinTempCell();         // ID of cell with lowest temperature
//sbyte1  updateMaxTemp();                // Temperature of hottest sensor
//sbyte1  updateMaxTempCell();         // ID of cell with highest temperature
//
//ubyte2 updatePackRes();                // resistance of entire pack
//ubyte1  updateMinRes();              // resistance of lowest resistance cells
//ubyte1  updateMinResCell();          // ID of cell with lowest resistance
//ubyte1  updateMaxRes();                // resistance of highest resistance cells
//ubyte1  updateMaxResCell();            // ID of cell with highest resistance
//
//
//LimitCause updateChargeLimitCause();
//LimitCause updateDischargeLimitCause();
//
//
////void getFaults(FaultOptions *presentFaults, StoredFault *storedFault, FaultOptions *presentWarnings);
//void clearStoredFault();
//
//IOFlags getIOFlags();
