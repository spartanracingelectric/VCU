
#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "serial.h"
#include "mathFunctions.h"

/*********************************************************
 *            *********** CAUTION ***********            *
 * MULTI-BYTE VALUES FOR THE STAFL BMS ARE LITTLE-ENDIAN *
 *                                                       *
 *********************************************************/

struct _BatteryManagementSystem
{

    ubyte2 canMessageBaseId;

    SerialManager *sm;

    // 0x622h //

    ubyte1 state;       // state of system
    ubyte2 timer;       // power up time
    ubyte1 flags;       // flags
    ubyte1 faultCode;   // fault code, stored
    ubyte1 levelFaults; // Level fault flags (e.g. over voltage, under voltage, etc)
    ubyte1 warnings;    // warning flags

    // 0x623h //

    //    ubyte2 packVoltage;        // Total voltage of pack
    ubyte1 minVtg;     // Voltage of least charged cell
    ubyte1 minVtgCell; // ID of cell with lowest voltage
    ubyte1 maxVtg;     // Voltage of most charged cell
    ubyte1 maxVtgCell; // ID of cell with highest voltage

    // 0x624h //

    //    sbyte2  packCurrent;        // Pack current
    ubyte2 chargeLimit;    // Maximum current acceptable (charge)
    ubyte2 dischargeLimit; // Maximum current available (discharge)

    // 0x625h //

    ubyte4 batteryEnergyIn;  // Total energy into battery
    ubyte4 batteryEnergyOut; // Total energy out of battery

    // 0x626h //

    ubyte1 SOC;      // state of charge
    ubyte2 DOD;      // depth of discharge
    ubyte2 capacity; // actual capacity of pack
    ubyte1 SOH;      // State of Health

    // 0x627h //

    sbyte1 packTemp;    // average pack temperature
    sbyte1 minTemp;     // Temperature of coldest sensor
    sbyte1 minTempCell; // ID of cell with lowest temperature
                        //    sbyte1  maxTemp;            // Temperature of hottest sensor
    sbyte1 maxTempCell; // ID of cell with highest temperature

    // 0x628h //

    ubyte2 packRes;    // resistance of entire pack
    ubyte1 minRes;     // resistance of lowest resistance cells
    ubyte1 minResCell; // ID of cell with lowest resistance
    ubyte1 maxRes;     // resistance of highest resistance cells
    ubyte1 maxResCell; // ID of cell with highest resistance

    // 0X629 //

    sbyte4 packVoltage; //Voltage(100mV)[022]
    sbyte4 packCurrent; //Current(100mA)[054]
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

    case 0x622:

        bms->state = bmsCanMessage->data[0];
        utemp16 = (ubyte2)bmsCanMessage->data[1] << 8 | bmsCanMessage->data[2];
        bms->timer = swap_uint16(utemp16);
        bms->flags = bmsCanMessage->data[3];
        bms->faultCode = bmsCanMessage->data[4];
        bms->levelFaults = bmsCanMessage->data[5];
        bms->warnings = bmsCanMessage->data[6];
        break;

    case 0x623:

        //    utemp16 = ((bmsCanMessage->data[0] << 8) | (bmsCanMessage->data[1]));
        //    bms->packVoltage = swap_uint16(utemp16);  //65535 = 65535V
        bms->minVtg = (bmsCanMessage->data[2] / 10); //255 = 25.5V
        bms->minVtgCell = bmsCanMessage->data[3];    //1-254
        bms->maxVtg = (bmsCanMessage->data[4] / 10); //255 = 25.5V
        bms->maxVtgCell = bmsCanMessage->data[5];    //1-254

        break;

    case 0x624:

        //    temp16 = ((bmsCanMessage->data[0] << 8) | (bmsCanMessage->data[1]));
        //    bms->packCurrent = swap_int16(temp16);

        utemp16 = ((bmsCanMessage->data[2] << 8) | (bmsCanMessage->data[3]));
        bms->chargeLimit = swap_uint16(utemp16);

        utemp16 = ((bmsCanMessage->data[4] << 8) | (bmsCanMessage->data[5]));
        bms->dischargeLimit = swap_uint16(utemp16);

        break;

    case 0x625:

        utemp32 = ((((ubyte4)bmsCanMessage->data[0] << 24) |
                    ((ubyte4)bmsCanMessage->data[1] << 16) |
                    ((ubyte4)bmsCanMessage->data[2] << 8) |
                    (bmsCanMessage->data[3])));
        bms->batteryEnergyIn = swap_uint32(utemp32);

        utemp32 = ((((ubyte4)bmsCanMessage->data[4] << 24) |
                    ((ubyte4)bmsCanMessage->data[5] << 16) |
                    ((ubyte4)bmsCanMessage->data[6] << 8) |
                    ((ubyte4)bmsCanMessage->data[7])));
        bms->batteryEnergyOut = swap_uint32(utemp32);

        break;

    case 0x626:

        bms->SOC = bmsCanMessage->data[0];

        utemp16 = ((bmsCanMessage->data[1] << 8) | (bmsCanMessage->data[2]));
        bms->DOD = swap_uint16(utemp16);

        utemp16 = ((bmsCanMessage->data[3] << 8) | (bmsCanMessage->data[4]));
        bms->capacity = swap_uint16(utemp16);
        bms->SOH = bmsCanMessage->data[6];

        break;

    case 0x627:

        //bms->packTemp = bmsCanMessage->data[0];
        bms->minTemp = bmsCanMessage->data[2];
        bms->minTempCell = bmsCanMessage->data[3];
        bms->maxTemp = bmsCanMessage->data[4];
        bms->maxTempCell = bmsCanMessage->data[5];

        break;

    case 0x628:

        utemp16 = ((bmsCanMessage->data[0] << 8) | (bmsCanMessage->data[1]));
        bms->packRes = ((swap_uint16(utemp16)) / 10000);
        //        1 = 100 mOhm
        //        10 = 1000 mOhm = 0.001 ohm = 10/10000
        //        100 = 10000 mohm = 0.01 ohm
        //        1000 = 100000 mOhM = 0.1 ohm
        //        10000 = 1000000 mOhm = 1 ohm
        bms->minRes = (bmsCanMessage->data[2] / 10000);
        bms->minResCell = bmsCanMessage->data[3];
        bms->maxRes = (bmsCanMessage->data[4] / 10000);
        bms->maxResCell = bmsCanMessage->data[5];
        break;

    case 0x629:
        //See https://onedrive.live.com/view.aspx?resid=F9BB8F0F8FDB5CF8!36803&ithint=file%2cxlsx&app=Excel&authkey=!AI-YHJrHmtUaWpI
        //Voltage(100mV)[022] - 2 bytes little endian (1=100mV) 0000000000000001 = 1 = 100 mV = 0.1 V
        //        5 = 500 mV = 0.5 V
        //        10 = 1000 mV = 1 V
        //       100 = 10000 mV = 10 V
        //         1000 = 100000 mV = 100 V
        //Current(100mA)[054] - 2 bytes little endian (1=100mA)
        //Max Temp[104] - in C
        //Avg Temp[096] - in C
        //SOC(%)[112] - %
        //DCL(%)[080]- %  //**CHANGED**

        bms->packVoltage = (((bmsCanMessage->data[1] << 8) | (bmsCanMessage->data[0])) / 10); //V
        bms->packCurrent = (((bmsCanMessage->data[3] << 8) | (bmsCanMessage->data[2])) / 10); //V
        bms->maxTemp = ((bmsCanMessage->data[4]));                                            //C
        bms->avgTemp = ((bmsCanMessage->data[5]));                                            //C
        bms->CCL = ((bmsCanMessage->data[6]));                                                //%
        bms->DCL = ((bmsCanMessage->data[7]));                                                //%

        break;
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
