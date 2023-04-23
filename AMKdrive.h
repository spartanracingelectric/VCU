#ifndef _AMKDRIVE_H
#define _AMKDRIVE_H

/***** Standard includes *****/
#include <stdlib.h> //Needed for malloc
#include "IO_CAN.h"

#include "IO_Driver.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "readyToDriveSound.h"

// Base CAN message ID for "setpoint" message - outgoing to inverter
#define DI_BASE_CAN_ID_OUTGOING 0x183
// Base CAN message ID for both "actual" messages - incoming from inverter
// Note that the address for "Actual2" is calculated as this ID + 2 (plus offset)
#define DI_BASE_CAN_ID_INCOMING 0x282

//----------------------------------------------------------------------------
// DI_Location_Address - This maps physical location to CAN ID offset (AMK calls this "node address").
// AMK does not specify which inverter is at which side/axle of the car.
// Note: Motor direction is set in Aipex tool, ID32773 'Service bits' bit 16 = 1.
//----------------------------------------------------------------------------
// Offset  Position     Setpoint Actual1 Actual2
// ----------------------------------------
//         Base address  0x183    0x282   0x284
// ----------------------------------------
//    1    Front left    0x184    0x283   0x285
//    2    Front right   0x185    0x284   0x286
//   3-4   NOT USABLE
//    5    Rear left     0x188    0x287   0x289
//    6    Rear right    0x189    0x288   0x290
//----------------------------------------------------------------------------

typedef enum _DI_Location_Address {
    FRONT_LEFT = 1,
    FRONT_RIGHT = 2,
    REAR_LEFT = 5,
    REAR_RIGHT = 6
} DI_Location_Address;

typedef struct _DriveInverter {
    DI_Location_Address location_address;  // CAN ID offset and physical location
    ubyte2 canIdOutgoing;   // Calculated CAN ID for "setpoint" message
    ubyte2 canIdIncoming;   // Calculated CAN ID for "Actual1" message, also used for "Actual2" message

    int startUpStage;

    // Setpoints (commands, outgoing: 0x183 + offset)
    bool AMK_bInverterOn;
    bool AMK_bDcOn;
    bool AMK_bEnable;
    bool AMK_bErrorReset;
    sbyte2 AMK_TorqueSetpoint;
    sbyte2 AMK_TorqueLimitPositiv;
    sbyte2 AMK_TorqueLimitNegativ;

    // Actual Values 1 (incoming: 0x282 + offset)
    bool AMK_bSystemReady;
    bool AMK_bError;
    bool AMK_bWarn;
    bool AMK_bQuitDcOn;
    bool AMK_bDcOnVal;
    bool AMK_bQuitInverterOnVal;
    bool AMK_bInverterOnVal;
    bool AMK_bDerating;
    sbyte2 AMK_ActualVelocity;  // RPM
    sbyte2 AMK_TorqueCurrent;
    sbyte2 AMK_MagnetizingCurrent;

    // Actual Values 2 (incoming: 0x284 + offset)
    sbyte2 AMK_TempMotor;       // 0.1degC
    sbyte2 AMK_TempInverter;    // 0.1degC
    ubyte2 AMK_ErrorInfo;       
    sbyte2 AMK_TorqueFeedback;        // 0.1degC
} _DriveInverter;

_DriveInverter* AmkDriver_new(DI_Location_Address location_address);

void DI_calculateInverterControl(_DriveInverter* Idv, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds);

void DI_calculateCommands(_DriveInverter* Idv, TorqueEncoder *tps, BrakePressureSensor *bps);

void DI_parseCanMessage(IO_CAN_DATA_FRAME* diCanMessage, _DriveInverter* Idv);

void DI_commandTorque(sbyte2 newTorque, _DriveInverter* Idv);

sbyte2 DI_getCommandedTorque(_DriveInverter* Idv);

#endif