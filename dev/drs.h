#ifndef _DRS_H
#define _DRS_H

#include "IO_Driver.h"
#include "WheelSpeeds.h"
#include "brakePressureSensor.h"
#include "torqueEncoder.h"
#include "motorController.h"
#include "sensors.h"

// Rotary Switch modes:
//      Mode 0 - Always off
//      Mode 1 - Always on
//      Mode 2 - Driver controlled
//      Mode 3 - Auto (sensor controlled)
enum
{
    STAY_CLOSED,
    STAY_OPEN,
    MANUAL,
    AUTO
};

//_DRS is a structure tag, used to reference the structure for future initializations
typedef struct _DRS
{
    // flag indicating if DRS should be active or inactive based on inputs and mode
    bool AutoDRSActive;

    // enum indicating current DRS mode selected based on rotary switch
    ubyte1 currentDRSMode;
    ubyte1 buttonPressed;
    ubyte1 drsFlap;

} DRS;
// DRS is an instance of the _DRS struct
// what is the purpose of this initialized struct?

// initialize new DRS objects
void DRS_new(DRS *me);

// DRS control logic
void DRS_update(DRS *me, ubyte1 pot_DRS_LC, bool lc_status);
// make sensor comparisons to thresholds and set drsActive flag
void runAuto(DRS *me);

// actuate the wing to physically activate DRS
void DRS_open(DRS *me);
// retract the wing to physically deactivate DRS
void DRS_close(DRS *me);

void update_knob(DRS *me);

#endif