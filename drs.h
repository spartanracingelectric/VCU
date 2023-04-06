#ifndef _DRS_H
#define _DRS_H

#include "IO_Driver.h"
#include "WheelSpeeds.h"
#include "brakePressureSensor.h"
#include "torqueEncoder.h"
#include "motorController.h"
#include "sensorCalculations.h"

// Rotary Switch modes:
//      Mode 0 - Always off
//      Mode 1 - Always on
//      Mode 2 - Driver controlled
//      Mode 3 - Auto (sensor controlled)
typedef enum { STAY_CLOSED, STAY_OPEN, MANUAL, AUTO } Mode;

//_DRS is a structure tag, used to reference the structure for future initializations 
typedef struct _DRS
{
    //flag indicating if DRS should be active or inactive based on inputs and mode
    bool AutoDRSActive;

    //enum indicating current DRS mode selected based on rotaryswitch
    Mode currentDRSMode;

    //state of open and close valves on 2 port DRS solenoid
    bool openSolenoidState;
    bool closeSolenoidState;
    //timer to implement timed closing functionality
    ubyte4 drsSolenoidTimer;

    //steering wheel inputs
    //DRS mode is set by a potentiometer, need an analog voltage reading
    //how to create voltage map for switch configurations?
    ubyte4 drsRotarySwitch;
    
    //digital input for button on steering wheel
    bool driverDRSButton;
} DRS;
//DRS is an instance of the _DRS struct
//what is the purpose of this initialized struct?

//initialize new DRS objects
DRS *DRS_new();

//DRS control logic
void update_DRS_mode(DRS *me, MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps);
//make sensor comparisons to thresholds and set drsActive flag
void updateAuto(DRS *me, MotorController *mcm, TorqueEncoder *tps, BrakePressureSensor *bps);

//decide to call either open or close based on conditions
void toggleDRS(DRS *me);
//actuate the wing to physically activate DRS
void DRS_open(DRS *me);
//retract the wing to physically deactivate DRS
void DRS_close(DRS *me);

float map(float val, float lower_a, float upper_a, float lower_b, float upper_b);

void update_knob(DRS *me);

#endif