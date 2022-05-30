// Traction control Header

#include "IO_Driver.h" 
//#include "IO_UART.h" //for sending serial data, need?
#include "pid.h"
#include "motorController.h"
#include "wheelspeeds.h"

#include "sensors.h"
#include "PID.h"

//what function, data type, or macro definitions needed?

//Traction Control Mode
typedef enum TCsetting { TC0 = 0, TC1, TC2, TC3 } TCSMode;

typedef struct _TractionControl TractionControl;

void TC_setMode(TractionControl *TCSMode); 
//void TC_setMode(TractionControl *TCSMode)// these aren't written correctly..
// void SlipRatio(MotorController *me, WheelSpeeds *me);