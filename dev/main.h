#include <IO_Driver.h>

#define CYCLE_TIME_US 10000 // in microseconds
#define CYCLE_TIME (100000.0 / CYCLE_TIME_US) // in seconds
#define CAN_0_BAUD 500 // in kbps
#define CAN_1_BAUD 500 // in kbps
//16 bumps per rotation, 16 hz = 1 rotation per second
#define F_WSS_TICKS 26 // number of ticks on the tone wheel
#define R_WSS_TICKS 22 // number of ticks on the tone wheel
#define WHEEL_DIAMETER 18 //Inches
#define LV_BATT_S 8 // number of cells in series for the LV Battery
#define LC_STEERING_THRESHOLD 15.0f // in degrees