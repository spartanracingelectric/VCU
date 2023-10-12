#include <IO_Driver.h>

#define CYCLE_TIME_US 10000 // in microseconds
#define CYCLE_TIME (100000.0 / CYCLE_TIME_US) // in seconds
#define CAN_0_BAUD 500 // in kbps
#define CAN_1_BAUD 500 // in kbps