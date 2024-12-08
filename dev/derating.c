#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "derating.h"
#include "bms.h" //cell temps
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h" //torque request
#include "brakePressureSensor.h" //brakes
#include "motorController.h" //torque info, feedback 
#include "sensorCalculations.h"

/* Torque Limit
Endurance Limit: 170Nm
    170Nm, stays below 80kW
1. Derating only activates for limp mode
    - 55C, 125Nm
    1.Reading Data
        - Use pre-made functions from other files (BMS & MCM)
        - Refer to bms.c and motorController.c for get_functions
2. Sofrware integration
*/