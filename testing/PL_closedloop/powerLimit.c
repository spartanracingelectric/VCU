
#include "PID.h"
#include "hashTable.h"
#include "mathFunctions.h"
#include "powerLimit.h"
#include "stdbool.h"
#include <stdlib.h>
PowerLimit* POWERLIMIT_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->pid = PID_new(20, 0, 0, 0);

    me->plMode = 0;

    me->plStatus = false;
    me->pidOutput = 0; 
    me->plTorqueCommand = 0; 
    me->plTargetPower = 80;
    me->plInitializationThreshold = me->plTargetPower - 15;
    return me;
}
