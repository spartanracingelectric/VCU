
#include "PID.h"
#include "hashTable.h"
#include "mathFunctions.h"
#include "powerLimit.h"
#include "stdbool.h"
#include <stdlib.h>
PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));

    me->hashtable = HashTable_new();

    me-> PLstatus = false;
   // me->pid = PID_new(1, 0, 0, 0);// fill this in  
    me->mcm_voltage = 0.0; 
     me->mcm_current = 0.0; 
      me->power = 0.0; 
       me->wheelspeed = 0.0; 

        me->LUT_val = 0.0; 
         me->error = 0.0; 
          me->estimatedtq = 0.0; 
           me->setpointtq = 0.0; 
     
    return me;
}
