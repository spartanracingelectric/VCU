#include "IO_Driver.h" 
#include "motorController.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "PID.h"

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
     me->value=0;

    me->PLStatus = FALSE;
    me->LUTval=0;
    me->setpoint=0;
    me->actual= 0;
    me->pltorque= 0;
    me->piderror= 0;
   
    return me;
    }

void testing(PowerLimit *me){
    me->PLStatus = TRUE;
    me->value = 1000;
}

void powerLimitTorqueCalculation_1(PowerLimit *me,  MotorController* mcm, PID* pid){
   float gain = 95.49; //for decinm
  sbyte4 watts = MCM_getPower(mcm);
  int wheelspeed = MCM_getMotorRPM(mcm);
  if(watts > 55000-10){
    me-> PLStatus = TRUE;
    ubyte2 pidsetpoint = (ubyte2)((55000*gain/wheelspeed));
    me->setpoint =pidsetpoint;
    ubyte2 pidactual = (ubyte2)((55000*gain/wheelspeed));
    me->actual= pidactual;
    PID_setpointUpdate(pid,pidsetpoint);
    sbyte2 PIDerror = PID_compute(pid, pidactual);
    me->piderror = PIDerror;
    ubyte2 PLfinaltq = (ubyte2)(pidactual+ PIDerror);
    me->pltorque= PLfinaltq;
  }
  else{
    me->PLStatus= FALSE;
  }
  MCM_update_PL_TorqueLimit(mcm,  me->pltorque); // we need to change this on mcm.c / pl.c/.h 
  MCM_update_PL_State(mcm, me->PLStatus);
}

void powerLimitTorqueCalculation_2(PowerLimit *me,  MotorController* mcm){

}