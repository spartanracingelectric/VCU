
#ifndef GLOBAL_CONSTANTS_H
#define GLOBAL_CONSTANTS_H
/** THE PURPOSE OF THIS FILE IS TO GLOBALLY DEFINE WIDELY USED VALUES. 
 * IF YOU WISH TO DEFINE SOMETHING WITH A NARROW AND SPECIFIC FOCUS, 
 * OR A NARROW AND SPECIFIC USE CASE, USE A MORE APPROPIATE FILE THAN THIS ONE. **/
#include "ptypes_xe167.h"

    #ifndef VCU_CYCLE_TIME_HZ
        #define VCU_CYCLE_TIME_HZ (ubyte1) 100
    #endif

    #ifndef VCU_CYCLE_TIME_SECONDS
        #define VCU_CYCLE_TIME_SECONDS (float4) 0.01 //10 ms
    #endif

    #ifndef VCU_CYCLE_TIME_MICROSECONDS
        #define VCU_CYCLE_TIME_MICROSECONDS (ubyte2) 10000 //1000 μs = 1 ms
    #endif

    #ifndef MOTOR_AND_CONTROLS_SYSTEMS
    #define MOTOR_AND_CONTROLS_SYSTEMS
        #ifndef MCM_MAX_TORQUE_DNm
            #define MCM_MAX_TORQUE_DNm (ubyte2) 2310
        #endif

        #ifndef KW_LIMIT
            #define KW_LIMIT (ubyte1) 80
        #endif

        #ifndef WATT_LIMIT
            #define WATT_LIMIT (ubyte4) 80000
        #endif
    #endif

    #ifndef MATHVARIABLES
    #define MATHVARIABLES
        #ifndef PI_FLOAT
            #define PI_FLOAT (float4) 3.14159
        #endif
        
        #ifndef PI_SIX_DIGITS_UBYTE4
            #define PI_SIX_DIGITS_UBYTE4 (ubyte4) 314159
        #endif
    #endif

    #define POWERLIMIT_ENABLE
    #define LAUNCHCONTROL_ENABLE

#endif /* GLOBAL_CONSTANTS_H */