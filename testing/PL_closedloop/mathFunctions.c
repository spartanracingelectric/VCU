#include <stdbool.h>
#include "mathFunctions.h"

/*****************************************************************************
* Helper functions
****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* Special features:
*   - Handles cases where "start" is less than "end" (value goes backwards)
* Automatically compensates for reverse-direction values: When min > max, value is assumed to travel in reverse direction.
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
* If range == 0, then 0 will be returned.  (Safety: in case of problem during regen torque calculation)
-------------------------------------------------------------------*/
int getPercent(int value, int start, int end, bool zeroToOneOnly)
{
    int retVal = (value - start) / (end - start);

    if (zeroToOneOnly == true)
    {
        if (retVal < 0)
        {
            return retVal = 0;
        }
        if (retVal > 1)
        {
            return retVal = 1;
        }
    }

    return retVal;
}

// A utility function to get maximum of two integers
int max(int a, int b)
{
    return (a > b) ? a : b;
}

// A utility function to get maximum of two integers


int int_lowerStepInterval(int value, int increment) {
    return value - (value % increment);
}
int int_upperStepInterval(int value, int increment) {
    return int_lowerStepInterval(value, increment) + increment; //For simplicity and understanding, keeping the function call. To decrease total function time, convert to pointers.
}
int floorToNearest5(int num)
{
     if (num % 5 != 0) {
        int remainder = num % 5;
        return num - remainder;
    } 
    else 
    {
        return num; 
    }
}
int ceilToNearest5(int num)
{
    if (num % 5 != 0) {
        int remainder = num % 5;
        return num + (5 - remainder);
    } 
    else
    {
        return num;
    }
}

int floorToNearest160(int num){
 if (num % 160 != 0) {
   if (num < 2000) {
        return 160; // If num is less than 2000, return 2000
    }
    
    // Calculate the nearest floor value
    int adjustedNum = num - 2000;
    int remainder = adjustedNum % 160;

    if (remainder != 0) {
        return num - remainder; // Round down to the nearest multiple of 160
    } else {
        return num; // Already a multiple of 160
    }
}
}
int ceilToNearest160(int num){
 if (num < 2000) {
        return 160; // If num is less than 2000, return 2000
    }
    
    // Calculate the nearest ceiling value
    int adjustedNum = num - 2000;
    int remainder = adjustedNum % 160;

    if (remainder != 0) {
        return num + (160 - remainder); // Round up to the nearest multiple of 160
    } else {
        return num; // Already a multiple of 160
    }
}