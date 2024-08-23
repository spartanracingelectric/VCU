#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include "mathFunctions.h"


float getPercent(float value, float start, float end, bool zeroToOneOnly)
{
    float retVal = (value - start) / (end - start);

    if (zeroToOneOnly)
    {
        if (retVal < 0)
        {
            return 0.0;
        }
        if (retVal > 1)
        {
            return 1.0;
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
int min(int a, int b)
{
    return (a < b) ? a : b;
}


int int_lowerStepInterval(int value, int increment) {
    return value - (value % increment);
}
int int_upperStepInterval(int value, int increment) {
    int temp = int_lowerStepInterval(value, increment);
    return (temp == value?temp:temp + increment);
}
