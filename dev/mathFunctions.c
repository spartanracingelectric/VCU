#include <stdarg.h>
#include <math.h>
#include "mathFunctions.h"
#include "IO_Driver.h"
#include "IO_RTC.h"

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
float4 getPercent(float4 value, float4 start, float4 end, bool zeroToOneOnly)
{
    float4 retVal = (value - start) / (end - start);

    if (zeroToOneOnly == TRUE)
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
ubyte2 max(ubyte2 a, ubyte2 b)
{
    return (a > b) ? a : b;
}

// A utility function to get maximum of two integers
ubyte2 min(ubyte2 a, ubyte2 b)
{
    return (a < b) ? a : b;
}

/**********************************************************************/ /**
 *
 * \brief Decides whether a blinking light should be on or off based on
 *        the system clock and period given
 *
 * \param timestamp     A timestamp from IO_RTC for when the blink was started
 * \param highPeriod    How long the blink should stay high, in ms (NOT ns)
 * \param lowPeriod     How long the blink should stay low -- NOT IMPLEMENTED;
 *                      Always the same as highPeriod
 *
 * \return boolean:
 * \retval TRUE      blink should be high
 * \retval FALSE     blink should be low
 *
 ***************************************************************************
 *
 * \remarks
 *   Blablabla
 *
 ***************************************************************************/
bool blink(ubyte4 *clock, ubyte2 highPeriod)
{
    //time passed since the start of the blinks divided by the period to get
    //count.
    //count % 2 gets the current state the blink should be in 0 or <0
    ubyte4 count = IO_RTC_GetTimeUS(*clock) / highPeriod;
    //removes decimal places. there may be a better way to do this but I got lazy
    count = count - count % 1;

    return !(count / highPeriod) % 2;
}

//byte swapping functions used by BMS

ubyte1 swap_uint8(ubyte1 val)
{
    return (val << 4) | (val >> 4);
}

sbyte1 swap_int8(sbyte1 val)
{
    return (val << 4) | (val >> 4);
}
ubyte2 swap_uint16(ubyte2 val)
{
    return (val << 8) | (val >> 8);
}

//! Byte swap short
sbyte2 swap_int16(sbyte2 val)
{
    return (val << 8) | ((val >> 8) & 0xFF);
}

//! Byte swap unsigned int
ubyte4 swap_uint32(ubyte4 val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | (val >> 16);
}

//! Byte swap int
sbyte4 swap_int32(sbyte4 val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

float4 get_lowerStepInterval(float4 value, float4 increment) {
    return value - (value % increment);
}
float4 get_upperStepInterval(float4 value, float4 increment) {
    return get_lowerStepInterval(value, increment) + increment; //For simplicity and understanding, keeping the function call. To decrease total function time, convert to pointers.
}