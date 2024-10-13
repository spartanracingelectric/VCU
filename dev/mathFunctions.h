#ifndef _MATHFUNCTIONS_H
#define _MATHFUNCTIONS_H


#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file

/*****************************************************************************
* Helper functions
****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
float4 getPercent(float4 value, float4 start, float4 end, bool zeroToOneOnly);

// A utility function to get maximum of two integers
ubyte2 max(ubyte2 a, ubyte2 b);


/*
*  Functions for endian conversion
*/
ubyte1 swap_uint8(ubyte1 val);
sbyte1 swap_int8(sbyte1 val);
ubyte2 swap_uint16(ubyte2 val);
sbyte2 swap_int16(sbyte2 val);
ubyte4 swap_uint32(ubyte4 val);
sbyte4 swap_int32(sbyte4 val);
/* Functions for Adjusting Value to a Step Function Increment */
ubyte4 ubyte4_lowerStepInterval(ubyte4 value, ubyte4 increment);
ubyte4 ubyte4_upperStepInterval(ubyte4 value, ubyte4 increment);
float4 float4_lowerStepInterval(float4 value, float4 increment);
 float4 float4_upperStepInterval(float4 value, float4 increment);
#endif //  _MATHFUNCTIONS_H
