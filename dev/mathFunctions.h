#ifndef _MATHFUNCTIONS_H
#define _MATHFUNCTIONS_H

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

/*****************************************************************************
 * Helper functions
 ****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
float4 getPercent(float4 value, float4 start, float4 end, bool zeroToOneOnly);

// Function to reassemble a 4-byte integer from a byte array starting from a specified index
ubyte4 reasm_ubyte4(const ubyte1 *data, ubyte1 start_index);
// Function to reassemble a 2-byte integer from a byte array starting from a specified index
ubyte2 reasm_ubyte2(const ubyte1 *data, ubyte1 start_index);
sbyte2 reasm_sbyte2(const ubyte1 *data, ubyte1 start_index);

//  Functions for endian conversion
ubyte1 swap_uint8(ubyte1 val);
sbyte1 swap_int8(sbyte1 val);
ubyte2 swap_uint16(ubyte2 val);
sbyte2 swap_int16(sbyte2 val);
ubyte4 swap_uint32(ubyte4 val);
sbyte4 swap_int32(sbyte4 val);

// make our own min and max because tasking has a fatal error with the stdlib min and max
#define max_ew(a, b) (((a) > (b)) ? (a) : (b))
#define min_ew(a, b) (((a) < (b)) ? (a) : (b))

#endif //  _MATHFUNCTIONS_H
