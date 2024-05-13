#ifndef _LUT_H
#define _LUT_H

#include "IO_Driver.h"

typedef struct LUT
{
    float4 min;
    float4 max;
    ubyte1 len;
    float4 *lut; // pointer to an array of floats
} LUT;

// Function to create a LUT with dynamic memory allocation
LUT *createLUT(float4 min, float4 max, ubyte1 len, float4 *initialValues);

// Function to free the memory allocated for the LUT
void freeLUT(LUT *lut);

// Function for linear interpolation between two float values
float4 lerp(float4 v0, float4 v1, float4 t);

// Function to get value from LUT using linear interpolation
float4 getValueFromLUT(LUT *lut, float4 input);

#endif // LUT_H