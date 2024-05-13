#include <stdio.h>
#include <stdlib.h>
#include "IO_Driver.h"

#include "lut.h"

// Function to create a LUT with dynamic memory allocation
LUT *createLUT(float4 min, float4 max, ubyte1 len, float4 *initialValues)
{
    if (len <= 0 || !initialValues)
    {
        return NULL;
    }

    LUT *newLut = (LUT *)malloc(sizeof(LUT));
    if (!newLut)
    {
        return NULL;
    }

    newLut->min = min;
    newLut->max = max;
    newLut->len = len;
    newLut->lut = (float4 *)malloc(sizeof(float4) * len);

    if (!newLut->lut)
    {
        free(newLut); // prevent memory leak by freeing already allocated LUT
        return NULL;
    }

    for (ubyte1 i = 0; i < len; i++)
    {
        newLut->lut[i] = initialValues[i];
    }

    return newLut;
}

// Function to free the memory allocated for the LUT
void freeLUT(LUT *lut)
{
    if (lut)
    {
        free(lut->lut); // free the array
        free(lut);      // free the struct itself
    }
}

// Function for linear interpolation between two float4 values
float4 lerp(float4 v0, float4 v1, float4 t)
{
    return (1 - t) * v0 + t * v1;
}

// Function to get value from LUT using linear interpolation
float4 getValueFromLUT(LUT *lut, float4 input)
{
    if (!lut || !lut->lut)
    {
        return -1; // Use an appropriate error value for your context
    }

    if (input < lut->min || input > lut->max)
    {
        return -1; // Use an appropriate error value for your context
    }

    // Scale the input value to the range of the LUT indices
    float4 scale = (lut->len - 1) / (lut->max - lut->min);
    float4 adjustedInput = (input - lut->min) * scale;

    // Find indices for interpolation
    ubyte1 indexLow = (ubyte1)adjustedInput;
    ubyte1 indexHigh = indexLow + 1;

    if (indexHigh >= lut->len)
    {
        // Clamp to the end of the LUT if at or beyond the end
        return lut->lut[lut->len - 1];
    }

    // Compute interpolation parameter
    float4 t = adjustedInput - (float4)indexLow;

    // Return the interpolated value
    return lerp(lut->lut[indexLow], lut->lut[indexHigh], t);
}

// float4 initialValues[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
// LUT* myLut = createLUT(0.0, 1.0, 10, initialValues);
// Use your LUT
// e.g., float4 value = getValueFromLUT(myLut, input);
