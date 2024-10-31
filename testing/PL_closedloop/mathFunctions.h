#ifndef _MATHFUNCTIONS_H
#define _MATHFUNCTIONS_H
#include <stdbool.h>

  //Includes datatypes, constants, etc - should be included in every c file

/*****************************************************************************
* Helper functions
****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
int getPercent(int value, int start, int end, bool zeroToOneOnly);

// A utility function to get maximum of two integers
int max(int a, int b);


/*
*  Functions for endian conversion
*/

/* Functions for Adjusting Value to a Step Function Increment */
int int_lowerStepInterval(int value, int increment);
int int_upperStepInterval(int value, int increment);
int floorToNearest5(int num);
int ceilToNearest5(int num);
int floorToNearest160(int num);
int ceilToNearest160(int num);
int max(int a, int b);
int getPercent(int value, int start, int end, bool zeroToOneOnly);
#endif //  _MATHFUNCTIONS_H
