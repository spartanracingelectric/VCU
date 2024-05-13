#ifndef _TIMERDEBUG_H
#define _TIMERDEBUG_H

#include "IO_Driver.h"
#include "IO_RTC.h"

#define TIMER_UPDATE_INTERVAL_LIM (ubyte2)500 // //only get time every n amount of VCU cycles

typedef struct _TimerDebug
{
    ubyte4 RTCTimer;
    ubyte4 prevTime;

    ubyte2 timerUpdateIntervalCounter;
} TimerDebug;

void TimerDebug_new(TimerDebug *me);
void TimerDebug_startTimer(TimerDebug *me);
void TimerDebug_stopTimer(TimerDebug *me);
ubyte4 TimerDebug_getTime(TimerDebug *me);
ubyte2 TimerDebug_getIntervalCounter(TimerDebug *me);
void TimerDebug_testTimer();

#endif