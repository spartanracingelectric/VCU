#include "watchdog.h"
#include "IO_Driver.h"
#include "IO_RTC.h"

/// @brief Generates a watchdog timer, timeout is in microseconds
/// @param wd
/// @param timeout
void WatchDog_new(WatchDog *wd, ubyte4 timeout)
{
    wd->timeout = timeout;
    wd->running = FALSE;
}

void WatchDog_reset(WatchDog *wd) { wd->running = TRUE; }

void WatchDog_pet(WatchDog *wd)
{
    if (wd->running)
    {
        IO_RTC_StartTime(&wd->timestamp);
    }
}

bool WatchDog_check(WatchDog *wd)
{
    if (wd->running)
    {
        if (IO_RTC_GetTimeUS(wd->timestamp) >= wd->timeout)
        {
            wd->running = TRUE;
            return TRUE;
        }
    }
    wd->running = FALSE;
    return FALSE;
}