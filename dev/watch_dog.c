#include "IO_Driver.h"
#include "IO_RTC.h"
#include "watch_dog.h"
#include "main.h"

/// @brief Generates a watchdog timer, timeout is in microseconds
/// @param wd
/// @param timeout
void WatchDog_new(WatchDog *wd, ubyte4 timeout)
{
    wd->timeout = timeout;
    wd->running = FALSE;
    wd->mood = 0;
}

void WatchDog_reset(WatchDog *wd)
{
    wd->running = TRUE;
    wd->mood = 0;
}

void WatchDog_pet(WatchDog *wd)
{
    IO_RTC_StartTime(&wd->timestamp);
    if (!wd->running)
    {
        if (IO_RTC_GetTimeUS(wd->timestamp) >= wd->timeout)
        {
            wd->mood++;
            if (wd->mood == BMS_WATCHDOG_CLEAR)
            {
                wd->running = TRUE;
            }
        }
        else
        {
            wd->mood = 0; // reset the counter if the messages are spaced too far apart
        }
    }
}

bool WatchDog_check(WatchDog *wd)
{
    return (wd->running && IO_RTC_GetTimeUS(wd->timestamp) >= wd->timeout);
}