#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include "IO_Driver.h"

typedef struct _WatchDog {
    ubyte4 timestamp;
    ubyte4 timeout;
    bool running;
} WatchDog;

void WatchDog_new(WatchDog* wd, ubyte4 timeout);
void WatchDog_pet(WatchDog* wd);
void WatchDog_reset(WatchDog* wd);
bool WatchDog_check(WatchDog* wd);

#endif // _WATCH_DOG_H