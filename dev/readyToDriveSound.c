#include <stdlib.h> //Needed for malloc

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "IO_PWM.h"

#include "readyToDriveSound.h"

struct _ReadyToDriveSound
{
    ubyte4 timeStamp_soundStarted; // from IO_RTC_StartTime(&)
    ubyte4 timeToSound;            // in microseconds: 1000 = 1ms, limit 4294967295 means 4294 sec max = about 71min max
    ubyte2 volumePercent;
};

ReadyToDriveSound *RTDS_new(void)
{
    ReadyToDriveSound *rtds = (ReadyToDriveSound *) malloc(sizeof(struct _ReadyToDriveSound));
    RTDS_setVolume(rtds, 0, 0);
    return rtds;
}

void RTDS_delete(ReadyToDriveSound *rtds)
{
    // some implementations pass a Person** to set the reference to 0
    // this implementation requires that the caller sets his own references to 0
    // free(person->name);
    free(rtds);
}

void RTDS_setVolume(ReadyToDriveSound *rtds, float4 volumePercent, ubyte4 timeToPlay)
{
    IO_PWM_SetDuty(IO_PWM_01, 65535 * volumePercent, NULL);
    IO_RTC_StartTime(&(rtds->timeStamp_soundStarted));
    rtds->timeToSound = timeToPlay;
}

void RTDS_shutdownHelper(ReadyToDriveSound *rtds)
{
    if (IO_RTC_GetTimeUS(rtds->timeStamp_soundStarted) > rtds->timeToSound)
    {
        RTDS_setVolume(rtds, 0, 0);
    }
}