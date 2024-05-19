#include <stdlib.h> //Needed for malloc

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

#include "readyToDriveSound.h"
#include "sensors.h"

// extern PWMOutput RTD_Sound;
extern DigitalOutput RTD_Sound;

void RTDS_new(ReadyToDriveSound *me, ubyte2 volumePercent, ubyte4 timeToPlay)
{
    me->volumePercent = volumePercent;
    me->timeToSound = timeToPlay;
}

void RTDS_play_sound(ReadyToDriveSound *rtds)
{
    // PWMOutput_set(&RTD_Sound, rtds->volumePercent);
    DigitalOutput_set(&RTD_Sound, TRUE);
    IO_RTC_StartTime(&(rtds->timeStamp_soundStarted));
}

void RTDS_shutdownHelper(ReadyToDriveSound *rtds)
{
    if (IO_RTC_GetTimeUS(rtds->timeStamp_soundStarted) > rtds->timeToSound)
    {
        // PWMOutput_set(&RTD_Sound, 0);
        DigitalOutput_set(&RTD_Sound, FALSE);
    }
}