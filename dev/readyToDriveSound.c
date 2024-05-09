#include <stdlib.h> //Needed for malloc

#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

#include "readyToDriveSound.h"
#include "sensors.h"

extern PWMOutput RTD_Sound;

ReadyToDriveSound *RTDS_new(ubyte2 volumePercent, ubyte4 timeToPlay) {
    ReadyToDriveSound *rtds = (ReadyToDriveSound *)malloc(sizeof(struct _ReadyToDriveSound));
    rtds->volumePercent = volumePercent;
    rtds->timeToSound = timeToPlay;
    return rtds;
}

void RTDS_play_sound(ReadyToDriveSound *rtds) {
    PWMOutput_set(&RTD_Sound, rtds->volumePercent);
    IO_RTC_StartTime(&(rtds->timeStamp_soundStarted));
}

void RTDS_shutdownHelper(ReadyToDriveSound *rtds) {
    if (IO_RTC_GetTimeUS(rtds->timeStamp_soundStarted) > rtds->timeToSound) {
        PWMOutput_set(&RTD_Sound, 0);
    }
}