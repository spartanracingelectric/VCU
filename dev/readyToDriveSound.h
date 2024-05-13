//"Include guard" - prevents this file from being #included more than once
#ifndef _READYTODRIVESOUND_H
#define _READYTODRIVESOUND_H

typedef struct _ReadyToDriveSound {
    ubyte4 timeStamp_soundStarted; //from IO_RTC_StartTime(&)
    ubyte4 timeToSound;            //in microseconds: 1000 = 1ms, limit 4294967295 means 4294 sec max = about 71min max
    ubyte2 volumePercent;
} ReadyToDriveSound;

void RTDS_new(ReadyToDriveSound *rtds, ubyte2 volumePercent, ubyte4 timeToPlay);

void RTDS_play_sound(ReadyToDriveSound *rtds);

void RTDS_shutdownHelper(ReadyToDriveSound* rtds);

#endif // _READYTODRIVESOUND_H
