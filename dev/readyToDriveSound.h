//"Include guard" - prevents this file from being #included more than once
#ifndef _READYTODRIVESOUND_H
#define _READYTODRIVESOUND_H

//struct _ReadyToDriveSound;
//typedef ReadyToDriveSound struct _ReadyToDriveSound;
typedef struct _ReadyToDriveSound ReadyToDriveSound;

ReadyToDriveSound* RTDS_new(void);

void RTDS_delete(ReadyToDriveSound* rtds);

void RTDS_setVolume(ReadyToDriveSound* rtds, float4 volumePercent, ubyte4 timeToPlay);

void RTDS_shutdownHelper(ReadyToDriveSound* rtds);

#endif // _READYTODRIVESOUND_H
