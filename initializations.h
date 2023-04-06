//----------------------------------------------------------------------------
// VCU Subsystem Initializations
//----------------------------------------------------------------------------
// This is where we turn on the VCU's internal power supplies and sensors.
//
// The supplies/sensors and their parameters are defined in IO_ADC.h.
// Each sensor/ADC channel/etc has different parameters, so make sure to
// check the .h files, the examples, and the documentation!
//----------------------------------------------------------------------------

#ifndef _INITIALIZEVCU_H
#define _INITIALIZEVCU_H

#include "IO_Driver.h"
#include "APDB.h"

//Application Database, needed for TTC-Downloader
//APDB appl_db;

//Functions
void vcu_initializeVCU(void);
void vcu_initializeADC(bool benchMode);
void vcu_initializeCAN(void);
void vcu_initializeMCU(void);
void vcu_ADCWasteLoop(void);
#endif //  _INITIALIZEVCU_H
