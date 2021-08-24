#ifndef PMD_SPI
#define PMD_SPI

#include "NI\ni845x.h"

typedef struct tag_PMDSPI_IOData {
    HANDLE m_Handle;
    NiHandle ConfigurationHandle;
    NiHandle DeviceHandle;
    NiHandle ScriptHandle;

    uInt16 NumBitsPerSample;
    int32  ClockPhase;
    int32  ClockPhaseRead;
    int32  ClockPolarity;

    /* supported NI clock rates: (MC58113 supports up to 10MHz)

       25 kHz,   32 kHz,  40 kHz,  50 kHz,  80 kHz, 100 kHz, 125 kHz,
       160 kHz, 200 kHz, 250 kHz, 400 kHz, 500 kHz, 625 kHz, 800 kHz,
       1 MHz,  1.25 MHz, 2.5 MHz, 3.125 MHz, 4 MHz, 5 MHz, 6.25 MHz,
       10 MHz, 12.5 MHz, 20 MHz, 25 MHz, 33.33 MHz, 50 MHz
    */
    uInt16 ClockRate;

    uInt32 ChipSelect;

    BOOL m_bUseScript;
    long m_Timeout;
    PMDuint8 bVerbose;

    char     ResourceName[256];
} PMDSPI_IOData;

// functions that can be called externally
void PMDSPI_InitData(PMDSPI_IOData* transport_data);

PMDresult PMDSPI_Init(PMDAxisHandle* axis_handle, int device);

// call this function to initialize the interface
PMDresult PMDSetupAxisInterface_SPI(PMDAxisHandle* axis_handle, PMDAxis axis_number, int device);

// Write/read up to 4 words in a single SPICS pulse.
PMDresult PMDSPI_WriteWords(PMDAxisHandle* axis_handle,
                                PMDuint16 *WriteData, int nwords, PMDuint16 *ReadData);

// Set new verbosity, return old verbosity.
int PMDSPI_SetVerbose(void *transport_data, int level);

// Return old timeout.
long PMDSPI_SetTimeout(void *transport_data, long msec);
#endif  // def PMD_SPI
