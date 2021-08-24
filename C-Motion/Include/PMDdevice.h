#ifndef PMD_Device
#define PMD_Device

//
//  Performance Motion Devices, Inc.
//

#include "PMDtypes.h"

typedef enum {
    InterfaceParallel,
    InterfaceISA,
    InterfacePCI,
    InterfaceSerial,
    InterfaceCAN,
    InterfaceSPI,
} PMDInterfaceType;

typedef struct tagPMDIOTransport {

// these have not yet been defined in any interface classes
//  void (*DisableInterrupt)(void*);
//  void (*EnableInterrupt)(void*);

    PMDresult (*SendCommand)(void*, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat);
    PMDresult (*Close)(void*);

    PMDuint16 (*GetStatus)(void*);
    PMDuint16 (*IsReady)(void*);
    PMDuint16 (*HasInterrupt)(void*);
    PMDuint16 (*HasError)(void*);
    PMDresult (*HardReset)(void*);

    int bHasDPRAM;
    PMDresult (*ReadDPRAM)(void* transport_data, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read);
    PMDresult (*WriteDPRAM)(void* transport_data, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write);

} PMDIOTransport;

typedef struct tagPMDAxisHandle {
    PMDAxis axis;
    PMDIOTransport transport;
    void* transport_data;
    PMDresult result;
    PMDInterfaceType InterfaceType;
} PMDAxisHandle;

typedef PMDAxisHandle* PMDAxisInterface;

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

// raw parallel IO using in() and out() functions
#ifdef PMD_PARALLEL_INTERFACE
#include "PMDpar.h"
#endif

// parallel ISA IO using WDM driver (PMDMCG2.sys)
#ifdef PMD_ISA_INTERFACE
#include "PMDdrv.h"
#endif

// PCI IO using PLX drivers (PLXAPI.dll pci9030.sys) via PMDpci.lib
#ifdef PMD_PCI_INTERFACE
#include "PMDpci.h"
#endif

// RS232 serial IO using windows comm driver
#ifdef PMD_W32SERIAL_INTERFACE
#include "PMDW32Ser.h"
#endif

// CAN IO using IXXAT drivers
#ifdef PMD_CAN_INTERFACE
#include "PMDCAN.h"
#endif

// SPI IO using NI drivers
#ifdef PMD_SPI_INTERFACE
#include "PMDNISPI.h"
#endif

PMDCFunc PMDCloseAxisInterface        (PMDAxisInterface axis_intf);
PMDCFunc PMDReadDPRAM                 (PMDAxisInterface axis_intf, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read);
PMDCFunc PMDWriteDPRAM                (PMDAxisInterface axis_intf, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write);
PMDCFunc PMDHardReset                 (PMDAxisInterface axis_intf);
PMDCFunc PMDAtlasAxisOpen             (PMDAxisInterface hSourceAxis, PMDAxisInterface hAtlasAxis);

#endif
