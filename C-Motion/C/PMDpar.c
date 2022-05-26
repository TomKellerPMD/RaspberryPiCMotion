//
//  PMDpar.c -- parallel interface command/data transfer functions 
//              This interface module uses direct port access which will 
//              only work on Linux, old Windows OSs and DOS.
//              
//  Performance Motion Devices, Inc.
//

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/io.h>

// undefine __TIME_WAITFORREADY if it is not desired to have a timeout while waiting for the ready signal
#define __TIME_WAITFORREADY
#ifdef __TIME_WAITFORREADY
#include <sys/timeb.h>
#endif

#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDdevice.h"
#include "PMDtrans.h"
#include "PMDsys.h"
#include "PMDpar.h"

// only include this if we are running in diagnostics mode
#include "PMDdiag.h"

extern unsigned GetTickCount();

enum {PMDParallelIOMode_16_16=0, PMDParallelIOMode_8_16, PMDParallelIOMode_8_8};

typedef struct tagPMDParallelIOTransportData {

	PMDuint16 dataPort;
	PMDuint16 commandPort;
	PMDuint16 statusPort;

	PMDuint16 readyMask;
	PMDuint16 readyValue;
	PMDuint16 hostInterruptMask;
	PMDuint16 hostInterruptValue;
	PMDuint16 commandStatusMask;
	PMDuint16 commandStatusValue;

	PMDuint16 bVerifyChecksum;
	PMDuint16 bDiagnostics;

	void (*OutPData)(PMDuint16 port, PMDuint16 dataword);
	void (*OutPCmd)(PMDuint16 port, PMDuint16 dataword);
	PMDuint16 (*InPData)(PMDuint16 port);
	PMDuint16 (*InPStatus)(PMDuint16 port);

} PMDParallelIOTransportData;


#define PMDBASE  0x300

// forward declarations
PMDresult PMDParallel_ReceiveResponse(PMDParallelIOTransportData* transport_data, PMDuint16 length, PMDuint16 buffer[]);

void OutP16Bit(PMDuint16 port, PMDuint16 dataword)
{
        outw(port, dataword);
}

PMDuint16 InP16Bit(PMDuint16 port)
{
        return inw(port);
}

void OutP8Bit(PMDuint16 port, PMDuint16 dataword)
{
    outb(port, (PMDuint8)(dataword>>8)&0xFF);
    outb(port, (PMDuint8)dataword&0xFF);
}

PMDuint16 InP8Bit(PMDuint16 port)
{
    return (PMDuint16) (((inb(port) & 0xFF) << 8) | (inb(port) & 0xFF));
}

void OutP8BitCmd(PMDuint16 port, PMDuint16 dataword)
{
    outb(port, (PMDuint8)dataword&0xFF);
}

PMDuint16 InP8BitStatus(PMDuint16 port)
{
    return (PMDuint16)(inb(port) & 0xFF);
}

PMDuint16 PMDParallel_GetStatus(void* transport_data)
{
    PMDParallelIOTransportData* PIOtransport_data = (PMDParallelIOTransportData*)transport_data;

    return PIOtransport_data->InPStatus(PIOtransport_data->statusPort);
}

PMDuint16 PMDParallel_IsReady(void* transport_data)
{
    PMDParallelIOTransportData* PIOtransport_data = (PMDParallelIOTransportData*)transport_data;

    return ((PIOtransport_data->InPStatus(PIOtransport_data->statusPort) 
        & PIOtransport_data->readyMask) == PIOtransport_data->readyValue);
}

PMDuint16 PMDParallel_HasInterrupt(void* transport_data)
{
    PMDParallelIOTransportData* PIOtransport_data = (PMDParallelIOTransportData*)transport_data;

    return ((PIOtransport_data->InPStatus(PIOtransport_data->statusPort) 
        & PIOtransport_data->hostInterruptMask) == PIOtransport_data->hostInterruptValue);
}

PMDuint16 PMDParallel_HasError(void* transport_data)
{
    PMDParallelIOTransportData* PIOtransport_data = (PMDParallelIOTransportData*)transport_data;

    return ((PIOtransport_data->InPStatus(PIOtransport_data->statusPort) 
        & PIOtransport_data->commandStatusMask) == PIOtransport_data->commandStatusValue);
}

PMDresult PMDParallel_HardReset(void* transport_data)
{
    PMDParallelIOTransportData* PIOtransport_data = (PMDParallelIOTransportData*)transport_data;

    // reset port for the Developer's Kit Board is base address + 8
    PIOtransport_data->OutPCmd((PMDuint16)(PIOtransport_data->dataPort + 8), 0);
    // reset port for the Motion Board is base address + 6
    PIOtransport_data->OutPCmd((PMDuint16)(PIOtransport_data->dataPort + 6), 0);

    return PMD_NOERROR;
}

// wait for the DB2000 to be ready for the next command
static PMDresult PMDParallel_WaitForReady(PMDParallelIOTransportData* transport_data)
{
    PMDuint16 in_val;
    int i;
#ifdef __TIME_WAITFORREADY
    PMDuint32 stopTime, currentTime;

    // For faster turnaround, check ready bit in tight loop before using timeout code
    // because getting time can be a lengthy function call.
    // Each InPStatus call takes 1 to 1.5 µs (based on ISA freq) and a command may 
    // take up to 150 µs for the PMD chip to process (based on servo loop time)
    // so check ready bit 150 times before dropping into timeout section.
    for(i=0;i<150;i++)
    {
        // poll ready port, if not ready, loop
        in_val = transport_data->InPStatus(transport_data->statusPort);

        if ((in_val != 0xFFFF) 
            && ((in_val & transport_data->readyMask) == transport_data->readyValue))
                return PMD_ERR_OK;
    }
    stopTime = GetCurrentMilliseconds() + 1000; // timeout is 1 sec

    for(;;)
    {

        // poll ready port, if not ready, loop
        in_val = transport_data->InPStatus(transport_data->statusPort);

        if ((in_val != 0xFFFF) 
            && ((in_val & transport_data->readyMask) == transport_data->readyValue))
                return PMD_ERR_OK;

        currentTime = GetCurrentMilliseconds();

        if (currentTime > stopTime)
        {
            // recheck the busy flag on last time
            in_val = transport_data->InPStatus(transport_data->statusPort);

            if ((in_val != 0xFFFF) 
                && ((in_val & transport_data->readyMask) == transport_data->readyValue))
                return PMD_ERR_OK;
            return PMD_ERR_CommTimeoutError;
        }
    }
#else
    // wait indefinitely
    for(;;)
    {
        // poll ready port, if not ready, loop
        in_val = transport_data->InPStatus(transport_data->statusPort);

        if ((in_val != 0xFFFF) 
            && ((in_val & transport_data->readyMask) == transport_data->readyValue))
                return PMD_ERR_OK;
    }
#endif
}

// wait for the device to be ready for the next command
static PMDresult PMDParallel_GetCommandStatus(PMDParallelIOTransportData* transport_data)
{
    PMDuint16 result;
    int     in_val;

    if((result = PMDParallel_WaitForReady(transport_data)) != PMD_ERR_OK)
            return result;

    in_val = transport_data->InPStatus(transport_data->statusPort);
    if ((in_val & transport_data->commandStatusMask) == transport_data->commandStatusValue)
    {
        if (transport_data->bDiagnostics)
            PMDprintf("C-Motion: Command Error bit set.\n");
        return PMD_ERR_CommandError;
    }

    return PMD_ERR_OK;
}

// send the command and data to the device
PMDresult PMDParallel_Send(PMDParallelIOTransportData* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat)
{
    PMDuint16 result;
    PMDuint16 commandstatus = PMD_ERR_OK;
    PMDuint16 index;
    long messageChecksum=0;
    PMDuint16 chipsetChecksum=0;
    result = PMD_ERR_InterfaceNotInitialized;
    if((result = PMDParallel_WaitForReady(transport_data)) == PMD_ERR_OK)
    {
            // put the command into the device
            transport_data->OutPCmd(transport_data->commandPort, xDat[0]);

            // put the data into the device
            // one word at a time
            for(index=1; index<xCt; index++)
            {
                    if((result = PMDParallel_WaitForReady(transport_data)) != PMD_ERR_OK)
                            break;
                    transport_data->OutPData(transport_data->dataPort,xDat[index]);
            }
            // get the status
            if(result == PMD_ERR_OK)
                    commandstatus = PMDParallel_GetCommandStatus(transport_data);
    }

    // get the data from the device
    for(index=0; index<rCt; index++)
    {
            if((result = PMDParallel_WaitForReady(transport_data)) != PMD_ERR_OK)
                    break;
            rDat[index] = transport_data->InPData(transport_data->dataPort);
    }

    if (transport_data->bVerifyChecksum)
    {
        for(index=0; index<xCt; index++)
                messageChecksum += xDat[index];
        for(index=0; index<rCt; index++)
                messageChecksum += rDat[index];
        messageChecksum = messageChecksum & 0xFFFF;

        result = PMDParallel_ReceiveResponse(transport_data, 1, &chipsetChecksum);

        if ( result == PMD_ERR_OK && messageChecksum != chipsetChecksum )
        {
            if (transport_data->bDiagnostics)
                PMDprintf("Checksum failure.  expected: %04x,  got: %04x\n",messageChecksum,chipsetChecksum);
            return PMD_ERR_ChecksumError;
        }
    }

    if (commandstatus!=PMD_ERR_OK)
    {
        if (transport_data->bDiagnostics)
        {
            PMDprintf("C-Motion: %s ",PMDGetOpcodeText(xDat[0]));
            for(index=0; index<xCt; index++)
                PMDprintf("%X ",xDat[index]);
            PMDprintf("\n");
        }
        return commandstatus;
    }

    return result;
}

// send the command to and get data from the device
PMDresult PMDParallel_ReceiveResponse(PMDParallelIOTransportData* transport_data, PMDuint16 length, PMDuint16 buffer[])
{
        PMDresult result;
        PMDuint16 index;

        result = PMD_ERR_InterfaceNotInitialized;

        // get the data from the device
        for(index=0; index<length; index++)
        {
                if((result = PMDParallel_WaitForReady(transport_data)) != PMD_ERR_OK)
                        break;
                buffer[index] = transport_data->InPData(transport_data->dataPort);
        }

        return result;
}

PMDresult PMDParallel_Close(void* transport_data)
{
    free(transport_data);

    return PMD_ERR_OK;
}

/******************************************************************************

    Function Name: ReadDPRAM()

    Reads from the dual port RAM instead of through the CP chip.
    Note: For use with the Prodigy PC/104 card only.

    DPRAM base address offsets are:

    0xA - Address Register
    0xC - Data Register
    0xE - Config Register

    Config Register Bits
    Bit 0: Burst mode enable.    0=enabled,   1=disabled  (default 0)
    Bit 1: Burst mode direction. 0=increment, 1=decrement (default 0)

 ******************************************************************************/
#define DPRAM_REG_ADDRESS   0x0A
#define DPRAM_REG_DATA      0x0C
#define DPRAM_REG_CONFIG    0x0E

PMDresult PMDParallel_ReadDPRAM(void* transport_data, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read)
{
    PMDParallelIOTransportData* PIOtransport_data = (PMDParallelIOTransportData*)transport_data;

    PMDuint32 i;
    PMDuint16 address;
    PMDuint16 words_to_read;
    PMDuint16 offset_in_words;
    PMDuint16 word_read_lo;
    PMDuint16 word_read_hi;

    // Prodigy PC/104 does not support more than 64 kBytes of RAM
    if (offset_in_dwords > 0x3FFF || dwords_to_read > 0x4000)
        return PMD_ERR_DPRAM; 

    words_to_read = (PMDuint16)dwords_to_read * 2;
    offset_in_words = (PMDuint16)offset_in_dwords * 2;

    address = PIOtransport_data->dataPort + DPRAM_REG_ADDRESS;
    PIOtransport_data->OutPData(address, offset_in_words);

    // read the data
    address = PIOtransport_data->dataPort + DPRAM_REG_DATA;
    for (i=0; i<dwords_to_read; i++)
    {
        word_read_hi = PIOtransport_data->InPData(address);
        word_read_lo = PIOtransport_data->InPData(address);
        data[i] = word_read_hi << 16 | word_read_lo;
    }

    return PMD_ERR_OK;
}

/******************************************************************************

    Function Name: WriteDPRAM()

    Writes to the dual port RAM instead of through the CP chip.
    Returns 0 if sucessful ERR_DPRAM otherwise.

 ******************************************************************************/
PMDresult PMDParallel_WriteDPRAM(void* transport_data, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write)
{
    PMDParallelIOTransportData* PIOtransport_data = (PMDParallelIOTransportData*)transport_data;
    PMDuint32 i;
    PMDuint16 address;
    PMDuint16 offset_in_words;
    PMDuint16 word_write_lo;
    PMDuint16 word_write_hi;

    if (offset_in_dwords > 0x3FFF || dwords_to_write > 0x4000)
        return PMD_ERR_DPRAM; // DPRAM boundary exceeded

    offset_in_words = (PMDuint16)offset_in_dwords * 2;

    address = PIOtransport_data->dataPort + DPRAM_REG_ADDRESS;
    PIOtransport_data->OutPData(address, offset_in_words);

    // write the data
    address = PIOtransport_data->dataPort + DPRAM_REG_DATA;
    for (i=0; i<dwords_to_write; i++)
    {
        word_write_hi = (PMDuint16)((data[i] >> 16) & 0x0000FFFF);
        word_write_lo = (PMDuint16)(data[i] & 0x0000FFFF);

        PIOtransport_data->OutPData(address, word_write_hi);
        PIOtransport_data->OutPData(address, word_write_lo);
    }

    return PMD_ERR_OK;
}

void PMDParallel_InitData(PMDParallelIOTransportData* transport_data, int IOMode)
{
    // assign default values
    if (transport_data->dataPort == 0)
        transport_data->dataPort = PMDBASE;
    transport_data->commandPort = transport_data->dataPort + 2;
    transport_data->statusPort = transport_data->dataPort + 2;

    transport_data->readyMask = 0x8000;
    transport_data->readyValue = 0x8000;
    transport_data->hostInterruptMask = 0x4000;
    transport_data->hostInterruptValue = 0x4000;
    transport_data->commandStatusMask = 0x2000;
    transport_data->commandStatusValue = 0x2000;

    // by default always verify the checksum
    transport_data->bVerifyChecksum = 1;
    // by default disable diagnostics
    transport_data->bDiagnostics = 1;

    // assign default handlers/masks according to IO mode
    switch (IOMode)
    {
    case PMDParallelIOMode_16_16:
        transport_data->OutPData = OutP16Bit;
        transport_data->OutPCmd = OutP16Bit;
        transport_data->InPData = InP16Bit;
        transport_data->InPStatus = InP16Bit;
        break;
    case PMDParallelIOMode_8_16:
        transport_data->OutPData = OutP8Bit;
        transport_data->OutPCmd = OutP8Bit;
        transport_data->InPData = InP8Bit;
        transport_data->InPStatus = InP8Bit;
        break;
    case PMDParallelIOMode_8_8:
        transport_data->OutPData = OutP8Bit;
        transport_data->OutPCmd = OutP8BitCmd;
        transport_data->InPData = InP8Bit;
        transport_data->InPStatus = InP8BitStatus;
        transport_data->readyMask = 0x80;
        transport_data->readyValue = 0x80;
        transport_data->hostInterruptMask = 0x40;
        transport_data->hostInterruptValue = 0x40;
        transport_data->commandStatusMask = 0x20;
        transport_data->commandStatusValue = 0x20;
        break;
    }
}

void PMDParallel_Init(PMDAxisHandle* axis_handle)
{
    // setup function pointers
    axis_handle->transport.SendCommand = (PMDresult (*)(void *, PMDuint8,  PMDuint16 *, PMDuint8,  PMDuint16 *)) PMDParallel_Send;
    axis_handle->transport.GetStatus = PMDParallel_GetStatus;
    axis_handle->transport.IsReady = PMDParallel_IsReady;
    axis_handle->transport.HasInterrupt = PMDParallel_HasInterrupt;
    axis_handle->transport.HasError = PMDParallel_HasError;
    axis_handle->transport.HardReset = PMDParallel_HardReset;
    axis_handle->transport.Close = PMDParallel_Close;

    axis_handle->transport.bHasDPRAM = TRUE;
    axis_handle->transport.ReadDPRAM = PMDParallel_ReadDPRAM;
    axis_handle->transport.WriteDPRAM = PMDParallel_WriteDPRAM;
}

/*****************************************************************************
  Set board_address to 0 to use the default base address (0x300) 
  otherwise set it to the base address of the ISA board
******************************************************************************/
PMDresult PMDSetupAxisInterface_Parallel(PMDAxisHandle* axis_handle, PMDAxis axis_number, PMDuint16 board_address)
{
    PMDParallelIOTransportData* transport_data;

    transport_data = (PMDParallelIOTransportData*) malloc( sizeof( PMDParallelIOTransportData ) );
    memset(transport_data, 0, sizeof(PMDParallelIOTransportData));

    // set the axis we are talking to with this handle
    axis_handle->axis = axis_number;

    // set the interface type 
    axis_handle->InterfaceType = InterfaceParallel;

    // if required the transport data defaults can be changed here
    // e.g. board IO address (dataPort,commandPort,statusPort)
    transport_data->dataPort = board_address;

    // the transport data is initialized first to setup the defaults
    // make sure the IO mode is set correctly
    PMDParallel_InitData(transport_data, PMDParallelIOMode_16_16);

    axis_handle->transport_data = (void*) transport_data;

    // initialize the transport (inits function pointers)
    PMDParallel_Init(axis_handle);

    return PMD_ERR_OK;
}

