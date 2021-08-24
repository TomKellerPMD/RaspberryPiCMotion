//  PMDNISPI.c -- SPI interface command/data transfer functions for National Instruments USB-8452.
//
//  Performance Motion Devices, Inc.
//

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

#include "PMDtypes.h"
#include "PMDdevice.h"
#include "NI\Ni845x.h"
#include "PMDNISPI.h"

#define Printf printf
#define STRING_SIZE 256

#define NI_RESULT(_code, _call)  _code = _call;  \
    if (_code) { \
        char StatusString[STRING_SIZE]; \
        ni845xStatusToString(_code, STRING_SIZE, StatusString); \
        Printf("Error: %s\n", StatusString); \
        Printf(" call: " #_call "\n");}


// ------------------------------------------------------------------------
PMDuint16 PMDSPI_GetStatus(void* transport_data)
{
    return 0;
}

// ------------------------------------------------------------------------
PMDuint16 PMDSPI_IsReady(void* transport_data)
{
    return 1;
}

// ------------------------------------------------------------------------
PMDuint16 PMDSPI_HasInterrupt(void* transport_data)
{
    return 0;
}

// ------------------------------------------------------------------------
PMDuint16 PMDSPI_HasError(void* transport_data)
{
    return 0;
}

// ------------------------------------------------------------------------
PMDresult PMDSPI_HardReset(void* transport_data)
{
    return PMD_ERR_InvalidOperation; // unsupported
}

PMDresult PMDSPI_Close(void* transport_data)
{
    PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData*)transport_data;
    int32 StatusCode;

    if (transport_data != NULL)
    {
        if (SPI_transport_data->m_Handle != INVALID_HANDLE_VALUE)
        {
             if (SPI_transport_data->m_bUseScript)  //SCRIPT_MODE
             {
                  NI_RESULT(StatusCode,
                            ni845xSpiScriptClose(SPI_transport_data->ScriptHandle) );
             }
             NI_RESULT(StatusCode,
                       ni845xSpiConfigurationClose( SPI_transport_data->ConfigurationHandle ));
/*
             NI_RESULT(StatusCode,
                       ni845xDeviceUnlock ( SPI_transport_data->DeviceHandle ));
*/
             NI_RESULT(StatusCode,
                       ni845xClose (SPI_transport_data->DeviceHandle));
        }
        SPI_transport_data->m_Handle = INVALID_HANDLE_VALUE;
    }
    free(transport_data);
    return PMD_ERR_OK;
}


// Set up default configuration for SPI port
void PMDSPI_InitData(PMDSPI_IOData* transport_data)
{
    transport_data->m_Handle = INVALID_HANDLE_VALUE;
    transport_data->m_bUseScript = FALSE;
    transport_data->m_Timeout = 500;
    transport_data->NumBitsPerSample = 16;
    transport_data->ClockPhase = kNi845xSpiClockPhaseSecondEdge;
    transport_data->ClockPhaseRead = transport_data->ClockPhase;
    transport_data->ClockPolarity = kNi845xSpiClockPolarityIdleLow;
    transport_data->ClockRate = 10000;
    transport_data->ChipSelect = 0;
    transport_data->ResourceName[0] = 0;
    transport_data->bVerbose = 0;
}

// Call into NI library to find and initialize the external device.
PMDuint16 PMDSPI_InitPort(PMDSPI_IOData* transport_data, int device)
{
    int32 StatusCode;
    NiHandle FindDeviceHandle;
    uInt32   NumberFound;
    int idev = 1;

    StatusCode = ni845xFindDevice ( transport_data->ResourceName, &FindDeviceHandle, &NumberFound );

    Printf("Found %d devices\n", NumberFound);
    Printf("  Device %d: \"%s\"\n", idev, transport_data->ResourceName);
    if (device > (int)NumberFound)
    {
         Printf("no such device %d\n", device);
         return PMD_ERR_OpeningPort;
    }
    while (idev < device) {
         StatusCode = ni845xFindDeviceNext (FindDeviceHandle, transport_data->ResourceName);
         Printf("  Device %d: \"%s\"\n", idev, transport_data->ResourceName);
         idev++;
    }

    if (StatusCode == 0)
    {
        StatusCode = ni845xOpen( transport_data->ResourceName, &transport_data->DeviceHandle );

        if (StatusCode == 0)
        {
            transport_data->m_Handle = (void*)transport_data->DeviceHandle;
        }

        // minimum is 1000ms, default is 30000
        NI_RESULT(StatusCode,
                  ni845xSetTimeout( transport_data->DeviceHandle, 1000));
        
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationOpen( &transport_data->ConfigurationHandle ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetChipSelect( transport_data->ConfigurationHandle,
                                                       transport_data->ChipSelect ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockRate( transport_data->ConfigurationHandle,
                                                      transport_data->ClockRate ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockPolarity( transport_data->ConfigurationHandle,
                                                          transport_data->ClockPolarity ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockPhase( transport_data->ConfigurationHandle,
                                                       transport_data->ClockPhase ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetNumBitsPerSample( transport_data->ConfigurationHandle,
                                                             transport_data->NumBitsPerSample ));
        /*
        NI_RESULT(StatusCode, ni845xSpiConfigurationSetPort( transport_data->ConfigurationHandle,
                                                             transport_data->PortNumber ));
        */

        if (transport_data->m_bUseScript)  //SCRIPT_MODE
        {
            NI_RESULT(StatusCode,
                      ni845xSpiScriptOpen(&transport_data->ScriptHandle) );
            NI_RESULT(StatusCode,
                      ni845xSpiScriptReset(transport_data->ScriptHandle) );
            NI_RESULT(StatusCode,
                      ni845xSpiScriptClockPolarityPhase (transport_data->ScriptHandle,
                                                         transport_data->ClockPolarity,
                                                         transport_data->ClockPhase) );
            NI_RESULT(StatusCode,
                      ni845xSpiScriptClockRate (transport_data->ScriptHandle,
                                                transport_data->ClockRate) );
            NI_RESULT(StatusCode,
                      ni845xSpiScriptNumBitsPerSample (transport_data->ScriptHandle,
                                                       transport_data->NumBitsPerSample) );
        }
    }

    if (StatusCode) 
    { 
        char StatusString[STRING_SIZE]; 
        ni845xStatusToString(StatusCode, STRING_SIZE, StatusString);
        Printf("Error: %s\n", StatusString);
        return PMD_ERR_OpeningPort;
    }
    return PMD_ERR_OK;
}

int GetHostSPIStatus(PMDSPI_IOData* transport_data)
{
    int32 StatusCode;
    uInt8 PortNumber = 0;
    uInt8 LineNumber = 0;
    int32 ReadData;

    NI_RESULT(StatusCode,
              ni845xDioReadLine (transport_data->DeviceHandle,
                                 PortNumber,
                                 LineNumber,
                                 &ReadData ));

    return ReadData;
};


int32 WaitUntilReady(PMDSPI_IOData* transport_data)
{
    unsigned long EndTime = GetTickCount() + transport_data->m_Timeout;

    do {
        if (GetHostSPIStatus(transport_data) == 0)
            return 0;
    } while (GetTickCount() < EndTime);

    return PMD_ERR_Timeout;
};


/*  
Sending a command
word    Host                        MC58113
------------------------------------------------------
        Assert ~HostSPIEnable   
1       8b axis 8b opcode           0
2       8b rsv  8b checksum         0
3       16b argument 1 (optional)   0
4       16b argument 2 (optional)   0
5       16b argument 3 (optional)   0
        De-assert ~HostSPIEnable    

        wait for HostSPIReady signal

Receiving a response
word    Host                        MC58113
------------------------------------------------------
        Assert ~HostSPIEnable   
1       0                           8b checksum  8b command status
2       0                           16b response 1 (optional)
3       0                           16b response 2 (optional)
4       0                           16b response 3 (optional)
        De-assert ~HostSPIEnable    
*/
PMDresult PMDSPI_Send(void* transport_data, 
                      PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat)
{
    PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData*)transport_data;

    int32 StatusCode;

    union buff 
    {
        WORD  w[8];
        BYTE  b[16];
    } txbuff, rxbuff;

    int  ProcessorError;
    char  nbytes;
    int c=0;
    int  i;
    WORD sum;
    WORD carry;
    uInt32 WriteSize;
    uInt8* WriteData = txbuff.b;
    uInt8  Nop[8];
    uInt32 ReadSize;
    uInt8* ReadData = rxbuff.b;

    if( SPI_transport_data->m_Handle == INVALID_HANDLE_VALUE ) 
         return PMD_ERR_NotConnected;

    memset(Nop, 0, 8);
    txbuff.b[c++] = (BYTE)(xDat[0]>>8);      // axis
    txbuff.b[c++] = (BYTE)(xDat[0] & 0xFF);  // opcode
    txbuff.b[c++] = 0;                       // reserved byte
    txbuff.b[c++] = 0;                       // checksum byte

    // byte swap command data
    for( i=1; i<xCt; i++ )
    {
        txbuff.b[c++] = (BYTE)(xDat[i] >> 8);
        txbuff.b[c++] = (BYTE)xDat[i];
    }
    nbytes = c;

    /* calculate checksum */
    sum = 0xAA; // seed
    for( i=0; i<nbytes; i++ ) 
        sum += txbuff.b[i];

    carry = sum >> 8;
    sum = sum & 0xFF;
    sum += carry;
    carry = sum >> 8;
    sum = sum & 0xFF;
    sum += carry;

    sum = ~sum;
    txbuff.b[3] = (PMDuint8)sum;        // checksum byte

//  NI_RESULT( ni845xDeviceLock( DeviceHandle ));
    // if the HostSPIStatus signal is already low, we're out of sync 
    // so do a read of any number of words to resync. 
    WriteSize = 2; 
    if (GetHostSPIStatus(SPI_transport_data) == 0)
    {
         if (SPI_transport_data->bVerbose > 0)
              Printf("SPI resync\n");
         NI_RESULT(StatusCode,
                   ni845xSpiWriteRead(SPI_transport_data->DeviceHandle,
                                      SPI_transport_data->ConfigurationHandle,
                                      WriteSize, Nop, &ReadSize, ReadData ));
    }

    if (SPI_transport_data->bVerbose > 0) {
         Printf("     send:");
         for (i = 0; 2*i < nbytes; i++)
             // Swap byte order for printing.
             Printf(" %02X%02X", txbuff.b[2*i], txbuff.b[2*i + 1]);
         Printf("\n");
    }

    // send command
    if (SPI_transport_data->m_bUseScript)  //SCRIPT_MODE is faster but cannot WaitUntilReady 
    {
        uInt32 ScriptReadIndexCommand;
        uInt32 ScriptReadIndexResponse;

        NI_RESULT(StatusCode,
                  ni845xSpiScriptReset(SPI_transport_data->ScriptHandle) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptClockPolarityPhase (SPI_transport_data->ScriptHandle,
                                                     SPI_transport_data->ClockPolarity,
                                                     SPI_transport_data->ClockPhase) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptClockRate (SPI_transport_data->ScriptHandle,
                                            SPI_transport_data->ClockRate) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptNumBitsPerSample (SPI_transport_data->ScriptHandle,
                                                   SPI_transport_data->NumBitsPerSample) );
        
        NI_RESULT(StatusCode,
                  ni845xSpiScriptEnableSPI(SPI_transport_data->ScriptHandle) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptCSLow(SPI_transport_data->ScriptHandle, 0) );
        WriteSize = nbytes;
        NI_RESULT(StatusCode,
                  ni845xSpiScriptWriteRead( SPI_transport_data->ScriptHandle,
                                            WriteSize,
                                            WriteData,
                                            &ScriptReadIndexCommand ));
        NI_RESULT(StatusCode,
                  ni845xSpiScriptCSHigh(SPI_transport_data->ScriptHandle, 0) );

        // script does not support waiting on a DIO so just delay the
        // max amount of time to process a command

        NI_RESULT(StatusCode,
                  ni845xSpiScriptUsDelay(SPI_transport_data->ScriptHandle,
                                         (uInt16)SPI_transport_data->m_Timeout) );

        WriteSize = (rCt + 1) * 2; // +1 to include checksum word
        NI_RESULT(StatusCode,
                  ni845xSpiScriptCSLow(SPI_transport_data->ScriptHandle, 0) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptWriteRead(SPI_transport_data->ScriptHandle,
                                           WriteSize,
                                           WriteData,
                                           &ScriptReadIndexResponse ));
        NI_RESULT(StatusCode,
                  ni845xSpiScriptCSHigh(SPI_transport_data->ScriptHandle, 0) );
//      NI_RESULT( ni845xSpiScriptDisableSPI(SPI_transport_data->ScriptHandle) );

        NI_RESULT(StatusCode,
                  ni845xSpiScriptRun(SPI_transport_data->ScriptHandle,
                                     SPI_transport_data->DeviceHandle,
                                     0) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptExtractReadDataSize(SPI_transport_data->ScriptHandle,
                                                     ScriptReadIndexResponse,
                                                     &ReadSize) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptExtractReadData(SPI_transport_data->ScriptHandle,
                                                 ScriptReadIndexResponse,
                                                 ReadData) );
    }
    else
    {
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockRate(SPI_transport_data->ConfigurationHandle,
                                                     SPI_transport_data->ClockRate ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockPolarity(SPI_transport_data->ConfigurationHandle,
                                                         SPI_transport_data->ClockPolarity ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockPhase(SPI_transport_data->ConfigurationHandle,
                                                      SPI_transport_data->ClockPhase ));
        WriteSize = nbytes;

        // Data provided in pWriteData and retrieved using pReadData is organized in big endian format.
        NI_RESULT(StatusCode,
                  ni845xSpiWriteRead(SPI_transport_data->DeviceHandle, 
                                     SPI_transport_data->ConfigurationHandle,
                                     WriteSize,
                                     WriteData,
                                     &ReadSize,
                                     ReadData ));
        WaitUntilReady(SPI_transport_data);

        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockPhase(SPI_transport_data->ConfigurationHandle,
                                                      SPI_transport_data->ClockPhaseRead ));
        WriteSize = (rCt + 1) * 2; // +1 to include checksum word
        NI_RESULT(StatusCode,
                  ni845xSpiWriteRead(SPI_transport_data->DeviceHandle,
                                     SPI_transport_data->ConfigurationHandle,
                                     WriteSize,
                                     Nop,
                                     &ReadSize,
                                     ReadData ));
    }

//  NI_RESULT( ni845xDeviceUnlock( DeviceHandle ));

    // byte swap return data
    for( i=0; i<(int)ReadSize; i+=2 )
    {
        BYTE temp = rxbuff.b[i];
        rxbuff.b[i] = rxbuff.b[i+1];
        rxbuff.b[i+1] = temp;
    }

    if (SPI_transport_data->bVerbose > 0)
    {
        Printf("  receive:");
        for (i = 0; 2*i < (int)ReadSize; i++)
            Printf(" %04X", rxbuff.w[i]);
        Printf("\n");
    }

    // first byte in response is the motion processor command status 
    // a positive command status is # result words that are available 
    ProcessorError = 0;
    if ((int8)(rxbuff.b[0]) < 0)
    {
        ProcessorError = -(int8)(rxbuff.b[0]);
    }

    sum = 0xAA;
    for( i=0; i<(int)ReadSize; i++ )
        sum += rxbuff.b[i];
    carry = sum >> 8;
    sum = sum & 0xFF;
    sum += carry;
    carry = sum >> 8;
    sum = sum & 0xFF;
    sum += carry;

    if( sum != 0xFF)
    {
        Printf("\n**Checksum error: (opcode 0x%02X)", xDat[0]>>8 );
        for(i=0; i<(int)ReadSize/2; i++ )
            Printf("  %04X", rxbuff.w[i] );
        Printf(" S: %02X **\n", sum );

        return PMD_ERR_ChecksumError;
    }

    // copy return data skipping checksum word
    for( i=0; i<rCt; i++ )
    {
        rDat[i] = rxbuff.w[i+1];
    }
    // report error if one occured
    if( ProcessorError )
        return (PMDErrorCode)ProcessorError;

    return PMD_ERR_OK;
}


PMDresult PMDSPI_WriteWords(PMDAxisHandle* axis_handle,
                            PMDuint16 *WriteData, int nwords, PMDuint16 *ReadData)
{
    PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData*)(axis_handle->transport_data);
    int32 StatusCode;
    uInt32 WriteSize = 2*nwords;
    uInt32 ReadSize;
    uInt8 outbuf[10], inbuf[10];
    int i;

    // The MC58113 will not accept more than 4 words at a time.
    if (nwords > sizeof(outbuf)/sizeof(uInt16))
        return PMD_ERR_CommandError;

    if (SPI_transport_data->bVerbose > 0) {
        Printf("     send:");
        for (i = 0; i < nwords; i++)
            Printf(" %04X", WriteData[i]);
        Printf("\n");
    }

    // Byte swap write data
    for (i = 0; i < nwords; i++)
    {
        outbuf[2*i] = (uInt8)(WriteData[i] >> 8);
        outbuf[2*i + 1] = (uInt8) (WriteData[i] & 0xFF);
    }

    // send words
    if (SPI_transport_data->m_bUseScript)  //SCRIPT_MODE is faster but cannot WaitUntilReady 
    {
        uInt32 ScriptRead;

        NI_RESULT(StatusCode,
                  ni845xSpiScriptReset(SPI_transport_data->ScriptHandle) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptClockPolarityPhase (SPI_transport_data->ScriptHandle,
                                                     SPI_transport_data->ClockPolarity,
                                                     SPI_transport_data->ClockPhase) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptClockRate (SPI_transport_data->ScriptHandle,
                                            SPI_transport_data->ClockRate) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptNumBitsPerSample (SPI_transport_data->ScriptHandle,
                                                   SPI_transport_data->NumBitsPerSample) );
        
        NI_RESULT(StatusCode,
                  ni845xSpiScriptEnableSPI(SPI_transport_data->ScriptHandle) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptCSLow(SPI_transport_data->ScriptHandle, 0) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptWriteRead( SPI_transport_data->ScriptHandle,
                                            WriteSize,
                                            outbuf,
                                            &ScriptRead ));
        NI_RESULT(StatusCode,
                  ni845xSpiScriptCSHigh(SPI_transport_data->ScriptHandle, 0) );

        // script does not support waiting on a DIO so just delay the
        // max amount of time to process a command

        NI_RESULT(StatusCode,
                  ni845xSpiScriptUsDelay(SPI_transport_data->ScriptHandle,
                                         (uInt16)SPI_transport_data->m_Timeout) );

        NI_RESULT(StatusCode,
                  ni845xSpiScriptCSLow(SPI_transport_data->ScriptHandle, 0) );


        NI_RESULT(StatusCode,
                  ni845xSpiScriptRun(SPI_transport_data->ScriptHandle,
                                     SPI_transport_data->DeviceHandle,
                                     0) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptExtractReadDataSize(SPI_transport_data->ScriptHandle,
                                                     ScriptRead,
                                                     &ReadSize) );
        NI_RESULT(StatusCode,
                  ni845xSpiScriptExtractReadData(SPI_transport_data->ScriptHandle,
                                                 ScriptRead,
                                                 inbuf) );
    }
    else
    {
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockRate(SPI_transport_data->ConfigurationHandle,
                                                     SPI_transport_data->ClockRate ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockPolarity(SPI_transport_data->ConfigurationHandle,
                                                         SPI_transport_data->ClockPolarity ));
        NI_RESULT(StatusCode,
                  ni845xSpiConfigurationSetClockPhase(SPI_transport_data->ConfigurationHandle,
                                                      SPI_transport_data->ClockPhase ));
        // Data provided in pWriteData and retrieved using pReadData is organized in big endian format.
        NI_RESULT(StatusCode,
                  ni845xSpiWriteRead(SPI_transport_data->DeviceHandle, 
                                     SPI_transport_data->ConfigurationHandle,
                                     WriteSize,
                                     outbuf,
                                     &ReadSize,
                                     inbuf ));
    }

    if (0==StatusCode && ReadData)
    {
        // Byte swap read data
        for (i = 0; i < nwords; i++)
        {
            ReadData[i] = ((uInt16)(inbuf[2*i]) << 8) + (uInt16)inbuf[2*i + 1];
        }
        if (SPI_transport_data->bVerbose > 0) {
            Printf("  receive:");
            for (i = 0; i < nwords; i++)
                Printf(" %04X", ReadData[i]);
            Printf("\n");
        }
    }
    return StatusCode;
}


PMDresult PMDSPI_Init(PMDAxisHandle* axis_handle, int device)
{
    // setup function pointers
    axis_handle->transport.SendCommand = PMDSPI_Send;

    axis_handle->transport.GetStatus = PMDSPI_GetStatus;
    axis_handle->transport.IsReady = PMDSPI_IsReady;
    axis_handle->transport.HasInterrupt = PMDSPI_HasInterrupt;
    axis_handle->transport.HasError = PMDSPI_HasError;
    axis_handle->transport.HardReset = PMDSPI_HardReset;

    axis_handle->transport.bHasDPRAM = FALSE;
    axis_handle->transport.ReadDPRAM = NULL;
    axis_handle->transport.WriteDPRAM = NULL;

    axis_handle->transport.Close = PMDSPI_Close;

    return PMDSPI_InitPort(axis_handle->transport_data, device);
}

/*
  Searches down the list of available devices, device=1 gives the first,
  device=2 the second, and so forth.
*/

PMDresult PMDSetupAxisInterface_SPI(PMDAxisHandle* axis_handle,
                                    PMDAxis axis_number,
                                    int device)
{
    PMDSPI_IOData* transport_data;

    transport_data = (PMDSPI_IOData*) malloc( sizeof( PMDSPI_IOData ) );
    memset(transport_data, 0, sizeof(PMDSPI_IOData));

    // set the axis we are talking to with this handle
    axis_handle->axis = axis_number;

    // set the interface type 
    axis_handle->InterfaceType = InterfaceSPI;

    // the transport data is initialized first to setup the defaults
    PMDSPI_InitData(transport_data);

    axis_handle->transport_data = (void*) transport_data;

    // initialize the transport (inits function pointers)
    return PMDSPI_Init(axis_handle, device);
}

int PMDSPI_SetVerbose(void* transport_data, int verbose)
{
     int ret;

     PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData *)transport_data;
     ret = SPI_transport_data->bVerbose;
     SPI_transport_data->bVerbose = verbose;
     return ret;
}

long PMDSPI_SetTimeout(void *transport_data, long msec)
{
     long ret;

     PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData *)transport_data;
     ret = SPI_transport_data->m_Timeout;
     SPI_transport_data->m_Timeout = msec;
     return ret;
}
