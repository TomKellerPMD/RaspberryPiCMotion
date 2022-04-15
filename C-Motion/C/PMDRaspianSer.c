//  PMDRaspianSer.c -- Rasperry Pi Linux serial interface command/data transfer functions
//  Replaces PMDW32Ser.c in standard C-Motion libary
//  Performance Motion Devices, Inc.
//  TLK 10/15/2021
//


#include <stdio.h>
#include <stdlib.h>
#include <termios.h>		//Used for UART
#include <sys/ioctl.h>
#include <errno.h>

#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDdevice.h"
#include "PMDtrans.h"
#include "PMDsys.h"
#include "PMDW32Ser.h"


// only need to include this if diagnostics mode is used
#include "PMDdiag.h"
#define error_message printf

// string input from command line
char LinuxCommPort[13] = "";

// ------------------------------------------------------------------------
PMDuint16 PMDSerial_GetStatus(void* transport_data)
{
    return 0;
}

// ------------------------------------------------------------------------
PMDuint16 PMDSerial_IsReady(void* transport_data)
{
    return 1;
}

// ------------------------------------------------------------------------
PMDuint16 PMDSerial_HasInterrupt(void* transport_data)
{
    return 0;
}

// ------------------------------------------------------------------------
PMDuint16 PMDSerial_HasError(void* transport_data)
{
    return 0;
}

// ------------------------------------------------------------------------
PMDresult PMDSerial_HardReset(void* transport_data)
{
    return PMD_ERR_InvalidOperation; // unsupported
}


// ------------------------------------------------------------------------
PMDresult PMDSerial_Close(void* transport_data)
{
    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;

    if (transport_data != NULL)
    {
        if ( SIOtransport_data->hPort!=INVALID_HANDLE_VALUE )
        {
            close(SIOtransport_data->hPort);
            flock(SIOtransport_data->hPort, LOCK_UN); /* free the port so that others can use it. */
            SIOtransport_data->hPort = INVALID_HANDLE_VALUE;

        }
        free(transport_data);
    }
    transport_data = NULL;

    return PMD_ERR_OK;
}



// ------------------------------------------------------------------------
PMDuint16 PMDSerial_InitPort(void* transport_data)
{
      
    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;

   	SIOtransport_data->port = open(LinuxCommPort, O_RDWR | O_NOCTTY);
    SIOtransport_data->hPort=SIOtransport_data->port;
    
   if (SIOtransport_data->hPort == INVALID_HANDLE_VALUE )
    {
        return PMD_ERR_OpeningPort;
    }
 
    else printf("Opened port %s\n",LinuxCommPort);

     //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
   // uart0_filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
  //  
    /* lock access so that another process can't also use the port */
    if (flock(SIOtransport_data->hPort, LOCK_EX | LOCK_NB) != 0)
    {
        PMDSerial_Close( transport_data );
        error_message("Another process has locked the comport.");
        return PMD_ERR_OpeningPort;
    }

    if( !PMDSerial_SetConfig( transport_data, SIOtransport_data->baud, PMDSerialParityNone) )
    {
        PMDSerial_Close( transport_data );
        return PMD_ERR_InvalidSerialPort;
    }

    if( !PMDSerial_SetTimeout( transport_data, 100 ) )
    {
        close( SIOtransport_data->hPort );
        SIOtransport_data->hPort = INVALID_HANDLE_VALUE;
        return PMD_ERR_InvalidSerialPort;
    }

    return PMD_ERR_OK;
}

// ------------------------------------------------------------------------
BOOL PMDSerial_SetConfig(void* transport_data, PMDuint32 baud, PMDuint8 parity)
{
    struct termios tty;
    int baudr;
    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;
        
    SIOtransport_data->baud   = baud;
    SIOtransport_data->parity = parity;
    SIOtransport_data->stop   = ONESTOPBIT;

    if( SIOtransport_data->hPort == INVALID_HANDLE_VALUE ) 
        return FALSE;

    memset (&tty, 0, sizeof tty);
    if (tcgetattr (SIOtransport_data->hPort, &tty) != 0)
    if ((errno=tcgetattr (SIOtransport_data->hPort,&tty)) !=0)
    {
        error_message ("error %d from tcgetattr", errno);
        return FALSE;
    }

 //    printf("baud=%d\n",baud);
    switch(baud)
    {
        case    2400 : 
            baudr = B2400;
            break;
        case    4800 : 
            baudr = B4800;
            break;
        case    9600 : 
            baudr = B9600;
            break;
        case   19200 : 
            baudr = B19200;
            break;
        case   57600 : 
            baudr = B57600;
            break;
        case  115200 : 
            baudr = B115200;
            break;
        case  230400 : 
            baudr = B230400;
            break;
        case  460800 : 
            baudr = B460800;
            break;

        default      :
            error_message("invalid baudrate\n");
            return FALSE;
            break;
    }

    int cbits=CS8,
        cpar=0,
        ipar=IGNPAR,
        bstop=0;

    switch(parity)
    {
        case PMDSerialParityNone: 
            cpar = 0;
            ipar = IGNPAR;
            break;
        case PMDSerialParityEven: 
            cpar = PARENB;
            ipar = INPCK;
            break;
        case PMDSerialParityOdd: 
            cpar = (PARENB | PARODD);
            ipar = INPCK;
            break;
        default : 
            error_message("invalid parity '%c'\n", parity);
            return FALSE;
            break;
    }

    cfsetospeed (&tty, baudr);
    cfsetispeed (&tty, baudr);
           
    tty.c_cflag = cbits | cpar | bstop;
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls, enable reading
    tty.c_cflag |= baudr;
    tty.c_iflag = ipar;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    /*
    VMIN = 0 and VTIME = 0
        This is a completely non-blocking read - the call is satisfied immediately directly from the driver's input queue. If data are available, it's transferred to the caller's buffer up to nbytes and returned. Otherwise zero is immediately returned to indicate "no data". We'll note that this is "polling" of the serial port, and it's almost always a bad idea. If done repeatedly, it can consume enormous amounts of processor time and is highly inefficient. Don't use this mode unless you really, really know what you're doing. 
    VMIN = 0 and VTIME > 0
        This is a pure timed read. If data are available in the input queue, it's transferred to the caller's buffer up to a maximum of nbytes, and returned immediately to the caller. Otherwise the driver blocks until data arrives, or when VTIME tenths expire from the start of the call. If the timer expires without data, zero is returned. A single byte is sufficient to satisfy this read call, but if more is available in the input queue, it's returned to the caller. Note that this is an overall timer, not an intercharacter one. 
    VMIN > 0 and VTIME > 0
        A read() is satisfied when either VMIN characters have been transferred to the caller's buffer, or when VTIME tenths expire between characters. Since this timer is not started until the first character arrives, this call can block indefinitely if the serial line is idle. This is the most common mode of operation, and we consider VTIME to be an intercharacter timeout, not an overall one. This call should never return zero bytes read. 
    VMIN > 0 and VTIME = 0
        This is a counted read that is satisfied only when at least VMIN characters have been transferred to the caller's buffer - there is no timing component involved. This read can be satisfied from the driver's input queue (where the call could return immediately), or by waiting for new data to arrive: in this respect the call could block indefinitely. We believe that it's undefined behavior if nbytes is less then VMIN. 
    */
    tty.c_cc[VMIN] = 0;      /* 1=block until n bytes are received */
    tty.c_cc[VTIME] = 0;     /* block until a timer expires (n * 100 mSec.) */

    tcflush(SIOtransport_data->hPort, TCIFLUSH); 
    if (errno = tcsetattr (SIOtransport_data->hPort, TCSANOW, &tty) != 0)
    {
        error_message ("error %d from tcsetattr", errno);
        return FALSE;
    }


     return TRUE;
}

// ------------------------------------------------------------------------
BOOL PMDSerial_SetTimeout(void* transport_data,long msec)
{
    struct termios tty;
    int error;

    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;

    if( SIOtransport_data->hPort == INVALID_HANDLE_VALUE ) return FALSE;

    int fd = SIOtransport_data->hPort;
    if (error = tcgetattr (fd, &tty) != 0)
    {
        error_message ("error %d from tcgetattr", errno);
        return FALSE;
    }

    tty.c_cc[VTIME] = msec / 100;     /* block until a timer expires (n * 100 mSec.) */

    printf("Timeout set to %dms\n",tty.c_cc[VTIME]*100);
    
    if (error = tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        error_message ("error %d from tcsetattr", errno);
        return FALSE;
    }
    
      return TRUE;
}

// ------------------------------------------------------------------------
void PMDSerial_SetProtocol(void* transport_data,PMDuint16 mode)
{
    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;

    SIOtransport_data->protocol = mode;
}

// ------------------------------------------------------------------------
void PMDSerial_SetMultiDropAddress(void* transport_data,PMDuint16 address)
{
    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;

    SIOtransport_data->multiDropAddress = address;
}

// ------------------------------------------------------------------------
PMDresult PMDSerial_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat)
{
    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;

    unsigned int c=0;
    unsigned int i,sumbytes;
    unsigned int nExpected = 2*rCt+2;
    char sum;
    int delay;   //ms
    DWORD bytes,tempbytes;
    PMDuint8 buffer[20];
    PMDuint8* pbuff = &buffer[0];
    PMDuint8 tempbuffer[20];
    PMDuint16 ProcessorError;
    int port_status=1;
    
    if( SIOtransport_data->hPort == INVALID_HANDLE_VALUE ) return PMD_ERR_NotConnected;

    /* Clear address byte & checksum byte */
    buffer[ c++ ] = (char)(SIOtransport_data->multiDropAddress);
    buffer[ c++ ] = (char)(0);

    /* Add axis number and command code */
    buffer[ c++ ] = (char)(xDat[0]>>8);
    buffer[ c++ ] = (char)(xDat[0]&0xff);

    /* add data (handling byte swapping) */
    for( i=1; i<xCt; i++ )
    {
        buffer[ c++ ] = (char)(xDat[i] >> 8);
        buffer[ c++ ] = (char)(xDat[i] & 0xFF);
    }

    /* calculate checksum */
    for( sum=i=0; i<c; i++ ) 
        sum += buffer[i];
    buffer[1] = -sum;

    // Flush the receive buffer in case any unexpected bytes have been received
    PMDSerial_FlushRecv(transport_data);

     
    bytes = write(SIOtransport_data->hPort, buffer, c);
   
    if( bytes != c )
        return PMD_ERR_CommPortWrite;
        
     //wait for transmission
     while(port_status) ioctl(SIOtransport_data->hPort,TIOCOUTQ,&port_status);   

          
    /* read return data */
    if (SIOtransport_data->protocol != PMDSerialProtocolMultiDropUsingIdleLineDetection)
    {
        sumbytes=0;
        bytes=1;
        while(bytes)
        {
            bytes = read(SIOtransport_data->hPort, tempbuffer, nExpected);
            for(i=0;i<bytes;i++) buffer[sumbytes+i]=tempbuffer[i];
            sumbytes+=bytes;
            if(sumbytes==nExpected) break;
        }
        bytes=sumbytes;
        
        if ( bytes ==0 )
        {
            if(SIOtransport_data->bDiagnostics)  printf("Timeout!!\n");   
            return PMD_ERR_CommTimeoutError;
        }
        if(bytes!=nExpected)
        {
            if(SIOtransport_data->bDiagnostics)  printf("Wrong Numer of Bytes.  Got: %ld  Expected: %d\n",bytes,nExpected);   
            return PMD_ERR_CommandError;
        }
    }
    else
    {
        // idle line returns an extra byte containing the slave address
        bytes = read(SIOtransport_data->hPort, buffer, nExpected+1);
               
        
        if ( bytes == 0 )
            return PMD_ERR_CommTimeoutError;

        if(buffer[0] != SIOtransport_data->multiDropAddress)
            return PMD_ERR_CommPortRead; // unexpected address  
    }
     
    // verify the checksum
    for( sum=i=0; i<bytes; i++ )
        sum += buffer[i];
    
    if (SIOtransport_data->protocol == PMDSerialProtocolMultiDropUsingIdleLineDetection)
    {
        // remove address byte from head of packet
        pbuff++; 
        bytes--;
    }

    // first byte is the error code 
    ProcessorError = pbuff[0];

    // if there was an error, don't attempt to receive any data
    if( ProcessorError && bytes==2 )
        rCt = 0;

    if( sum )
    {
        if(SIOtransport_data->bDiagnostics) PMDprintf("Checksum Error: bytes received %ld\n",bytes);
        return PMD_ERR_ChecksumError;
    }
    
    /* byte swap return data */
    for( i=0, c=2; i<rCt; i++ )
    {
        rDat[i]  = (PMDuint16)((pbuff[c++])<<8);
        rDat[i] |= (PMDuint16)(pbuff[c++]);
    }

    if (ProcessorError && SIOtransport_data->bDiagnostics)
    {
        PMDprintf("Processor status: %s\r\n", PMDGetErrorMessage(ProcessorError));
        PMDprintf("C-Motion: %s ",PMDGetOpcodeText(xDat[0]));
        for(i=1; i<xCt; i++)
            PMDprintf(" TXdata%d %X ",i, xDat[i]);
        for(i=0; i<rCt; i++)
            PMDprintf(" RXdata%d %X ",i, rDat[i]);
        PMDprintf("\r\n");
    }
    // some errors require resyncing the serial port when in point-to-point serial mode.
    if (ProcessorError && bytes == 2)
    {
        // we might be out of sync if any of these error codes are returned,
        // especially if the command contains parameters.
        if (ProcessorError == PMD_ERR_HardFault ||
            ProcessorError == PMD_ERR_BadSerialChecksum ||
            ProcessorError == PMD_ERR_InvalidInstruction ||
            ProcessorError == PMD_ERR_InvalidAxis)
        {
            PMDSerial_Sync(transport_data);
        }
    }
    else
    {
        if ( bytes != (unsigned)(nExpected) )
        {
            if(SIOtransport_data->bDiagnostics) PMDprintf("Error: Unexpected Number of Bytes %ld\n",bytes);
            return PMD_ERR_CommTimeoutError;
        }
    }

    return ( ProcessorError );
}

// ------------------------------------------------------------------------
PMDresult PMDSerial_Sync(void* transport_data)
{
    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;
    const int maxSend = 15;
    DWORD bytes;
    int i;
    char tx = 0;
    char rx[2];

    if( SIOtransport_data->hPort == INVALID_HANDLE_VALUE ) return PMD_ERR_NotConnected;
    // sync is not required with multi-drop protocols because the command buffer resets
    // after the idle time has elapsed.
    
    if (SIOtransport_data->protocol != Protocol_PointToPoint)
        return PMD_ERR_InvalidOperation;

    // Flush the receive buffer in case any unexpected bytes have been received
    PMDSerial_FlushRecv(transport_data);
    
    for( i=0; i<maxSend; i++ )
    {
        /* write a zero character */
        bytes = write(SIOtransport_data->hPort, &tx, 1);

        if( bytes != 1 )
            return PMD_ERR_CommPortWrite;
        
        PMDTaskWait(1);
        /* Attempt to read the 2 byte response */
        bytes = read(SIOtransport_data->hPort, rx, 2);

        if ( bytes == 2 )
            break;
    }

    /* If no data was seen, return an error */
    if( i== maxSend ) 
        return PMD_ERR_CommTimeoutError;

    /* flush any other data read */
    PMDSerial_FlushRecv(transport_data);

    return PMD_ERR_OK;
}

// ------------------------------------------------------------------------
PMDresult PMDSerial_FlushRecv(void* transport_data)
{
    PMDSerialIOData* SIOtransport_data = (PMDSerialIOData*)transport_data;

    if( SIOtransport_data->hPort == INVALID_HANDLE_VALUE ) return PMD_ERR_NotConnected;
    
    tcflush(SIOtransport_data->hPort, TCIOFLUSH);
    fflush(stdin);
    fflush(stdout);

    return PMD_ERR_OK;
}

// ------------------------------------------------------------------------
void PMDSerial_InitData(PMDSerialIOData* transport_data)
{
    // assign default values
    transport_data->multiDropAddress = 0;
    transport_data->protocol = Protocol_PointToPoint;
    transport_data->baud = 57600;

    // default port is COM1
    transport_data->port = 1;

    // by default always verify the checksum
    transport_data->bVerifyChecksum = 1;
    // by default disable diagnostics
    transport_data->bDiagnostics = 1;
}

// ------------------------------------------------------------------------
PMDresult PMDSerial_Init(PMDAxisHandle* handle)
{
    PMDAxisHandle* axis_handle = (PMDAxisHandle*) handle;

    // setup function pointers
    axis_handle->transport.SendCommand = PMDSerial_Send;

    axis_handle->transport.GetStatus = PMDSerial_GetStatus;
    axis_handle->transport.IsReady = PMDSerial_IsReady;
    axis_handle->transport.HasInterrupt = PMDSerial_HasInterrupt;
    axis_handle->transport.HasError = PMDSerial_HasError;
    axis_handle->transport.HardReset = PMDSerial_HardReset;

    axis_handle->transport.bHasDPRAM = FALSE;
    axis_handle->transport.ReadDPRAM = NULL;
    axis_handle->transport.WriteDPRAM = NULL;

    axis_handle->transport.Close = PMDSerial_Close;

    return PMDSerial_InitPort(axis_handle->transport_data);
}

// ------------------------------------------------------------------------
// Use this function to copy an existing handle to an open port 
// and set the multi-drop address and axis number.
void PMDCreateMultiDropHandle(PMDAxisHandle* dest_axis_handle, PMDAxisHandle* src_axis_handle, PMDAxis axis_number, PMDuint8 nodeID)
{
    PMDSerialIOData* transport_data;

    // copy the handle
    memcpy(dest_axis_handle, src_axis_handle, sizeof( PMDAxisHandle ) );
    // allocate a new transport_data structure because the multi-drop address will be different for each axis handle
    transport_data = (PMDSerialIOData*) malloc( sizeof( PMDSerialIOData ) );
    dest_axis_handle->transport_data = transport_data;
    memcpy(dest_axis_handle->transport_data, src_axis_handle->transport_data, sizeof(PMDSerialIOData));

    // set the axis number and multi-drop address 
    dest_axis_handle->axis = axis_number;
    transport_data->multiDropAddress = nodeID;
}   

/*****************************************************************************
Set port_number to COMn port number (1 = COM1, 0 = default)
*****************************************************************************/
PMDresult PMDSetupAxisInterface_Serial(PMDAxisHandle* axis_handle, PMDAxis axis_number, PMDuint8 port_number)
{
    PMDSerialIOData* transport_data;

    transport_data = (PMDSerialIOData*) malloc( sizeof( PMDSerialIOData ) );
    memset(transport_data, 0, sizeof(PMDSerialIOData));

    // set the axis we are talking to with this handle
    axis_handle->axis = axis_number;

    // set the interface type 
    axis_handle->InterfaceType = InterfaceSerial;

    // the transport data is initialized first to setup the defaults
    PMDSerial_InitData(transport_data);
    
    // assign port number that this axis handle will use
    // if 0 use default (COM1)
    if (port_number)
        transport_data->port = port_number;

    axis_handle->transport_data = (void*) transport_data;

    // initialize the transport (inits function pointers)
    return PMDSerial_Init(axis_handle);
}

