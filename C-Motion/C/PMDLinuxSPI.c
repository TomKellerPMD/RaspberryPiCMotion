//
//	PMDLinuxSPI.c -- SPI interface command/data transfer functions for Linux SPI drivers.
//
//	Performance Motion Devices, Inc.
//

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "PMDsys.h"
#include "PMDtypes.h"
#include "PMDdevice.h"
#include "PMDSPI.h"

#define Printf printf
#define STRING_SIZE 256

#define SPI_RESULT(_call) \
	ret = _call; \
	if (-1 == ret) { \
		Printf(" call: " #_call "\n"); \
		return PMD_ERR_CommunicationsError; \
	}

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

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

	close(SPI_transport_data->m_Handle);

	SPI_transport_data->m_Handle = INVALID_HANDLE_VALUE;
	free(transport_data);

	return PMD_ERR_OK;
}


// Call into NI library to find and initialize the external device.
PMDuint16 PMDSPI_InitPort(PMDSPI_IOData* transport_data, int device, int chipselect)
{
	PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData*)transport_data;
	int fd;
	int ret;
	char sDevice[30];
	int mode = 0;
	int bits = 8;
	int speed = 10000000;

	sprintf(sDevice, "/dev/spidev%d.%d", device, chipselect);
	Printf("Device %s\n", sDevice);
	fd = open(sDevice, O_RDWR);
	if (fd < 0)
		return PMD_ERR_OpeningPort;
	SPI_transport_data->m_Handle = fd;

	/*
	mode |= SPI_LOOP;		// loopback
	mode |= SPI_CPHA;		// clock phase
	mode |= SPI_CPOL;		// clock polarity
	mode |= SPI_LSB_FIRST;		// least significant bit first
	mode |= SPI_CS_HIGH;		// chip select active high
	mode |= SPI_3WIRE;		// SI/SO signals shared
	mode |= SPI_NO_CS;		// no chip select
	mode |= SPI_READY;		// spi ready
	*/
	mode |= SPI_CPHA;		// clock phase
	// spi mode
	SPI_RESULT(ioctl(fd, SPI_IOC_WR_MODE, &mode));
	SPI_RESULT(ioctl(fd, SPI_IOC_RD_MODE, &mode));
	// bits per word
	SPI_RESULT(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits));
	SPI_RESULT(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits));
	// max speed hz
	SPI_RESULT(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed));
	SPI_RESULT(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed));

	Printf("spi mode: %d\n", mode);
	Printf("bits per word: %d\n", bits);
	Printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	Printf("speed: %d Hz (%d KHz)\n", SPI_transport_data->ClockRate, SPI_transport_data->ClockRate/1000);

	return PMD_ERR_OK;
}

int GetHostSPIStatus(PMDSPI_IOData* transport_data)
{
	int ReadData = 1;
	// add code to read the GPIO pin that is connected to the HostSPIstatus pin.

	return ReadData;
};


int WaitUntilReady(PMDSPI_IOData* transport_data)
{
#if 1
	usleep(50);
#else
	//if using the HostSIPStatus signal then enabled this code
	unsigned long EndTime = GetTickCount() + transport_data->m_Timeout;

	do {
		if (GetHostSPIStatus(transport_data) == 0)
			return 0;
	} while (GetTickCount() < EndTime);

	return PMD_ERR_Timeout;
#endif
	return PMD_ERR_OK;
};


/*
SPI protocol format
Sending a command
word	Host						MC58113
------------------------------------------------------
		Assert ~HostSPIEnable	
1		8b axis 8b opcode			0
2		8b rsv	8b checksum 		0
3		16b argument 1 (optional)	0
4		16b argument 2 (optional)	0
5		16b argument 3 (optional)	0
		De-assert ~HostSPIEnable	

		wait for HostSPIReady signal

Receiving a response
word	Host						MC58113
------------------------------------------------------
		Assert ~HostSPIEnable	
1		0							8b checksum  8b command status
2		0							16b response 1 (optional)
3		0							16b response 2 (optional)
4		0							16b response 3 (optional)
		De-assert ~HostSPIEnable	
*/
PMDresult PMDSPI_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat)
{
	PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData*)transport_data;


	union buff 
	{
		WORD  w[8];
		BYTE  b[16];
	} txbuff, rxbuff;

	int  ProcessorError;
	char  nbytes;
	char  nwords;
	int  c=0;
	int  i;
	int  sum;
	int  carry;
	WORD* WriteData = txbuff.w;
	WORD  Nop[8];
	WORD* ReadData = rxbuff.w;

	if( SPI_transport_data->m_Handle == INVALID_HANDLE_VALUE ) 
		 return PMD_ERR_NotConnected;

	memset(Nop, 0, 8);
	txbuff.b[c++] = (BYTE)(xDat[0]>>8); 	 // axis
	txbuff.b[c++] = (BYTE)(xDat[0] & 0xFF);  // opcode
	txbuff.b[c++] = 0;						 // reserved byte
	txbuff.b[c++] = 0;						 // checksum byte

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
	txbuff.b[3] = (PMDuint8)sum;		// checksum byte

	// if the HostSPIStatus signal is already low, we're out of sync 
	// so do a read of any number of words to resync. 
	if (GetHostSPIStatus(SPI_transport_data) == 0)
	{
		if (SPI_transport_data->bVerbose > 0)
			  Printf("SPI resync\n");
		PMDSPI_WriteWords(transport_data, Nop, 1, ReadData);
	}
	// send command
	nwords = nbytes / 2;
	PMDSPI_WriteWords(transport_data, WriteData, nwords, ReadData);
	
	// if any words in the return data are non-zero then we are out of sync so resend the command.
	if (ReadData[0] != 0)
	{
		Printf("SPI resync\n");
		PMDSPI_WriteWords(transport_data, WriteData, nwords, ReadData);
	}

	WaitUntilReady(SPI_transport_data);

	// read return words
	nwords = (rCt + 1); // +1 to include checksum word
	PMDSPI_WriteWords(transport_data, Nop, nwords, ReadData);

	// The first byte in response is the motion processor command status. 
	// A positive command status is # result words that are available. 
	ProcessorError = 0;
	if ((rxbuff.b[0]) < 0)
	{
		ProcessorError = -(rxbuff.b[0]);
	}

	sum = 0xAA;
	nbytes = nwords * 2;
	for( i=0; i<nbytes; i++ )
		sum += rxbuff.b[i];
	carry = sum >> 8;
	sum = sum & 0xFF;
	sum += carry;
	carry = sum >> 8;
	sum = sum & 0xFF;
	sum += carry;

	if( sum != 0xFF)
	{
		Printf("\n**Checksum error: (opcode 0x%02X)", xDat[0] );
		for(i=0; i<nwords; i++ )
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

// Data provided in pWriteData and retrieved using pReadData is organized in big endian format.
PMDresult PMDSPI_WriteWords(void* transport_data, PMDuint16 *WriteData, int nwords, PMDuint16 *ReadData)
{
	PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData*)(transport_data);
	int nbytes = 2*nwords;
	uint8_t rxbuff[20];
	int i;
	int ret = 0;
	int delay = 0;

	if (nbytes > ARRAY_SIZE(rxbuff))
		return PMD_ERR_CommandError;

	if (SPI_transport_data->bVerbose > 0 && WriteData[0] != 0) {
		Printf("     send:");
		for (i = 0; i < nwords; i++)
			Printf(" %04X", WriteData[i]);
		Printf("\n");
	}

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)WriteData,
		.rx_buf = (unsigned long)rxbuff,
		.len = nbytes,
		.delay_usecs = delay,
		.speed_hz = SPI_transport_data->ClockRate,
		.bits_per_word = SPI_transport_data->NumBitsPerSample
	};

	SPI_RESULT(ioctl(SPI_transport_data->m_Handle, SPI_IOC_MESSAGE(1), &tr));


	if (ret >= 0 && ReadData)
	{
		// Byte swap read data
		for (i = 0; i < nwords; i++)
		{
			ReadData[i] = ((rxbuff[2*i]) << 8) + rxbuff[2*i + 1];
		}
		if (SPI_transport_data->bVerbose > 0) 
		{
			Printf("  receive:");
			for (i = 0; i < nwords; i++)
				Printf(" %04X", ReadData[i]);
			Printf("\n");
		}
	}
	return PMD_ERR_OK;
}

// Set up default configuration for SPI port
void PMDSPI_InitData(PMDSPI_IOData* transport_data)
{
	transport_data->m_Handle = INVALID_HANDLE_VALUE;
	transport_data->m_bUseScript = FALSE;
	transport_data->m_Timeout = 200;
	transport_data->NumBitsPerSample = 8;
	transport_data->ClockRate = 500000; // Hz
	transport_data->bVerbose = 0;
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
  device is the chip select #
*/
PMDresult PMDSetupAxisInterface_SPI(PMDAxisHandle* axis_handle,
									PMDAxis axis_number,
									int device,
									int chipselect)
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
	return PMDSPI_Init(axis_handle, device, chipselect);
}

long PMDSPI_SetTimeout(void *transport_data, long msec)
{
	long ret;

	PMDSPI_IOData* SPI_transport_data = (PMDSPI_IOData *)transport_data;
	ret = SPI_transport_data->m_Timeout;
	SPI_transport_data->m_Timeout = msec;
	return ret;
}
