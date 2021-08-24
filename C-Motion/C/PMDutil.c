// ****************************************************************
// PMDutil.c : Defines sample test and utility functions
//
// Performance Motion Devices, Inc.
//

#include <memory.h>
#include <malloc.h>

#include "C-Motion.h"
#include "PMDsys.h"
#include "PMDutil.h"
#include "PMDdiag.h"
#include "PMDocode.h"
#include "PMDecode.h"
#include <time.h>
#include <math.h>



PMDuint32 GetTickCount(void);



//*****************************************************************************
// Initialize array of axis handles
// define the PMD_?_INTERFACE definitions in the project file
// as a preprocessor definition based on the interface used
int InitializeAxisHandles(PMDAxisHandle* phAxis)
{
	PMDAxisHandle* pAxis0 = phAxis;
	PMDAxis axis;
	PMDuint16 status;
	PMDuint16 result = PMD_ERR_OK;

#ifdef PMD_PARALLEL_INTERFACE
	if (pAxis0->transport_data == NULL) // prevent multiple initializations
		PMDSetupAxisInterface_Parallel( pAxis0, PMDAxis1, 0x340 );
	PMDprintf("Using direct parallel IO interface\n");
#endif

#ifdef PMD_W32SERIAL_INTERFACE
	if (pAxis0->transport_data == NULL) // prevent multiple initializations
		PMDSetupAxisInterface_Serial( pAxis0, PMDAxis1, 0 );
	PMDprintf("Using serial interface\n");
#endif

#ifdef PMD_PCI_INTERFACE
	if (pAxis0->transport_data == NULL) // prevent multiple initializations
		PMDSetupAxisInterface_PCI( pAxis0, PMDAxis1, 0 );
	PMDprintf("Using PCI interface\n");
#endif

#ifndef PMD_PARALLEL_INTERFACE
#ifndef PMD_W32SERIAL_INTERFACE
#ifndef PMD_PCI_INTERFACE
	PMDprintf("Error: No PMD Interface defined in PMDutil.c \n");
#endif
#endif
#endif

	if (pAxis0->transport_data == NULL)
	{
		PMDprintf("Failed to initialize board interface\n");
		return 0;
	}

	// create handles to the other axes
	for (axis=PMDAxis2; axis<PMD_MAX_AXES; axis++)
		PMDCopyAxisInterface(&phAxis[axis], pAxis0, axis);

	// the command error bit may have been set after the last
	// reset, so clear it
	result = PMDGetInstructionError( pAxis0, &status );

/*	enable this if you always want the processor in a known state at start up
	// attempt board reset 
	if (!PMDChipsetReset(pAxis0))
	{
		PMDprintf("Reset Failed.\n");
		return 0;
	}
*/

	return (result == PMD_ERR_OK);
}


//*****************************************************************************
void PMDCopyAxisInterface(PMDAxisHandle* dest_axis_handle, PMDAxisHandle* src_axis_handle, PMDAxis axis_number)
{
	// copy the handle
	memcpy(dest_axis_handle, src_axis_handle, sizeof( PMDAxisHandle ) );

	// set the axis we are talking to with this handle
	dest_axis_handle->axis = axis_number;
}

//*****************************************************************************
PMDresult PMDProcessorReset(PMDAxisHandle* phAxis)
{
	PMDresult result;
	PMDuint16 status;

	// reset the PMD chip set that this axis resides on
	// if more than one chip set is present, all of them should be
	// reset here
	result = PMDReset(phAxis);

	if ( result != PMD_NOERROR )
	{
		PMDprintf("Error: %s\n", PMDGetErrorMessage(result));
		return result;
	}

	// wait for chip to reset
	PMDTaskWait(400); // ms

	// After the reset the InstructionError register should return PMD_ERR_Reset.
	// The call to GetInstructionError will reset the InstructionError register 
	// back to PMD_NOERROR
	result = PMDGetInstructionError(phAxis, &status);

	// the above command should execute without error but we need to check
	if ( (result != PMD_NOERROR) || (status != PMD_ERR_Reset && status != PMD_NOERROR) )
	{
		PMDprintf("Error: %s\n", PMDGetErrorMessage(result));
		PMDprintf("Status: %s\n", PMDGetErrorMessage(status));
	}
	return result;
}

//*****************************************************************************
void SetupTrace(PMDAxisHandle* phAxis, PMDuint32 bufferlength)
{
	PMDSetTraceStop( phAxis, 0, PMDTraceConditionImmediate, 0, PMDTraceTriggerStateHigh );
	
	// setup the trace buffer (buffer ID = 0)
	PMDSetBufferLength( phAxis, 0, bufferlength );
	PMDSetTracePeriod( phAxis, 1); // cycle(s) per trace
	PMDSetTraceMode( phAxis, PMDTraceModeOneTime );

	// single axis dynamics trace
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber1, PMDAxis1, PMDTraceVariableActualPosition );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber2, PMDAxis1, 84 );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber3, PMDAxis1, PMDTraceVariableNone ); 
	//PMDSetTraceVariable( phAxis, PMDTraceVariableNumber4, PMDAxis1, PMDTraceVariableMotionProcessorTime );

/*
	// single axis commanded vs actual position trace
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber1, PMDAxis1, PMDTraceVariableCommandedPosition );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber2, PMDAxis1, PMDTraceVariableActualPosition );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber3, PMDAxis1, PMDTraceVariableActiveMotorCommand );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber4, PMDAxis1, PMDTraceVariableNone );

	// 4 axis position trace
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber1, PMDAxis1, PMDTraceVariableCommandedPosition );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber1, PMDAxis2, PMDTraceVariableCommandedPosition );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber1, PMDAxis3, PMDTraceVariableCommandedPosition );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber1, PMDAxis4, PMDTraceVariableCommandedPosition );

	// 2 axis commanded position vs velocity trace
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber1, PMDAxis1, PMDTraceVariableCommandedPosition );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber2, PMDAxis2, PMDTraceVariableCommandedPosition );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber3, PMDAxis1, PMDTraceVariableCommandedVelocity );
	PMDSetTraceVariable( phAxis, PMDTraceVariableNumber4, PMDAxis2, PMDTraceVariableCommandedVelocity );
*/	
	// trace will start on the next update command on Axis 1
	PMDSetTraceStart( phAxis, PMDAxis1, PMDTraceConditionNextUpdate, 0, PMDTraceTriggerStateHigh );
	// or trigger a trace start on InMotion bit instead of an update 
	//PMDSetTraceStart( phAxis, PMDAxis1, PMDTraceConditionActivityStatus, PMDActivityInMotionBit, PMDTraceTriggerStateHigh );
}

/***********************************************************************************

  ReadBuffer

  Implements similar functionality as the chip command PMDReadBuffer except it reads the data
  directly from the DPRAM if the interface supports it.  It is much faster than
  calling PMDReadBuffer for the same number of data reads.
  To further improve performance the PMDGet... commands can be removed and the values stored
  in local variables

***********************************************************************************/
PMDresult ReadBuffer(PMDAxisHandle* phAxis, PMDuint16 bufferID, PMDuint32* pbuffer, PMDuint32 dwords_to_read)
{
	PMDuint32 i, index,length,start;

	if (phAxis->transport.bHasDPRAM)
	{
		PMDGetBufferReadIndex(phAxis, bufferID, &index);
		PMDGetBufferLength(phAxis, bufferID, &length);
		PMDGetBufferStart(phAxis, bufferID, &start);

		// can't read more data than the buffer can hold!
		if (dwords_to_read>length)
			return PMD_ERR_InvalidOperation;

		// check if this is a wrap-around condition
		if (dwords_to_read > (length-index))
		{
			// read the first segment
			PMDReadDPRAM( phAxis, pbuffer, index+start, (length-index));
			// read the next segment
			PMDReadDPRAM( phAxis, pbuffer+(length-index), start, dwords_to_read-(length-index));
		}
		else
			PMDReadDPRAM( phAxis, pbuffer, index+start, dwords_to_read);

		PMDSetBufferReadIndex(phAxis, bufferID, dwords_to_read%length);
	}
	else  
	{
		for (i=0; i<dwords_to_read; i++) 
			PMDReadBuffer(phAxis, bufferID, pbuffer++);
	}
	return PMD_ERR_OK;
}

//*****************************************************************************
void DisplayTraceResults(PMDAxisHandle* phAxis)
{
	PMDuint32 i;
	PMDuint32 readindex = 0;
	PMDlong32 *pbuffer;
	PMDlong32 *pbufferindex;
	PMDuint16 status;
	PMDuint16 tracemode;
	PMDuint16 bufferid = 0;
	PMDuint32 bufferlength;
	PMDuint32 tracecount;
	PMDuint32 nTraces;
	PMDuint16 traceperiod;
	PMDuint16 nVariables = 0;
	PMDuint8 tracevar;
	PMDAxis axisno;
	int bDone, bActive, bStarted;


	// get number of trace variables so we can display each one in it's own column
	do
	{
		PMDGetTraceVariable( phAxis, nVariables, &axisno, &tracevar );
		nVariables++;
	}
	while (tracevar != PMDTraceVariableNone && nVariables < 4);

	PMDGetBufferLength(phAxis, bufferid, &bufferlength );
	PMDGetTraceMode(phAxis, &tracemode);
	PMDGetTraceCount(phAxis, &tracecount);
	PMDGetTraceStatus(phAxis, &status);
	PMDGetTracePeriod(phAxis, &traceperiod);
	nTraces = bufferlength / nVariables;

	PMDprintf("Trace status  = %d\r\n", status);
	PMDprintf("Trace count   = %ld\r\n", tracecount);
	PMDprintf("Trace period  = %d cycles\r\n", traceperiod);
	PMDprintf("Buffer length = %ld\r\n", bufferlength );
	PMDprintf("# traces      = %ld\r\n", nTraces );

	bActive = (status & PMDTraceStatusActivity);
	bDone = (tracecount == bufferlength && !bActive);
	bStarted = bActive || bDone;

	if (!bStarted)
	{
		PMDprintf("Error. Trace was not started.\r\n");
		return;
	}

//	PMDSetBufferReadIndex(phAxis, bufferid, readindex);

	pbuffer = (PMDlong32*)malloc(sizeof(PMDlong32) * bufferlength);
	
	if (bActive) // wait until done (not necessary but simpler)
	{
		while (tracecount < bufferlength) 
			PMDGetTraceCount( phAxis, &tracecount);
	}

	// read the data
	ReadBuffer(phAxis, bufferid, pbuffer, tracecount);
	
	// display the data
	tracecount = 0;
	pbufferindex = pbuffer;
	PMDprintf("\n");
	PMDprintf("#\tVar1\tVar2\tVar3\tVar4\n");
	PMDprintf("----\t----\t----\t----\t----\n");
	do 
	{
		PMDprintf("%ld", tracecount);
		for (i=0; i<nVariables; i++) 
		{
			PMDprintf("\t%ld", *pbufferindex++);
		}
		PMDprintf("\n");
		tracecount++;
	}
	while (tracecount < nTraces);

	free(pbuffer);
}

//*****************************************************************************
// WaitForEvent 
//
//   Waits for the specified event to be set in the EventStatus register.
//
//*****************************************************************************

PMDresult WaitForEvent(PMDAxisHandle* phAxis, PMDuint16 eventmask, PMDuint32 timeoutms)
{
    PMDuint16 status = 0;
    PMDresult result = PMD_NOERROR;
    PMDuint32 starttime;
    PMDuint32 endtime;

    PMDprintf("Waiting for event %04X on axis %d.\n", eventmask, phAxis->axis + 1);

    starttime = PMDDeviceGetTickCount();
    while (!(status & eventmask))
    {
        PMDGetEventStatus( phAxis, &status );
        if (PMDDeviceGetTickCount() > starttime + timeoutms)
        {
            return PMD_ERR_Timeout;
        }
    }
    endtime = PMDDeviceGetTickCount();
    PMDprintf("Event(s) %04X set on axis #%d in %ld ms\n", status, phAxis->axis+1, endtime - starttime);
    // clear the event for the next time
    PMDResetEventStatus( phAxis, (PMDuint16)~eventmask );

    return result;
}


// ****************************************************************************
PMDresult WaitForAlgorithmicPhaseInitialization(PMDAxisHandle* phAxis)
{
	PMDuint16 status;
	PMDresult result;

	// poll ActivityStatus to determine when Phase Initilization is complete
	PMDprintf("Waiting for algorithmic phase initialization to complete.\n");
	do
	{
		result = PMDGetActivityStatus(phAxis, &status);
	}	
	while(!(status & PMDActivityStatusPhasingInitialized));

	PMDprintf("Algorithmic phase initialization completed.\n");
	return result;
}

PMDuint32 GetTickCount()
{
	struct timespec spec;
	time_t s;
	long ms;
	clock_gettime(CLOCK_REALTIME,&spec);

	s = spec.tv_sec;
	ms = round(spec.tv_nsec/1.0e6);
	if(ms > 999)
	{
		s++;
		ms=0;
	}	

	return s*1000+ms;
}


// ****************************************************************************
// this function will perform the appropriate phase initialize method for the current settings
PMDresult InitializePhase(PMDAxisHandle* phAxis)
{
	PMDuint16 PhaseInitializeMode;
	PMDuint16 MotorType;
	PMDresult result;

	result = PMDGetMotorType( phAxis, &MotorType );

	if (MotorType == PMDMotorTypeBrushlessDC3Phase)
	{
		PMDGetPhaseInitializeMode( phAxis, &PhaseInitializeMode );
		if (PhaseInitializeMode == PMDPhaseInitializeModeAlgorithmic)
		{
			//assumes SetMotorCommand has been set
			PMD_RESULT(PMDInitializePhase( phAxis ))
			if (result == PMD_NOERROR)
				WaitForAlgorithmicPhaseInitialization( phAxis );
		}
		else if (PhaseInitializeMode == PMDPhaseInitializeModeHallBased)
		{
			result = PMDInitializePhase(phAxis);
		}
	}
	return result;
}


// ****************************************************************************
// wait for the "DriveNotConnected" bit in the Magellan's Drive Status register to go low.
PMDresult WaitForAtlasToConnect(PMDAxisHandle* phAxis)
{
	PMDuint16 status = 0;
	PMDuint32 starttime;
	PMDuint32 timeoutms = 1000;

	starttime = GetCurrentMilliseconds();
	do
	{
		PMDGetDriveStatus( phAxis, &status );
		if (GetCurrentMilliseconds() > starttime + timeoutms)
		{
			PMDprintf("Timeout waiting for Atlas to connect.\n");
			return PMD_ERR_Timeout;
		}
	}
	while (status & PMDDriveStatusAtlasNotConnected);

	PMDGetDriveFaultStatus( phAxis, &status );
	if (status & PMDDriveFaultStatusMotorTypeMismatch)
		PMDprintf("Error: Magellan motor type does not match Atlas motor type.\n");

	return PMD_NOERROR;
}

PMDresult DisplayVersion(PMDAxisHandle* phAxis)
{
    PMDresult result = PMD_NOERROR;
	char cPartNumber[20];
	char* pPartNumber = &cPartNumber[0];
	int i;
    PMDuint16 generation, motorType, numberAxes, special, custom, major, minor;
	PMDuint32 version;

    PMD_RESULT(PMDGetVersion(phAxis, &generation, &motorType, &numberAxes, &special, &custom, &major, &minor));

    if (result == PMD_NOERROR)
    {
		if (generation != 0) // GetVersion is 0 for MC7x113 (Juno)
		{
			if ((special & 0x3) == 0) // MCxx113
				PMDprintf("PMD processor MC%d%d%d13 v%d.%d\n\n", generation, motorType, numberAxes, major, minor);
			else
				PMDprintf("PMD processor MC%d%d%d%d0 v%d.%d\n\n", generation, motorType, numberAxes, special, major, minor);
		}
		else
		{
			for (i=0; i<4; i++)
			{
				PMD_RESULT(PMDGetProductInfo(phAxis,  PMDProductInfoPartNumberWord1+i, (PMDuint32*)pPartNumber+i));
			}
			PMD_RESULT(PMDGetProductInfo(phAxis,  PMDProductInfoVersion, &version));
			minor = (version >> 0) & 0xFF;
			major = (version >> 8) & 0xFF;
			custom = (version >> 16) & 0xFF;
			if (custom)
				PMDprintf("PMD processor %s v%d.%d.%d\n\n", cPartNumber, custom, major, minor);
			else
				PMDprintf("PMD processor %s v%d.%d\n\n", cPartNumber, major, minor);
		}
	}
	
	return result;
}

// convert GetProductInfo version format to GetVersion format
PMDCFunc GetVersionDWord(PMDAxisInterface axis_intf, PMDuint32 *version)
{
	PMDresult result;
	int Offset = 0;
	PMDuint32 mode;
	PMDuint32 ver;

	result = PMDGetVersion32(axis_intf, version);
	// For MC7x113 GetVersion returns 0
	if (result == PMD_NOERROR && *version == 0)
	{
		result = PMDGetProductInfo(axis_intf, 0, &mode); // index 0 returns currently running firmware 0=motion, 1=bootloader
		Offset = mode << 8;

		result = PMDGetProductInfo(axis_intf,  PMDProductInfoVersion + Offset, &ver);
		// ver is 4 bytes -> 0 | custom | major | minor
		*version |= (ver >> 0) & 0x0F;
		*version |= ((ver >> 8) & 0x0F) << 4;
		*version |= ((ver >> 16) & 0xFF) << 8;
		*version |= mode << 18;
		result = PMDGetProductInfo(axis_intf,  PMDProductInfoProductClass + Offset, &ver);
		// ver is 4 bytes -> generation | motortype | numaxes | 0
		*version |= ((ver >> 8) & 0x0F) << 20;	// numaxes
		*version |= ((ver >> 16) & 0x0F) << 24;	// motortype
		*version |= ((ver >> 24) & 0x0F) << 28;	// generation
	}
	return result;
}
