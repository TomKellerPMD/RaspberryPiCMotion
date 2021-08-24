#include "PMDdevice.h"

///////////////////////////////////////////////////////////////////////////
// C-Motion provided functions

PMDCFunc PMDCloseAxisInterface(PMDAxisInterface axis_intf)
{
	axis_intf->transport.Close(axis_intf->transport_data);
	return PMD_ERR_OK;
}

PMDCFunc PMDReadDPRAM(PMDAxisInterface axis_intf, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read)
{
	if (axis_intf->transport.bHasDPRAM)
		return axis_intf->transport.ReadDPRAM(axis_intf->transport_data, data, offset_in_dwords, dwords_to_read);
	else
		return PMD_ERR_NoDPRAM;
}

PMDCFunc PMDWriteDPRAM(PMDAxisInterface axis_intf, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write)
{
	if (axis_intf->transport.bHasDPRAM)
		return axis_intf->transport.WriteDPRAM(axis_intf->transport_data, data, offset_in_dwords, dwords_to_write);
	else
		return PMD_ERR_NoDPRAM;
}

// peforms a hard reset (valid for parallel interfaces only)
// make sure to add approx 100 ms wait period after a reset 
// and before attempting to communicate with the chip.
PMDCFunc PMDHardReset(PMDAxisInterface axis_intf)
{
	return axis_intf->transport.HardReset(axis_intf->transport_data);
}

// Create an axis handle for communicating directly with an Atlas rather than via the Magellan
// This is required for certain functions such as Atlas trace.
PMDCFunc PMDAtlasAxisOpen(PMDAxisInterface hSourceAxis, PMDAxisInterface hAtlasAxis)
{
	// copy the source axis handle info
	*hAtlasAxis = *hSourceAxis;
	// set the axis number to indicate it is an Atlas axis
	hAtlasAxis->axis = hAtlasAxis->axis | PMDAtlasAxisMask;

	return PMD_ERR_OK;
}

