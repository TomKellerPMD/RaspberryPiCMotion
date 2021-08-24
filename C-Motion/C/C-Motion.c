///////////////////////////////////////////////////////////////////////////
//
//	C-Motion.c -- PMD C-Motion API implementation
//
//	Performance Motion Devices, Inc.
//
///////////////////////////////////////////////////////////////////////////

#include "C-Motion.h"
#include "PMDocode.h"


///////////////////////////////////////////////////////////////////////////
// Profile Generation
PMDCFunc PMDSetProfileMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetProfileMode, mode);
}

PMDCFunc PMDGetProfileMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetProfileMode, mode);
}

PMDCFunc PMDSetPosition(PMDAxisInterface axis_intf, PMDint32 position)
{
	return SendCommandLong(axis_intf, PMDOPSetPosition, position);
}

PMDCFunc PMDGetPosition(PMDAxisInterface axis_intf, PMDint32* position)
{
	return SendCommandGetLong(axis_intf, PMDOPGetPosition,(PMDuint32*)position);
}

PMDCFunc PMDSetVelocity(PMDAxisInterface axis_intf, PMDint32 velocity)
{
	return SendCommandLong(axis_intf, PMDOPSetVelocity, velocity);
}

PMDCFunc PMDGetVelocity(PMDAxisInterface axis_intf, PMDint32* velocity)
{
	return SendCommandGetLong(axis_intf, PMDOPGetVelocity,(PMDuint32*)velocity);
}

PMDCFunc PMDSetStartVelocity(PMDAxisInterface axis_intf, PMDuint32 velocity)
{
	return SendCommandLong(axis_intf, PMDOPSetStartVelocity, velocity);
}

PMDCFunc PMDGetStartVelocity(PMDAxisInterface axis_intf, PMDuint32* velocity)
{
	return SendCommandGetLong(axis_intf, PMDOPGetStartVelocity, velocity);
}

PMDCFunc PMDSetAcceleration(PMDAxisInterface axis_intf, PMDuint32 acceleration)
{
	return SendCommandLong(axis_intf, PMDOPSetAcceleration, acceleration);
}

PMDCFunc PMDGetAcceleration(PMDAxisInterface axis_intf, PMDuint32* acceleration)
{
	return SendCommandGetLong(axis_intf, PMDOPGetAcceleration, acceleration);
}

PMDCFunc PMDSetDeceleration(PMDAxisInterface axis_intf, PMDuint32 deceleration)
{
	return SendCommandLong(axis_intf, PMDOPSetDeceleration, deceleration);
}

PMDCFunc PMDGetDeceleration(PMDAxisInterface axis_intf, PMDuint32* deceleration)
{
	return SendCommandGetLong(axis_intf, PMDOPGetDeceleration, deceleration);
}

PMDCFunc PMDSetJerk(PMDAxisInterface axis_intf, PMDuint32 jerk)
{
	return SendCommandLong(axis_intf, PMDOPSetJerk, jerk);
}

PMDCFunc PMDGetJerk(PMDAxisInterface axis_intf, PMDuint32* jerk)
{
	return SendCommandGetLong(axis_intf, PMDOPGetJerk, jerk);
}

PMDCFunc PMDSetGearRatio(PMDAxisInterface axis_intf, PMDint32 ratio)
{
	return SendCommandLong(axis_intf, PMDOPSetGearRatio,(PMDuint32)ratio);
}

PMDCFunc PMDGetGearRatio(PMDAxisInterface axis_intf, PMDint32* ratio)
{
	return SendCommandGetLong(axis_intf, PMDOPGetGearRatio,(PMDuint32*)ratio);
}

PMDCFunc PMDSetGearMaster(PMDAxisInterface axis_intf, PMDAxis masterAxis, PMDuint16 source)
{
	PMDuint16 value =(PMDuint16)((masterAxis & nibbleMask) |((source & nibbleMask) << 8));
	return SendCommandWord(axis_intf, PMDOPSetGearMaster, value);
}

PMDCFunc PMDGetGearMaster(PMDAxisInterface axis_intf, PMDAxis* masterAxis, PMDuint16* source)
{
	PMDresult result;
	PMDuint16 value;

	result = SendCommandGetWord(axis_intf, PMDOPGetGearMaster, &value);
	*masterAxis =(PMDAxis)(value & nibbleMask);
	*source =((value >> 8) & nibbleMask);

	return result;
}

PMDCFunc PMDSetStopMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetStopMode, mode);
}

PMDCFunc PMDGetStopMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetStopMode, mode);
}

PMDCFunc PMDGetCommandedPosition(PMDAxisInterface axis_intf, PMDint32* position)
{
	return SendCommandGetLong(axis_intf, PMDOPGetCommandedPosition, (PMDuint32*)position);
}

PMDCFunc PMDGetCommandedVelocity(PMDAxisInterface axis_intf, PMDint32* velocity)
{
	return SendCommandGetLong(axis_intf, PMDOPGetCommandedVelocity, (PMDuint32*)velocity);
}

PMDCFunc PMDGetCommandedAcceleration(PMDAxisInterface axis_intf, PMDint32 *acceleration)
{
	return SendCommandGetLong(axis_intf, PMDOPGetCommandedAcceleration, (PMDuint32*)acceleration);
}


///////////////////////////////////////////////////////////////////////////
// Servo Filter
PMDCFunc PMDSetMotorLimit(PMDAxisInterface axis_intf, PMDuint16 limit)
{
	return SendCommandWord(axis_intf, PMDOPSetMotorLimit, limit);
}

PMDCFunc PMDGetMotorLimit(PMDAxisInterface axis_intf, PMDuint16* limit)
{
	return SendCommandGetWord(axis_intf, PMDOPGetMotorLimit, limit);
}

PMDCFunc PMDSetMotorBias(PMDAxisInterface axis_intf, PMDint16 bias)
{
	return SendCommandWord(axis_intf, PMDOPSetMotorBias,(PMDuint16)bias);
}

PMDCFunc PMDGetMotorBias(PMDAxisInterface axis_intf, PMDint16* bias)
{
	return SendCommandGetWord(axis_intf, PMDOPGetMotorBias, (PMDuint16*)bias);
}

PMDCFunc PMDSetPositionErrorLimit(PMDAxisInterface axis_intf, PMDuint32 limit)
{
	return SendCommandLong(axis_intf, PMDOPSetPositionErrorLimit, limit);
}

PMDCFunc PMDGetPositionErrorLimit(PMDAxisInterface axis_intf, PMDuint32* limit)
{
	return SendCommandGetLong(axis_intf, PMDOPGetPositionErrorLimit, limit);
}

PMDCFunc PMDSetSettleTime(PMDAxisInterface axis_intf, PMDuint16 time)
{
	return SendCommandWord(axis_intf, PMDOPSetSettleTime, time);
}

PMDCFunc PMDGetSettleTime(PMDAxisInterface axis_intf, PMDuint16* time)
{
	return SendCommandGetWord(axis_intf, PMDOPGetSettleTime, time);
}

PMDCFunc PMDSetSettleWindow(PMDAxisInterface axis_intf, PMDuint16 window)
{
	return SendCommandWord(axis_intf, PMDOPSetSettleWindow, window);
}

PMDCFunc PMDGetSettleWindow(PMDAxisInterface axis_intf, PMDuint16* window)
{
	return SendCommandGetWord(axis_intf, PMDOPGetSettleWindow, window);
}

PMDCFunc PMDSetTrackingWindow(PMDAxisInterface axis_intf, PMDuint16 window)
{
	return SendCommandWord(axis_intf, PMDOPSetTrackingWindow, window);
}

PMDCFunc PMDGetTrackingWindow(PMDAxisInterface axis_intf, PMDuint16* window)
{
	return SendCommandGetWord(axis_intf, PMDOPGetTrackingWindow, window);
}

PMDCFunc PMDSetMotionCompleteMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetMotionCompleteMode, mode);
}

PMDCFunc PMDGetMotionCompleteMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetMotionCompleteMode, mode);
}

PMDCFunc PMDClearPositionError(PMDAxisInterface axis_intf)
{
	return SendCommand(axis_intf, PMDOPClearPositionError);
}

PMDCFunc PMDGetPositionError(PMDAxisInterface axis_intf, PMDint32* error)
{
	return SendCommandGetLong(axis_intf, PMDOPGetPositionError,(PMDuint32*)error);
}

PMDCFunc PMDSetSampleTime(PMDAxisInterface axis_intf, PMDuint32 time)
{
	return SendCommandLong(axis_intf, PMDOPSetSampleTime, time);
}

PMDCFunc PMDGetSampleTime(PMDAxisInterface axis_intf, PMDuint32* time)
{
	return SendCommandGetLong(axis_intf, PMDOPGetSampleTime, time);
}


///////////////////////////////////////////////////////////////////////////
// Parameter Update & Breakpoints
PMDCFunc PMDSetBreakpoint(PMDAxisInterface axis_intf, PMDuint16 breakpointID,
						 PMDAxis sourceAxis, PMDuint8 action, PMDuint8 trigger)
{
	PMDuint16 value =(PMDuint16)((sourceAxis & nibbleMask) |
		((action & nibbleMask) << 4) |((trigger & byteMask) << 8));
	return SendCommandWordWord(axis_intf, PMDOPSetBreakpoint, breakpointID, value);
}

PMDCFunc PMDGetBreakpoint(PMDAxisInterface axis_intf, PMDuint16 breakpointID,
						 PMDAxis* sourceAxis, PMDuint8* action, PMDuint8* trigger)
{
	PMDresult result;
	PMDuint16 value;

	result = SendCommandWordGetWord(axis_intf, PMDOPGetBreakpoint, breakpointID, &value);

	*sourceAxis =(PMDAxis)(value & nibbleMask);
	*action =(PMDuint8)((value >> 4) & nibbleMask);
	*trigger =(PMDuint8)((value >> 8) & byteMask);

	return result;
}

PMDCFunc PMDSetBreakpointValue(PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDint32 value)
{
	return SendCommandWordLong(axis_intf, PMDOPSetBreakpointValue, breakpointID,(PMDuint32)value);
}

PMDCFunc PMDGetBreakpointValue(PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetBreakpointValue, breakpointID,(PMDuint32*)value);
}

PMDCFunc PMDUpdate(PMDAxisInterface axis_intf)
{
	return SendCommand(axis_intf, PMDOPUpdate);
}

PMDCFunc PMDMultiUpdate(PMDAxisInterface axis_intf, PMDuint16 mask)
{
	return SendCommandWord(axis_intf, PMDOPMultiUpdate, mask);
}


// Interrupt Processing
PMDCFunc PMDSetInterruptMask(PMDAxisInterface axis_intf, PMDuint16 mask)
{
	return SendCommandWord(axis_intf, PMDOPSetInterruptMask, mask);
}

PMDCFunc PMDGetInterruptMask(PMDAxisInterface axis_intf, PMDuint16* mask)
{
	return SendCommandGetWord(axis_intf, PMDOPGetInterruptMask, mask);
}

PMDCFunc PMDClearInterrupt(PMDAxisInterface axis_intf)
{
	return SendCommand(axis_intf, PMDOPClearInterrupt);
}

PMDCFunc PMDGetInterruptAxis(PMDAxisInterface axis_intf, PMDuint16* mask)
{
	return SendCommandGetWord(axis_intf, PMDOPGetInterruptAxis, mask);
}


///////////////////////////////////////////////////////////////////////////
// Status Register Control
PMDCFunc PMDResetEventStatus(PMDAxisInterface axis_intf, PMDuint16 status)
{
	return SendCommandWord(axis_intf, PMDOPResetEventStatus, status);
}

PMDCFunc PMDGetEventStatus(PMDAxisInterface axis_intf, PMDuint16* status)
{
	return SendCommandGetWord(axis_intf, PMDOPGetEventStatus, status);
}

PMDCFunc PMDGetActivityStatus(PMDAxisInterface axis_intf, PMDuint16* status)
{
	return SendCommandGetWord(axis_intf, PMDOPGetActivityStatus, status);
}

PMDCFunc PMDSetSignalSense(PMDAxisInterface axis_intf, PMDuint16 sense)
{
	return SendCommandWord(axis_intf, PMDOPSetSignalSense, sense);
}

PMDCFunc PMDGetSignalSense(PMDAxisInterface axis_intf, PMDuint16* sense)
{
	return SendCommandGetWord(axis_intf, PMDOPGetSignalSense, sense);
}

PMDCFunc PMDGetSignalStatus(PMDAxisInterface axis_intf, PMDuint16* status)
{
	return SendCommandGetWord(axis_intf, PMDOPGetSignalStatus, status);
}


///////////////////////////////////////////////////////////////////////////
// Encoder
PMDCFunc PMDAdjustActualPosition(PMDAxisInterface axis_intf, PMDint32 position)
{
	return SendCommandLong(axis_intf, PMDOPAdjustActualPosition,(PMDuint32)position);
}

PMDCFunc PMDSetActualPosition(PMDAxisInterface axis_intf, PMDint32 position)
{
	return SendCommandLong(axis_intf, PMDOPSetActualPosition,(PMDuint32)position);
}

PMDCFunc PMDGetActualPosition(PMDAxisInterface axis_intf, PMDint32* position)
{
	return SendCommandGetLong(axis_intf, PMDOPGetActualPosition,(PMDuint32*)position);
}

PMDCFunc PMDSetActualPositionUnits(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetActualPositionUnits, mode);
}

PMDCFunc PMDGetActualPositionUnits(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetActualPositionUnits, mode);
}

PMDCFunc PMDSetCaptureSource(PMDAxisInterface axis_intf, PMDuint16 source)
{
	return SendCommandWord(axis_intf, PMDOPSetCaptureSource, source);
}

PMDCFunc PMDGetCaptureSource(PMDAxisInterface axis_intf, PMDuint16* source)
{
	return SendCommandGetWord(axis_intf, PMDOPGetCaptureSource, source);
}

PMDCFunc PMDSetEncoderSource(PMDAxisInterface axis_intf, PMDuint16 source)
{
	return SendCommandWord(axis_intf, PMDOPSetEncoderSource, source);
}

PMDCFunc PMDGetEncoderSource(PMDAxisInterface axis_intf, PMDuint16* source)
{
	return SendCommandGetWord(axis_intf, PMDOPGetEncoderSource, source);
}

PMDCFunc PMDSetAuxiliaryEncoderSource(PMDAxisInterface axis_intf, PMDuint8 mode, PMDAxis auxillaryAxis)
{
	return SendCommandWord(axis_intf, PMDOPSetAuxiliaryEncoderSource,(PMDuint16)(((PMDuint16)mode<<8) | auxillaryAxis));
}

PMDCFunc PMDGetAuxiliaryEncoderSource(PMDAxisInterface axis_intf, PMDuint8* mode, PMDAxis* auxillaryAxis)
{
	PMDuint16 value;
	PMDresult result;

	result = SendCommandGetWord(axis_intf, PMDOPGetAuxiliaryEncoderSource, &value );
	*mode =(PMDuint8)(value>>8);
	*auxillaryAxis = value & 0x0f;

	return result;
}

PMDCFunc PMDSetEncoderModulus(PMDAxisInterface axis_intf, PMDuint16 modulus)
{
	return SendCommandWord(axis_intf, PMDOPSetEncoderModulus, modulus);
}

PMDCFunc PMDGetEncoderModulus(PMDAxisInterface axis_intf, PMDuint16* modulus)
{
	return SendCommandGetWord(axis_intf, PMDOPGetEncoderModulus, modulus);
}

PMDCFunc PMDSetEncoderToStepRatio(PMDAxisInterface axis_intf, PMDuint16 counts, PMDuint16 steps)
{
	return SendCommandWordWord(axis_intf, PMDOPSetEncoderToStepRatio, counts, steps);
}

PMDCFunc PMDGetEncoderToStepRatio(PMDAxisInterface axis_intf, PMDuint16* counts, PMDuint16* steps)
{
	return SendCommandGetWordWord(axis_intf, PMDOPGetEncoderToStepRatio, counts, steps);
}

PMDCFunc PMDGetActualVelocity(PMDAxisInterface axis_intf, PMDint32* velocity)
{
	return SendCommandGetLong(axis_intf, PMDOPGetActualVelocity,(PMDuint32*)velocity);
}

PMDCFunc PMDGetCaptureValue(PMDAxisInterface axis_intf, PMDint32* position)
{
	return SendCommandGetLong(axis_intf, PMDOPGetCaptureValue,(PMDuint32*)position);
}


///////////////////////////////////////////////////////////////////////////
// Motor
PMDCFunc PMDSetMotorType(PMDAxisInterface axis_intf, PMDuint16 type)
{
	return SendCommandWord(axis_intf, PMDOPSetMotorType, type);
}

PMDCFunc PMDGetMotorType(PMDAxisInterface axis_intf, PMDuint16* type)
{
	PMDuint16 value;
	PMDresult result;
	result = SendCommandGetWord(axis_intf, PMDOPGetMotorType, &value);
	*type = value;
	return result;
}

PMDCFunc PMDSetMotorCommand(PMDAxisInterface axis_intf, PMDint16 command)
{
	return SendCommandWord(axis_intf, PMDOPSetMotorCommand,(PMDuint16)command);
}

PMDCFunc PMDGetMotorCommand(PMDAxisInterface axis_intf, PMDint16* command)
{
	return SendCommandGetWord(axis_intf, PMDOPGetMotorCommand,(PMDuint16*)command);
}

PMDCFunc PMDGetActiveMotorCommand(PMDAxisInterface axis_intf, PMDint16* command)
{
	return SendCommandGetWord(axis_intf, PMDOPGetActiveMotorCommand,(PMDuint16*)command);
}


///////////////////////////////////////////////////////////////////////////
// Commutation
PMDCFunc PMDSetOutputMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetOutputMode, mode);
}

PMDCFunc PMDGetOutputMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetOutputMode, mode);
}

PMDCFunc PMDSetPWMFrequency(PMDAxisInterface axis_intf, PMDuint16 frequency)
{
	return SendCommandWord(axis_intf, PMDOPSetPWMFrequency, frequency);
}

PMDCFunc PMDGetPWMFrequency(PMDAxisInterface axis_intf, PMDuint16* frequency)
{
	return SendCommandGetWord(axis_intf, PMDOPGetPWMFrequency, frequency);
}

PMDCFunc PMDSetCommutationMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetCommutationMode, mode);
}

PMDCFunc PMDGetCommutationMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetCommutationMode, mode);
}

PMDCFunc PMDSetPhaseInitializeMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetPhaseInitializeMode, mode);
}

PMDCFunc PMDGetPhaseInitializeMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetPhaseInitializeMode, mode);
}

PMDCFunc PMDSetPhasePrescale(PMDAxisInterface axis_intf, PMDuint16 scale)
{
	return SendCommandWord(axis_intf, PMDOPSetPhasePrescale, scale);
}

PMDCFunc PMDGetPhasePrescale(PMDAxisInterface axis_intf, PMDuint16* scale)
{
	return SendCommandGetWord(axis_intf, PMDOPGetPhasePrescale, scale);
}

PMDCFunc PMDSetPhaseCounts(PMDAxisInterface axis_intf, PMDuint16 counts)
{
	return SendCommandWord(axis_intf, PMDOPSetPhaseCounts, counts);
}

PMDCFunc PMDGetPhaseCounts(PMDAxisInterface axis_intf, PMDuint16* counts)
{
	return SendCommandGetWord(axis_intf, PMDOPGetPhaseCounts, counts);
}

PMDCFunc PMDSetPhaseInitializeTime(PMDAxisInterface axis_intf, PMDuint16 time)
{
	return SendCommandWord(axis_intf, PMDOPSetPhaseInitializeTime, time);
}

PMDCFunc PMDGetPhaseInitializeTime(PMDAxisInterface axis_intf, PMDuint16* time)
{
	return SendCommandGetWord(axis_intf, PMDOPGetPhaseInitializeTime, time);
}

PMDCFunc PMDSetPhaseOffset(PMDAxisInterface axis_intf, PMDuint16 offset)
{
	return SendCommandWord(axis_intf, PMDOPSetPhaseOffset, offset);
}

PMDCFunc PMDGetPhaseOffset(PMDAxisInterface axis_intf, PMDuint16* offset)
{
	return SendCommandGetWord(axis_intf, PMDOPGetPhaseOffset, offset);
}

PMDCFunc PMDSetPhaseAngle(PMDAxisInterface axis_intf, PMDuint16 angle)
{
	return SendCommandWord(axis_intf, PMDOPSetPhaseAngle, angle);
}

PMDCFunc PMDGetPhaseAngle(PMDAxisInterface axis_intf, PMDuint16* angle)
{
	return SendCommandGetWord(axis_intf, PMDOPGetPhaseAngle, angle);
}

PMDCFunc PMDSetPhaseCorrectionMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetPhaseCorrectionMode, mode);
}

PMDCFunc PMDGetPhaseCorrectionMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetPhaseCorrectionMode, mode);
}

PMDCFunc PMDInitializePhase(PMDAxisInterface axis_intf)
{
	return SendCommand(axis_intf, PMDOPInitializePhase);
}

PMDCFunc PMDGetPhaseCommand(PMDAxisInterface axis_intf, PMDuint16 phase, PMDint16* command)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetPhaseCommand, phase,(PMDuint16*)command);
}


///////////////////////////////////////////////////////////////////////////
// External Memory
PMDCFunc PMDSetBufferStart(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32 address)
{
	return SendCommandWordLong(axis_intf, PMDOPSetBufferStart, bufferID, address);
}

PMDCFunc PMDGetBufferStart(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32* address)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetBufferStart, bufferID, address);
}

PMDCFunc PMDSetBufferLength(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32 length)
{
	return SendCommandWordLong(axis_intf, PMDOPSetBufferLength, bufferID, length);
}

PMDCFunc PMDGetBufferLength(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32* length)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetBufferLength, bufferID, length);
}

PMDCFunc PMDWriteBuffer(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDint32 data)
{
	return SendCommandWordLong(axis_intf, PMDOPWriteBuffer, bufferID,(PMDuint32)data);
}

PMDCFunc PMDReadBuffer(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDint32* data)
{
	return SendCommandWordGetLong(axis_intf, PMDOPReadBuffer, bufferID,(PMDuint32*)data);
}

PMDCFunc PMDSetBufferWriteIndex(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32 index)
{
	return SendCommandWordLong(axis_intf, PMDOPSetBufferWriteIndex, bufferID, index);
}

PMDCFunc PMDGetBufferWriteIndex(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32* index)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetBufferWriteIndex, bufferID, index);
}

PMDCFunc PMDSetBufferReadIndex(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32 index)
{
	return SendCommandWordLong(axis_intf, PMDOPSetBufferReadIndex, bufferID, index);
}

PMDCFunc PMDGetBufferReadIndex(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32* index)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetBufferReadIndex, bufferID, index);
}


///////////////////////////////////////////////////////////////////////////
// Trace Operations
PMDCFunc PMDSetTraceMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return	SendCommandWord(axis_intf, PMDOPSetTraceMode, mode);
}

PMDCFunc PMDGetTraceMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return	SendCommandGetWord(axis_intf, PMDOPGetTraceMode, mode);
}

PMDCFunc PMDSetTracePeriod(PMDAxisInterface axis_intf, PMDuint16 period)
{
	return	SendCommandWord(axis_intf, PMDOPSetTracePeriod, period);
}

PMDCFunc PMDGetTracePeriod(PMDAxisInterface axis_intf, PMDuint16* period)
{
	return	SendCommandGetWord(axis_intf, PMDOPGetTracePeriod, period);
}

PMDCFunc PMDSetTraceVariable(PMDAxisInterface axis_intf, PMDuint16 variableNumber, PMDAxis traceAxis, PMDuint8 variable)
{
	PMDuint16 value;
	value =(PMDuint16)((traceAxis & nibbleMask) |((variable & byteMask) << 8));
	return	SendCommandWordWord(axis_intf, PMDOPSetTraceVariable, variableNumber, value);
}

PMDCFunc PMDGetTraceVariable(PMDAxisInterface axis_intf, PMDuint16 variableNumber, PMDAxis *traceAxis, PMDuint8* variable)
{
	PMDresult result;
	PMDuint16 value;
	result = SendCommandWordGetWord(axis_intf, PMDOPGetTraceVariable, variableNumber, &value);
	*traceAxis =(PMDAxis)(value & nibbleMask);
	*variable =(PMDuint8)((value >> 8) & byteMask);
	return result;
}

PMDCFunc PMDSetTraceStart(PMDAxisInterface axis_intf, PMDAxis triggerAxis, PMDuint8 condition, PMDuint8 bit, PMDuint8 state)
{
	PMDuint16 value =(PMDuint16)((triggerAxis & nibbleMask) |((condition & nibbleMask) << 4));
	value |=((bit & nibbleMask) << 8) |((state & nibbleMask) << 12);

	return	SendCommandWord(axis_intf, PMDOPSetTraceStart, value);
}

PMDCFunc PMDGetTraceStart(PMDAxisInterface axis_intf, PMDAxis *triggerAxis, PMDuint8* condition, PMDuint8* bit, PMDuint8* state)
{
	PMDresult result;
	PMDuint16 value;
	result = SendCommandGetWord(axis_intf, PMDOPGetTraceStart, &value);
	*triggerAxis =(PMDAxis)((value & nibbleMask));
	*condition =(PMDuint8)((value >> 4) & nibbleMask);
	*bit =(PMDuint8)((value >> 8) & nibbleMask);
	*state =(PMDuint8)((value >> 12) & nibbleMask);

	return result;
}

PMDCFunc PMDSetTraceStop(PMDAxisInterface axis_intf, PMDAxis triggerAxis, PMDuint8 condition, PMDuint8 bit, PMDuint8 state)
{
	PMDuint16 value =(PMDuint16)((triggerAxis & nibbleMask) |((condition & nibbleMask) << 4));
	value |=((bit & nibbleMask) << 8) |((state & nibbleMask) << 12);

	return	SendCommandWord(axis_intf, PMDOPSetTraceStop, value);
}

PMDCFunc PMDGetTraceStop(PMDAxisInterface axis_intf, PMDAxis *triggerAxis, PMDuint8* condition, PMDuint8* bit, PMDuint8* state)
{
	PMDresult result;
	PMDuint16 value;
	result = SendCommandGetWord(axis_intf, PMDOPGetTraceStop, &value);
	*triggerAxis =(PMDAxis)((value & nibbleMask));
	*condition =(PMDuint8)((value >> 4) & nibbleMask);
	*bit =(PMDuint8)((value >> 8) & nibbleMask);
	*state =(PMDuint8)((value >> 12) & nibbleMask);

	return result;
}


PMDCFunc PMDGetTraceStatus(PMDAxisInterface axis_intf, PMDuint16* status)
{
	return	SendCommandGetWord(axis_intf, PMDOPGetTraceStatus, status);
}

PMDCFunc PMDGetTraceCount(PMDAxisInterface axis_intf, PMDuint32* count)
{
	return	SendCommandGetLong(axis_intf, PMDOPGetTraceCount, count);
}

PMDCFunc PMDGetTraceValue(PMDAxisInterface axis_intf, PMDuint8 variable, PMDint32* value)
{
	return	SendCommandWordGetLong(axis_intf, PMDOPGetTraceValue, variable,(PMDuint32*)value);
}

///////////////////////////////////////////////////////////////////////////
// Miscellaneous

PMDCFunc PMDWriteIO(PMDAxisInterface axis_intf, PMDuint16 address, PMDuint16 data)
{
	return SendCommandWordWord(axis_intf, PMDOPWriteIO, address, data);
}

PMDCFunc PMDReadIO(PMDAxisInterface axis_intf, PMDuint16 address, PMDuint16* data)
{
	return SendCommandWordGetWord(axis_intf, PMDOPReadIO, address, data);
}

PMDCFunc PMDReadAnalog(PMDAxisInterface axis_intf, PMDuint16 portID, PMDuint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPReadAnalog, portID, value);
}

PMDCFunc PMDReset(PMDAxisInterface axis_intf)
{
	PMDresult result;

	result = SendCommand(axis_intf, PMDOPReset);

	return result;
}

PMDCFunc PMDNoOperation(PMDAxisInterface axis_intf)
{
	return SendCommand(axis_intf, PMDOPNoOperation);
}

PMDCFunc PMDGetVersion(PMDAxisInterface axis_intf, PMDuint16 *family, PMDuint16 *motorType,
						PMDuint16 *numberAxes, PMDuint16 *special,
						PMDuint16 *custom, PMDuint16 *major, PMDuint16 *minor)
{
	PMDuint16 chip = 0, software = 0;
	PMDresult result;

	result = SendCommandGetWordWord(axis_intf, PMDOPGetVersion, &chip, &software);

	*family =(PMDuint16)((chip & 0xF000) >> 12);
	*motorType	=(PMDuint16)((chip & 0x0F00) >> 8);
	*numberAxes =(PMDuint16)((chip & 0x00F0) >> 4);
	*special =(PMDuint16)(chip & 0x000F);

	*custom =(PMDuint16)((software & 0xFF00) >> 8);
	*major	=(PMDuint16)((software & 0x00F0) >> 4);
	*minor	=(PMDuint16)(software & 0x000F);

	return result;
}

PMDCFunc PMDGetVersion32(PMDAxisInterface axis_intf, PMDuint32 *version)
{
	return SendCommandGetLong(axis_intf, PMDOPGetVersion, version);
}

PMDCFunc PMDGetInstructionError(PMDAxisInterface axis_intf, PMDuint16 *error)
{
	return SendCommandGetWord(axis_intf, PMDOPGetInstructionError, error);
}

PMDCFunc PMDSetSerialPortMode(PMDAxisInterface axis_intf, PMDuint8 baud, PMDuint8 parity,
					PMDuint8 stopBits, PMDuint8 protocol, PMDuint8 multiDropID)
{
	PMDuint16 serialData;

	serialData =(PMDuint16)(baud |(parity << 4) |(stopBits << 6) |
		(protocol << 7) |
		// bits 9 and 10 are reserved
		(multiDropID << 11));
	return SendCommandWord(axis_intf, PMDOPSetSerialPortMode, serialData);
}

PMDCFunc PMDGetSerialPortMode(PMDAxisInterface axis_intf, PMDuint8* baud, PMDuint8* parity,
					PMDuint8* stopBits, PMDuint8* protocol, PMDuint8* multiDropID)
{
	PMDuint16 serialData;
	PMDresult result;

	result = SendCommandGetWord(axis_intf, PMDOPGetSerialPortMode, &serialData);

	*baud =(PMDuint8)(serialData & 0x000F);
	*parity =(PMDuint8)((serialData & 0x0030) >> 4);
	*stopBits =(PMDuint8)((serialData & 0x0040) >> 6);
	*protocol =(PMDuint8)((serialData & 0x0180) >> 7);
	*multiDropID =(PMDuint8)((serialData & 0xF800) >> 11);

	return result;
}

PMDCFunc PMDSetCANMode(PMDAxisInterface axis_intf, PMDuint8 nodeID, PMDuint8 transmission_rate)
{
	PMDuint16 canData;

	canData =(PMDuint16)((transmission_rate << 13) | nodeID );

	return SendCommandWord(axis_intf, PMDOPSetCANMode, canData);
}

PMDCFunc PMDGetCANMode(PMDAxisInterface axis_intf, PMDuint8* nodeID, PMDuint8* transmission_rate)
{
	PMDuint16 canData;
	PMDresult result;

	result = SendCommandGetWord(axis_intf, PMDOPGetCANMode, &canData);

	*transmission_rate =(PMDuint8)((canData & 0xE000) >> 13);
	*nodeID =(PMDuint8)(canData & 0x007F);

	return result;
}

PMDCFunc PMDSetSPIMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetSPIMode, mode);
}

PMDCFunc PMDGetSPIMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetSPIMode, mode);
}

PMDCFunc PMDGetTime(PMDAxisInterface axis_intf, PMDuint32* time)
{
	return SendCommandGetLong(axis_intf, PMDOPGetTime, time);
}

PMDCFunc PMDGetChecksum(PMDAxisInterface axis_intf, PMDuint32* checksum)
{
	return SendCommandGetLong(axis_intf, PMDOPGetChecksum, checksum);
}

PMDCFunc PMDSetStepRange(PMDAxisInterface axis_intf, PMDuint16 range)
{
	return SendCommandWord(axis_intf, PMDOPSetStepRange, range);
}

PMDCFunc PMDGetStepRange(PMDAxisInterface axis_intf, PMDuint16* range)
{
	return SendCommandGetWord(axis_intf, PMDOPGetStepRange, range);
}

PMDCFunc PMDSetSynchronizationMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetSynchronizationMode, mode);
}

PMDCFunc PMDGetSynchronizationMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetSynchronizationMode, mode);
}


// ION specific functions
PMDCFunc PMDGetDriveStatus(PMDAxisInterface axis_intf, PMDuint16* status)
{
	return SendCommandGetWord(axis_intf, PMDOPGetDriveStatus, status);
}

PMDCFunc PMDSetBreakpointUpdateMask(PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDuint16 mask)
{
	return SendCommandWordWord(axis_intf, PMDOPSetBreakpointUpdateMask, breakpointID, mask);
}

PMDCFunc PMDGetBreakpointUpdateMask(PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDuint16* mask)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetBreakpointUpdateMask, breakpointID, mask);
}

PMDCFunc PMDSetPositionLoop(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint32 value)
{
	return SendCommandWordLong(axis_intf, PMDOPSetPositionLoop, parameter, value);
}

PMDCFunc PMDGetPositionLoop(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetPositionLoop, parameter,(PMDuint32*)value);
}

PMDCFunc PMDGetPositionLoopValue(PMDAxisInterface axis_intf, PMDuint16 node, PMDint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetPositionLoopValue, node,(PMDuint32*)value);
}

PMDCFunc PMDSetOperatingMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetOperatingMode, mode);
}

PMDCFunc PMDGetOperatingMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetOperatingMode, mode);
}

PMDCFunc PMDGetActiveOperatingMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetActiveOperatingMode, mode);
}

PMDCFunc PMDRestoreOperatingMode(PMDAxisInterface axis_intf)
{
	return SendCommand(axis_intf, PMDOPRestoreOperatingMode);
}

PMDCFunc PMDSetCurrentFoldback(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPSetCurrentFoldback, parameter, value);
}

PMDCFunc PMDGetCurrentFoldback(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetCurrentFoldback, parameter, value);
}
// PMDSetHoldingCurrent has been replaced with PMDSetCurrent but is left here for compatibility
PMDCFunc PMDSetHoldingCurrent(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPSetCurrent, parameter, value);
}
// PMDGetHoldingCurrent has been replaced with PMDGetCurrent but is left here for compatibility
PMDCFunc PMDGetHoldingCurrent(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetCurrent, parameter, value);
}

PMDCFunc PMDSetCurrentControlMode(PMDAxisInterface axis_intf, PMDuint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetCurrentControlMode, mode);
}

PMDCFunc PMDGetCurrentControlMode(PMDAxisInterface axis_intf, PMDuint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetCurrentControlMode, mode);
}

PMDCFunc PMDSetAxisOutMask(PMDAxisInterface axis_intf, PMDAxis sourceAxis, PMDuint8 sourceRegister, PMDuint16 selectionMask, PMDuint16 senseMask)
{
	PMDuint16 value =(PMDuint16)((sourceAxis & nibbleMask) |((sourceRegister & nibbleMask) << 8));

	return SendCommandWordWordWord(axis_intf, PMDOPSetAxisOutMask, value, selectionMask, senseMask);
}

PMDCFunc PMDGetAxisOutMask(PMDAxisInterface axis_intf, PMDAxis *sourceAxis, PMDuint8* sourceRegister, PMDuint16* selectionMask, PMDuint16* senseMask)
{
	PMDresult result;
	PMDuint16 value;

	result = SendCommandGetWordWordWord(axis_intf, PMDOPGetAxisOutMask, &value, selectionMask, senseMask);

	*sourceAxis =(PMDAxis)(value & nibbleMask);

	value =(PMDuint16)(value >> 8);
	*sourceRegister =(PMDuint8)(value & nibbleMask);

	return result;
}

PMDCFunc PMDSetEventAction(PMDAxisInterface axis_intf, PMDuint16 eventid, PMDuint16 action)
{
	return SendCommandWordWord(axis_intf, PMDOPSetEventAction, eventid, action);
}

PMDCFunc PMDGetEventAction(PMDAxisInterface axis_intf, PMDuint16 eventid, PMDuint16* action)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetEventAction, eventid, action);
}
// SetBusVoltageLimits has been replaced with SetDriveFaultParameter but is left here for compatibility
PMDCFunc PMDSetBusVoltageLimits(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPSetDriveFaultParameter, parameter, value);
}
// GetBusVoltageLimits has been replaced with GetDriveFaultParameter but is left here for compatibility
PMDCFunc PMDGetBusVoltageLimits(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetDriveFaultParameter, parameter, value);
}

PMDCFunc PMDGetBusVoltage(PMDAxisInterface axis_intf, PMDuint16* value)
{
	return SendCommandGetWord(axis_intf, PMDOPGetBusVoltage, value);
}

PMDCFunc PMDSetOvertemperatureLimit(PMDAxisInterface axis_intf, PMDuint16 limit)
{
	return SendCommandWord(axis_intf, PMDOPSetOvertemperatureLimit, limit);
}

PMDCFunc PMDGetOvertemperatureLimit(PMDAxisInterface axis_intf, PMDuint16* limit)
{
	return SendCommandGetWord(axis_intf, PMDOPGetOvertemperatureLimit, limit);
}

PMDCFunc PMDGetTemperature(PMDAxisInterface axis_intf, PMDint16* temperature)
{
	return SendCommandGetWord(axis_intf, PMDOPGetTemperature,(PMDuint16*)temperature);
}

PMDCFunc PMDClearDriveFaultStatus(PMDAxisInterface axis_intf)
{
	return SendCommand(axis_intf, PMDOPClearDriveFaultStatus);
}

PMDCFunc PMDGetDriveFaultStatus(PMDAxisInterface axis_intf, PMDuint16* status)
{
	return SendCommandGetWord(axis_intf, PMDOPGetDriveFaultStatus, status);
}

PMDCFunc PMDSetUpdateMask(PMDAxisInterface axis_intf, PMDuint16 mask)
{
	return SendCommandWord(axis_intf, PMDOPSetUpdateMask, mask);
}

PMDCFunc PMDGetUpdateMask(PMDAxisInterface axis_intf, PMDuint16* mask)
{
	return SendCommandGetWord(axis_intf, PMDOPGetUpdateMask, mask);
}

PMDCFunc PMDSetCurrentLoop(PMDAxisInterface axis_intf, PMDuint8 phase, PMDuint8 parameter, PMDuint16 value)
{
	PMDuint16 selector =(PMDuint16)((phase << 8) | parameter );
	return SendCommandWordWord(axis_intf, PMDOPSetCurrentLoop, selector, value);
}

PMDCFunc PMDGetCurrentLoop(PMDAxisInterface axis_intf, PMDuint8 phase, PMDuint8 parameter, PMDuint16* value)
{
	PMDuint16 selector =(PMDuint16)((phase << 8) | parameter );
	return SendCommandWordGetWord(axis_intf, PMDOPGetCurrentLoop, selector, value);
}

PMDCFunc PMDGetCurrentLoopValue(PMDAxisInterface axis_intf, PMDuint8 phase, PMDuint8 node, PMDint32* value)
{
	PMDuint16 selector =(PMDuint16)((phase << 8) | node );
	return SendCommandWordGetLong(axis_intf, PMDOPGetCurrentLoopValue, selector, (PMDuint32*)value);
}

PMDCFunc PMDSetDefault(PMDAxisInterface axis_intf, PMDuint16 variable, PMDuint32 value)
{
	return SendCommandWordLong(axis_intf, PMDOPSetDefault, variable, value);
}

PMDCFunc PMDGetDefault(PMDAxisInterface axis_intf, PMDuint16 variable, PMDuint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetDefault, variable, value);
}

PMDCFunc PMDSetFOC(PMDAxisInterface axis_intf, PMDuint8 loop, PMDuint8 parameter, PMDuint16 value)
{
	PMDuint16 selector =(PMDuint16)((loop << 8) | parameter );
	return SendCommandWordWord(axis_intf, PMDOPSetFOC, selector, value);
}

PMDCFunc PMDGetFOC(PMDAxisInterface axis_intf, PMDuint8 loop, PMDuint8 parameter, PMDuint16* value)
{
	PMDuint16 selector =(PMDuint16)((loop << 8) | parameter );
	return SendCommandWordGetWord(axis_intf, PMDOPGetFOC, selector, value);
}

PMDCFunc PMDGetFOCValue(PMDAxisInterface axis_intf, PMDuint8 loop, PMDuint8 node, PMDint32* value)
{
	PMDuint16 selector =(PMDuint16)((loop << 8) | node );
	return SendCommandWordGetLong(axis_intf, PMDOPGetFOCValue, selector, (PMDuint32*)value);
}

PMDCFunc PMDSetFaultOutMask(PMDAxisInterface axis_intf, PMDuint16 mask)
{
	return SendCommandWord(axis_intf, PMDOPSetFaultOutMask, mask);
}

PMDCFunc PMDGetFaultOutMask(PMDAxisInterface axis_intf, PMDuint16* mask)
{
	return SendCommandGetWord(axis_intf, PMDOPGetFaultOutMask, mask);
}


///////////////////////////////////////////////////////////////////////////
// C-Motion version
PMDCFunc PMDGetCMotionVersion(PMDuint32* MajorVersion, PMDuint32* MinorVersion)
{
	*MajorVersion = CMOTION_MAJOR_VERSION;
	*MinorVersion = CMOTION_MINOR_VERSION;
return PMD_ERR_OK;
}

///////////////////////////////////////////////////////////////////////////
// Atlas and MC5x113 specific functions

PMDCFunc PMDDriveNVRAM(PMDAxisInterface axis_intf, PMDuint16 option, PMDuint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPDriveNVRAM, option, value);
}

PMDCFunc PMDReadBuffer16(PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDint16* data)
{
	return SendCommandWordGetWord(axis_intf, PMDOPReadBuffer16, bufferID,(PMDuint16*)data);
}

PMDCFunc PMDSetCurrent(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPSetCurrent, parameter, value);
}

PMDCFunc PMDGetCurrent(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetCurrent, parameter, value);
}

PMDCFunc PMDSetDriveFaultParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPSetDriveFaultParameter, parameter, value);
}

PMDCFunc PMDGetDriveFaultParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetDriveFaultParameter, parameter, value);
}

PMDCFunc PMDSetDrivePWM(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPSetDrivePWM, parameter, value);
}

PMDCFunc PMDGetDrivePWM(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetDrivePWM, parameter, value);
}

PMDCFunc PMDSetFeedbackParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint32 value)
{
	return SendCommandWordLong(axis_intf, PMDOPSetFeedbackParameter, parameter, value);
}

PMDCFunc PMDGetFeedbackParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetFeedbackParameter, parameter, value);
}


///////////////////////////////////////////////////////////////////////////
// MC5x113 and MC7x113 specific functions

PMDCFunc PMDSetAnalogCalibration(PMDAxisInterface axis_intf, PMDuint16 channel, PMDint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPSetAnalogCalibration, channel, value);
}

PMDCFunc PMDGetAnalogCalibration(PMDAxisInterface axis_intf, PMDuint16 channel, PMDint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetAnalogCalibration, channel, (PMDuint16*)value);
}

PMDCFunc PMDGetDriveValue(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDuint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetDriveValue, parameter, value);
}

PMDCFunc PMDCalibrateAnalog(PMDAxisInterface axis_intf, PMDuint16 option)
{
	return SendCommandWord(axis_intf, PMDOPCalibrateAnalog, option);
}


///////////////////////////////////////////////////////////////////////////
// MC7x113 specific functions

PMDCFunc PMDSetLoop(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint32 value)
{
	return SendCommandWordLong(axis_intf, PMDOPSetVelocityLoop, parameter, value);
}

PMDCFunc PMDGetLoop(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetVelocityLoop, parameter, (PMDuint32*)value);
}

PMDCFunc PMDGetLoopValue(PMDAxisInterface axis_intf, PMDuint16 node, PMDint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetVelocityLoopValue, node,(PMDuint32*)value);
}

PMDCFunc PMDSetProfileParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint32 value)
{
	return SendCommandWordLong(axis_intf, PMDOPSetProfileParameter, parameter, value);
}

PMDCFunc PMDGetProfileParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetProfileParameter, parameter,(PMDuint32*)value);
}

PMDCFunc PMDSetCommutationParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint32 value)
{
	return SendCommandWordLong(axis_intf, PMDOPSetCommutationParameter, parameter, value);
}

PMDCFunc PMDGetCommutationParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetCommutationParameter, parameter,(PMDuint32*)value);
}

PMDCFunc PMDSetPhaseParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint16 value)
{
	return SendCommandWordWord(axis_intf, PMDOPSetPhaseParameter, parameter, value);
}

PMDCFunc PMDGetPhaseParameter(PMDAxisInterface axis_intf, PMDuint16 parameter, PMDint16* value)
{
	return SendCommandWordGetWord(axis_intf, PMDOPGetPhaseParameter, parameter,(PMDuint16*)value);
}

PMDCFunc PMDSetDriveCommandMode(PMDAxisInterface axis_intf, PMDint16 mode)
{
	return SendCommandWord(axis_intf, PMDOPSetDriveCommandMode, mode);
}

PMDCFunc PMDGetDriveCommandMode(PMDAxisInterface axis_intf, PMDint16* mode)
{
	return SendCommandGetWord(axis_intf, PMDOPGetDriveCommandMode,(PMDuint16*)mode);
}

PMDCFunc PMDGetProductInfo(PMDAxisInterface axis_intf, PMDuint16 index, PMDuint32* value)
{
	return SendCommandWordGetLong(axis_intf, PMDOPGetProductInfo, index, value);
}

PMDCFunc PMDGetRuntimeError(PMDAxisInterface axis_intf, PMDuint16* error)
{
	return SendCommandGetWord(axis_intf, PMDOPGetRuntimeError, error);
}

PMDCFunc PMDSetCurrentLimit(PMDAxisInterface axis_intf, PMDuint16 limit)
{
	return SendCommandWord(axis_intf, PMDOPSetMotorLimit, limit);
}

PMDCFunc PMDGetCurrentLimit(PMDAxisInterface axis_intf, PMDuint16* limit)
{
	return SendCommandGetWord(axis_intf, PMDOPGetMotorLimit, limit);
}

// PMDExecutionControl is only valid when executed from NVRAM.
PMDCFunc PMDExecutionControl(PMDAxisInterface axis_intf, PMDuint16 option, PMDint32 value)
{
	return SendCommandWordLong(axis_intf, PMDOPExecutionControl, option, value);
}
