#ifndef _DEF_INC_CMotion
#define	_DEF_INC_CMotion

//
//  C-Motion.h -- C-Motion API
//
//  Performance Motion Devices, Inc.
//

#define CMOTION_MAJOR_VERSION	5
#define CMOTION_MINOR_VERSION	8

#if defined(__cplusplus)
extern "C" {
#endif

#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDdevice.h"
#include "PMDtrans.h"

// C-Motion provided functions
PMDCFunc PMDGetCMotionVersion(PMDuint32* MajorVersion, PMDuint32* MinorVersion);


// Profile Generation
PMDCFunc PMDSetProfileMode            (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetProfileMode            (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDSetPosition               (PMDAxisInterface axis_intf, PMDint32 position);
PMDCFunc PMDGetPosition               (PMDAxisInterface axis_intf, PMDint32* position);
PMDCFunc PMDSetVelocity               (PMDAxisInterface axis_intf, PMDint32 velocity);
PMDCFunc PMDGetVelocity               (PMDAxisInterface axis_intf, PMDint32* velocity);
PMDCFunc PMDSetStartVelocity          (PMDAxisInterface axis_intf, PMDuint32 velocity);
PMDCFunc PMDGetStartVelocity          (PMDAxisInterface axis_intf, PMDuint32* velocity);
PMDCFunc PMDSetAcceleration           (PMDAxisInterface axis_intf, PMDuint32 acceleration);
PMDCFunc PMDGetAcceleration           (PMDAxisInterface axis_intf, PMDuint32* acceleration);
PMDCFunc PMDSetDeceleration           (PMDAxisInterface axis_intf, PMDuint32 deceleration);
PMDCFunc PMDGetDeceleration           (PMDAxisInterface axis_intf, PMDuint32* deceleration);
PMDCFunc PMDSetJerk                   (PMDAxisInterface axis_intf, PMDuint32 jerk);
PMDCFunc PMDGetJerk                   (PMDAxisInterface axis_intf, PMDuint32* jerk);
PMDCFunc PMDSetGearRatio              (PMDAxisInterface axis_intf, PMDint32 ratio);
PMDCFunc PMDGetGearRatio              (PMDAxisInterface axis_intf, PMDint32* ratio);
PMDCFunc PMDSetGearMaster             (PMDAxisInterface axis_intf, PMDAxis masterAxis, PMDuint16 source);
PMDCFunc PMDGetGearMaster             (PMDAxisInterface axis_intf, PMDAxis* masterAxis, PMDuint16* source);
PMDCFunc PMDSetStopMode               (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetStopMode               (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDGetCommandedPosition      (PMDAxisInterface axis_intf, PMDint32* position);
PMDCFunc PMDGetCommandedVelocity      (PMDAxisInterface axis_intf, PMDint32* velocity);
PMDCFunc PMDGetCommandedAcceleration  (PMDAxisInterface axis_intf, PMDint32* acceleration);


// Position Loop
PMDCFunc PMDSetMotorLimit             (PMDAxisInterface axis_intf, PMDuint16 limit);
PMDCFunc PMDGetMotorLimit             (PMDAxisInterface axis_intf, PMDuint16* limit);
PMDCFunc PMDSetMotorBias              (PMDAxisInterface axis_intf, PMDint16 bias);
PMDCFunc PMDGetMotorBias              (PMDAxisInterface axis_intf, PMDint16* bias);
PMDCFunc PMDSetPositionErrorLimit     (PMDAxisInterface axis_intf, PMDuint32 limit);
PMDCFunc PMDGetPositionErrorLimit     (PMDAxisInterface axis_intf, PMDuint32* limit);
PMDCFunc PMDSetSettleTime             (PMDAxisInterface axis_intf, PMDuint16 time);
PMDCFunc PMDGetSettleTime             (PMDAxisInterface axis_intf, PMDuint16* time);
PMDCFunc PMDSetSettleWindow           (PMDAxisInterface axis_intf, PMDuint16 window);
PMDCFunc PMDGetSettleWindow           (PMDAxisInterface axis_intf, PMDuint16* window);
PMDCFunc PMDSetTrackingWindow         (PMDAxisInterface axis_intf, PMDuint16 window);
PMDCFunc PMDGetTrackingWindow         (PMDAxisInterface axis_intf, PMDuint16* window);
PMDCFunc PMDSetMotionCompleteMode     (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetMotionCompleteMode     (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDClearPositionError        (PMDAxisInterface axis_intf);
PMDCFunc PMDGetPositionError          (PMDAxisInterface axis_intf, PMDint32* error);
PMDCFunc PMDSetSampleTime             (PMDAxisInterface axis_intf, PMDuint32 time);
PMDCFunc PMDGetSampleTime             (PMDAxisInterface axis_intf, PMDuint32* time);


// Parameter Update & Breakpoints
PMDCFunc PMDSetBreakpoint             (PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDAxis sourceAxis, PMDuint8 action, PMDuint8 trigger);
PMDCFunc PMDGetBreakpoint             (PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDAxis *sourceAxis, PMDuint8* action, PMDuint8* trigger);
PMDCFunc PMDSetBreakpointValue        (PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDint32 value);
PMDCFunc PMDGetBreakpointValue        (PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDint32* value);
PMDCFunc PMDSetBreakpointUpdateMask   (PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDuint16  mask);
PMDCFunc PMDGetBreakpointUpdateMask   (PMDAxisInterface axis_intf, PMDuint16 breakpointID, PMDuint16* mask);
PMDCFunc PMDUpdate                    (PMDAxisInterface axis_intf);
PMDCFunc PMDMultiUpdate               (PMDAxisInterface axis_intf, PMDuint16 mask);


// Interrupt Processing
PMDCFunc PMDSetInterruptMask          (PMDAxisInterface axis_intf, PMDuint16 mask);
PMDCFunc PMDGetInterruptMask          (PMDAxisInterface axis_intf, PMDuint16* mask);
PMDCFunc PMDClearInterrupt            (PMDAxisInterface axis_intf);
PMDCFunc PMDGetInterruptAxis          (PMDAxisInterface axis_intf, PMDuint16* mask);


// Status Register Control
PMDCFunc PMDResetEventStatus          (PMDAxisInterface axis_intf, PMDuint16 status);
PMDCFunc PMDGetEventStatus            (PMDAxisInterface axis_intf, PMDuint16* status);
PMDCFunc PMDGetActivityStatus         (PMDAxisInterface axis_intf, PMDuint16* status);
PMDCFunc PMDSetSignalSense            (PMDAxisInterface axis_intf, PMDuint16 sense);
PMDCFunc PMDGetSignalSense            (PMDAxisInterface axis_intf, PMDuint16* sense);
PMDCFunc PMDGetSignalStatus           (PMDAxisInterface axis_intf, PMDuint16* status);


// Encoder
PMDCFunc PMDAdjustActualPosition      (PMDAxisInterface axis_intf, PMDint32 position);
PMDCFunc PMDSetActualPosition         (PMDAxisInterface axis_intf, PMDint32 position);
PMDCFunc PMDGetActualPosition         (PMDAxisInterface axis_intf, PMDint32* position);
PMDCFunc PMDSetActualPositionUnits    (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetActualPositionUnits    (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDSetCaptureSource          (PMDAxisInterface axis_intf, PMDuint16 source);
PMDCFunc PMDGetCaptureSource          (PMDAxisInterface axis_intf, PMDuint16* source);
PMDCFunc PMDSetEncoderSource          (PMDAxisInterface axis_intf, PMDuint16 source);
PMDCFunc PMDGetEncoderSource          (PMDAxisInterface axis_intf, PMDuint16* source);
PMDCFunc PMDSetEncoderModulus         (PMDAxisInterface axis_intf, PMDuint16 modulus);
PMDCFunc PMDGetEncoderModulus         (PMDAxisInterface axis_intf, PMDuint16* modulus);
PMDCFunc PMDSetEncoderToStepRatio     (PMDAxisInterface axis_intf, PMDuint16 counts, PMDuint16 steps);
PMDCFunc PMDGetEncoderToStepRatio     (PMDAxisInterface axis_intf, PMDuint16* counts, PMDuint16* steps);
PMDCFunc PMDGetActualVelocity         (PMDAxisInterface axis_intf, PMDint32* velocity);
PMDCFunc PMDGetCaptureValue           (PMDAxisInterface axis_intf, PMDint32* position);
PMDCFunc PMDSetAuxiliaryEncoderSource (PMDAxisInterface axis_intf, PMDuint8 mode, PMDAxis auxillaryAxis);
PMDCFunc PMDGetAuxiliaryEncoderSource (PMDAxisInterface axis_intf, PMDuint8* mode, PMDAxis* auxillaryAxis);


// Motor
PMDCFunc PMDSetMotorType              (PMDAxisInterface axis_intf, PMDuint16 type);
PMDCFunc PMDGetMotorType              (PMDAxisInterface axis_intf, PMDuint16* type);
PMDCFunc PMDSetMotorCommand           (PMDAxisInterface axis_intf, PMDint16 motorCommand);
PMDCFunc PMDGetMotorCommand           (PMDAxisInterface axis_intf, PMDint16* motorCommand);
PMDCFunc PMDGetActiveMotorCommand     (PMDAxisInterface axis_intf, PMDint16* motorCommand); // same as PMDGetCurrentMotorCommand


// Commutation
PMDCFunc PMDSetOutputMode             (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetOutputMode             (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDSetPWMFrequency           (PMDAxisInterface axis_intf, PMDuint16 frequency);
PMDCFunc PMDGetPWMFrequency           (PMDAxisInterface axis_intf, PMDuint16* frequency);
PMDCFunc PMDSetCommutationMode        (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetCommutationMode        (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDSetPhaseInitializeMode    (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetPhaseInitializeMode    (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDSetPhasePrescale          (PMDAxisInterface axis_intf, PMDuint16 scale);
PMDCFunc PMDGetPhasePrescale          (PMDAxisInterface axis_intf, PMDuint16* scale);
PMDCFunc PMDSetPhaseCounts            (PMDAxisInterface axis_intf, PMDuint16 counts);
PMDCFunc PMDGetPhaseCounts            (PMDAxisInterface axis_intf, PMDuint16* counts);
PMDCFunc PMDSetPhaseInitializeTime    (PMDAxisInterface axis_intf, PMDuint16 time);
PMDCFunc PMDGetPhaseInitializeTime    (PMDAxisInterface axis_intf, PMDuint16* time);
PMDCFunc PMDSetPhaseOffset            (PMDAxisInterface axis_intf, PMDuint16 offset);
PMDCFunc PMDGetPhaseOffset            (PMDAxisInterface axis_intf, PMDuint16* offset);
PMDCFunc PMDSetPhaseAngle             (PMDAxisInterface axis_intf, PMDuint16 angle);
PMDCFunc PMDGetPhaseAngle             (PMDAxisInterface axis_intf, PMDuint16* angle);
PMDCFunc PMDSetPhaseCorrectionMode    (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetPhaseCorrectionMode    (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDInitializePhase           (PMDAxisInterface axis_intf);
PMDCFunc PMDGetPhaseCommand           (PMDAxisInterface axis_intf, PMDuint16 phase, PMDint16* command);


// External Memory
PMDCFunc PMDSetBufferStart            (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32 address);
PMDCFunc PMDGetBufferStart            (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32* address);
PMDCFunc PMDSetBufferLength           (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32 length);
PMDCFunc PMDGetBufferLength           (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32* length);
PMDCFunc PMDWriteBuffer               (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDint32 data);
PMDCFunc PMDReadBuffer                (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDint32* data);
PMDCFunc PMDSetBufferWriteIndex       (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32 index);
PMDCFunc PMDGetBufferWriteIndex       (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32* index);
PMDCFunc PMDSetBufferReadIndex        (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32 index);
PMDCFunc PMDGetBufferReadIndex        (PMDAxisInterface axis_intf, PMDuint16 bufferID, PMDuint32* index);


// Trace Operations
PMDCFunc PMDSetTraceMode              (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetTraceMode              (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDSetTracePeriod            (PMDAxisInterface axis_intf, PMDuint16 tracePeriod);
PMDCFunc PMDGetTracePeriod            (PMDAxisInterface axis_intf, PMDuint16 *tracePeriod);
PMDCFunc PMDSetTraceVariable          (PMDAxisInterface axis_intf, PMDuint16 variableNumber, PMDAxis traceAxis, PMDuint8 variable);
PMDCFunc PMDGetTraceVariable          (PMDAxisInterface axis_intf, PMDuint16 variableNumber, PMDAxis *traceAxis, PMDuint8* variable);
PMDCFunc PMDSetTraceStart             (PMDAxisInterface axis_intf, PMDAxis triggerAxis, PMDuint8 condition, PMDuint8 bit, PMDuint8 state);
PMDCFunc PMDGetTraceStart             (PMDAxisInterface axis_intf, PMDAxis *triggerAxis, PMDuint8* condition, PMDuint8* bit, PMDuint8* state);
PMDCFunc PMDSetTraceStop              (PMDAxisInterface axis_intf, PMDAxis triggerAxis, PMDuint8 condition, PMDuint8 bit, PMDuint8 state);
PMDCFunc PMDGetTraceStop              (PMDAxisInterface axis_intf, PMDAxis *triggerAxis, PMDuint8* condition, PMDuint8* bit, PMDuint8* state);
PMDCFunc PMDGetTraceStatus            (PMDAxisInterface axis_intf, PMDuint16* status);
PMDCFunc PMDGetTraceCount             (PMDAxisInterface axis_intf, PMDuint32* count);
PMDCFunc PMDGetTraceValue             (PMDAxisInterface axis_intf, PMDuint8 variable, PMDint32* value);


// Miscellaneous
PMDCFunc PMDWriteIO                   (PMDAxisInterface axis_intf, PMDuint16 address, PMDuint16 data);
PMDCFunc PMDReadIO                    (PMDAxisInterface axis_intf, PMDuint16 address, PMDuint16 *data);
PMDCFunc PMDReadAnalog                (PMDAxisInterface axis_intf, PMDuint16 portID, PMDuint16* value);
PMDCFunc PMDReset                     (PMDAxisInterface axis_intf);
PMDCFunc PMDNoOperation               (PMDAxisInterface axis_intf);
PMDCFunc PMDGetVersion                (PMDAxisInterface axis_intf, PMDuint16 *family, PMDuint16 *motorType, PMDuint16 *numberAxes, PMDuint16 *special, PMDuint16 *custom, PMDuint16 *major, PMDuint16 *minor);
PMDCFunc PMDGetVersion32              (PMDAxisInterface axis_intf, PMDuint32 *version);
PMDCFunc PMDGetInstructionError       (PMDAxisInterface axis_intf, PMDuint16 *instructionError); //was GetHostIOError

PMDCFunc PMDSetSerialPortMode         (PMDAxisInterface axis_intf, PMDuint8 baud, PMDuint8 parity, PMDuint8 stopBits, PMDuint8 protocol, PMDuint8 multiDropID);
PMDCFunc PMDGetSerialPortMode         (PMDAxisInterface axis_intf, PMDuint8* baud, PMDuint8* parity, PMDuint8* stopBits, PMDuint8* protocol, PMDuint8* multiDropID);
PMDCFunc PMDSetCANMode                (PMDAxisInterface axis_intf, PMDuint8 nodeID, PMDuint8 transmission_rate);
PMDCFunc PMDGetCANMode                (PMDAxisInterface axis_intf, PMDuint8* nodeID, PMDuint8* transmission_rate);

PMDCFunc PMDSetSPIMode                (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetSPIMode                (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDGetTime                   (PMDAxisInterface axis_intf, PMDuint32 *numberServoCycles);
PMDCFunc PMDGetChecksum               (PMDAxisInterface axis_intf, PMDuint32* checksum);
PMDCFunc PMDSetStepRange              (PMDAxisInterface axis_intf, PMDuint16 range);
PMDCFunc PMDGetStepRange              (PMDAxisInterface axis_intf, PMDuint16* range);
PMDCFunc PMDSetSynchronizationMode    (PMDAxisInterface axis_intf, PMDuint16 mode);
PMDCFunc PMDGetSynchronizationMode    (PMDAxisInterface axis_intf, PMDuint16* mode);


// ION and Atlas specific functions
PMDCFunc PMDGetDriveStatus            (PMDAxisInterface axis_intf, PMDuint16* status);
PMDCFunc PMDSetPositionLoop           (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32  value);
PMDCFunc PMDGetPositionLoop           (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32* value);
PMDCFunc PMDGetPositionLoopValue      (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32* value);
PMDCFunc PMDSetOperatingMode          (PMDAxisInterface axis_intf, PMDuint16  mode);
PMDCFunc PMDGetOperatingMode          (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDGetActiveOperatingMode    (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDRestoreOperatingMode      (PMDAxisInterface axis_intf);
PMDCFunc PMDSetCurrentFoldback        (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16  value);
PMDCFunc PMDGetCurrentFoldback        (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16* value);
PMDCFunc PMDSetHoldingCurrent         (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16  value);
PMDCFunc PMDGetHoldingCurrent         (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16* value);
PMDCFunc PMDSetCurrentControlMode     (PMDAxisInterface axis_intf, PMDuint16  mode);
PMDCFunc PMDGetCurrentControlMode     (PMDAxisInterface axis_intf, PMDuint16* mode);
PMDCFunc PMDSetAxisOutMask            (PMDAxisInterface axis_intf, PMDAxis    sourceAxis, PMDuint8 sourceRegister, PMDuint16 selectionMask, PMDuint16 senseMask);
PMDCFunc PMDGetAxisOutMask            (PMDAxisInterface axis_intf, PMDAxis*   sourceAxis, PMDuint8* sourceRegister, PMDuint16* selectionMask, PMDuint16* senseMask);
PMDCFunc PMDSetEventAction            (PMDAxisInterface axis_intf, PMDuint16  eventid, PMDuint16  action);
PMDCFunc PMDGetEventAction            (PMDAxisInterface axis_intf, PMDuint16  eventid, PMDuint16* action);
PMDCFunc PMDSetBusVoltageLimits       (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16  value);
PMDCFunc PMDGetBusVoltageLimits       (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16* value);
PMDCFunc PMDGetBusVoltage             (PMDAxisInterface axis_intf, PMDuint16* voltage);
PMDCFunc PMDSetOvertemperatureLimit   (PMDAxisInterface axis_intf, PMDuint16  limit);
PMDCFunc PMDGetOvertemperatureLimit   (PMDAxisInterface axis_intf, PMDuint16* limit);
PMDCFunc PMDGetTemperature            (PMDAxisInterface axis_intf, PMDint16*  temperature);
PMDCFunc PMDClearDriveFaultStatus     (PMDAxisInterface axis_intf);
PMDCFunc PMDGetDriveFaultStatus       (PMDAxisInterface axis_intf, PMDuint16* status);
PMDCFunc PMDSetUpdateMask             (PMDAxisInterface axis_intf, PMDuint16  mask);
PMDCFunc PMDGetUpdateMask             (PMDAxisInterface axis_intf, PMDuint16* mask);
PMDCFunc PMDSetCurrentLoop            (PMDAxisInterface axis_intf, PMDuint8   phase, PMDuint8 parameter, PMDuint16 value);
PMDCFunc PMDGetCurrentLoop            (PMDAxisInterface axis_intf, PMDuint8   phase, PMDuint8 parameter, PMDuint16* value);
PMDCFunc PMDGetCurrentLoopValue       (PMDAxisInterface axis_intf, PMDuint8   phase, PMDuint8 node, PMDint32* value);
PMDCFunc PMDSetDefault                (PMDAxisInterface axis_intf, PMDuint16  variable, PMDuint32 value);
PMDCFunc PMDGetDefault                (PMDAxisInterface axis_intf, PMDuint16  variable, PMDuint32* value);
PMDCFunc PMDSetFOC                    (PMDAxisInterface axis_intf, PMDuint8   loop, PMDuint8 parameter, PMDuint16  value);
PMDCFunc PMDGetFOC                    (PMDAxisInterface axis_intf, PMDuint8   loop, PMDuint8 parameter, PMDuint16* value);
PMDCFunc PMDGetFOCValue               (PMDAxisInterface axis_intf, PMDuint8   loop, PMDuint8 node, PMDint32* value);
PMDCFunc PMDSetFaultOutMask           (PMDAxisInterface axis_intf, PMDuint16  mask);
PMDCFunc PMDGetFaultOutMask           (PMDAxisInterface axis_intf, PMDuint16* mask);


// Atlas, MC5x113 and MC7x113 specific functions
PMDCFunc PMDDriveNVRAM                (PMDAxisInterface axis_intf, PMDuint16  option,    PMDuint16  value);
PMDCFunc PMDReadBuffer16              (PMDAxisInterface axis_intf, PMDuint16  bufferID,  PMDint16*  data);
PMDCFunc PMDSetCurrent                (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16  value);
PMDCFunc PMDGetCurrent                (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16* value);
PMDCFunc PMDSetDriveFaultParameter    (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16  value);
PMDCFunc PMDGetDriveFaultParameter    (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16* value);
PMDCFunc PMDSetDrivePWM               (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16  value);
PMDCFunc PMDGetDrivePWM               (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16* value);
PMDCFunc PMDSetFeedbackParameter      (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint32  value);
PMDCFunc PMDGetFeedbackParameter      (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint32* value);

// MC5x113 specific functions
PMDCFunc PMDSetAnalogCalibration      (PMDAxisInterface axis_intf, PMDuint16  channel,   PMDint16   value);
PMDCFunc PMDGetAnalogCalibration      (PMDAxisInterface axis_intf, PMDuint16  channel,   PMDint16*  value);
PMDCFunc PMDGetDriveValue             (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDuint16* value);
PMDCFunc PMDCalibrateAnalog           (PMDAxisInterface axis_intf, PMDuint16  option);

// MC7x113 specific functions
PMDCFunc PMDSetLoop                   (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32   value);
PMDCFunc PMDGetLoop                   (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32*  value);
PMDCFunc PMDGetLoopValue              (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32*  value);
PMDCFunc PMDSetProfileParameter       (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32   value);
PMDCFunc PMDGetProfileParameter       (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32*  value);
PMDCFunc PMDGetProductInfo            (PMDAxisInterface axis_intf, PMDuint16  index,     PMDuint32* value);
PMDCFunc PMDExecutionControl          (PMDAxisInterface axis_intf, PMDuint16  option,    PMDint32   value);
PMDCFunc PMDSetCommutationParameter   (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32   value);
PMDCFunc PMDGetCommutationParameter   (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint32*  value);
PMDCFunc PMDSetPhaseParameter         (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint16   value);
PMDCFunc PMDGetPhaseParameter         (PMDAxisInterface axis_intf, PMDuint16  parameter, PMDint16*  value);
PMDCFunc PMDSetDriveCommandMode       (PMDAxisInterface axis_intf, PMDint16   mode);
PMDCFunc PMDGetDriveCommandMode       (PMDAxisInterface axis_intf, PMDint16*  mode);
PMDCFunc PMDGetRuntimeError           (PMDAxisInterface axis_intf, PMDuint16* error);
PMDCFunc PMDSetCurrentLimit           (PMDAxisInterface axis_intf, PMDuint16  limit);
PMDCFunc PMDGetCurrentLimit           (PMDAxisInterface axis_intf, PMDuint16* limit);


#if defined(__cplusplus)
}
#endif

#endif






