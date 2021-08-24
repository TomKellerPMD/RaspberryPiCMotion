#ifndef PMD_CommonTypes
#define PMD_CommonTypes

//  PMDtypes.h -- common type declarations for Magellan-based products
//
//  Performance Motion Devices, Inc.
//

#include "PMDecode.h"


// Global definitions

// Macros to control the export of DLL entry points.
// The stdcall calling convention is used so that Visual Basic
// can call these procedures.

#if defined(_WIN32) && (defined(PMD_EXPORTS) || defined(PMD_IMPORTS))

# define PMD_CCONV __stdcall
# ifdef PMD_EXPORTS
// For the library build
#  define PMD_API __declspec(dllexport)
# else
// For library users
#  define PMD_API __declspec(dllimport)
# endif

#else

// For non-Windows builds
# define PMD_CCONV /**/
# define PMD_API extern
#endif

// This macro can be used as the return type for almost all user-callable library procedures.
#define PMDCFunc PMD_API PMDresult PMD_CCONV

#define PMD_MAX_AXES 4

#define PMDnull 0

#define PMDparam                PMDuint32


// common types
typedef long PMDint32;
typedef unsigned long  PMDuint32;
typedef long PMDlong32;
typedef unsigned short PMDuint16;
typedef short PMDint16;
typedef unsigned char  PMDuint8;
typedef PMDErrorCode PMDresult;


enum {nibbleMask = 0x0F, byteMask = 0xFF};

enum {PMDAtlasAxisMask = 0x20};

typedef enum {
        PMDAxis1 = 0,
        PMDAxis2 = 1,
        PMDAxis3 = 2,
        PMDAxis4 = 3,
        PMDAtlasAxis1 = PMDAxis1 + PMDAtlasAxisMask,
        PMDAtlasAxis2 = PMDAxis2 + PMDAtlasAxisMask,
        PMDAtlasAxis3 = PMDAxis3 + PMDAtlasAxisMask,
        PMDAtlasAxis4 = PMDAxis4 + PMDAtlasAxisMask
} PMDAxisNumber;

typedef PMDuint16 PMDAxis;


enum {
        PMDDisable                                              = 0,
        PMDEnable                                               = 1
}; // generic enable switch

typedef enum {
        PMDProductInfoVersion                                   = 1,
        PMDProductInfoProductClass                              = 2,
        PMDProductInfoChecksum                                  = 3,
        PMDProductInfoPartNumberWord1                           = 5,
        PMDProductInfoPartNumberWord2                           = 6,
        PMDProductInfoPartNumberWord3                           = 7,
        PMDProductInfoPartNumberWord4                           = 8,
} PMDProductInfo;

typedef enum {
        PMDFamilyFirstGen                                       = 1,
        PMDFamilyNavigator                                      = 2,
        PMDFamilyPilot                                          = 3,
        PMDFamilyMagellan                                       = 5,
        PMDFamilyMotorControl                                   = 7,
        PMDFamilyMagellanION                                    = 9
} PMDFamily;

typedef enum {
        PMDMotorTypeVersionBrushedServo                         = 1,
        PMDMotorTypeVersionBrushlessServo                       = 3,
        PMDMotorTypeVersionMicroStepping                        = 4,
        PMDMotorTypeVersionStepping                             = 5,
        PMDMotorTypeVersionAllMotor                             = 8,
        PMDMotorTypeVersionAnyMotor                             = 9
} PMDMotorTypeVersion;

typedef enum {
        PMDMotorTypeBrushlessDC3Phase                           = 0,
        PMDMotorTypeBrushlessDC2Phase                           = 1,
        PMDMotorTypeMicrostep3Phase                             = 2,
        PMDMotorTypeMicrostep2Phase                             = 3,
        PMDMotorTypeStepping                                    = 4,
        PMDMotorTypeDCBrush                                     = 7
} PMDMotorType;

// Profile Generation
typedef enum {
        PMDProfileModeTrapezoidal                               = 0,
        PMDProfileModeVelocity                                  = 1,
        PMDProfileModeSCurve                                    = 2,
        PMDProfileModeElectronicGear                            = 3
} PMDProfileMode;

typedef enum {
        PMDStopModeNone                                         = 0,
        PMDStopModeAbrupt                                       = 1,
        PMDStopModeSmooth                                       = 2
} PMDStopMode;

typedef enum {
        PMDMotionCompleteModeCommandedPosition                  = 0,
        PMDMotionCompleteModeActualPosition                     = 1
} PMDMotionCompleteMode;

typedef enum {
        PMDActualPositionUnitsCounts                            = 0,
        PMDActualPositionUnitsSteps                             = 1
} PMDActualPositionUnits;

typedef enum {
        PMDAuxiliaryEncoderModeNone                             = 0,
        PMDAuxiliaryEncoderModeQuadrature                       = 1,
        PMDAuxiliaryEncoderModePulseAndDirection                = 2
} PMDAuxiliaryEncoderMode;

typedef enum {
        PMDGearMasterSourceActual                               = 0,
        PMDGearMasterSourceCommanded                            = 1
} PMDGearMasterSource;

typedef enum {
        PMDSynchronizationModeDisabled                          = 0,
        PMDSynchronizationModeMaster                            = 1,
        PMDSynchronizationModeSlave                             = 2
} PMDSynchronizationMode;

// Parameter Update & Breakpoints
typedef enum {
        PMDBreakpoint1                                          = 0,
        PMDBreakpoint2                                          = 1
} PMDBreakpoint;

typedef enum {
        PMDBreakpointTriggerDisable                             = 0,
        PMDBreakpointTriggerGreaterOrEqualCommandedPosition     = 1,
        PMDBreakpointTriggerLessOrEqualCommandedPosition        = 2,
        PMDBreakpointTriggerGreaterOrEqualActualPosition        = 3,
        PMDBreakpointTriggerLessOrEqualActualPosition           = 4,
        PMDBreakpointTriggerCommandedPositionCrossed            = 5,
        PMDBreakpointTriggerActualPositionCrossed               = 6,
        PMDBreakpointTriggerTime                                = 7,
        PMDBreakpointTriggerEventStatus                         = 8,
        PMDBreakpointTriggerActivityStatus                      = 9,
        PMDBreakpointTriggerSignalStatus                        = 10,
        PMDBreakpointTriggerDriveStatus                         = 11
} PMDBreakpointTrigger;

typedef enum {
        PMDBreakpointActionNone                                 = 0,
        PMDBreakpointActionUpdate                               = 1,
        PMDBreakpointActionAbruptStop                           = 2,
        PMDBreakpointActionSmoothStop                           = 3,
        PMDBreakpointActionMotorOff                             = 4,
        PMDBreakpointActionDisablePositionLoopAndHigherModules  = 5,
        PMDBreakpointActionDisableCurrentLoopAndHigherModules   = 6,
        PMDBreakpointActionDisableMotorOutputAndHigherModules   = 7,
        PMDBreakpointActionAbruptStopWithPositionErrorClear     = 8
} PMDBreakpointAction;

// Status Register masks
typedef enum {
        PMDActivityStatusPhasingInitialized                     = 0x0001,
        PMDActivityStatusAtMaximumVelocity                      = 0x0002,
        PMDActivityStatusTracking                               = 0x0004,
        PMDActivityStatusAxisSettled                            = 0x0080,
        PMDActivityStatusMotorOn                                = 0x0100,
        PMDActivityStatusPositionCapture                        = 0x0200,
        PMDActivityStatusInMotion                               = 0x0400,
        PMDActivityStatusInPositiveLimit                        = 0x0800,
        PMDActivityStatusInNegativeLimit                        = 0x1000,
} PMDActivityStatus;

typedef enum {
        PMDEventStatusMotionComplete                            = 0x0001,
        PMDEventStatusWrapAround                                = 0x0002,
        PMDEventStatusBreakpoint1                               = 0x0004,
        PMDEventStatusCaptureReceived                           = 0x0008,
        PMDEventStatusMotionError                               = 0x0010,
        PMDEventStatusInPositiveLimit                           = 0x0020,
        PMDEventStatusInNegativeLimit                           = 0x0040,
        PMDEventStatusInstructionError                          = 0x0080,
        PMDEventStatusDriveDisabled                             = 0x0100,
        PMDEventStatusOvertemperatureFault                      = 0x0200,
        PMDEventStatusBusVoltageFault                           = 0x0400, // ION
        PMDEventStatusDriveException                            = 0x0400,
        PMDEventStatusCommutationError                          = 0x0800,
        PMDEventStatusCurrentFoldback                           = 0x1000,
        PMDEventStatusRuntimeError                              = 0x2000, // Juno
        PMDEventStatusBreakpoint2                               = 0x4000
} PMDEventStatus;

typedef enum {
        PMDSignalStatusEncoderA                                 = 0x0001,
        PMDSignalStatusEncoderB                                 = 0x0002,
        PMDSignalStatusEncoderIndex                             = 0x0004,
        PMDSignalStatusEncoderHome                              = 0x0008,
        PMDSignalStatusPositiveLimit                            = 0x0010,
        PMDSignalStatusNegativeLimit                            = 0x0020,
        PMDSignalStatusAxisIn                                   = 0x0040,
        PMDSignalStatusHallA                                    = 0x0080,
        PMDSignalStatusHallB                                    = 0x0100,
        PMDSignalStatusHallC                                    = 0x0200,
        PMDSignalStatusAxisOut                                  = 0x0400,
        PMDSignalStatusStepOutputInvert                         = 0x0800,
        PMDSignalStatusStepInputInvert                          = 0x0800, // Juno
        PMDSignalStatusMotorDirection                           = 0x1000,
        PMDSignalStatusEnableIn                                 = 0x2000,
        PMDSignalStatusFaultOut                                 = 0x4000,
        PMDSignalStatusStepDirectionInvert                      = 0x8000  // Juno
} PMDSignalStatus;

// Status Register bit positions
typedef enum {
        PMDActivityStatusBitPhasingInitialized                  = 0,
        PMDActivityStatusBitAtMaximumVelocity                   = 1,
        PMDActivityStatusBitTracking                            = 2,
        PMDActivityStatusBitAxisSettled                         = 7,
        PMDActivityStatusBitMotorMode                           = 8,
        PMDActivityStatusBitPositionCapture                     = 9,
        PMDActivityStatusBitInMotion                            = 10,
        PMDActivityStatusBitInPositiveLimit                     = 11,
        PMDActivityStatusBitInNegitiveLimit                     = 12
} PMDActivityStatusBit;

typedef enum {
        PMDEventStatusBitMotionComplete                         = 0,
        PMDEventStatusBitWrapAround                             = 1,
        PMDEventStatusBitBreakpoint1                            = 2,
        PMDEventStatusBitCaptureReceived                        = 3,
        PMDEventStatusBitMotionError                            = 4,
        PMDEventStatusBitPositiveLimit                          = 5,
        PMDEventStatusBitNegativeLimit                          = 6,
        PMDEventStatusBitInstructionError                       = 7,
        PMDEventStatusBitDriveDisabled                          = 8,
        PMDEventStatusBitOvertemperatureFault                   = 9,
        PMDEventStatusBitBusVoltageFault                        = 10, // ION
        PMDEventStatusBitDriveException                         = 10,
        PMDEventStatusBitCommutationError                       = 11,
        PMDEventStatusBitCurrentFoldback                        = 12,
        PMDEventStatusBitRuntimeError                           = 13,
        PMDEventStatusBitBreakpoint2                            = 14
} PMDEventStatusBit;

typedef enum {
        PMDSignalStatusBitEncoderA                              = 0,
        PMDSignalStatusBitEncoderB                              = 1,
        PMDSignalStatusBitEncoderIndex                          = 2,
        PMDSignalStatusBitEncoderHome                           = 3,
        PMDSignalStatusBitPositiveLimit                         = 4,
        PMDSignalStatusBitNegativeLimit                         = 5,
        PMDSignalStatusBitAxisIn                                = 6,
        PMDSignalStatusBitHallA                                 = 7,
        PMDSignalStatusBitHallB                                 = 8,
        PMDSignalStatusBitHallC                                 = 9,
        PMDSignalStatusBitAxisOut                               = 10,
        PMDSignalStatusBitStepOutput                            = 11,
        PMDSignalStatusBitMotorDirection                        = 12,
        PMDSignalStatusBitEnableIn                              = 13,
        PMDSignalStatusBitFaultOut                              = 14
} PMDSignalStatusBit;

// Encoder
typedef enum {
        PMDCaptureSourceIndex                                   = 0,
        PMDCaptureSourceHome                                    = 1,
        PMDCaptureSourceHSI                                     = 2
} PMDCaptureSource;

typedef enum {
        PMDEncoderSourceIncremental                             = 0,
        PMDEncoderSourceParallel                                = 1,
        PMDEncoderSourceNone                                    = 2,
        PMDEncoderSourceLoopback                                = 3,
        PMDEncoderSourcePulseAndDirection                       = 4,
        PMDEncoderSourceHall                                    = 5,
        PMDEncoderSourceParallel32                              = 6,
        PMDEncoderSourceAnalogHall                              = 7 
} PMDEncoderSource;

// Motor
typedef enum {
        PMDOutputModeBipolarDAC                                 = 0,
        PMDOutputModePWMSignMagnitude                           = 1,
        PMDOutputModePWM5050Magnitude                           = 2,
        PMDOutputModeSPIDACOffsetBinary                         = 3,
        PMDOutputModeUnipolarDAC                                = 4,
        PMDOutputModeSPIDAC2sComplement                         = 5,
        PMDOutputModeAtlas                                      = 6,
        PMDOutputModePWMHighLow                                 = 7,
        PMDOutputModePulseAndDirection                          = 8,
        PMDOutputModeAtlasRecovery                              = 9,
        PMDOutputModeNone                                       = 10
} PMDOutputMode;

// Commutation
typedef enum {
        PMDCommutationModeSinusoidal                            = 0,
        PMDCommutationModeHallBased                             = 1
} PMDCommutationMode;

typedef enum {
        PMDPhaseInitializeModeAlgorithmic                       = 0,
        PMDPhaseInitializeModeHallBased                         = 1,
        PMDPhaseInitializeModePulse                             = 2 
} PMDPhaseInitializeMode;

typedef enum {
        PMDPhaseCorrectionModeDisabled                          = 0,
        PMDPhaseCorrectionModeIndex                             = 1,
        PMDPhaseCorrectionModeHall                              = 2 
} PMDPhaseCorrectionMode;

typedef enum {
        PMDPhasePrescaleOff                                     = 0,
        PMDPhasePrescale64                                      = 1,
        PMDPhasePrescale128                                     = 2,
        PMDPhasePrescale256                                     = 3
} PMDPhasePrescale;

typedef enum {
        PMDPhaseA                                               = 0,
        PMDPhaseB                                               = 1,
        PMDPhaseC                                               = 2
} PMDPhase;

// Trace Operations
typedef enum {
        PMDTraceVariableNumber1                                 = 0,
        PMDTraceVariableNumber2                                 = 1,
        PMDTraceVariableNumber3                                 = 2,
        PMDTraceVariableNumber4                                 = 3
} PMDTraceVariableNumber;

typedef enum {
        PMDTraceModeOneTime                                     = 0,
        PMDTraceModeRollingBuffer                               = 1,
        PMDTraceModeExternalTrigger                             = 0x0100
} PMDTraceMode;

typedef enum {
        PMDTraceVariableNone                                    = 0,
        PMDTraceVariablePositionError                           = 1,
        PMDTraceVariableCommandedPosition                       = 2,
        PMDTraceVariableCommandedVelocity                       = 3,
        PMDTraceVariableCommandedAcceleration                   = 4,
        PMDTraceVariableActualPosition                          = 5,
        PMDTraceVariableActualVelocity                          = 6,
        PMDTraceVariableActiveMotorCommand                      = 7,
        PMDTraceVariableMotionProcessorTime                     = 8,
        PMDTraceVariableCaptureRegister                         = 9,
        PMDTraceVariableActualVelocity32                        = 83,
        PMDTraceVariableRawEncoderValue                         = 84,

        PMDTraceVariablePositionLoopIntegralSum                 = 10,
        PMDTraceVariablePositionLoopIntegralContribution        = 57,
        PMDTraceVariablePositionLoopDerivative                  = 11,
        PMDTraceVariablePIDOutput                               = 64,
        PMDTraceVariableBiquad1Output                           = 65,

        PMDTraceVariableEventStatusRegister                     = 12,
        PMDTraceVariableActivityStatusRegister                  = 13,
        PMDTraceVariableSignalStatusRegister                    = 14,
        PMDTraceVariableDriveStatusRegister                     = 56,
        PMDTraceVariableDriveFaultStatusRegister                = 79,
        PMDTraceVariableAtlasSPIStatus                          = 80,

        PMDTraceVariablePhaseAngle                              = 15,
        PMDTraceVariablePhaseOffset                             = 16,
        PMDTraceVariablePhaseACommand                           = 17,
        PMDTraceVariablePhaseBCommand                           = 18,
        PMDTraceVariablePhaseCCommand                           = 19,

        PMDTraceVariableAnalogInput0                            = 20,
        PMDTraceVariableAnalogInput1                            = 21,
        PMDTraceVariableAnalogInput2                            = 22,
        PMDTraceVariableAnalogInput3                            = 23,
        PMDTraceVariableAnalogInput4                            = 24,
        PMDTraceVariableAnalogInput5                            = 25,
        PMDTraceVariableAnalogInput6                            = 26,
        PMDTraceVariableAnalogInput7                            = 27,
        PMDTraceVariablePhaseAngleScaled                        = 29,

        PMDTraceVariableCurrentLoopAReference                   = 66,// 42h
        PMDTraceVariableCurrentLoopAError                       = 30,
        PMDTraceVariableCurrentLoopActualCurrentA               = 31,
        PMDTraceVariableCurrentLoopAIntegratorSum               = 32,
        PMDTraceVariableCurrentLoopAIntegralContribution        = 33,
        PMDTraceVariableCurrentLoopAOutput                      = 34,

        PMDTraceVariableCurrentLoopBReference                   = 67,// 43h
        PMDTraceVariableCurrentLoopBError                       = 35,
        PMDTraceVariableCurrentLoopActualCurrentB               = 36,
        PMDTraceVariableCurrentLoopBIntegratorSum               = 37,
        PMDTraceVariableCurrentLoopBIntegralContribution        = 38,
        PMDTraceVariableCurrentLoopBOutput                      = 39,

        PMDTraceVariableFOCDReference                           = 40,// 28h
        PMDTraceVariableFOCDError                               = 41,
        PMDTraceVariableFOCDFeedback                            = 42,
        PMDTraceVariableFOCDIntegratorSum                       = 43,
        PMDTraceVariableFOCDIntegralContribution                = 44,
        PMDTraceVariableFOCDOutput                              = 45,

        PMDTraceVariableFOCQReference                           = 46,// 2Eh
        PMDTraceVariableFOCQError                               = 47,
        PMDTraceVariableFOCQFeedback                            = 48,
        PMDTraceVariableFOCQIntegratorSum                       = 49,
        PMDTraceVariableFOCQIntegralContribution                = 50,
        PMDTraceVariableFOCQOutput                              = 51,

        PMDTraceVariableFOCAlphaOutput                          = 52,// 34h
        PMDTraceVariableFOCBetaOutput                           = 53,
        PMDTraceVariableFOCActualCurrentA                       = 31,
        PMDTraceVariableFOCActualCurrentB                       = 36,

        PMDTraceVariableBusVoltage                              = 54,// 36h
        PMDTraceVariableTemperature                             = 55,
        PMDTraceVariableI2tEnergy                               = 68,
        // Atlas
        PMDTraceVariableLegCurrentA                             = 69,
        PMDTraceVariableLegCurrentB                             = 70,
        PMDTraceVariableLegCurrentC                             = 71,
        PMDTraceVariableLegCurrentD                             = 72,
        PMDTraceVariablePhaseCurrentAlpha                       = 73,
        PMDTraceVariablePhaseCurrentBeta                        = 74,
        PMDTraceVariablePWMOutputA                              = 75,
        PMDTraceVariablePWMOutputB                              = 76,
        PMDTraceVariablePWMOutputC                              = 77,

        //  Juno only
        PMDTraceVariableEstimatedVelocity                       = 95,
        PMDTraceVariableCommandedVelocityScaled                 = 96,
        PMDTraceVariableVelocityLoopError                       = 97,
        PMDTraceVariableVelocityLoopErrorIntegral               = 98,
        PMDTraceVariableVelocityLoopOutput                      = 99,
        PMDTraceVariableVelocityLoopBiQuad1Input                = 100,
        PMDTraceVariableVelocityLoopBiQuad2Input                = 101,
        PMDTraceVariableTachometerReading                       = 102,
        PMDTraceVariableAnalogCommandReading                    = 103,
        PMDTraceVariableIntegratedPositionLoopOutput            = 104,
        PMDTraceVariableRawSPICommand                           = 105,
        PMDTraceVariableOuterLoopReference                      = 117,
        PMDTraceVariableOuterLoopFeedback                       = 118,
} PMDTraceVariable;

typedef enum {
        PMDTraceConditionImmediate                              = 0,
        PMDTraceConditionNextUpdate                             = 1,
        PMDTraceConditionEventStatus                            = 2,
        PMDTraceConditionActivityStatus                         = 3,
        PMDTraceConditionSignalStatus                           = 4,
        PMDTraceConditionDriveStatus                            = 5,
        PMDTraceConditionSPICommand                             = 6 
} PMDTraceCondition;

typedef enum {
        PMDTraceTriggerStateLow                                 = 0,
        PMDTraceTriggerStateHigh                                = 1
} PMDTraceTriggerState;

typedef enum {
        PMDTraceStatusMode                                      = 0x0001,
        PMDTraceStatusActivity                                  = 0x0002,
        PMDTraceStatusDataWrap                                  = 0x0004,
        PMDTraceStatusOverrun                                   = 0x0008,
        PMDTraceStatusDataAvailable                             = 0x0010 
} PMDTraceStatus;

typedef enum {
        PMDAxisOutRegisterNone                                  = 0,
        PMDAxisOutRegisterEventStatus                           = 1,
        PMDAxisOutRegisterActivityStatus                        = 2,
        PMDAxisOutRegisterSignalStatus                          = 3,
        PMDAxisOutRegisterDriveStatus                           = 4
} PMDAxisOutRegister;

typedef enum {
        PMDSerialBaud1200                                       = 0,
        PMDSerialBaud2400                                       = 1,
        PMDSerialBaud9600                                       = 2,
        PMDSerialBaud19200                                      = 3,
        PMDSerialBaud57600                                      = 4,
        PMDSerialBaud115200                                     = 5,
        PMDSerialBaud230400                                     = 6,
        PMDSerialBaud460800                                     = 7
} PMDSerialBaud;

typedef enum {
        PMDSerialStopBits1                                      = 0,
        PMDSerialStopBits2                                      = 1
} PMDSerialStopBits;

typedef enum {
        PMDSerialProtocolPoint2Point                            = 0,
        PMDSerialProtocolMultiDropUsingIdleLineDetection        = 1,
} PMDSerialProtocol;

typedef enum {
        PMDSerialParityNone                                     = 0,
        PMDSerialParityOdd                                      = 1,
        PMDSerialParityEven                                     = 2
} PMDSerialParity;

typedef enum {
        PMDCANBaud1000000                                       = 0,
        PMDCANBaud800000                                        = 1,
        PMDCANBaud500000                                        = 2,
        PMDCANBaud250000                                        = 3,
        PMDCANBaud125000                                        = 4,
        PMDCANBaud50000                                         = 5,
        PMDCANBaud20000                                         = 6,
        PMDCANBaud10000                                         = 7
} PMDCANBaud;

typedef enum {
        PMDSPIModeRisingEdge                                    = 0,
        PMDSPIModeRisingEdgeDelay                               = 1,
        PMDSPIModeFallingEdge                                   = 2,
        PMDSPIModeFallingEdgeDelay                              = 3
} PMDSPIMode;

//ION and Atlas specific
typedef enum {
        PMDCurrentControlModeCurrentLoop                        = 0,
        PMDCurrentControlModeFOC                                = 1,
        PMDCurrentControlModeThirdLegFloating                   = 2
} PMDCurrentControlMode;

typedef enum {
        PMDFOCDirect                                            = 0,
        PMDFOCQuadrature                                        = 1,
        PMDFOCBoth                                              = 2
} PMDFOC;

typedef enum {
        PMDFOCParameterKp                                       = 0,
        PMDFOCParameterKi                                       = 1,
        PMDFOCParameterIlimit                                   = 2
} PMDFOCParameter;

typedef enum {
        PMDCurrentLoopPhaseA                                    = 0,
        PMDCurrentLoopPhaseB                                    = 1,
        PMDCurrentLoopBoth                                      = 2
} PMDCurrentLoop;

typedef enum {
        PMDCurrentLoopParameterKp                               = 0,
        PMDCurrentLoopParameterKi                               = 1,
        PMDCurrentLoopParameterIlimit                           = 2
} PMDCurrentLoopParameter;

typedef enum {
        PMDCurrentLoopValueNodeReference                        = 0,
        PMDCurrentLoopValueNodeActualCurrent                    = 1,
        PMDCurrentLoopValueNodeLoopError                        = 2,
        PMDCurrentLoopValueNodeIntegratorSum                    = 3,
        PMDCurrentLoopValueNodeIntegralContribution             = 5,
        PMDCurrentLoopValueNodeOutput                           = 6,
        PMDCurrentLoopValueNodeI2tEnergy                        = 10
} PMDCurrentLoopValueNode;

typedef enum {
        PMDFOCValueNodeReference                                = 0,
        PMDFOCValueNodeFeedback                                 = 1,
        PMDFOCValueNodeLoopError                                = 2,
        PMDFOCValueNodeIntegratorSum                            = 3,
        PMDFOCValueNodeIntegralContribution                     = 5,
        PMDFOCValueNodeOutput                                   = 6,
        PMDFOCValueNodeFOCOutput                                = 7,
        PMDFOCValueNodeActualCurrent                            = 8,
        PMDFOCValueNodeI2tEnergy                                = 10
} PMDFOCValueNode;

typedef enum {
        PMDDriveStatusCalibrated                                = 1 << 0,
        PMDDriveStatusInFoldback                                = 1 << 1,
        PMDDriveStatusOverTemperature                           = 1 << 2,
        PMDDriveStatusShuntActive                               = 1 << 3,
        PMDDriveStatusInHolding                                 = 1 << 4,
        PMDDriveStatusOverVoltage                               = 1 << 5,
        PMDDriveStatusUnderVoltage                              = 1 << 6,
        PMDDriveStatusDisabled                                  = 1 << 7,
        PMDDriveStatusOutputClipped                             = 1 << 12,
        PMDDriveStatusNVRAMInitialization                       = 1 << 14,
        PMDDriveStatusAtlasNotConnected                         = 1 << 15
} PMDDriveStatus;

typedef enum {
        PMDDriveFaultStatusOvercurrent                          = 1 << 0,
        PMDDriveFaultStatusGround                               = 1 << 1, // ION
        PMDDriveFaultStatusExternalLogic                        = 1 << 2, // ION
        PMDDriveFaultStatusOpmodeMismatch                       = 1 << 3, // Atlas
        PMDDriveFaultStatusInternalLogic                        = 1 << 4,
        PMDDriveFaultStatusOvervoltage                          = 1 << 5,
        PMDDriveFaultStatusUndervoltage                         = 1 << 6,
        PMDDriveFaultStatusDisabled                             = 1 << 7,
        PMDDriveFaultStatusFoldback                             = 1 << 8,
        PMDDriveFaultStatusOvertemperature                      = 1 << 9,
        PMDDriveFaultStatusAtlasDetectedSPIChecksum             = 1 << 10, // Atlas
        PMDDriveFaultStatusWatchdog                             = 1 << 11,
        PMDDriveFaultStatusPWMDisabled                          = 1 << 11,
        PMDDriveFaultStatusBrake                                = 1 << 13,
        PMDDriveFaultStatusMagellanDetectedSPIChecksum          = 1 << 14, // Atlas
        PMDDriveFaultStatusMotorTypeMismatch                    = 1 << 15  // Atlas
} PMDDriveFaultStatus;

typedef enum {
        PMDCurrentHoldingMotorLimit                             = 0,
        PMDCurrentHoldingDelay                                  = 1,
        PMDCurrentDriveCurrent                                  = 2,
} PMDCurrent;

// PMDHoldingCurrent has been replaced by PMDCurrent but is left in for compatibility
typedef enum {
        PMDHoldingMotorLimit                                    = 0,
        PMDHoldingDelay                                         = 1
} PMDHoldingCurrent;

typedef enum {
        PMDCurrentFoldbackContinuousCurrentLimit                = 0,
        PMDCurrentFoldbackI2tEnergyLimit                        = 1,
} PMDCurrentFoldback;

typedef enum {
        PMDOperatingModeAxisEnabled                             = 0x0001,
        PMDOperatingModeMotorOutputEnabled                      = 0x0002,
        PMDOperatingModeCurrentControlEnabled                   = 0x0004,
        PMDOperatingModeVelocityLoopEnabled                     = 0x0008, // Juno
        PMDOperatingModePositionLoopEnabled                     = 0x0010,
        PMDOperatingModeTrajectoryEnabled                       = 0x0020,
        PMDOperatingModeCommandSourceEnabled                    = 0x0020, // Juno
        PMDOperatingModeAllEnabled                              = 0x0037,
} PMDOperatingMode;

typedef enum {
        PMDEventActionEventImmediate                            = 0,
        PMDEventActionEventPositiveLimit                        = 1,
        PMDEventActionEventNegativeLimit                        = 2,
        PMDEventActionEventMotionError                          = 3,
        PMDEventActionEventCurrentFoldback                      = 4,
        // The following are Juno only
        PMDEventActionEventCaptureReceived                      = 5,
        PMDEventActionEventOvertemperature                      = 6,
        PMDEventActionEventDisabled                             = 7,
        PMDEventActionEventCommutationError                     = 8,
        PMDEventActionEventOvercurrent                          = 9,
        PMDEventActionEventOvervoltage                          = 10,
        PMDEventActionEventUndervoltage                         = 11,
        PMDEventActionEventWatchdogTimeout                      = 12,
        PMDEventActionEventBrakeAsserted                        = 13
} PMDEventActionEvent;

typedef enum {
        PMDEventActionNone                                      = 0,
        PMDEventActionAbruptStop                                = 2,
        PMDEventActionSmoothStop                                = 3,
        PMDEventActionDisableVelocityLoopAndHigherModules       = 4, // Juno
        PMDEventActionDisablePositionLoopAndHigherModules       = 5,
        PMDEventActionDisableCurrentLoopAndHigherModules        = 6,
        PMDEventActionDisableMotorOutputAndHigherModules        = 7,
        PMDEventActionAbruptStopWithPositionErrorClear          = 8,
        PMDEventActionPassiveBraking                            = 10 // Juno
} PMDEventAction;

typedef enum {
        PMDUpdateMaskTrajectory                                 = 0x0001,
        PMDUpdateMaskPositionLoop                               = 0x0002,
        PMDUpdateMaskCurrentLoop                                = 0x0008
} PMDUpdateMask;

typedef enum {
        PMDPositionLoopValueNodeIntegratorSum                   = 0,
        PMDPositionLoopValueNodeIntegralContribution            = 1,
        PMDPositionLoopValueNodeDerivative                      = 2,
        PMDPositionLoopValueNodeBiquad1Input                    = 3,
        PMDPositionLoopValueNodeBiquad2Input                    = 4,
} PMDPositionLoopValueNode;

typedef enum {
        PMDPositionLoopKp                                       = 0, // Proportional gain
        PMDPositionLoopKi                                       = 1, // Integrator gain
        PMDPositionLoopIlimit                                   = 2, // Integrator limit
        PMDPositionLoopKd                                       = 3, // Derivative gain
        PMDPositionLoopDerivativeTime                           = 4,
        PMDPositionLoopKout                                     = 5, // Output gain
        PMDPositionLoopKvff                                     = 6, // Velocity feed forward gain
        PMDPositionLoopKaff                                     = 7, // Acceleration feed forward gain
        PMDPositionLoopBiquad1Enable                            = 8,
        PMDPositionLoopBiquad1B0                                = 9,
        PMDPositionLoopBiquad1B1                                = 10,
        PMDPositionLoopBiquad1B2                                = 11,
        PMDPositionLoopBiquad1A1                                = 12,
        PMDPositionLoopBiquad1A2                                = 13,
        PMDPositionLoopBiquad1K                                 = 14,
        PMDPositionLoopBiquad2Enable                            = 15,
        PMDPositionLoopBiquad2B0                                = 16,
        PMDPositionLoopBiquad2B1                                = 17,
        PMDPositionLoopBiquad2B2                                = 18,
        PMDPositionLoopBiquad2A1                                = 19,
        PMDPositionLoopBiquad2A2                                = 20,
        PMDPositionLoopBiquad2K                                 = 21
} PMDPositionLoop;

typedef enum {
        PMDVoltageLimitOvervoltage,
        PMDVoltageLimitUndervoltage
} PMDVoltageLimit;

// Atlas, Juno & MC5x113  specific types
typedef enum {
        PMDNVRAMOptionModeAtlas                                 = 0,
        PMDNVRAMOptionMode58113                                 = 0x100, 
        PMDNVRAMOptionErase                                     = 1,
        PMDNVRAMOptionWrite                                     = 2,
        PMDNVRAMOptionBlockWriteBegin                           = 3,
        PMDNVRAMOptionBlockWriteEnd                             = 4,
        PMDNVRAMOptionSkip                                      = 8
} PMDNVRAMOption;

typedef enum {
        PMDDriveFaultParameterOvervoltageLimit                  = 0,
        PMDDriveFaultParameterUndervoltageLimit                 = 1,
        PMDDriveFaultParameterRecoveryMode                      = 2,
        PMDDriveFaultParameterWatchdogLimit                     = 3,
        PMDDriveFaultParameterTemperatureLimit                  = 4,
        PMDDriveFaultParameterTemperatureHysteresis             = 5,
        PMDDriveFaultParameterShuntVoltage                      = 8,
        PMDDriveFaultParameterShuntPWMDutyCycle                 = 9,
        PMDDriveFaultParameterBusCurrentSupplyLimit             = 10,
        PMDDriveFaultParameterBusCurrentReturnLimit             = 11,
} PMDDriveFaultParameter;

typedef enum {
        PMDDrivePWMLimit                                        = 0,
        PMDDrivePWMDeadTime                                     = 1,
        PMDDrivePWMSignalSense                                  = 2,
        PMDDrivePWMFrequency                                    = 3,
        PMDDrivePWMRefreshPeriod                                = 4,
        PMDDrivePWMRefreshTime                                  = 5,
        PMDDrivePWMCurrentSenseTime                             = 6
} PMDDrivePWM;

typedef enum {
        PMDAnalogCalibrationLegCurrentA                         = 0,
        PMDAnalogCalibrationLegCurrentB                         = 1,
        PMDAnalogCalibrationLegCurrentC                         = 2,
        PMDAnalogCalibrationLegCurrentD                         = 3
} PMDAnalogCalibration;

typedef enum {
        PMDFeedbackParameterModulus                             = 0
} PMDFeedbackParameter;

typedef enum {
        PMDDriveValueBusVoltage                                 = 0,
        PMDDriveValueTemperature                                = 1,
        PMDDriveValueBusCurrentSupply                           = 2,
        PMDDriveValueBusCurrentReturn                           = 3
} PMDDriveValue;

typedef enum {
        // Atlas modes
        PMDDriveCommandModeBLDC_SPI                             = 0x0,
        PMDDriveCommandModeStep_SPI                             = 0x1,
        PMDDriveCommandModeDC_SPI                               = 0x2,
        PMDDriveCommandModeStep_PulseAndDirection               = 0x101,
        // Juno modes
        PMDDriveCommandModeAnalog                               = 0x20,
        PMDDriveCommandModeSPI                                  = 0x21,
        PMDDriveCommandModeInternalProfile                      = 0x22,
        PMDDriveCommandModePulseAndDirection                    = 0x23,
} PMDDriveCommandMode;

typedef enum {
        PMDCommutationParameterPhaseCounts                      = 0,
        PMDCommutationParameterPhaseAngle                       = 1,
        PMDCommutationParameterPhaseOffset                      = 2,
        PMDCommutationParameterPhaseDenominator                 = 3,
} PMDCommutationParameter;

typedef enum {
        PMDLoopVelocityKp                                       = 0,
        PMDLoopVelocityKi                                       = 1,
        PMDLoopVelocityIlimit                                   = 2,
        PMDLoopVelocityKout                                     = 5,
        PMDLoopVelocityVelocityErrorLimit                       = 7,
        PMDLoopVelocityBiquad1Enable                            = 8,
        PMDLoopVelocityBiquad1B0                                = 9,
        PMDLoopVelocityBiquad1B1                                = 10,
        PMDLoopVelocityBiquad1B2                                = 11,
        PMDLoopVelocityBiquad1A1                                = 12,
        PMDLoopVelocityBiquad1A2                                = 13,
        PMDLoopVelocityBiquad2Enable                            = 16,
        PMDLoopVelocityBiquad2B0                                = 17,
        PMDLoopVelocityBiquad2B1                                = 18,
        PMDLoopVelocityBiquad2B2                                = 19,
        PMDLoopVelocityBiquad2A1                                = 20,
        PMDLoopVelocityBiquad2A2                                = 21,
        PMDLoopVelocityFeedbackSource                           = 0x40,
        PMDLoopVelocityScalar                                   = 0x41,
        PMDLoopPositionFeedbackSource                           = 0x42,
        PMDLoopLowerLimit                                       = 0x43,
        PMDLoopUpperLimit                                       = 0x44,
        PMDLoopPositionKp                                       = 0x100,
        PMDLoopPositionKi                                       = 0x101,
        PMDLoopPositionIlimit                                   = 0x102,
        PMDLoopPositionKd                                       = 0x103,
        PMDLoopPositionDerivativeTime                           = 0x104,
        PMDLoopPositionKout                                     = 0x105,
        PMDLoopPositionPeriodMultiplier                         = 0x106,
        PMDLoopPositionPositionErrorLimit                       = 0x107,
        PMDLoopPositionDeadbandLowerLimit                       = 0x108,
        PMDLoopPositionDeadbandUpperLimit                       = 0x109,
} PMDLoop;  // Juno only

typedef enum {
        PMDLoopValueVelocityReference                           = 0,
        PMDLoopValueVelocityFeedback                            = 1,
        PMDLoopValueVelocityError                               = 2,
        PMDLoopValueVelocityIntegratorSum                       = 3,
        PMDLoopValueVelocityOutput                              = 5,
        PMDLoopValueVelocityBiquad1Input                        = 6,
        PMDLoopValueVelocityBiquad2Input                        = 7,
        PMDLoopValuePositionReference                           = 0x100,
        PMDLoopValuePositionFeedback                            = 0x101,
        PMDLoopValuePositionError                               = 0x102,
        PMDLoopValuePositionIntegratorSum                       = 0x103,
        PMDLoopValuePositionDerivative                          = 0x104,
        PMDLoopValuePositionOutput                              = 0x105,
        PMDLoopValuePositionBiquad1Input                        = 0x106,
        PMDLoopValuePositionBiquad2Input                        = 0x107,
} PMDLoopValue;

typedef enum {
        PMDCalibrateLegCurrentSensors                           = 0,
        PMDCalibrateAnalogCommand                               = 1,
        PMDCalibrateTachometer                                  = 2,
} PMDCalibrate;

typedef enum {
        PMDPhaseParameterRampTime                               = 0,
        PMDPhaseParameterPositivePulseTime                      = 1,
        PMDPhaseParameterNegativePulseTime                      = 2,
        PMDPhaseParameterPulseMotorCommand                      = 3,
        PMDPhaseParameterRampCommand                            = 5,
} PMDPhaseParameter;

typedef enum {
        PMDProfileParameterWaveAmplitude                        = 0x20,
        PMDProfileParameterWavePhase                            = 0x21,
        PMDProfileParameterWaveStartingFrequency                = 0x22,
        PMDProfileParameterWaveMaximumFrequency                 = 0x23,
        PMDProfileParameterWaveAngularAcceleration              = 0x24,
} PMDProfileParameter;

typedef enum {
        PMDRuntimeErrorVelocityScalarOverflow                   = 5,
} PMDRuntimeError;

#endif
