#ifndef PMD_ErrorCodes
#define	PMD_ErrorCodes

//  PMDecode.h -- error codes
//
//  Performance Motion Devices, Inc.
//
#if defined(__cplusplus)
extern "C" {
#endif

typedef enum PMDErrorCodesEnum {

	// processor error codes
	PMD_NOERROR                             = 0,
	PMD_ERR_OK                              = 0,
	PMD_ERR_Reset                           = 0x01,
	PMD_ERR_InvalidInstruction              = 0x02,
	PMD_ERR_InvalidAxis                     = 0x03,
	PMD_ERR_InvalidParameter                = 0x04,
	PMD_ERR_TraceRunning                    = 0x05,
	PMD_ERR_BlockOutOfBounds                = 0x07,
	PMD_ERR_TraceBufferZero                 = 0x08,
	PMD_ERR_BadSerialChecksum               = 0x09,
	PMD_ERR_InvalidNegativeValue            = 0x0B,
	PMD_ERR_InvalidParameterChange          = 0x0C,
	PMD_ERR_LimitEventPending               = 0x0D,
	PMD_ERR_InvalidMoveIntoLimit            = 0x0E,
	PMD_ERR_InvalidOperatingModeRestore     = 0x10,
	PMD_ERR_InvalidOperatingModeForCommand  = 0x11,
	PMD_ERR_BadState                        = 0x12,
	PMD_ERR_AtlasNotDetected                = 0x14,
	PMD_ERR_HardFault                       = 0x13,
	PMD_ERR_BadSPIChecksum                  = 0x15,
	PMD_ERR_InvalidSPIprotocol              = 0x16,
	PMD_ERR_InvalidTorqueCommand            = 0x18,
	PMD_ERR_BadFlashChecksum                = 0x19,
	PMD_ERR_InvalidFlashModeCommand         = 0x1A,
	PMD_ERR_ReadOnly                        = 0x1B,
	PMD_ERR_InitializationOnlyCommand       = 0x1C,
	PMD_ERR_IncorrectDataCount              = 0x1D,
	PMD_ERR_MoveWhileInError                = 0x1E,
	PMD_ERR_WaitTimedOut                    = 0x1F,
	PMD_ERR_InitializationRunning           = 0x20,
	PMD_ERR_InvalidClock                    = 0x21,
	PMD_ERR_InitializationSkipped           = 0x22,
	PMD_ERR_InvalidInterface                = 0x23,

	// non-processor errors
	PMD_ERR_InvalidOperation                = 0x7FD0,
	PMD_ERR_NotConnected                    = 0x7FD1,
	PMD_ERR_NotResponding                   = 0x7FD2,
	PMD_ERR_CommPortRead                    = 0x7FD3,
	PMD_ERR_CommPortWrite                   = 0x7FD4,
	PMD_ERR_InvalidSerialPort               = 0x7FDB,
	PMD_ERR_InterfaceNotInitialized         = 0x7FDF,
	PMD_ERR_OpeningPort                     = 0x7FE0,
	PMD_ERR_Driver                          = 0x7FE1,
	PMD_ERR_NoDPRAM                         = 0x7FE2,
	PMD_ERR_DPRAM                           = 0x7FE3,
	PMD_ERR_Timeout                         = 0x7FE4,
	PMD_ERR_WaitCancelled                   = 0x7FE5,
	PMD_ERR_CommunicationsError             = 0x7FFC,
	PMD_ERR_CommTimeoutError                = 0x7FFD,
	PMD_ERR_ChecksumError                   = 0x7FFE,
	PMD_ERR_CommandError                    = 0x7FFF

} PMDErrorCode;

#if defined(__cplusplus)
}
#endif

#endif
