// ****************************************************************
// PMDdiag.c : converts opcodes and error codes to strings
//
// Performance Motion Devices, Inc.
//

#include "PMDtypes.h"
#include "PMDdiag.h"
#include "PMDecode.h"

//*****************************************************************************
const char* PMDOpcodeText[] = {
    "NoOperation               ",
    "GetProductInfo            ",
    "SetMotorType              ",
    "GetMotorType              ",
    "SetBiQuadCoefficient      ",
    "GetBiQuadCoefficient      ",
    "SetMotorLimit             ",
    "GetMotorLimit             ",
    "SetAuxiliaryEncoderSource ",
    "GetAuxiliaryEncoderSource ",
    "SetSPIMode                ",
    "GetSPIMode                ",
    "SetPWMFrequency           ",
    "GetPWMFrequency           ",
    "GetDriveStatus            ",
    "SetMotorBias              ",

    "SetPosition               ",
    "SetVelocity               ",
    "SetCANMode                ",
    "SetJerk                   ",
    "SetGearRatio              ",
    "GetCANMode                ",
    "SetProfileParameter       ",
    "GetProfileParameter       ",
    "Invalid opcode!           ",
    "Invalid opcode!           ",
    "Update                    ",
    "SetOvertemperatureLimit   ",
    "GetOvertemperatureLimit   ",
    "GetCommandedPosition      ",
    "GetCommandedVelocity      ",
    "Invalid opcode!           ",

    "Invalid opcode!           ",
    "SetFeedbackParameter      ",
    "GetFeedbackParameter      ",
    "SetDrivePWM               ",
    "GetDrivePWM               ",
    "SetKp                     ",
    "SetKi                     ",
    "SetKd                     ",
    "GetTraceValue             ",
    "SetAnalogCalibration      ",
    "GetAnalogCalibration      ",
    "SetKvff                   ",
    "GetPhaseAngle             ",
    "GetMotorBias              ",
    "RestoreOperatingMode      ",
    "SetInterruptMask          ",

    "DriveNVRAM                ",
    "GetEventStatus            ",
    "SetBreakpointUpdateMask   ",
    "GetBreakpointUpdateMask   ",
    "ResetEventStatus          ",
    "ExecutionControl          ",
    "GetCaptureValue           ",
    "GetActualPosition         ",
    "GetVelocityLoopValue      ",
    "Reset                     ",
    "GetActiveMotorCommand     ",
    "SetSampleTime             ",
    "GetSampleTime             ",
    "GetRuntimeError           ",
    "GetTime                   ",
    "Invalid opcode!           ",

    "GetBusVoltage             ",
    "SetCurrentFoldback        ",
    "GetCurrentFoldback        ",
    "SetCurrentControlMode     ",
    "GetCurrentControlMode     ",
    "SetAxisOutMask            ",
    "GetAxisOutMask            ",
    "ClearPositionError        ",
    "SetEventAction            ",
    "GetEventAction            ",
    "GetPosition               ",
    "GetVelocity               ",
    "GetAcceleration           ",
    "SetActualPosition         ",
    "Invalid opcode!           ",
    "Invalid opcode!           ",

    "GetKp                     ",
    "GetKi                     ",
    "GetKd                     ",
    "GetTemperature            ",
    "GetKvff                   ",
    "GetPositionLoopValue      ",
    "GetInterruptMask          ",
    "GetActiveOperatingMode    ",
    "GetJerk                   ",
    "GetGearRatio              ",
    "GetFOCValue               ",
    "MultiUpdate               ",
    "Invalid opcode!           ",
    "Invalid opcode!           ",
    "SetCurrent                ",
    "GetCurrent                ",

    "GetDriveFaultParameter    ",
    "Invalid opcode!           ",
    "SetDriveFaultParameter    ",
    "SetCommutationParameter   ",
    "GetCommutationParameter   ",
    "SetOperatingMode          ",
    "GetOperatingMode          ",
    "SetPositionLoop           ",
    "GetPositionLoop           ",
    "GetMotorCommand           ",
    "SetStartVelocity          ",
    "GetStartVelocity          ",
    "ClearDriveFaultStatus     ",
    "GetDriveFaultStatus       ",
    "GetOutputMode             ",
    "CalibrateAnalog           ",

    "GetDriveValue             ",
    "GetCurrentLoopValue       ",
    "SetPhaseInitializeTime    ",
    "SetCurrentLoop            ",
    "GetCurrentLoop            ",
    "SetPhaseCounts            ",
    "SetPhaseOffset            ",
    "SetMotorCommand           ",
    "SetVelocityLoop           ",
    "GetVelocityLoop           ",
    "InitializePhase           ",
    "GetPhaseOffset            ",
    "GetPhaseInitializeTime    ",
    "GetPhaseCounts            ",
    "SetDriveCommandMode       ",
    "GetDriveCommandMode       ",

    "SetLimitSwitchMode        ",
    "GetLimitSwitchMode        ",
    "WriteIO                   ",
    "ReadIO                    ",
    "SetPhaseAngle             ",
    "SetPhaseParameter         ",
    "GetPhaseParameter         ",
    "SetAxisMode               ",
    "GetAxisMode               ",
    "SetDefault                ",
    "GetDefault                ",
    "SetSerialPortMode         ",
    "GetSerialPortMode         ",
    "SetEncoderModulus         ",
    "GetEncoderModulus         ",
    "GetVersion                ",

    "SetAcceleration           ",
    "SetDeceleration           ",
    "GetDeceleration           ",
    "SetKaff                   ",
    "GetKaff                   ",
    "SetIntegrationLimit       ",
    "GetIntegrationLimit       ",
    "SetPositionErrorLimit     ",
    "GetPositionErrorLimit     ",
    "GetPositionError          ",
    "GetIntegral               ",
    "GetDerivative             ",
    "SetDerivativeTime         ",
    "GetDerivativeTime         ",
    "SetKout                   ",
    "GetKout                   ",

    "SetProfileMode            ",
    "GetProfileMode            ",
    "SetSignalSense            ",
    "GetSignalSense            ",
    "GetSignalStatus           ",
    "GetInstructionError       ",
    "GetActivityStatus         ",
    "GetCommandedAcceleration  ",
    "SetTrackingWindow         ",
    "GetTrackingWindow         ",
    "SetSettleTime             ",
    "GetSettleTime             ",
    "ClearInterrupt            ",
    "GetActualVelocity         ",
    "SetGearMaster             ",
    "GetGearMaster             ",

    "SetTraceMode              ",
    "GetTraceMode              ",
    "SetTraceStart             ",
    "GetTraceStart             ",
    "SetTraceStop              ",
    "GetTraceStop              ",
    "SetTraceVariable          ",
    "GetTraceVariable          ",
    "SetTracePeriod            ",
    "GetTracePeriod            ",
    "GetTraceStatus            ",
    "GetTraceCount             ",
    "SetSettleWindow           ",
    "GetSettleWindow           ",
    "SetActualPositionUnits    ",
    "GetActualPositionUnits    ",

    "SetBufferStart            ",
    "GetBufferStart            ",
    "SetBufferLength           ",
    "GetBufferLength           ",
    "SetBufferWriteIndex       ",
    "GetBufferWriteIndex       ",
    "SetBufferReadIndex        ",
    "GetBufferReadIndex        ",
    "WriteBuffer               ",
    "ReadBuffer                ",
    "SetBufferFunction         ",
    "GetBufferFunction         ",
    "Invalid opcode!           ",
    "ReadBuffer16              ",
    "GetStepRange              ",
    "SetStepRange              ",

    "SetStopMode               ",
    "GetStopMode               ",
    "SetAutoStopMode           ",
    "GetAutoStopMode           ",
    "SetBreakpoint             ",
    "GetBreakpoint             ",
    "SetBreakpointValue        ",
    "GetBreakpointValue        ",
    "SetCaptureSource          ",
    "GetCaptureSource          ",
    "SetEncoderSource          ",
    "GetEncoderSource          ",
    "SetMotorMode              ",
    "GetMotorMode              ",
    "SetEncoderToStepRatio     ",
    "GetEncoderToStepRatio     ",

    "SetOutputMode             ",
    "GetInterruptAxis          ",
    "SetCommutationMode        ",
    "GetCommutationMode        ",
    "SetPhaseInitializeMode    ",
    "GetPhaseInitializeMode    ",
    "SetPhasePrescale          ",
    "GetPhasePrescale          ",
    "SetPhaseCorrectionMode    ",
    "GetPhaseCorrectionMode    ",
    "GetPhaseCommand           ",
    "SetMotionCompleteMode     ",
    "GetMotionCompleteMode     ",
    "SetAxisOutSource          ",
    "GetAxisOutSource          ",
    "ReadAnalog                ",

    "Invalid opcode!           ",
    "Invalid opcode!           ",
    "SetSynchronizationMode    ",
    "GetSynchronizationMode    ",
    "Invalid opcode!           ",
    "AdjustActualPosition      ",
    "SetFOC                    ",
    "GetFOC                    ",
    "GetChecksum               ",
    "SetUpdateMask             ",
    "GetUpdateMask             ",
    "SetFaultOutMask           ",
    "GetFaultOutMask           ",
    "Invalid opcode!           ",
    "Invalid opcode!           ",
    "Invalid opcode!           "

};


//*****************************************************************************
const char *PMDGetOpcodeText(PMDuint16 opCode)
{
    return PMDOpcodeText[opCode & 0xff];
}

//*****************************************************************************
const char *PMDGetErrorMessage(PMDresult errorCode)
{
    return errorCode == PMD_ERR_OK                         ? "No error" :

    // Motion processor errors
    errorCode == PMD_ERR_Reset                             ? "Processor reset" :
    errorCode == PMD_ERR_InvalidInstruction                ? "Invalid instruction" :
    errorCode == PMD_ERR_InvalidAxis                       ? "Invalid axis" :
    errorCode == PMD_ERR_InvalidParameter                  ? "Invalid data parameter" :
    errorCode == PMD_ERR_TraceRunning                      ? "Trace currently running" :
    errorCode == PMD_ERR_BlockOutOfBounds                  ? "Block out of bounds" :
    errorCode == PMD_ERR_TraceBufferZero                   ? "Zero length trace buffer" :
    errorCode == PMD_ERR_BadSerialChecksum                 ? "Invalid checksum" :
    errorCode == PMD_ERR_InvalidNegativeValue              ? "Invalid negative value for profile mode" :
    errorCode == PMD_ERR_InvalidParameterChange            ? "Invalid parameter change for profile mode" :
    errorCode == PMD_ERR_LimitEventPending                 ? "Invalid move with limit event pending" :
    errorCode == PMD_ERR_InvalidMoveIntoLimit              ? "Invalid move into limit" :
    errorCode == PMD_ERR_InvalidOperatingModeRestore       ? "Invalid operating mode restore" :
    errorCode == PMD_ERR_InvalidOperatingModeForCommand    ? "Command not valid in this operating mode" :
    errorCode == PMD_ERR_BadState                          ? "Command not accepted in current state" :
    errorCode == PMD_ERR_AtlasNotDetected                  ? "Atlas command specified but no Atlas detected." :
    errorCode == PMD_ERR_HardFault                         ? "A hard fault has occured. The processor must be reset " :
    errorCode == PMD_ERR_BadSPIChecksum                    ? "Bad SPI command checksum" :
    errorCode == PMD_ERR_InvalidSPIprotocol                ? "Incorrect SPI command protocol" :
    errorCode == PMD_ERR_InvalidTorqueCommand              ? "Invalid torque command" :
    errorCode == PMD_ERR_BadFlashChecksum                  ? "Bad flash checksum" :
    errorCode == PMD_ERR_InvalidFlashModeCommand           ? "Command not valid in flash mode" :
    errorCode == PMD_ERR_ReadOnly                          ? "Write to read only buffer" :
    errorCode == PMD_ERR_InitializationOnlyCommand         ? "Command valid only for initialization" :
    errorCode == PMD_ERR_IncorrectDataCount                ? "Incorrect amount of data received by processor" :
    errorCode == PMD_ERR_MoveWhileInError                  ? "Attempt to move while an event bit is set" :
    errorCode == PMD_ERR_WaitTimedOut                      ? "ExecutionControl command timed out" :
    errorCode == PMD_ERR_InitializationRunning             ? "Attempt to change buffer 1 state while NVRAM init is running" :
    errorCode == PMD_ERR_InvalidClock                      ? "Invalid external clock detected" :
    errorCode == PMD_ERR_InitializationSkipped             ? "Reset was performed with NVRAM initialization inhibited" :
    errorCode == PMD_ERR_InvalidInterface                  ? "Operation not permitted on command interface" :

    errorCode == PMD_ERR_InvalidOperation                  ? "Invalid operation" :
    errorCode == PMD_ERR_NotConnected                      ? "Not connected" :
    errorCode == PMD_ERR_NotResponding                     ? "Device not responding" :
    errorCode == PMD_ERR_CommPortRead                      ? "Comm port read" :
    errorCode == PMD_ERR_CommPortWrite                     ? "Comm port write" :
    errorCode == PMD_ERR_InvalidSerialPort                 ? "Invalid serial port" :
    errorCode == PMD_ERR_InterfaceNotInitialized           ? "Interface not initialized" :
    errorCode == PMD_ERR_OpeningPort                       ? "Cannot open port" :
    errorCode == PMD_ERR_Driver                            ? "Windows driver error" :
    errorCode == PMD_ERR_NoDPRAM                           ? "DPRAM not supported" :
    errorCode == PMD_ERR_DPRAM                             ? "Error accessing DPRAM" :
    errorCode == PMD_ERR_Timeout                           ? "Timeout" :
    errorCode == PMD_ERR_WaitCancelled                     ? "Cancelled" :
    errorCode == PMD_ERR_CommunicationsError               ? "Communication error" :
    errorCode == PMD_ERR_CommTimeoutError                  ? "Communication timeout error" :
    errorCode == PMD_ERR_ChecksumError                     ? "Checksum error" :
    errorCode == PMD_ERR_CommandError                      ? "Command error" :

    "Undefined error occurred";
}

