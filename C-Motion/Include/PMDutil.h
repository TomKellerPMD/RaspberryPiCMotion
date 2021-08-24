#ifndef PMD_Util
#define PMD_Util
// ****************************************************************
// PMD_Util.h : sample test and utility function declerations
//
// Performance Motion Devices, Inc.
//

#if defined(__cplusplus)
extern "C" {
#endif

int InitializeAxisHandles(PMDAxisHandle* phAxis);
void PMDCopyAxisInterface(PMDAxisHandle* dest_axis_handle, PMDAxisHandle* src_axis_handle, PMDAxis axis_number);
PMDresult PMDProcessorReset(PMDAxisHandle* phAxis);
void SetupTrace(PMDAxisHandle* phAxis, PMDuint32 bufferlength);
void DisplayTraceResults(PMDAxisHandle* phAxis);
PMDresult WaitForEvent(PMDAxisHandle* phAxis, PMDuint16 eventmask, PMDuint32 timeoutms);
PMDresult ReadBuffer(PMDAxisHandle* phAxis, PMDuint16 bufferID, PMDuint32* pbuffer, PMDuint32 dwords_to_read);

PMDresult WaitForAlgorithmicPhaseInitialization(PMDAxisHandle* phAxis);
PMDresult InitializePhase(PMDAxisHandle* phAxis);
PMDresult WaitForAtlasToConnect(PMDAxisHandle* phAxis);
PMDresult DisplayVersion(PMDAxisHandle* phAxis);
PMDresult GetVersionDWord(PMDAxisInterface axis_intf, PMDuint32 *version);

// in Pro-MotionExport.c
PMDresult SetupAxis1(PMDAxisHandle* phAxis); 
PMDresult SetupAxis2(PMDAxisHandle* phAxis); 
PMDresult SetupAxis3(PMDAxisHandle* phAxis); 
PMDresult SetupAxis4(PMDAxisHandle* phAxis); 

#if defined(__cplusplus)
}
#endif

#endif
