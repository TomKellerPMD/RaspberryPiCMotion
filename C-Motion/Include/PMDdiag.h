#ifndef PMD_Diagnostics
#define	PMD_Diagnostics

//  PMDdiag.h -- diagnostic functions
//
//  Performance Motion Devices, Inc.
//
#include "PMDsys.h"

#define PMD_RESULT(_call)  {result = _call; if (result) PMDprintf("Error: %s, %30s at line: %d in file: %s\n", PMDGetErrorMessage(result), #_call, __LINE__, __FILE__);}
#define PMD_ABORTONERROR(_call)  PMD_RESULT(_call) if (result) PMDTaskAbort(result);

#if defined(__cplusplus)
extern "C" {
#endif

const char *PMDGetOpcodeText(PMDuint16 opCode);
const char *PMDGetErrorMessage(PMDresult errorCode);

#if defined(__cplusplus)
}
#endif

#endif

