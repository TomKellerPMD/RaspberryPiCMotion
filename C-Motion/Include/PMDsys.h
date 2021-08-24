/*****************************************************************************

   $Workfile: PMDsys.h $
   $Revision: 2 $

    Purpose: Operating System functions and includes

****************************************************************************/
#ifndef _PMDSYS_H
#define _PMDSYS_H

#include <stdlib.h>
#include <stdio.h>

#if defined(_WIN32)
// disable deprecation warnings for vsprintf and other CRT functions
#pragma warning( disable : 4996 )

#include <windows.h>
#define PMDputch                  putch
#define PMDputs                   puts
#define PMDprintf                 printf
#define PMDsprintf                sprintf
#define PMDTaskWait(ms)           Sleep(ms)
#define PMDTaskAbort(code)        exit(code)
#define PMDDeviceGetTickCount()   GetTickCount()
#define GetCurrentMilliseconds()  GetTickCount()

#else // Linux

#define PMDputch                  putch
#define PMDputs                   puts
#define PMDprintf                 printf
#define PMDsprintf                sprintf
#define PMDTaskWait(ms)           usleep(ms*1000)
#define PMDTaskAbort(code)        abort()
#define PMDDeviceGetTickCount()   GetTickCount()
#define GetCurrentMilliseconds()  GetTickCount()
#define INVALID_HANDLE_VALUE      ((long)-1)
#define HANDLE                    int
#define BOOL                      int

#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <linux/types.h>
#include <sys/time.h>
#include <sys/file.h>
//unsigned GetTickCount();

#endif


#endif
