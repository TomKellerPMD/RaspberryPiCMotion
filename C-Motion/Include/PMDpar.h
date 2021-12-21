#ifndef PMD_Parallel
#define PMD_Parallel

//  PMDpar.h -- parallel IO
//
//  Performance Motion Devices, Inc.
//

// call this function to initialize the interface
PMDresult PMDSetupAxisInterface_Parallel(PMDAxisHandle* axis_handle, PMDAxis axis_number, PMDuint16 board_address);

PMDresult PMDParallel_ReadDPRAM(void* transport_data, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 words_to_read);
PMDresult PMDParallel_WriteDPRAM(void* transport_data, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 words_to_write);

#endif