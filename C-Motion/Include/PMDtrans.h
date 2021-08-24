#ifndef _DEF_INC_PMDtrans
#define	_DEF_INC_PMDtrans

//
//  PMDtrans.h -- Transport functions for motion processor commands 
//
//  Performance Motion Devices, Inc.
//

PMDuint16 BuildCommand(PMDuint16 OPCode, PMDAxis axis);

PMDresult SendCommand(PMDAxisInterface axis_intf, PMDuint16 OPCode);
PMDresult SendCommandWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1);
PMDresult SendCommandWordWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint16 data2);
PMDresult SendCommandWordWordWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint16 data2, PMDuint16 data3);
PMDresult SendCommandLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint32 data1);
PMDresult SendCommandLongWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint32 data1, PMDuint16 data2);
PMDresult SendCommandWordLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint32 data2);
PMDresult SendCommandGetWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 *data1);
PMDresult SendCommandGetWordWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 *data1, PMDuint16 *data2);
PMDresult SendCommandGetWordWordWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 *data1, PMDuint16 *data2, PMDuint16 *data3);
PMDresult SendCommandGetLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint32 *data1);
PMDresult SendCommandGetWordLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 *data1, PMDuint32 *data2);
PMDresult SendCommandWordGetWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint16 *data2);
PMDresult SendCommandWordGetLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint32 *data2);
PMDresult SendCommandLongGetWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint32 data1, PMDuint16* data2);
PMDresult SendCommandWordWordGetWord(PMDAxisInterface axis_intf, PMDuint16 OPCode,
									 PMDuint16 data1, PMDuint16 data2, PMDuint16 *data3);


#endif
