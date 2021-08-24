//  PMDtrans.c -- generic transport interface
//
//  Performance Motion Devices, Inc.
//

#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDdevice.h"
#include "PMDtrans.h"


PMDuint16 BuildCommand(PMDuint16 OPCode, PMDAxis axis)
{
    return ((PMDuint16)((PMDuint32)OPCode | ((PMDuint32)axis << 8)));
}

PMDresult SendCommand(PMDAxisInterface axis_intf, PMDuint16 OPCode)
{
    PMDuint16 buffer[1];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);

    return axis_intf->transport.SendCommand(axis_intf->transport_data, 1, buffer, 0, PMDnull);
}


PMDresult SendCommandWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1)
{
    PMDuint16 buffer[2];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)data1;

    return axis_intf->transport.SendCommand(axis_intf->transport_data, 2, buffer, 0, PMDnull);
}


PMDresult SendCommandWordWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint16 data2)
{
    PMDuint16 buffer[3];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)data1;
    buffer[2] = (PMDuint16)data2;

    return axis_intf->transport.SendCommand(axis_intf->transport_data, 3, buffer, 0, PMDnull);
}

PMDresult SendCommandWordWordWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint16 data2, PMDuint16 data3)
{
    PMDuint16 buffer[4];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)data1;
    buffer[2] = (PMDuint16)data2;
    buffer[3] = (PMDuint16)data3;

    return axis_intf->transport.SendCommand(axis_intf->transport_data, 4, buffer, 0, PMDnull);
}

PMDresult SendCommandLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint32 data1)
{
    PMDuint16 buffer[3];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)(data1 >> 16); //Write MSW first
    buffer[2] = (PMDuint16) data1;

    return axis_intf->transport.SendCommand(axis_intf->transport_data, 3, buffer, 0, PMDnull);
}

PMDresult SendCommandLongWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint32 data1, PMDuint16 data2)
{
    PMDuint16 buffer[4];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)(data1 >> 16); //Write MSW first
    buffer[2] = (PMDuint16) data1;
    buffer[3] = data2;

    return axis_intf->transport.SendCommand(axis_intf->transport_data, 4, buffer, 0, PMDnull);
}

PMDresult SendCommandWordLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint32 data2)
{
    PMDuint16 buffer[4];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = data1;
    buffer[2] = (PMDuint16)(data2 >> 16); //Write MSW first
    buffer[3] = (PMDuint16) data2;

    return axis_intf->transport.SendCommand(axis_intf->transport_data, 4, buffer, 0, PMDnull);
}


PMDresult SendCommandGetWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 *data1)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[2];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 1, buffer, 1, buffer+1);

    if (result == PMD_ERR_OK)
        *data1 = (PMDuint16)buffer[1];

    return result;
}

PMDresult SendCommandGetWordWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 *data1, PMDuint16 *data2)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[3];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 1, buffer, 2, buffer+1);

    if (result == PMD_ERR_OK)
    {
        *data1 = (PMDuint16)buffer[1];
        *data2 = (PMDuint16)buffer[2];
    }
    return result;
}

PMDresult SendCommandGetWordWordWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 *data1, PMDuint16 *data2, PMDuint16 *data3)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[4];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 1, buffer, 3, buffer+1);

    if (result == PMD_ERR_OK)
    {
        *data1 = (PMDuint16)buffer[1];
        *data2 = (PMDuint16)buffer[2];
        *data3 = (PMDuint16)buffer[3];
    }
    return result;
}

PMDresult SendCommandGetLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint32 *data1)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[3];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 1, buffer, 2, buffer+1);

    if (result == PMD_ERR_OK)
        *data1 = ((PMDuint32)buffer[1])<<16 | ((PMDuint32)buffer[2]);

    return result;
}

PMDresult SendCommandGetWordLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 *data1, PMDuint32 *data2)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[4];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 1, buffer, 3, buffer+1);

    if (result == PMD_ERR_OK)
    {
        *data1 = (PMDuint16)buffer[1];
        *data2 = ((PMDuint32)buffer[2])<<16 | ((PMDuint32)buffer[3]);
    }
    return result;
}

PMDresult SendCommandWordGetWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint16 *data2)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[3];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)data1;

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 2, buffer, 1, buffer+2);

    if (result == PMD_ERR_OK)
        *data2 = (PMDuint16)buffer[2];

    return result;
}

PMDresult SendCommandWordGetLong(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint32 *data2)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[4];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)data1;

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 2, buffer, 2, buffer+2);

    if (result == PMD_ERR_OK)
        *data2 = ((PMDuint32)buffer[2])<<16 | ((PMDuint32)buffer[3]);

    return result;
}

PMDresult SendCommandLongGetWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint32 data1, PMDuint16* data2)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[4];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)(data1 >> 16); //Write MSW first
    buffer[2] = (PMDuint16) data1;

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 2, buffer, 1, buffer+3);

    if (result == PMD_ERR_OK)
        *data2 = (PMDuint16)buffer[3];

    return result;
}

PMDresult SendCommandWordWordGetWord(PMDAxisInterface axis_intf, PMDuint16 OPCode, PMDuint16 data1, PMDuint16 data2, PMDuint16 *data3)
{
    PMDresult result = PMD_ERR_OK;
    PMDuint16 buffer[4];

    buffer[0] = BuildCommand(OPCode,axis_intf->axis);
    buffer[1] = (PMDuint16)data1;
    buffer[2] = (PMDuint16)data2;

    result = axis_intf->transport.SendCommand(axis_intf->transport_data, 3, buffer, 1, buffer+3);

    if (result == PMD_ERR_OK)
        *data3 = (PMDuint16)buffer[3];

    return result;
}

