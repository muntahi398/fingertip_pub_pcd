#include "FtdiUsbDevice.h"

#include <string.h>
#include <stdio.h>

void FtdiUsbDevice::SetBaudrate(int baudrate)
{
	context->PrintStatus(ftdi_set_baudrate(context->GetContext(), baudrate), "ftdi_set_baudrate");
}
void FtdiUsbDevice::SetLatencyTimer(unsigned char latency)
{
	context->PrintStatus(ftdi_set_latency_timer(context->GetContext(), latency), "ftdi_set_latency_timer");
}

FtdiUsbDevice::FtdiUsbDevice(ftdi_interface interface, int vendorID, int productID)
{
	context = new FtdiContext();
    
    context->SetInterface(interface);
    int ret = ftdi_usb_open(context->GetContext(), vendorID, productID);
    context->PrintStatus(ret, "ftdi_usb_open");
    if (ret < 0)
    {
        delete context;
        context = NULL;
    }

}

FtdiUsbDevice::~FtdiUsbDevice()
{
	context->PrintStatus(ftdi_usb_close(context->GetContext()), "ftdi_usb_close");
	
	delete context;
}

int FtdiUsbDevice::Read(unsigned char* buffer, int length)
{
	int result = ftdi_read_data(context->GetContext(), buffer, length);
	
	context->PrintStatus(result, "ftdi_read_data");
	
	return result;
}
int FtdiUsbDevice::Write(unsigned char* buffer, int length)
{
	int result = ftdi_write_data(context->GetContext(), buffer, length);
	
	context->PrintStatus(result, "ftdi_write_data");
	
	return result;
}
int FtdiUsbDevice::Read(Buffer* buffer)
{
	const int chunkSize = 0x400;

	int bytesRead = 0;
	unsigned char chunk[chunkSize];
	
	while ((bytesRead = Read(chunk, chunkSize)) > 0)
	{
		Buffer chunkBuffer(chunk, bytesRead);

		buffer->Append(&chunkBuffer);
	}

	return buffer->GetLength();
}
int FtdiUsbDevice::Write(Buffer* buffer)
{
	return Write(buffer->GetData(), buffer->GetLength());
}
void FtdiUsbDevice::ClearReadBuffer()
{
	context->PrintStatus(ftdi_usb_purge_rx_buffer(context->GetContext()), "ftdi_usb_purge_rx_buffer");
}
void FtdiUsbDevice::ClearWriteBuffer()
{
	context->PrintStatus(ftdi_usb_purge_tx_buffer(context->GetContext()), "ftdi_usb_purge_tx_buffer");
}
void FtdiUsbDevice::ClearBuffers()
{
	context->PrintStatus(ftdi_usb_purge_buffers(context->GetContext()), "ftdi_usb_purge_buffers");
}

bool FtdiUsbDevice::DeviceFound()
{
    return (context != NULL);
}

