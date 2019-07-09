#ifndef FtdiUsbDevice_h
#define FtdiUsbDevice_h

#include <ftdi.h>

#include "FtdiContext.h"
#include "Buffer.h"

class FtdiUsbDevice
{
	private: FtdiContext* context;

	public: void SetBaudrate(int);
	public: void SetLatencyTimer(unsigned char);

	public: FtdiUsbDevice(ftdi_interface, int, int);
	public: ~FtdiUsbDevice();
	
	public: int Read(unsigned char*, int);
	public: int Write(unsigned char*, int);
	public: int Read(Buffer*);
	public: int Write(Buffer*);
	public: void ClearReadBuffer();
	public: void ClearWriteBuffer();
	public: void ClearBuffers();
    public: bool DeviceFound();
};

#endif
