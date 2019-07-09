#ifndef AdnsReader_h
#define AdnsReader_h

#include "FtdiUsbDevice.h"

class AdnsReader
{
	private: FtdiUsbDevice* device;
	
	public: AdnsReader(FtdiUsbDevice* device);
	public: ~AdnsReader();
	
	public: Buffer* Read(const char*, int);
	public: Buffer* ReadWithoutCommand(int);
};

#endif
