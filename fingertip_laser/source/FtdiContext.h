#ifndef FtdiContext_h
#define FtdiContext_h

#include <ftdi.h>

class FtdiContext
{
	private: ftdi_context* context;
	
	public: ftdi_context* GetContext();
	public: void SetInterface(ftdi_interface);
	
	public: FtdiContext();
	public: ~FtdiContext();

	public: void PrintStatus(int, const char*);
};

#endif
