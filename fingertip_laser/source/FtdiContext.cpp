#include "FtdiContext.h"

#include <iostream>

using namespace std;

ftdi_context* FtdiContext::GetContext() { return context; }
void FtdiContext::SetInterface(ftdi_interface interface)
{
	PrintStatus(ftdi_set_interface(context, interface), "ftdi_set_interface");
}

FtdiContext::FtdiContext()
{
    context = ftdi_new();

}
FtdiContext::~FtdiContext()
{
	ftdi_free(context);
}

void FtdiContext::PrintStatus(int status, const char* function)
{
	if (status >= 0)
		; //cout << "Executed '" << function << "' (Result " << status << ")" << endl;
	else
        cerr << "Error on '" << function << "': " << ftdi_get_error_string(context) << " (Code " << status << ")" << endl;
}
