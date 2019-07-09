#include "AdnsReader.h"

#include <stdio.h>
#include <time.h>

AdnsReader::AdnsReader(FtdiUsbDevice* device)
{
	this->device = device;
}
AdnsReader::~AdnsReader() { }

Buffer* AdnsReader::Read(const char* command, int length)
{
	device->ClearBuffers();

	Buffer commandBuffer(command);
	int bytesWritten = device->Write(&commandBuffer);

	if (bytesWritten == 0)
	{
		printf("Command '%s' failed, could not send command.\n", command);
		return NULL;
	}

	Buffer startMarker("-I-\n");
	Buffer endMarker("\n-O-\n");
	Buffer totalResult;
	Buffer* data = NULL;

	timespec required;
	required.tv_sec = 0;
	required.tv_nsec = 1000000;

	for (int i = 0; i < 1000 && data == NULL; i++)
	{
		Buffer result;
		device->Read(&result);
		totalResult.Append(&result);

		data = totalResult.Extract(&startMarker, &endMarker);

		if (data == NULL) nanosleep(&required, NULL);
	}
	if (length != 0)
	{
		if (data == NULL)
		{
			printf("Command '%s' failed, markers not found (%i bytes read).\n", command, totalResult.GetLength());
			return NULL;
		}
		if (data->GetLength() != length)
		{
			printf("Command '%s' failed, payload data has length %i (%i expected).\n", command, data->GetLength(), length);
			delete data;
			return NULL;
		}
	}

	return data;
}
