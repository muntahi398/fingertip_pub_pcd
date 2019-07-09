#include "Buffer.h"

#include <string.h>

int Buffer::GetCapacity() { return capacity; }
void Buffer::SetCapacity(int value)
{
	capacity = value;

	unsigned char* newData = new unsigned char[capacity];
	Copy(data, newData, length);	
	delete[] data;
	data = newData;
}
unsigned char* Buffer::GetData() { return data; }
int Buffer::GetLength() { return length; }

Buffer::Buffer() { Initialize(); }
Buffer::Buffer(unsigned char* initialData, int initialLength) { Initialize(initialData, initialLength); }
Buffer::Buffer(const char* text) { Initialize((unsigned char*)text, strlen(text)); }
Buffer::~Buffer() { delete[] data; }

void Buffer::Append(Buffer* appendBuffer)
{
	while (length + appendBuffer->length > capacity) SetCapacity(capacity * 2);
	Copy(appendBuffer->data, data + length, appendBuffer->length);
	length += appendBuffer->length;
}
void Buffer::Clear()
{
	length = 0;
}
int Buffer::Find(Buffer* string)
{
	for (int i = 0; i <= length - string->length; i++)
		if (Equals(data + i, string->data, string->length))
			return i;

	return -1;
}
Buffer* Buffer::Extract(Buffer* startMarker, Buffer* endMarker)
{
	int startIndex = Find(startMarker);
	int endIndex = Find(endMarker);

	if (startIndex == -1 || endIndex == -1 || startIndex > endIndex) return NULL;

	return new Buffer(data + startIndex + startMarker->length, endIndex - (startIndex + startMarker->length));
}
Buffer* Buffer::SubBuffer(int startIndex, int endIndex)
{
    if (startIndex == -1 || endIndex == -1 || startIndex > endIndex) return NULL;

	return new Buffer(data + startIndex, endIndex - startIndex );
}
void Buffer::CopyTo(void* destination)
{
	Copy(data, (unsigned char*)destination, length);
}

void Buffer::Initialize()
{
	capacity = 1;
	data = new unsigned char[capacity];
	length = 0;
}
void Buffer::Initialize(unsigned char* initialData, int initialLength)
{
	Initialize();

	while (initialLength > capacity) SetCapacity(capacity * 2);
	Copy(initialData, data, initialLength);
	length = initialLength;
}

void Buffer::Copy(unsigned char* source, unsigned char* destination, int length, int start)
{
	for (int i = start; i < length+start; i++) destination[i] = source[i];
}
bool Buffer::Equals(unsigned char* data1, unsigned char* data2, int length)
{
	for (int i = 0; i < length; i++)
		if (data1[i] != data2[i])
			return false;

	return true;
}
