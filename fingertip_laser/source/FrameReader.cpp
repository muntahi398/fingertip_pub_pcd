#include "FrameReader.h"

#include <stdio.h>
#include <math.h>

Buffer* FrameReader::GetFrameBuffer()
{
	return frameBuffer;
}
unsigned int FrameReader::GetPixelAverage()
{
	return pixelAverage;
}
unsigned char FrameReader::GetPixelMinimum()
{
	return pixelMinimum;
}
unsigned char FrameReader::GetPixelMaximum()
{
	return pixelMaximum;
}
unsigned char FrameReader::getActiveFrame()
{
    return activeFrame;
}
unsigned char* FrameReader::GetBitmap()
{
	return bitmap;
}
unsigned char* FrameReader::GetBitmap(unsigned int sensID)
{
    switch(sensID){
        case 0 : return bitmap;
        case 1 : return bitmap2;
        case 2 : return bitmap3;
        case 3 : return bitmap4;
        default : return bitmap;
    }
}
unsigned char FrameReader::getActiveSensorMap()
{
    return activeSensors;
}
void FrameReader::setActiveSensorMap(unsigned char map)
{
    this->activeSensors = map;
}
bool FrameReader::IsFrameAvailable()
{
	return frameBuffer != NULL;
}
double FrameReader::GetContrast()
{
	if (frameBuffer == NULL) return 0;

	unsigned char* frameData = frameBuffer->GetData();
	double histogram[0x100];
	
	for (int i = 0; i < 0x100; i++) histogram[i] = 0;
	for (int i = 1; i < 901; i++) histogram[frameData[i-1]] += 1.0 / 900.0;
	
	double mean = 0;
	for (int i = 0; i < 0x100; i++) mean += i * histogram[i];
	
	double variance = 0;
	for (int i = 0; i < 0x100; i++) variance += (i - mean) * (i - mean) * histogram[i];
	
	double meanDeviation = fabs(0x80 - mean) / 0x80;
	double standardDeviation = sqrt(variance);
	
	//printf("Mean: %f, Variance: %f, Mean Deviation: %f, standardDeviation: %f\n", mean, variance, meanDeviation, standardDeviation);

	return meanDeviation * standardDeviation;
}
bool FrameReader::getReadAll(){
    return this->readAllPictures;
}

FrameReader::FrameReader(AdnsReader* reader)
{
	this->reader = reader;
	this->frameBuffer = NULL;
	this->bitmap = NULL;
	this->bitmap2 = NULL;
	this->bitmap3 = NULL;
	this->bitmap4 = NULL;
    this->activeFrame = 0;
    this->activeSensors = 0x0f;
    unsigned char* buffer = reader->Read("M",1)->GetData();
    this->activeSensors = buffer[0];
    
    this->activeSensorCount = 0;
    for(int i=0;i<4;i++)
        this->activeSensorCount += activeSensors & (0x1<<i) ? 1 : 0;
    this->readAllPictures = true;

	crcErrors = 0;
	crcChecks = 0;
}
FrameReader::~FrameReader()
{
	delete frameBuffer;
	delete[] bitmap;
}

void FrameReader::UpdateAll()
{
	delete frameBuffer;
	frameBuffer = reader->Read("i", 906);
	
	if (frameBuffer != NULL)
	{
		unsigned char* frameData = frameBuffer->GetData();

		pixelAverage = 0;
		pixelMinimum = 0xFF;
		pixelMaximum = 0x00;
		
        // sensor-id at index 0
        activeFrame = frameData[0];

		delete[] bitmap;
		bitmap = new unsigned char[900 * 4];
		
		for (int i = 0; i < 900; i++)
		{
			unsigned char value = frameData[i+1] << 1;
			
			pixelAverage += value;
			if (value < pixelMinimum) pixelMinimum = value;
			if (value > pixelMaximum) pixelMaximum = value;

			bitmap[i * 4 + 0] = value;
			bitmap[i * 4 + 1] = value;
			bitmap[i * 4 + 2] = value;
			bitmap[i * 4 + 3] = 0xFF;
		}

		pixelAverage /= 900;
		
		bool crcCheckCorrect = false;
		// crc-sum starts at index 902
		int i = 902;
		uint32_t remoteCheckSum = *((uint32_t*)(frameData+i));
		// compute local checksum
		uint32_t localCheckSum = 0xFFFFFFFF;
		localCheckSum = crc32(localCheckSum,(uint32_t*)(frameData+1), 225); //225 instead 900 because now using 32bit words
		crcChecks++;
		// print checksum-results
		crcCheckCorrect = remoteCheckSum==localCheckSum;
		crcErrors += !crcCheckCorrect?1:0;
		if (!crcCheckCorrect)
            printf("CRC:\r\n\tremote\t%08X\n\tlocal\t%08X\n\t%s\r\nerrors:\t%i\t%favg\r\n",
                remoteCheckSum,localCheckSum,
                crcCheckCorrect?"correct":"error",crcErrors,(float)(crcErrors/crcChecks));
	}
}
void FrameReader::Update()
{
    
    unsigned char* buffer = reader->Read("M",1)->GetData();
    this->activeSensors = buffer[0];
	delete frameBuffer;
    this->activeSensorCount = 0;
    for(int i=0;i<4;i++)
        this->activeSensorCount += this->activeSensors & (0x1<<i) ? 1 : 0;
        
    frameBuffer = reader->Read("j",activeSensorCount*907);

    for(int s=0;s<activeSensorCount;s++){
        Buffer *tempBuffer = frameBuffer->SubBuffer(s*907,(s+1)*907);
	    if (frameBuffer != NULL)
	    {
		    unsigned char* frameData = tempBuffer->GetData();
            pixelAverage = 0;
		    pixelMinimum = 0xFF;
		    pixelMaximum = 0x00;

            // sensor-id at index 0
            activeFrame = frameData[0];
		    
            if(activeFrame==0){
                delete[] bitmap;
                bitmap = new unsigned char[900 * 4];
		        for (int i = 0; i < 900; i++)
		        {
			        unsigned char value = frameData[i+1] << 1;
			
			        pixelAverage += value;
			        if (value < pixelMinimum) pixelMinimum = value;
			        if (value > pixelMaximum) pixelMaximum = value;
                        bitmap[i * 4 + 0] = value;
		                bitmap[i * 4 + 1] = value;
		                bitmap[i * 4 + 2] = value;
		                bitmap[i * 4 + 3] = 0xFF;
		        }
            }else if(activeFrame==1){
                delete[] bitmap2;
                bitmap2 = new unsigned char[900 * 4];
		        for (int i = 0; i < 900; i++)
		        {
			        unsigned char value = frameData[i+1] << 1;
			
			        pixelAverage += value;
			        if (value < pixelMinimum) pixelMinimum = value;
			        if (value > pixelMaximum) pixelMaximum = value;
                        bitmap2[i * 4 + 0] = value;
		                bitmap2[i * 4 + 1] = value;
		                bitmap2[i * 4 + 2] = value;
		                bitmap2[i * 4 + 3] = 0xFF;
		        }
            }else if(activeFrame==2){
                delete[] bitmap3;
                bitmap3 = new unsigned char[900 * 4];
		        for (int i = 0; i < 900; i++)
		        {
			        unsigned char value = frameData[i+1] << 1;
			
			        pixelAverage += value;
			        if (value < pixelMinimum) pixelMinimum = value;
			        if (value > pixelMaximum) pixelMaximum = value;
                        bitmap3[i * 4 + 0] = value;
		                bitmap3[i * 4 + 1] = value;
		                bitmap3[i * 4 + 2] = value;
		                bitmap3[i * 4 + 3] = 0xFF;
		        }
            }else if(activeFrame==3){
                delete[] bitmap4;
                bitmap4 = new unsigned char[900 * 4];
		        for (int i = 0; i < 900; i++)
		        {
			        unsigned char value = frameData[i+1] << 1;
			
			        pixelAverage += value;
			        if (value < pixelMinimum) pixelMinimum = value;
			        if (value > pixelMaximum) pixelMaximum = value;
                        bitmap4[i * 4 + 0] = value;
		                bitmap4[i * 4 + 1] = value;
		                bitmap4[i * 4 + 2] = value;
		                bitmap4[i * 4 + 3] = 0xFF;
		        }
            }
		    pixelAverage /= 900;
		
		    bool crcCheckCorrect = false;
		    // crc-sum starts at index 902
		    int i = 902;
		    uint32_t remoteCheckSum = *((uint32_t*)(frameData+i));
		    // compute local checksum
		    uint32_t localCheckSum = 0xFFFFFFFF;
		    localCheckSum = crc32(localCheckSum,(uint32_t*)(frameData+1), 225); //225 instead 900 because now using 32bit words
		    crcChecks++;
		    // print checksum-results
		    crcCheckCorrect = remoteCheckSum==localCheckSum;
		    crcErrors += !crcCheckCorrect?1:0;
		    if (!crcCheckCorrect) printf("CRC:\r\n\tremote\t%08X\n\tlocal\t%08X\n\t%s\r\nerrors:\t%i\t%f avg\r\n",remoteCheckSum,localCheckSum,crcCheckCorrect?"correct":"error",crcErrors,(float)(crcErrors/crcChecks));
	    }
    }
}


/* CRC-32 as in the STM32 CRC Module 

crc_model.cm_width = 32;            // 32-bit CRC
crc_model.cm_poly  = 0x04C11DB7;    // CRC-32 polynomial
crc_model.cm_init  = 0xFFFFFFFF;    // CRC initialized to 1's
crc_model.cm_refin = FALSE;         // CRC calculated MSB first
crc_model.cm_refot = FALSE;         // Final result is not bit-reversed
crc_model.cm_xorot = 0x00000000;    // Final result XOR'ed with this

http://www.ross.net/crc/download/crc_v3.txt
*/
uint32_t FrameReader::crc32(uint32_t crc, uint32_t *data, uint32_t len)
{
	int i;
	int j;
	uint32_t tmpWord;
	
	while (len--)
	{
		tmpWord = *data++;
		for (i = 0; i < 4; i++)
		{
			crc ^= tmpWord & 0xFF000000;
			tmpWord <<= 8;
			for (j = 0; j<8; j++)
			{
				crc = (crc << 1) ^ (crc & 0x80000000 ? CRCPOLY_LE : 0);
			}
		}
	}
	
	return crc;
	
	//linux kernel crc function
	/*int i;
	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? CRCPOLY_LE : 0);
	}
	return crc;*/
}
