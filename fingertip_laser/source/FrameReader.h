#ifndef FrameReader_h
#define FrameReader_h

#include "AdnsReader.h"
#include <inttypes.h>

// Polynom for CRC-Sum
#define CRCPOLY_LE 0x4C11DB7

class FrameReader
{
	private: AdnsReader* reader;
	private: Buffer* frameBuffer;
	private: unsigned int pixelAverage;
	private: unsigned char pixelMinimum;
	private: unsigned char pixelMaximum;
	private: unsigned char* bitmap;
	private: unsigned char* bitmap2;
	private: unsigned char* bitmap3;
	private: unsigned char* bitmap4;
    private: unsigned char activeFrame;
    // bitmap displaying sensor active
    private: unsigned char activeSensors;
    private: unsigned char activeSensorCount;
    private: bool readAllPictures;
	
	private: int crcErrors;
	private: long crcChecks;
	
	public: Buffer* GetFrameBuffer();
	public: unsigned int GetPixelAverage();
	public: unsigned char GetPixelMinimum();
	public: unsigned char GetPixelMaximum();
    public: unsigned char getActiveFrame();
    public: unsigned char getActiveSensorMap();
    public: void setActiveSensorMap(unsigned char map);
	public: unsigned char* GetBitmap();
	public: unsigned char* GetBitmap(unsigned int sensID);
	public: bool IsFrameAvailable();
	public: double GetContrast();
    public: bool getReadAll();
	
	public: FrameReader(AdnsReader*);
	public: ~FrameReader();
	
	public: void Update();
	public: void UpdateAll();

	private: static uint32_t crc32(uint32_t crc, uint32_t *data, uint32_t len);
};

#endif
