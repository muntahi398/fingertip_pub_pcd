#ifndef AdnsMotionBurstReport_h
#define AdnsMotionBurstReport_h

#pragma pack(push)

#pragma pack(1)

class AdnsMotionBurstReport
{
        public: unsigned        SensorID                :  8;
	public: unsigned	FirstPixel		:  1;
	public: unsigned	OperationMode	        :  2;
	public: unsigned	/* Reserved */	        :  2;
	public: unsigned	PowerValid		:  1;
	public: unsigned	Fault			:  1;
	public: unsigned	Motion			:  1;

	public: unsigned	Observation		:  6;
	public: unsigned	SRomCode		:  1;
	public: unsigned	/* Unused */	:  1;

	public: signed		DeltaX			: 16;
	public: signed		DeltaY			: 16;

	public: unsigned	SurfaceQuality	        :  8;

	public: unsigned	PixelSum		:  8;
	public: unsigned	PixelMaximum	        :  8;
	public: unsigned	PixelMinimum	        :  8;

	public: unsigned	Shutter			: 16;

	public: unsigned	FramePeriod		: 16;

	public: void Initialize();
};

#pragma pack(pop)

#endif
