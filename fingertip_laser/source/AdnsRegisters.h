#ifndef AdnsRegisters_h
#define AdnsRegisters_h

#pragma pack(push)

#pragma pack(1)

class AdnsRegisters
{
    public: unsigned    SensorID                    :  8;
	public: unsigned	ProductID					:  8;
	public: unsigned	RevisionID					:  8;

	public: unsigned	FirstPixel					:  1;
	public: unsigned	OperationMode				:  2;
	public: unsigned	/* Reserved */				:  2;
	public: unsigned	PowerValid					:  1;
	public: unsigned	Fault						:  1;
	public: unsigned	Motion						:  1;

	public: signed		DeltaX						: 16;
	public: signed		DeltaY						: 16;

	public: unsigned	SurfaceQuality				:  8;

	public: unsigned	PixelSum					:  8;
	public: unsigned	PixelMaximum				:  8;
	public: unsigned	PixelMinimum				:  8;

	public: unsigned	Shutter						: 16;

	public: unsigned	FramePeriod					: 16;

	public: unsigned	ResolutionX					:  6;
	public: unsigned	/* Reserved */				:  2;

	public: unsigned	/* Zeroes */				:  2;
	public: unsigned	IndependentResolution		:  1;
	public: unsigned	FixedFramerate				:  1;
	public: unsigned	NoAgc						:  1;
	public: unsigned	RestEnabled					:  1;
	public: unsigned	/* Set Operation Mode */	:  2;

	public: unsigned	Rest1Delay					:  8;
	public: unsigned	Rest1Rate					:  8;
	public: unsigned	Rest2Delay					:  8;
	public: unsigned	Rest2Rate					:  8;
	public: unsigned	Rest3Delay					:  8;
	public: unsigned	Rest3Rate					:  8;
	
	public: unsigned	FramePeriodMaximum			: 16;
	public: unsigned	FramePeriodMinimum			: 16;

	public: unsigned	ShutterMaximum				: 16;

	public: unsigned	NoForce						:  1;
	public: unsigned	DriveMode					:  3;
	public: unsigned	/* Reserved */				:  4;

	public: unsigned	Observation					:  6;
	public: unsigned	SRomCode					:  1;
	public: unsigned	/* Unused */				:  1;

	public: unsigned	SRomID						:  8;

	public: unsigned	LiftDetectionThreshold		:  5;
	public: unsigned	/* Reserved */				:  3;

	public: unsigned	ResolutionY					:  8;

	public: unsigned	/* Reserved */				:  1;
	public: unsigned	SRomSize					:  1;
	public: unsigned	/* Reserved */				:  6;

	public: unsigned	InverseProductID			:  8;

	public: void Initialize();
};

#pragma pack(pop)

#endif
