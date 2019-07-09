#include "AdnsRegisters.h"

#include <arpa/inet.h>

void AdnsRegisters::Initialize()
{
	Shutter = ntohs(Shutter);
	FramePeriod = ntohs(FramePeriod);
	FramePeriodMaximum = ntohs(FramePeriodMaximum);
	FramePeriodMinimum = ntohs(FramePeriodMinimum);
	ShutterMaximum = ntohs(ShutterMaximum);
}
