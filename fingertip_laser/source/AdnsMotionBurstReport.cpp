#include "AdnsMotionBurstReport.h"

#include <arpa/inet.h>

void AdnsMotionBurstReport::Initialize()
{
	Shutter = ntohs(Shutter);
	FramePeriod = ntohs(FramePeriod);
}
