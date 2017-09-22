#ifndef REAL_TIME_MODE_H
#define REAL_TIME_MODE_H

#include "Mode.h"
#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"

using namespace qrk;

class OnlineMode : public Mode
{
private:
	Urg_driver _lidar;

public:
	OnlineMode(FusionType type);
	virtual ~OnlineMode();
	void Run();
};

#endif