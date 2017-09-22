#ifndef RECORD_MODE_H
#define RECORD_MODE_H

#include "Mode.h"
#include <string>
#include <fstream>
#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"

using namespace qrk;
using namespace std;


/*!
* ��class���n�i��v���Plidar�����s�ɩҨϥΡC
*/
class RecordMode : public Mode
{
private:
	string _videoFileName;
	string _lidarFileName;
	int _lidarDataQuantity;
	int _frameDataQuantity;
	Urg_driver _lidar;
	ofstream _lidarTextFile;

	string GetCurrentTime();
	bool InitializeLidar(string &deviceStateMessage);
	bool WriteLidarData(string &deviceStateMessage);

public:
	RecordMode(string videoFileName, string lidarFileName);
	virtual ~RecordMode();
	void Run();
};

#endif