#ifndef RECORD_MODE_H
#define RECORD_MODE_H

#include "Mode.h"
#include <string>
#include <fstream>
#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <time.h>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace qrk;
using namespace std;
using namespace std::chrono;
using cv::Size;
using cv::VideoCapture;
using cv::VideoWriter;
using cv::Mat;
using cv::waitKey;
using cv::destroyAllWindows;


/*!
* 此class為要進行影像與lidar的錄製時所使用。
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