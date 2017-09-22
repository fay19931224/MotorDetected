#include "RecordMode.h"
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <time.h>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace cv;
using namespace std::chrono;


/*!
* 初始化影像名稱以及LIDAR檔案名稱
*/
RecordMode::RecordMode(string videoFileName, string lidarFileName)
{
	_videoFileName = videoFileName;
	_lidarFileName = lidarFileName;
}

RecordMode::~RecordMode()
{
	_lidarTextFile.close();
}


/*!
* 取得當前時間
*/
string RecordMode::GetCurrentTime()
{
	system_clock::time_point today = system_clock::now();
	time_t currentTime = system_clock::to_time_t(today);
	struct tm *date = localtime(&currentTime);

	stringstream s;
	s << put_time(date, "%Y-%m-%d %H:%M:%S.");
	char ss[5];
	sprintf(ss, "%03d", duration_cast<milliseconds>(today.time_since_epoch()).count() % 1000);
	s << string(ss);
	return s.str();
}

/*!
* 確認lidar是否已經正確連上
*/
bool RecordMode::InitializeLidar(string &deviceStateMessage)
{
	_lidarTextFile.open(_lidarFileName.c_str(), ios::out);
	if (!_lidarTextFile.is_open())
	{
		deviceStateMessage = "Lidar text file error!";
		return false;
	}

	Connection_information information;
	// Connects to the sensor
	if (!_lidar.open(information.device_or_ip_name(), information.baudrate_or_port_number(), information.connection_type()))
	{
		deviceStateMessage = "Urg_driver::open(): " + string(information.device_or_ip_name()) + ": " + string(_lidar.what());
		return false;
	}
	// Case where the measurement range (start/end steps) is defined
	_lidar.set_scanning_parameter(_lidar.deg2step(-95), _lidar.deg2step(+95));
	return true;
}

/*!
* 寫入LIDAR資料
*/
bool RecordMode::WriteLidarData(string &deviceStateMessage)
{
	bool isSuccess = true;
	vector<long> data;
	vector<unsigned short> intensity;
	long time_stamp = 0;
	_lidar.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);
	if (!_lidar.get_distance_intensity(data, intensity, &time_stamp))
	{
		deviceStateMessage = "Urg_driver::get_distance(): " + string(_lidar.what());
		isSuccess = false;
	}

	int totalLidarAngleQuantity = data.size();
	_lidarTextFile << totalLidarAngleQuantity << " " << GetCurrentTime() << endl;
	for (int i = 0; i < totalLidarAngleQuantity; ++i)
	{
		_lidarTextFile << data[i] << ", " << intensity[i] << endl;
	}
	_lidarDataQuantity++;
	return isSuccess;
}

/*!
* 錄製影片並寫入lidar資料，並輸出成功訊息，以及frame及LIDAR的筆數資料，用以確認數量是否相同
*/
void RecordMode::Run()
{
	string deviceStateMessage;
	if (InitializeLidar(deviceStateMessage))
	{
		const float FPS = 10;
		VideoCapture inputVideo(1);
		inputVideo.set(CV_CAP_PROP_FPS, FPS);
		Size videoSize = Size((int)inputVideo.get(CV_CAP_PROP_FRAME_WIDTH), (int)inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));
		VideoWriter outputVideo(_videoFileName, CV_FOURCC('D', 'I', 'V', 'X'), FPS, videoSize);
		while (true)
		{
			float st = clock();
			bool isLidarWorkFine = WriteLidarData(deviceStateMessage);
			if (!isLidarWorkFine)
			{
				cout << deviceStateMessage << endl;
			}
			float ed = clock();
			float delay = ed - st;

			Mat frame;
			Mat grayFrame;
			inputVideo.read(frame);
			outputVideo.write(frame);
			_frameDataQuantity++;

			delay = max(1.0f, delay);
			imshow("", frame);
			if (waitKey(delay) == 27) //ESC
			{
				deviceStateMessage = "Record Success, total " + to_string(_lidarDataQuantity) + " lidar data, " + to_string(_frameDataQuantity) + " frame data";
				break;
			}
		}
		destroyAllWindows();
	}
	cout << deviceStateMessage << endl;
}