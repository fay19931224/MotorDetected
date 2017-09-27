#ifndef FUSION_MANAGER_H
#define FUSION_MANAGER_H

#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <vector>
#include "ClassifierType.h"

using cv::FONT_HERSHEY_COMPLEX;
using cv::Mat;
using cv::Rect;
using cv::Scalar;
using cv::Size;
using cv::getTextSize;
using cv::Point;
using namespace std;

/*!
* 此class用來進行影像及Lidar資料的對齊，並能計算偵測物件的距離，且能對偵測物件進行標籤的動作及根據距離改變警示等級
*/
class FusionManager
{
private:
	enum DRIVING_STATE
	{
		EMPTY,
		LOW,
		MID,
		HIGH
	};

	const float FONT_SCALE = 0.5;
	const int FONT_FACE = FONT_HERSHEY_COMPLEX;
	const int TEXT_LINE_SPACE = 2;
	const int FONT_THICKNESS = 1;

	float CAMERA_ANGLE_ST;
	float CAMERA_ANGLE_ED;
	float CAMERA_ANGLE_TOTAL;
	float LIDAR_ANGLE_ST;
	float LIDAR_ANGLE_ED;
	float LIDAR_ANGLE_TOTAL;

	int LOW_THRES_DISTNACE;
	int MID_THRES_DISTANCE;
	DRIVING_STATE _currentState = DRIVING_STATE::EMPTY;

	float CalculateDistance(Mat &frame, int st, int ed, vector<long> &lidarDistanceData);

public:
	FusionManager();
	virtual ~FusionManager();
	int GetLowDistance();
	int GetMidDistance();
	float RequestDistance(Mat &frame, Rect& roi, vector<long> &lidarDistanceData);
	void AddInformationOnObject(Mat &frame, Rect& roi, ClassiferType classifierType, float distance, Scalar textColor);
	void AddInformationOnObject(Mat &frame, Rect& roi, string text, Scalar textColor);
	vector<Rect> FilterPosibleArea(Mat &frame, int stDistance, int edDistance, vector<long> &lidarDistanceData, int minimumRoiWidth, FusionType type = FusionType::CarFront);
	void ShowLidarBar(Mat &frame, vector<long> &lidarDistanceData);
	void InititalizeDistanceLimit(int lowDistance, int midDistance);
	void SyncLidarAndCamera(float lidarAngleSt = -95, float lidarAngleEd = 95, float cameraAngleSt = -30, float cameraAngleEd = 30);
	bool IsEmptyState();
	void ShowState(Mat &frame);

};

#endif