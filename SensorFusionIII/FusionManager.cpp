#include "FusionManager.h"
#include <algorithm>

FusionManager::FusionManager()
{
}

FusionManager::~FusionManager()
{
}

/*!
* 回傳預設的近距離偵測的距離值
*/
int FusionManager::GetLowDistance()
{
	return LOW_THRES_DISTNACE;
}

/*!
* 回傳預設的中距離偵測的距離值
*/
int FusionManager::GetMidDistance()
{
	return MID_THRES_DISTANCE;
}

/*!
* 根據得到的距離決定警示狀態，以及根據分類器型別決定該物件屬於哪類並顯示於原始影像
* @param frame 為 Mat型態，為原始輸入影像
* @param roi 為 Rect型態，為原始影像切割出的ROI影像
* @param classifierType 為ClassiferType型態，為分類器的類別
* @param distance 為float型態，為從切割出的ROI計算出的距離
* @param textColor 為 Scalar型態，為字型使用的顏色
*/
void FusionManager::AddInformationOnObject(Mat &frame, Rect& roi, ClassiferType classifierType, float distance, Scalar textColor)
{
	DRIVING_STATE state = DRIVING_STATE::EMPTY;
	if (distance < LOW_THRES_DISTNACE)
	{
		state = DRIVING_STATE::HIGH;
	}
	else if (distance < MID_THRES_DISTANCE)
	{
		state = DRIVING_STATE::MID;
	}
	else
	{
		state = DRIVING_STATE::LOW;
	}
	if (state > _currentState)
	{
		_currentState = state;
	}
	
	vector<string> text;
	switch (classifierType)
	{
	case Vehicle:
		text.push_back("Vehicle");
		break;
	case Pedestrian:
		text.push_back("Pedestrian");
		break;
	case Motorbike:
		text.push_back("Motorbike");
		break;
	case UnKnown:
		text.push_back("UnKnown");
		break;
	}
	char s[20];
	sprintf(s, "%.2f", distance);
	text.push_back(string(s) + "mm");

	int baseline = 0;
	int totalTextHeight = (text.size() + 1)*TEXT_LINE_SPACE;
	for (int i = 0; i < text.size(); i++)
	{
		Size textSize = getTextSize(text[i], FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
		totalTextHeight += textSize.height + FONT_THICKNESS / 2;
	}

	int stY = roi.y + (roi.height - totalTextHeight) / 2;
	for (int i = 0; i < text.size(); i++)
	{
		Size textSize = getTextSize(text[i], FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
		stY += (textSize.height + TEXT_LINE_SPACE);
		putText(frame, text[i], Point(roi.x + roi.width / 2 - textSize.width / 2, stY), FONT_FACE, FONT_SCALE, textColor, FONT_THICKNESS);
	}
}


void FusionManager::AddInformationOnObject(Mat &frame, Rect& roi, string text, Scalar textColor)
{
	int baseline = 0;
	Size textSize = getTextSize(text, FONT_FACE, 0.33, FONT_THICKNESS, &baseline);
	//int totalTextHeight = 
		int stY = roi.y + textSize.height + FONT_THICKNESS / 2;
	putText(frame, text, Point(roi.x + roi.width / 2 - textSize.width / 2, stY), FONT_FACE, 0.33, textColor, FONT_THICKNESS);
}

/*!
* 排序使用，找出較小的資料
* @param p1 為 pair<int, long>型態，
* @param p2 為 pair<int, long>型態，
*/
bool isComp(pair<int, long> &p1, pair<int, long>&p2)
{
	return p1.second < p2.second;
}

/*!
* 取前1/10小的距離資料計算平均距離，再將前1/10小的距離資料中小於平均距離的資料取出進行計算，得到較為可信的距離
* @param frame 為 Mat型態，為切割過後的ROI影像
* @param st 為 int型態，為ROI右上方的像素值
* @param ed 為 int型態，為ROI左上方的像素值
* @param lidatDistanceData 為 vector<long>型態，為當前Frame的Lidar距離資料
* @return 回傳float型態的值，為計算出的距離值
*/
float FusionManager::CalculateDistance(Mat &frame, int st, int ed, vector<long> &lidarDistanceData)
{
	vector<pair<int, long>> leadList;
	for (int i = st; i < ed; i++)
	{
		leadList.push_back(make_pair(i, lidarDistanceData[i]));
	}

	sort(leadList.begin(), leadList.end(), isComp);
	float sumDistance = 0.0f;
	int per = leadList.size() / 10;
	int base = per;
	for (int i = 0; i < per; i++)
	{
		if (leadList[i].second == 120000)
		{
			base--;
			continue;
		}
		sumDistance += leadList[i].second;
	}
	sumDistance /= base;

	base = 0;
	float result = 0.0f;
	for (int i = 0; i < per; i++)
	{
		if (leadList[i].second < sumDistance)
		{
			result += leadList[i].second;
			base++;
		}
	}
	return result / base;
}

/*!
* 對齊影像以及Lidar資料，並將計算結果傳給CalculateDistance計算距離
* @param frame 為 Mat型態，為輸入的原始影像
* @param roi 為Rect型態，為從原始影像切割後的ROI影像
* @param lidarDistanceData 為 vector<long>型態，用來存放當前Frame的Lidar距離資料
*/
float FusionManager::RequestDistance(Mat &frame, Rect& roi, vector<long> &lidarDistanceData)
{
	int dataLen = lidarDistanceData.size();
	float total = CAMERA_ANGLE_TOTAL*dataLen / LIDAR_ANGLE_TOTAL;
	int st = (CAMERA_ANGLE_ST - LIDAR_ANGLE_ST)*dataLen / LIDAR_ANGLE_TOTAL;
	int ed = st + total;
	int posEd = roi.x + roi.width;
	return CalculateDistance(frame, ed - (posEd - 1)*total / frame.cols, ed - roi.x*total / frame.cols, lidarDistanceData);
}

/*!
* 補償Lidar資料，透過距離及fusion類型決定ROI高度比例，並回傳切割出的ROI區域結果
* @param frame 為 Mat型態，為輸入的原始影像
* @param stDistance 為int型態，為偵測的起始距離
* @param edDistnace 為inr型態，為偵測的最遠距離
* @param lidarDistanceData 為vector<long>型態，用來存放當前Frame的Lidar距離資料
* @param minimumRoiWidth 為int型態，為判斷切出來的ROI是否過小的依據
* @param type 為 FusionType型態，為當前使用的fusion方式
* @return 回傳vector<cv::Rect>的roiList，為切割出的ROI區域
*/
vector<Rect> FusionManager::FilterPosibleArea(Mat &frame, int stDistance, int edDistance, vector<long> &lidarDistanceData, int minimumRoiWidth,FusionType type)
{
	float dataLen = lidarDistanceData.size();
	float total = CAMERA_ANGLE_TOTAL*dataLen / LIDAR_ANGLE_TOTAL;
	int st = (CAMERA_ANGLE_ST - LIDAR_ANGLE_ST)*dataLen / LIDAR_ANGLE_TOTAL;
	int ed = st + total;

	int offset = 4 * dataLen / LIDAR_ANGLE_TOTAL;
	bool *flag = new bool[lidarDistanceData.size()];
	memset(flag, false, sizeof(bool)*lidarDistanceData.size());
	for (int i = st; i < ed; i++)
	{
		if (stDistance <= lidarDistanceData[i] && lidarDistanceData[i] <= edDistance)
		{
			//for detect lidar accuracy
			flag[i] = true;
			int len = i + offset;
			for (int j = i - offset; j <= len; j++)
			{
				if (st <= j && j < ed)
				{
					flag[j] = true;
				}
			}
		}
	}

	int stIndex;
	bool isFirst = true;
	vector<Rect> roiList;
	for (int i = st; i < ed; i++)
	{
		if (flag[i] && isFirst)
		{
			stIndex = i;
			isFirst = false;
		}
		else if ((!flag[i] && !isFirst) || (flag[i] && i == ed - 1 && !isFirst))
		{
			int x = (ed - i)*frame.cols / total;
			int width = (i - stIndex)*frame.cols / total;
			if (width >= minimumRoiWidth)
			{
				int heightSt;
				int height;
				float distance = CalculateDistance(frame, stIndex, i, lidarDistanceData);

				//determin roi by distance of lidar
				if (distance < LOW_THRES_DISTNACE)
				{
					heightSt = 0;
					height = frame.rows - 1;
				}
				else if (type==FusionType::CarLeftSide)
				{
					heightSt = 2.0 / 10.0*frame.rows;
					height = 7.0 / 10.0*frame.rows;
				}
				else if (type == FusionType::CarRightSide)
				{
					heightSt = 4.0 / 10.0*frame.rows;
					height = 6.0 / 10.0*frame.rows;
				}
				else
				{
					heightSt = 1.0 / 7.0*frame.rows;
					height = 3.0 / 5.0*frame.rows;
				}

				roiList.push_back(Rect(x, heightSt, width, height));
			}
			isFirst = true;
		}
	}

	delete flag;
	return roiList;
}

void FusionManager::ShowLidarBar(Mat &frame, vector<long> &lidarDistanceData)
{
	int dataLen = lidarDistanceData.size();
	int total = CAMERA_ANGLE_TOTAL*dataLen / LIDAR_ANGLE_TOTAL;
	int st = (CAMERA_ANGLE_ST - LIDAR_ANGLE_ST)*dataLen / LIDAR_ANGLE_TOTAL;
	int ed = st + total;

	int barHeight = 10;
	int horizon = frame.rows*0.57;
	for (int i = 0; i < frame.cols; i++)
	{
		int index = ed - i*total / frame.cols;
		int value = (float)lidarDistanceData[index] / 120000 * 255;
		line(frame, Point(i, horizon - barHeight / 2), Point(i, horizon + barHeight / 2), Scalar(value, value, value));
	}
}

/*!
檢查當前狀態是否為Empty，Empty表示畫面沒有偵側到目標物件
*/
bool FusionManager::IsEmptyState()
{
	return _currentState == DRIVING_STATE::EMPTY;
}

/*!
* 將偵測後的狀態顯示於當前影像上
* @param frame 為Mat型態，為輸入的原始影像
*/
void FusionManager::ShowState(Mat &frame)
{
	if (_currentState == DRIVING_STATE::HIGH)
	{
		circle(frame, Point(frame.cols - 20, 20), 10, Scalar(0, 0, 255), -1);
	}
	else if (_currentState == DRIVING_STATE::MID)
	{
		circle(frame, Point(frame.cols - 20, 20), 10, Scalar(0, 255, 255), -1);
	}
	else if (_currentState == DRIVING_STATE::LOW)
	{
		circle(frame, Point(frame.cols - 20, 20), 10, Scalar(0, 255, 0), -1);
	}
	_currentState = DRIVING_STATE::EMPTY;
}

/*!
* 初始化預設的近距離與中距離數值
* @param lowDistance 為int型態，為預設的近距離值
* @param midDistance 為int型態，為預設的中距離值
*/
void FusionManager::InititalizeDistanceLimit(int lowDistance, int midDistance)
{
	LOW_THRES_DISTNACE = lowDistance;
	MID_THRES_DISTANCE = midDistance;
}

/*!
* 輸入影像及Lidar資料的角度資料，並計算影像及Lidar偵測的度數範圍
* @param lidarAngleSt 為float型態，為Lidar資料的起始度數
* @param lidarAngleEd 為float型態，為Lidar資料的最大度數
* @param cameraAngleSt 為float型態，為影像資料的起始度數
* @param cameraAngleEd 為float型態，為影像資料的最大度數
*/
void FusionManager::SyncLidarAndCamera(float lidarAngleSt, float lidarAngleEd, float cameraAngleSt, float cameraAngleEd)
{
	LIDAR_ANGLE_ST = lidarAngleSt;
	LIDAR_ANGLE_ED = lidarAngleEd;
	LIDAR_ANGLE_TOTAL = LIDAR_ANGLE_ED - LIDAR_ANGLE_ST;
	CAMERA_ANGLE_ST = cameraAngleSt;
	CAMERA_ANGLE_ED = cameraAngleEd;
	CAMERA_ANGLE_TOTAL = CAMERA_ANGLE_ED - CAMERA_ANGLE_ST;
}