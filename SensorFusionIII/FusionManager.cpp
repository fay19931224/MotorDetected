#include "FusionManager.h"
#include <algorithm>

FusionManager::FusionManager()
{
}

FusionManager::~FusionManager()
{
}

/*!
* �^�ǹw�]����Z���������Z����
*/
int FusionManager::GetLowDistance()
{
	return LOW_THRES_DISTNACE;
}

/*!
* �^�ǹw�]�����Z���������Z����
*/
int FusionManager::GetMidDistance()
{
	return MID_THRES_DISTANCE;
}

/*!
* �ھڱo�쪺�Z���M�wĵ�ܪ��A�A�H�ήھڤ��������O�M�w�Ӫ����ݩ��������ܩ��l�v��
* @param frame �� Mat���A�A����l��J�v��
* @param roi �� Rect���A�A����l�v�����ΥX��ROI�v��
* @param classifierType ��ClassiferType���A�A�������������O
* @param distance ��float���A�A���q���ΥX��ROI�p��X���Z��
* @param textColor �� Scalar���A�A���r���ϥΪ��C��
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
* �ƧǨϥΡA��X���p�����
* @param p1 �� pair<int, long>���A�A
* @param p2 �� pair<int, long>���A�A
*/
bool isComp(pair<int, long> &p1, pair<int, long>&p2)
{
	return p1.second < p2.second;
}

/*!
* ���e1/10�p���Z����ƭp�⥭���Z���A�A�N�e1/10�p���Z����Ƥ��p�󥭧��Z������ƨ��X�i��p��A�o������i�H���Z��
* @param frame �� Mat���A�A�����ιL�᪺ROI�v��
* @param st �� int���A�A��ROI�k�W�誺������
* @param ed �� int���A�A��ROI���W�誺������
* @param lidatDistanceData �� vector<long>���A�A����eFrame��Lidar�Z�����
* @return �^��float���A���ȡA���p��X���Z����
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
* ����v���H��Lidar��ơA�ñN�p�⵲�G�ǵ�CalculateDistance�p��Z��
* @param frame �� Mat���A�A����J����l�v��
* @param roi ��Rect���A�A���q��l�v�����Ϋ᪺ROI�v��
* @param lidarDistanceData �� vector<long>���A�A�ΨӦs���eFrame��Lidar�Z�����
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
* ���vLidar��ơA�z�L�Z����fusion�����M�wROI���פ�ҡA�æ^�Ǥ��ΥX��ROI�ϰ쵲�G
* @param frame �� Mat���A�A����J����l�v��
* @param stDistance ��int���A�A���������_�l�Z��
* @param edDistnace ��inr���A�A���������̻��Z��
* @param lidarDistanceData ��vector<long>���A�A�ΨӦs���eFrame��Lidar�Z�����
* @param minimumRoiWidth ��int���A�A���P�_���X�Ӫ�ROI�O�_�L�p���̾�
* @param type �� FusionType���A�A����e�ϥΪ�fusion�覡
* @return �^��vector<cv::Rect>��roiList�A�����ΥX��ROI�ϰ�
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
�ˬd��e���A�O�_��Empty�AEmpty��ܵe���S��������ؼЪ���
*/
bool FusionManager::IsEmptyState()
{
	return _currentState == DRIVING_STATE::EMPTY;
}

/*!
* �N�����᪺���A��ܩ��e�v���W
* @param frame ��Mat���A�A����J����l�v��
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
* ��l�ƹw�]����Z���P���Z���ƭ�
* @param lowDistance ��int���A�A���w�]����Z����
* @param midDistance ��int���A�A���w�]�����Z����
*/
void FusionManager::InititalizeDistanceLimit(int lowDistance, int midDistance)
{
	LOW_THRES_DISTNACE = lowDistance;
	MID_THRES_DISTANCE = midDistance;
}

/*!
* ��J�v����Lidar��ƪ����׸�ơA�íp��v����Lidar�������׼ƽd��
* @param lidarAngleSt ��float���A�A��Lidar��ƪ��_�l�׼�
* @param lidarAngleEd ��float���A�A��Lidar��ƪ��̤j�׼�
* @param cameraAngleSt ��float���A�A���v����ƪ��_�l�׼�
* @param cameraAngleEd ��float���A�A���v����ƪ��̤j�׼�
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