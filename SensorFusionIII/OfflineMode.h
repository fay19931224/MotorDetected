#ifndef OFFLINE_MODE_H
#define OFFLINE_MODE_H

#include "Mode.h"

/*!
* ��class�����o�v���H��Lidar��ƫ�A�ھڳ]�w���Z���b�v���W�i��ROI�����ΥH�Ϊ��󪺰����ΰl��
*/
class OfflineMode : public Mode 
{
private:
	string _videoFileName;	
	int _filterRoiDistance;
	int _waitKeySec;
	int _waitKeyChoosen;
	FusionType _type;
	bool WaitKey();	
	void Detect(Mat &frame, Mat &grayFrame);
	void pureDetect(Mat &frame, Mat &grayFrame);	

public:
	OfflineMode(string videoFileName, FusionType type, int currentModelType = 0);
	virtual ~OfflineMode();
	int GetChoiceVideoType();
	void Run();
	Rect adjustROI(Mat, Rect);
	vector<Rect> _posibleROI;	
};

#endif