#ifndef OFFLINE_MODE_H
#define OFFLINE_MODE_H

#include "Mode.h"

/*!
* 此class為取得影像以及Lidar資料後，根據設定的距離在影像上進行ROI的切割以及物件的偵測及追蹤
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