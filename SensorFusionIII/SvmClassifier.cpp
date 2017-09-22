#include "SvmClassifier.h"


/*!
* ��l��SVM������
* @param featureName ��string �����A��Ū�J��SVM�ҫ��W��
* @param type ��ClassiferType �����A������������
* @param rectangleColor ��Scalar �����A����������������ɨϥΪ��ت��C��
* @param windowSize ��Size �����A�����f�j�p�A�P�V�m�ɥ��˥���SIZE�ۦP
* @param threshold ��float �����A�����������e�ȡA�ȶV�C�N�V�e�P�A�ȶV���N�V�Y��
*/
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, Size windowSize, float threshold) : Classifier(type, rectangleColor, threshold), WINDOW_SIZE(windowSize)
{
	_svm.load(featureName.c_str());
	_descriptor = HOGDescriptor(windowSize, Size(CELL_SIZE.width * 2, CELL_SIZE.height * 2), CELL_SIZE, CELL_SIZE, 9);
	vector<float> hogVector;
	_svm.getSupportVector(hogVector);
	_descriptor.setSVMDetector(hogVector);
}

SvmClassifier::~SvmClassifier()
{
}

void SvmClassifier::Classify(Mat &frame)
{
	_descriptor.detectMultiScale(frame, _resultROI, THRESHOLD, CELL_SIZE);
	
}


/*!
* �NROI�v�������󰻴����G������_resultROI
* @param frame ��Mat �����A����l��J�v��
* @param roiList ��vector<cv::Rect> �����A�����ιL�᪺ROI�v��
*/
void SvmClassifier::Classify(Mat &frame, vector<Rect> &roiList)
{
	_resultROI.clear();
	for (int i = 0; i < roiList.size(); i++)
	{
		if (roiList[i].width < WINDOW_SIZE.width || roiList[i].height < WINDOW_SIZE.height)//?
		{
			continue;
		}
		vector<Rect> resultList;
		_descriptor.detectMultiScale(Mat(frame, roiList[i]), resultList, THRESHOLD, CELL_SIZE);		            
		for (int j = 0; j < resultList.size(); j++)
		{
			_resultROI.push_back(Rect(resultList[j].x + roiList[i].x, resultList[j].y + roiList[i].y, resultList[j].width, resultList[j].height));
		}
	}
}