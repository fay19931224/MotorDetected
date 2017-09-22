#include "CasClassifier.h"

CasClassifier::CasClassifier(string featureName, ClassiferType type, Scalar rectangleColor, float threshold) : Classifier(type, rectangleColor, threshold)
{
	_cascade.load(featureName);
}

CasClassifier::~CasClassifier()
{
}

void CasClassifier::Classify(Mat &frame)
{
	_cascade.detectMultiScale(frame, _resultROI, 1.1, 3, CV_HAAR_SCALE_IMAGE, Size(64, 32));
}

void CasClassifier::Classify(Mat &frame, vector<Rect> &roiList)
{
}