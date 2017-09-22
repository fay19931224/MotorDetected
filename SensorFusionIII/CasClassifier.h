#ifndef CAS_CLASSIFIER_H
#define CAS_CLASSIFIER_H

#include "Classifier.h"

class CasClassifier : public Classifier
{
private:
	CascadeClassifier _cascade;

public:
	CasClassifier(string featureName, ClassiferType type, Scalar rectangleColor = Scalar(0, 255, 0), float threshold = 1);
	virtual ~CasClassifier();
	void Classify(Mat &frame);
	void Classify(Mat &frame, vector<Rect> &roiList);
};

#endif