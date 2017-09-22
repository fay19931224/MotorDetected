#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H

#include "Classifier.h"
#include "PrimalSVM.h"

/*!
* ��class�Ψӳ]�w��������HOG��CELLSIZE�H�Ϊ�l��SVM�������A�ô��Ѥ�������k�C
*/
class SvmClassifier : public Classifier
{
private:
	const Size CELL_SIZE = Size(8, 8);
	const Size WINDOW_SIZE;
	HOGDescriptor _descriptor;
	PrimalSVM _svm;

public:
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor = Scalar(0, 255, 0), Size windowSize = Size(64, 128), float threshold=1);
	virtual ~SvmClassifier();
	void Classify(Mat &frame);
	void Classify(Mat &frame, vector<Rect> &roiList);
};

#endif