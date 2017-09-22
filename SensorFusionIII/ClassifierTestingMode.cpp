#include "ClassifierTestingMode.h"
#include "SvmClassifier.h"

ClassifierTestingMode::ClassifierTestingMode()
{
	//_classifierList.push_back(new SvmClassifier("K-fold\\1\\svmFeature.xml", ClassiferType::Vehicle, Scalar(255, 0, 0), Size(40, 64), 1));
	//_classifierList.push_back(new SvmClassifier("K-fold\\2\\svmFeature.xml", ClassiferType::Vehicle, Scalar(255, 0, 0), Size(40, 64), 1));
	//_classifierList.push_back(new SvmClassifier("K-fold\\3\\svmFeature.xml", ClassiferType::Vehicle, Scalar(255, 0, 0), Size(40, 64), 1));
	//_classifierList.push_back(new SvmClassifier("Features\\pedestrianFeature.xml", ClassiferType::Pedestrian, Scalar(0, 0, 255), Size(64, 144), 2));
	_classifierList.push_back(new SvmClassifier("Features\\svmFeature (2).xml", ClassiferType::Motorbike, Scalar(0, 255, 255), Size(160, 200), 1)); //orangle
	//_classifierList.push_back(new SvmClassifier("Features\\vehicleRowFeature_new.xml", ClassiferType::Vehicle, Scalar(255, 0, 0), Size(160, 120), 1.5));     //blue
}

ClassifierTestingMode::~ClassifierTestingMode()
{
}

void ClassifierTestingMode::Run()
{
	int per = 0;
	Mat frame;
	//VideoCapture capture("K-fold\\1\\test1.avi");
	//VideoCapture capture("K-fold\\2\\test2.mp4");
	//VideoCapture capture("K-fold\\3\\test3.avi");
	VideoCapture capture("Lidar Test Data\\CarRight\\videoData1216motor.avi");
	while (capture.read(frame))
	{
		//resize(frame, frame, Size(640, 480));
		Mat grayFrame;
		cvtColor(frame, grayFrame, CV_BGR2GRAY);

		for (int i = 0; i < _classifierList.size(); i++)
		{
			_classifierList[i]->Classify(grayFrame);
			_classifierList[i]->Update(frame);
		}
		//imwrite("K-fold\\1\\result\\" + to_string(per++) + ".jpg", frame);
		//imwrite("K-fold\\2\\result\\" + to_string(per++) + ".jpg", frame);
		//imwrite("K-fold\\3\\result\\" + to_string(per++) + ".jpg", frame);
		imshow("", frame);
		waitKey(1);
		printf("%d\n", per++);
		//cout << lidarHeader << endl;
		//system("pause");
	}
}