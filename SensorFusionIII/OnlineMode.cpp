#include "OnlineMode.h"
#include "FusionManager.h"
#include "SvmClassifier.h"
#include <time.h>

OnlineMode::OnlineMode(FusionType type)
{
	//_fusionManager.SyncLidarAndCamera(-95, 95,-34.15,34.15); //front
	_fusionManager.SyncLidarAndCamera(-94.5, 95.5, -20, 20);  //back
	/*_classifierList.push_back(new SvmClassifier("Features\\pedestrianFeature.xml", ClassiferType::Pedestrian, Scalar(0, 0, 255), Size(40, 96), 1));
	if (type == FusionType::CarFront)
	{
		_classifierList.push_back(new SvmClassifier("Features\\vehicleBackFeature.xml", ClassiferType::Vehicle, Scalar(255, 0, 0), Size(64, 48), 1));
		_classifierList.push_back(new SvmClassifier("Features\\motorbikeBackFeature.xml", ClassiferType::Motorbike, Scalar(0, 255, 255), Size(56, 144), 0.8));
	}
	else if (type == FusionType::CarLeftSide || type == FusionType::CarRightSide)
	{
		//_classifierList.push_back(new SvmClassifier("Features\\vehicleRowFeature.xml", ClassiferType::Vehicle, Scalar(255, 0, 0), Size(64, 48), 1));
		//_classifierList.push_back(new SvmClassifier("Features\\motorbikeRowFeature.xml", ClassiferType::Motorbike, Scalar(0, 255, 255), Size(56, 144), 0.8));
	}
	else if (type == FusionType::CarBack)
	{
		_classifierList.push_back(new SvmClassifier("Features\\vehicleFrontFeature.xml", ClassiferType::Vehicle, Scalar(255, 0, 0), Size(64, 48), 1));
		_classifierList.push_back(new SvmClassifier("Features\\motorbikeFrontFeature.xml", ClassiferType::Motorbike, Scalar(0, 255, 255), Size(56, 144), 0.8));
	}*/
}

OnlineMode::~OnlineMode()
{
}

void OnlineMode::Run()
{
	string deviceStateMessage;
	Connection_information information;
	// Connects to the sensor
	if (!_lidar.open(information.device_or_ip_name(), information.baudrate_or_port_number(), information.connection_type()))
	{
		deviceStateMessage = "Urg_driver::open(): " + string(information.device_or_ip_name()) + ": " + string(_lidar.what());
		cout << deviceStateMessage;
	}
	// Case where the measurement range (start/end steps) is defined
	_lidar.set_scanning_parameter(_lidar.deg2step(-95), _lidar.deg2step(+95));

	VideoCapture inputVideo(0);
	while (true)
	{
		float st = clock();
		vector<long> lidarDistanceData;
		vector<unsigned short> lidarSignalData;
		long time_stamp = 0;
		_lidar.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);
		if (!_lidar.get_distance_intensity(lidarDistanceData, lidarSignalData, &time_stamp))
		{
			deviceStateMessage = "Urg_driver::get_distance(): " + string(_lidar.what());
			cout << deviceStateMessage << endl;
		}
		float ed = clock();
		float delay = ed - st;

		Mat frame;
		Mat grayFrame;
		inputVideo.read(frame);
		cvtColor(frame, grayFrame, CV_BGR2GRAY);
		vector<Rect> roi = _fusionManager.FilterPosibleArea(grayFrame,0, 1000, lidarDistanceData, 1);
		for (int j = 0; j < roi.size(); j++)
		{
			//float distance = _fusionManager.RequestDistance(grayFrame, roi[j], lidarDistanceData);
			rectangle(frame, roi[j], Scalar(0, 0, 255));
			//_fusionManager.AddInformationOnObject(frame, roi[j], ClassiferType::Vehicle, distance, Scalar(0, 0, 255));
		}

		//vector<Rect> posibleROI = _fusionManager.FilterPosibleArea(grayFrame, 500, lidarDistanceData, 10);
		//for (int i = 0; i < _classifierList.size(); i++)
		//{
		//	_classifierList[i]->Classify(grayFrame, posibleROI);
		//	_classifierList[i]->Update(frame, _fusionManager, lidarDistanceData);
		//}

		delay = max(1.0f, delay);
		imshow("", frame);
		if (waitKey(delay) == 27) //ESC
		{
			break;
		}
	}
	destroyAllWindows();
}