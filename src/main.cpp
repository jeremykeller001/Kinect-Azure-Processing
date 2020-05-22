#include "IOUtils.h"
#include "KinectAzureUtils.h"
#include "Ply.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <float.h>
#include <math.h>
#include <Eigen/Dense>
#include "PclUtils.h"

using namespace std;

int main(int argc, char** argv) {
	//std::string mkvDirectory = "C:\\Users\\Jeremy\\Desktop\\DU COB\\ForJeremy\\ForJeremy";
	//std::string transformFilePath = "C:\\Users\\Jeremy\\Desktop\\DU COB\\ForJeremy\\ForJeremy\\CalFeb17Transv2.txt";

	std::string mkvDirectory = "E:\\Kinect Azure\\Captures";
	std::string transformFilePath = "E:\\Kinect Azure\\CalFeb17Transv2.txt";

	if (argc < 1) {
		cerr << "Usage: project.exe transformFile recordingDirectory" << endl;
		return -1;
	}
	
	//string transformFilePath = string(argv[1]);
	//string mkvDirectory = string(argv[2]);

	try {
		return KinectAzureUtils::outputRecordingsToPlyFiles(mkvDirectory, transformFilePath);
	}
	//Catches all exceptions - bad practice
	catch (...) {
		// ...
	}

	//
	// Debug Only - For running PCL methods on a single point cloud file
	//
	// Read in Ply
	/*
	//ifstream ply("C:\\Users\\Jeremy\\git\\Kinect-Azure-Processor\\out\\build\\x64-Debug\\2cam.ply");
	ifstream ply("C:\\Users\\Jeremy\\\Desktop\\DU COB\\MkvTestFiles\\Group150filtered.ply");
	string str;

	bool proceed = false;

	while (!ply.eof()) {
		ply >> str;
		if (str.compare("end_header") == 0) {
			proceed = true;
			break;
		}
	}

	if (!proceed) {
		return -1;
	}

	Ply pointCloud;

	while (!ply.eof()) {
		Eigen::RowVector3d point;
		ply >> str;
		point(0) = atof(str.c_str());
		ply >> str;
		point(1) = atof(str.c_str());
		ply >> str;
		point(2) = atof(str.c_str());
		pointCloud.addPoint(point);
	}

	cout << "Finished reading file" << endl;
	ply.close();

	// Convert to pcl::PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = PclUtils::convertPlyToPointCloud(pointCloud);

	PclUtils::filterAndMesh(pclPointCloud, "poissonMesh4Cam");

	return 0;
	*/
}