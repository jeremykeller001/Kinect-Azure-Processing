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
	std::string mkvDirectory = "C:\\Users\\Jeremy\\Desktop\\DU COB\\MkvTestFiles";
	std::string transformFilePath = "C:\\Users\\Jeremy\\Desktop\\DU COB\\MkvTestFiles\\Transforms.txt";

	return KinectAzureUtils::outputRecordingsToPlyFiles(mkvDirectory, transformFilePath);
	/*
	// Read in Ply
	ifstream ply("C:\\Users\\Jeremy\\Desktop\\DU COB\\MkvTestFiles\\160.ply");
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

	// apply Nearest Neighbor Filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPc (new pcl::PointCloud<pcl::PointXYZ>);
	PclUtils::applyNearestNeighborFilter(pclPointCloud,  filteredPc);
	PclUtils::outputToFile(filteredPc, "filteredPc.pcd");

	// Generate Mesh
	*/


	return 0;
}