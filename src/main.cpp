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

	//std::string mkvDirectory = "C:\\Users\\Jeremy\\Desktop\\DU COB\\2CameraCapture\\2CameraCapture";
	//std::string transformFilePath = "C:\\Users\\Jeremy\\Desktop\\DU COB\\2CameraCapture\\2CameraCapture\\CalMar5.txt";

	
	if (argc < 3) {
		cerr << "Usage: project.exe transformFile recordingDirectory" << endl;
		return -1;
	}
	

	string transformFilePath = string(argv[1]);
	string mkvDirectory = string(argv[2]);
	return KinectAzureUtils::outputRecordingsToPlyFiles(mkvDirectory, transformFilePath);

	return 0;
}