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

void usage(char* projectName) {
	cout << "Usage: " << projectName << " [options]" << endl <<
		"	Options:" << endl <<
		"	-h | --help\t\t\tShow this help message" << endl <<
		"	-t | --transform FILE_PATH\tFile path to transform file" << endl <<
		"	-c | --calibration\t\tRun in calibration mode" << endl <<
		"	-f | --frame FRAME\t\tSpecify to only output an individual frame (default: 15 in calibration mode)" << endl <<
		"	-d | --debug\t\t\tEnable debug mode logging and outputs" << endl <<
		"	--disableMesh\t\t\tIf specified, meshing functionality and output will be disabled" << endl;
}

int main(int argc, char** argv) {
	//std::string mkvDirectory = "F:\\DU COB\\Walk2";
	//std::string transformFilePath = "F:\\DU COB\\Walk2\\trans2.txt";

	std::string mkvDirectory = "E:\\Kinect Azure\\Captures";
	std::string transformFilePath = "E:\\Kinect Azure\\CalFeb17Transv2.txt";

	if (argc < 1) {
		cerr << "Usage: project.exe transformFile recordingDirectory" << endl;
		return -1;
	}
	
	//string transformFilePath = string(argv[1]);
	//string mkvDirectory = string(argv[2]);

	try {
		return KinectAzureUtils::outputRecordingsToPlyFiles(mkvDirectory, transformFilePath, 250, true, false, true);
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
=======
	//std::string mkvDirectory = "C:\\Users\\Jeremy\\Desktop\\DU COB\\Walk";
	//std::string transformFilePath = "C:\\Users\\Jeremy\\Desktop\\DU COB\\Walk\\CalApril29Trans.txt";

	
	if (argc < 2) {
		usage(argv[0]);
		return 1;
>>>>>>> master
	}

	string captureDirectory = argv[1];
	string transformPath = "";
	bool calibrationMode = false;
	int frame = -1;
	bool debugMode = false;
	bool skipMesh = false;
	for (int i = 1; i < argc; i++) {
		string arg = argv[i];
		if ((arg == "-h") || (arg == "--help")) {
			usage(argv[0]);
			return 0;
		}
		else if ((arg == "-t") || (arg == "--transform")) {
			if (i + 1 < argc) {
				transformPath = argv[++i];
			}
			else {
				cerr << "--transformation option requires a file path argument to be specified." << endl;
			}
		}
		else if ((arg == "-c") || (arg == "--calibration")) {
			calibrationMode = true;
			if (frame == -1) {
				// Set default frame if it has not already been specified
				frame = 15;
			}
		}
		else if ((arg == "-f") || (arg == "--frame")) {
			if (i + 1 < argc) {
				frame = atoi(argv[++i]);
			}
			else {
				cerr << "--frame option requires a frame number argument to be specified." << endl;
			}
		}
		else if ((arg == "-d") || (arg == "--debug")) {
			debugMode = true;
		}
		else if (arg == "--disableMesh") {
			skipMesh = true;
		}
	}
	
	return KinectAzureUtils::outputRecordingsToPlyFiles(captureDirectory, transformPath, frame, calibrationMode, debugMode, skipMesh);

	*/
}