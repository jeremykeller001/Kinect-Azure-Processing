#include "IOUtils.h"
#include "KinectAzureProcessor.h"
#include "Ply.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <float.h>
#include <math.h>
#include <Eigen/Dense>

using namespace std;

void usage(char* projectName) {
	cout << "Usage: " << projectName << " transform/file/path [options]" << endl <<
		"\tOptions:" << endl <<
		"\t-h | --help\t\t\tShow this help message" << endl <<
		"\t-t | --transform FILE_PATH\tFile path to transform configuration file" << endl <<
		"\t-c | --calibration\t\tRun in calibration mode" << endl <<
		"\t-f | --frame FRAME\t\tSpecify to only output an individual frame (default: 15 in calibration mode)" << endl <<
		"\t-d | --debug\t\t\tEnable debug mode logging and outputs" << endl <<
		"\t--disableMesh\t\t\tIf specified, meshing functionality and output will be disabled" << endl <<
		"\t--bodyTrackingOnly\t\tIf specified, only body tracking joint locations will be output" << endl;
}

int main(int argc, char** argv) {
	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	string captureDirectory = argv[1];
	string transformPath = "";
	bool calibrationMode = false;
	int frame = -1;
	bool debugMode = false;
	bool skipMesh = false;
	bool bodyTrackingOnly = false;
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
		else if (arg == "--bodyTrackingOnly") {
			bodyTrackingOnly = true;
		}
		else if (i != 1) {
			cerr << "Warning: unidentified argument sent in: " << arg << endl;
			cerr << "Would you like to continue? 'Y' / 'N'";
			string input;
			cin >> input;
			if (input == "n" || input == "N") {
				return 1;
			}
		}
	}

	if (!calibrationMode && transformPath == "") {
		cerr << "Warning: A transform file path was not provided, are you sure you want to continue? 'Y' / 'N'" << endl;
		string input;
		cin >> input;
		if (input == "n" || input == "N") {
			return 1;
		}
	}

	// Obtain Eigen transforms, body tracking suffix, and capture space bounds if exists
	std::string bodyTrackingFileSuffix = IOUtils::obtainBodyTrackingFileSuffix(transformPath);
	BodyTrackingUtils::BoundingBox captureSpaceBounds = IOUtils::obtainCaptureSpaceBounds(transformPath);
	unordered_map<string, Eigen::Matrix4Xd> transforms = IOUtils::readTransformationFile(transformPath);

	// Create KinectAzureUtils object and process captures
	KinectAzureProcessor kinectAzureProcessor(captureDirectory);
	kinectAzureProcessor.setBodyTrackingOutputOnly(bodyTrackingOnly);
	kinectAzureProcessor.setCalibrationMode(calibrationMode);
	kinectAzureProcessor.setDebugMode(debugMode);
	kinectAzureProcessor.setDisableMeshOutput(skipMesh);
	kinectAzureProcessor.setIndividualFrameIndex(frame);
	return kinectAzureProcessor.outputRecordingsToPlyFiles(transforms, bodyTrackingFileSuffix, captureSpaceBounds);
}