//
// IOUtils.cpp
// Utilities dealing with file i/o
//
// Created by: Jeremy Keller
// 2/06/2020
//

#include "IOUtils.h"

#include "boost/filesystem.hpp" 
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

static string MKV_FILE_EXT = ".mkv";

bool IOUtils::endsWith(string const fileName, string const fileExtension) {
	if (fileName.length() > fileExtension.length()) {
		return (0 == fileName.compare(fileName.length() - fileExtension.length(), fileExtension.length(), fileExtension));
	}
	else {
		return false;
	}
}

vector<string> IOUtils::obtainMkvFilesFromDirectory(string dirPath) {
	vector<string> mkvFiles;
	for (const auto& entry : boost::filesystem::directory_iterator(dirPath)) {
		string fileName = entry.path().string();
		if (endsWith(fileName, MKV_FILE_EXT)) {
			mkvFiles.push_back(fileName);
		}
	}

	return mkvFiles;
}

Eigen::RowVector4d IOUtils::readTransformLine(std::string line) {
	Eigen::RowVector4d rowVector;
	istringstream iss(line);
	iss >> rowVector(0) >> rowVector(1) >> rowVector(2) >> rowVector(3);
	return rowVector;
}


unordered_map<string, Eigen::Matrix4Xd> IOUtils::readTransformationFile(string fileName) {
	string startChars = "###";
	unordered_map<string, Eigen::Matrix4Xd> fileTransformMap;
	ifstream infile(fileName);

	string line;
	while (getline(infile, line)) {
		// Look for start chars
		if (line.compare(startChars) == 0) {
			// Assume next 5 lines exist and are in correct format

			// File name
			string fileSuffix;
			if (getline(infile, line)) {
				fileSuffix = line;
			}
			else {
				throw "Error obtaining file name suffix in transformation matrix file. File name suffix must be specified in the line following the \"###\" regex.";
			}

			// 4x4 transform over the next 4 lines
			Eigen::Matrix4Xd transform(4, 4);
			for (int i = 0; i < 4; i++) {
				if (getline(infile, line)) {
					// Read line into row vector
					Eigen::RowVector4d transformRow = readTransformLine(line);

					// Assign row vector to 4x4 matrix elements
					for (int j = 0; j < 4; j++) {
						transform(i, j) = transformRow(j);
					}
				}
				else {
					// If line cannot be read, there is an issue with the transform file, throw exception
					throw "Error reading transformation matrix line.";
				}
			}

			fileTransformMap.insert({ fileSuffix, transform });
		}
	}

	return fileTransformMap;
}

std::string IOUtils::obtainBodyTrackingFileSuffix(std::string fileName) {
	string startChars = "///";
	ifstream infile(fileName);

	string fileNameSuffix = "";
	string line;
	while (getline(infile, line)) {
		if (line.compare(startChars) == 0) {
			// Assume next line is the body tracking file name
			if (getline(infile, fileNameSuffix)) {
				// Body tracking file suffix is found, no need to continue processing
				break;
			}
			else {
				throw "Error obtaining body tracking file suffix. Body tracking file name suffix must be specified in the line following the \"///\" regex.";
			}
		}
	}

	return fileNameSuffix;
}