//
// IOUtils.cpp
// Utilities dealing with file i/o
//
// Created by: Jeremy Keller
// 2/06/2020
//

#include "IOUtils.h"

#include <filesystem>
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
	for (const auto& entry : filesystem::directory_iterator(dirPath)) {
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
				// throw exception
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
					// TODO: If line cannot be read, there is an issue with the transform file, throw exception
				}
			}

			fileTransformMap.insert({ fileSuffix, transform });
		}
	}

	return fileTransformMap;
}