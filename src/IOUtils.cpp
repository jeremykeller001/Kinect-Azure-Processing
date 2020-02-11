//
// IOUtils.cpp
// Utilities dealing with file i/o
//
// Created by: Jeremy Keller
// 2/06/2020
//

#include "IOUtils.h"

#include <filesystem>

using namespace std;

static string MKV_FILE_EXT = ".mkv";

bool endsWith(string const fileName, string const fileExtension) {
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