#include "IOUtils.h"

#include <filesystem>

using namespace std;

static string MKV_FILE_EXT = ".mkv";

static vector<string> IOUtils::obtainMkvFilesFromDirectory(string dirPath) {
	vector<string> mkvFiles;
	for (const auto& entry : fs::directory_iterator(dirPath)) {
		string fileName = entry.path().toString();
		if (endsWith(&fileName, MKV_FILE_EXT)) {
			mkvFiles.push_back(fileName);
		}
	}

	return mkvFiles;
}

bool endsWith(string const& fileName, string const fileExtension) {
	if (fileName.length() > fileExtension.length()) {
		return (0 == fileName.compare (fileName.length() - fileExtension.length(), fileExtension.length(), fileExtension))
	}
	else {
		return false;
	}
}