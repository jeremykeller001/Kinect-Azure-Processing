#include "IOUtils.h"
#include "KinectAzureUtils.h"
#include "Ply.h"

int main(int argc, char** argv) {
	std::string mkvDirectory = "C:\\Users\\Jeremy\\Desktop\\DU COB\\mkvRecordings";
	std::string transformFilePath = "C:\\Users\\Jeremy\\Desktop\\DU COB\\4x4TransformCalTemplate.txt";

	return KinectAzureUtils::outputRecordingsToPlyFiles(mkvDirectory, transformFilePath);
}