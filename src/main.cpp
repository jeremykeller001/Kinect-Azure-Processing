#include "IOUtils.h"
#include "KinectAzureUtils.h"
#include "Ply.h"

int main(int argc, char** argv) {
	std::string mkvDirectory = "C:\\Users\\Jeremy\\Desktop\\DU COB\\mkvRecordings";

	return KinectAzureUtils::outputRecordingsToPlyFiles(mkvDirectory);
}