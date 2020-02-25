#include "IOUtils.h"
#include "KinectAzureUtils.h"
#include "Ply.h"

int main(int argc, char** argv) {
	std::string mkvDirectory = "C:\\Users\\Jeremy\\Desktop\\DU COB\\mkvRecordings";
	std::string transformFile = "C:\\Users\\Jeremy\\Desktop\\DU COB\\4x4TransformCalTemplate.txt";

	std::unordered_map<std::string, Eigen::Matrix4Xd> transforms = IOUtils::readTransformationFile(transformFile);
	transforms.at("Master.mkv");
	//return KinectAzureUtils::outputRecordingsToPlyFiles(mkvDirectory);
}