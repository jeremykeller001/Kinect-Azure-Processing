//
// KinectAzureUtils.cpp
// Custom Utilities dealing with the Kinect Azure SDK
//
// Created by: Jeremy Keller
// 2/03/2020
//


#include "KinectAzureProcessor.h"
#include "BodyTrackingUtils.h"
#include <regex>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

using boost::property_tree::ptree;

void KinectAzureProcessor::print_capture_info(recording_t* file)
{
	k4a_image_t image = k4a_capture_get_depth_image(file->capture);

	printf("%-32s", file->filename.c_str());

	if (image != NULL)
	{
		uint64_t timestamp = k4a_image_get_device_timestamp_usec(image);
		printf("  %7ju usec", timestamp);
		k4a_image_release(image);
		image = NULL;
	}
	else
	{
		printf("  %12s", "");
	}
	printf("\n");
}

uint64_t KinectAzureProcessor::first_capture_timestamp(k4a_capture_t capture)
{
	uint64_t timestamp = k4a_image_get_device_timestamp_usec(k4a_capture_get_depth_image(capture));
	k4a_image_release(k4a_capture_get_depth_image(capture));
	return timestamp;
}

std::string KinectAzureProcessor::getStartRecording(recording_t* files, std::vector<CalibrationInfo> frameInfo, int fileCount) {
	std::unordered_map<uint64_t, TimestampCounter> timestampDifferenceCounts;

	// Count the occurences of each timestamp difference
	for (int i = 0 + 1; i < frameInfo.size(); i++) {
		// Incremement timestampDifferenceCounts
		uint64_t timestampDiff = frameInfo.at(i).timestampDiff;
		std::string fileName = frameInfo.at(i).fileName;
		if (timestampDifferenceCounts.count(timestampDiff) > 0) {
			// Counter for frame timestamp diff is the same, increment
			TimestampCounter count = timestampDifferenceCounts.at(timestampDiff);
			count.timestampCount++;
			timestampDifferenceCounts.erase(timestampDiff);
			timestampDifferenceCounts.insert({ timestampDiff, count });
		}
		else {
			// Counter for frame timestamp is new, insert with count of 1
			timestampDifferenceCounts.insert({ timestampDiff, {fileName, 1} });
		}
	}

	// Determine the frame with the max usec timestamp difference before it, this will be the true starting capture
	uint64_t maxTimestampDiff = 0;
	for (auto& x : timestampDifferenceCounts) {
		uint64_t timestampDiff = x.first;
		TimestampCounter timestampCounter = x.second;
		std::string fileName = timestampCounter.fileName;
		int count = timestampCounter.timestampCount;

		// Only process timestamp info with a count of > 3 and if the timestamp difference is greater than the max tracked timestamp so far
		if (count > 3 && timestampDiff > maxTimestampDiff) {
			maxTimestampDiff = timestampDiff;
		}
	}

	// If no timestamp could be evaluated, return empty name
	if (maxTimestampDiff == 0) {
		return "";
	}

	//CalibrationInfo startFrameInfo = { timestampDifferenceCounts.at(maxTimestampDiff).fileName, maxTimestampDiff };
	return timestampDifferenceCounts.at(maxTimestampDiff).fileName;
}

KinectAzureProcessor::FrameInfo KinectAzureProcessor::getNextFrame(int fileCount, recording_t* files) {
	uint64_t minTimestamp = (uint64_t)-1;
	KinectAzureProcessor::recording_t* nextFile = NULL;

	int index = 0;
	// Find the lowest timestamp out of each of the current captures.
	for (size_t i = 0; i < fileCount; i++)
	{
		if (files[i].capture != NULL)
		{
			uint64_t timestamp = first_capture_timestamp(files[i].capture);
			if (timestamp < minTimestamp)
			{
				index = i;
				minTimestamp = timestamp;
				nextFile = &files[i];
			}
		}
	}

	return {index, minTimestamp, nextFile};
}

Ply KinectAzureProcessor::generatePointCloud(KinectAzureProcessor::FrameInfo frameInfo, k4a_calibration_t* calibrations) {
	// Generate xy table
	k4a_image_t xyTable = NULL;

	k4a_result_t result1 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		calibrations[frameInfo.index].depth_camera_calibration.resolution_width,
		calibrations[frameInfo.index].depth_camera_calibration.resolution_height,
		calibrations[frameInfo.index].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
		&xyTable);


	// Create xy table
	k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xyTable);


	int width = calibrations[frameInfo.index].depth_camera_calibration.resolution_width;
	int height = calibrations[frameInfo.index].depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				&calibrations[frameInfo.index], &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
		}
	}

	// Generate point cloud
	const k4a_image_t depthImage = k4a_capture_get_depth_image(frameInfo.file->capture);
	int depthWidth = k4a_image_get_width_pixels(depthImage);
	int depthHeight = k4a_image_get_height_pixels(depthImage);

	uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depthImage);
	k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xyTable);

	Ply ply = Ply();
	ply.setFileName(frameInfo.file->filename);
	for (int i = 0; i < depthWidth * depthHeight; i++)
	{
		if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
		{
			Eigen::RowVector3d point;
			point(0) = xy_table_data[i].xy.x * (float)depth_data[i];
			point(1) = xy_table_data[i].xy.y * (float)depth_data[i];
			point(2) = (float)depth_data[i];
			ply.addPoint(point);
		}
	}

	return ply;

}

bool KinectAzureProcessor::checkSubjectWithinCaptureSpace(std::vector<Eigen::RowVector3d> jointPositions, BodyTrackingUtils::BoundingBox captureSpaceBounds) {
	// Check if all joint locations are contained within the capture space bounds. If not, then the frame should not be processed.
	for (Eigen::RowVector3d jointPosition : jointPositions) {
		if (!BodyTrackingUtils::withinBounds(jointPosition, captureSpaceBounds)) {
			return false;
		}
	}

	return true;
}

void KinectAzureProcessor::outputPointCloudGroup(std::vector<Ply> plys, uint64_t groupCount, std::unordered_map<std::string, Eigen::Matrix4Xd> transformations,
	std::vector<Eigen::RowVector3d> jointPositions, BodyTrackingUtils::BoundingBox captureSpaceBounds, std::string bodyTrackingFileSuffix) {

	// Calculate different behavior settings:
	bool individualOutputUnfiltered = debugMode || (calibrationMode && transformations.size() == 0);
	bool individualOutputFiltered = debugMode || (calibrationMode && transformations.size() > 0);
	bool unprocessedMergedOutput = debugMode;
	bool filterAndMeshing = !calibrationMode;

	std::stringstream outputFileName;
	outputFileName << "Group" << groupCount << "_";

	// Apply transformations to move all point clouds into master coordinate system
	std::vector<Ply> transformedPlys = MatrixUtils::applyTransforms(plys, transformations);

	BodyTrackingUtils::BoundingBox boundingBox;
	// If there are joint positions, create a bounding box based off of them
	if (jointPositions.size() > 0) {
		// Grab transformation for body tracking file to apply to jointPositions
		bool jointTransformFound = false;
		Eigen::Matrix4Xd jointPositionTransformation;
		for (auto it = transformations.begin(); it != transformations.end(); it++) {
			// For each pc, loop through transform and find matches
			if (it->first.compare(bodyTrackingFileSuffix) == 0) {
				jointPositionTransformation = it->second;
				jointTransformFound = true;
				break;
			}
		}

		if (debugMode) {
			// Output individual json for frame
			std::stringstream jointOutputName;
			jointOutputName << captureDirectory << "\\Joints_" << groupCount << ".txt";
			ofstream out;
			out.open(jointOutputName.str(), ios::out);
			for (Eigen::RowVector3d jointXYZ : jointPositions) {
				out << jointXYZ(0) << " " << jointXYZ(1) << " " << jointXYZ(2) << endl;
			}
			out.close();
		}

		// Apply transform to joint locations
		// Check capture space bounding box
		//	If there are any joints located outside of the capture space, the output will be skipped
		// Create Bounding box
		if (jointTransformFound) {
			std::vector<Eigen::RowVector3d> jointPositionsTransformed = MatrixUtils::applyJointTrackingTransform(jointPositions, jointPositionTransformation);
			if (!checkSubjectWithinCaptureSpace(jointPositionsTransformed, captureSpaceBounds)) {
				std::cerr << "Subject is outside of capture space. Skipping output." << endl;
				return;
			}
			boundingBox = BodyTrackingUtils::createBoundingBox(jointPositionsTransformed);

		}
		else {
			if (!checkSubjectWithinCaptureSpace(jointPositions, captureSpaceBounds)) {
				std::cerr << "Subject is outside of capture space. Skipping output." << endl;
				return;
			}
			boundingBox = BodyTrackingUtils::createBoundingBox(jointPositions);
		}

		// Add 'f' modifier to output file name
		outputFileName << "f";
	}
	else {
		// Case when there is no joint locations

		// Bounding box is just going to be the entire capture space bounds, since we do not have joint locations to isolate a subject
		boundingBox = captureSpaceBounds;

		// Skip filtering and meshing since the capture will not be isolated
		filterAndMeshing = false;

		// Output unprocessed merged point cloud output, since additonal meshing and processing will not occur
		unprocessedMergedOutput = true;

		// Add 'a' modifier to output file name
		outputFileName << "a";
	}

	// Apply bounds, convert to pcd file, and filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudCombined(new pcl::PointCloud<pcl::PointXYZ>);
	int index = 0;
	for (Ply pc : transformedPlys) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudSingle(new pcl::PointCloud<pcl::PointXYZ>);
		for (Eigen::RowVector3d point : pc.getPoints()) {
			if (BodyTrackingUtils::withinBounds(point, boundingBox)) {
				pclCloudSingle->points.push_back({ (float)point(0), (float)point(1), (float)point(2) });
			}
		}

		// Calculate individual file name
		std::string individualFileNameIdentifier;
		// Only calculate if any individual file output is specified
		if (individualOutputUnfiltered || individualOutputFiltered) {
			individualFileNameIdentifier = IOUtils::extractFileName(pc.getFileName());
		}
		
		// In calibration mode, output point cloud before statistical outlier filtering is applied
		if (pclCloudSingle->size() > 0 && individualOutputUnfiltered) {
			pclCloudSingle->resize(pclCloudSingle->size());
			std::stringstream singleFileName;
			singleFileName << captureDirectory << "\\Unfiltered_IndividualPc_" << groupCount << "_" << individualFileNameIdentifier << ".pcd";
			PclUtils::outputToFile(pclCloudSingle, singleFileName.str());
		}

		// Apply statistical outlier filter
		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPc(new pcl::PointCloud<pcl::PointXYZ>);
		PclUtils::applyStatisticalOutlierFilter(pclCloudSingle, filteredPc);

		// For single frame output
		if (filteredPc->size() > 0 && individualOutputFiltered) {
			std::stringstream filteredFileName;
			filteredFileName << captureDirectory << "\\IndividualPc_" << groupCount << "_" << individualFileNameIdentifier << ".pcd";
			PclUtils::outputToFile(filteredPc, filteredFileName.str());
		}

		*pclCloudCombined += *filteredPc;
		index++;
	}

	// For merged point cloud output before downsampling and smoothing
	if (unprocessedMergedOutput) {
		std::string pcFileName = std::string(outputFileName.str());
		pcFileName += "_full.pcd";
		std::string fullFilePath = std::string(captureDirectory);
		fullFilePath += "\\";
		fullFilePath += pcFileName;
		PclUtils::outputToFile(pclCloudCombined, fullFilePath);
	}

	// Meshing
	if (filterAndMeshing) {
		std::string meshFileName = std::string(outputFileName.str());
		std::string fullMeshPath = std::string(captureDirectory);
		fullMeshPath += "\\";
		fullMeshPath += meshFileName;
		PclUtils::resampleAndMesh(pclCloudCombined, fullMeshPath, disableMeshOutput);
	}
}

bool KinectAzureProcessor::openFiles(KinectAzureProcessor::recording_t** filess, k4a_calibration_t** calibrationss, k4abt_tracker_t &tracker, std::vector<std::string> mkvFiles, std::string btFileSuffix) {
	if (mkvFiles.size() < 1) {
		printf("At least one mkv video must be found for processing");
		return false;
	}

	KinectAzureProcessor::recording_t* files = *filess;
	k4a_calibration_t* calibrations = *calibrationss;

	size_t file_count = (size_t)(mkvFiles.size());
	bool master_found = false;
	k4a_result_t result = K4A_RESULT_SUCCEEDED;

	// Open each recording file and validate they were recorded in master/subordinate mode.
	for (int i = 0; i < file_count; i++)
	{
		char* filename = const_cast<char*>(mkvFiles.at(i).c_str());
		files[i].filename = mkvFiles.at(i);
		result = k4a_playback_open(filename, &files[i].handle);

		if (result != K4A_RESULT_SUCCEEDED)
		{
			printf("Failed to open file: %s\n", filename);
			break;
		}

		result = k4a_playback_get_record_configuration(files[i].handle, &files[i].record_config);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			printf("Failed to get record configuration for file: %s\n", filename);
			break;
		}

		if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
		{
			printf("Opened master recording file: %s\n", filename);
			if (master_found)
			{
				printf("ERROR: Multiple master recordings listed!\n");
				result = K4A_RESULT_FAILED;
				break;
			}
			else
			{
				master_found = true;
			}
		}
		else if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
		{
			printf("Opened subordinate recording file: %s\n", filename);
		}
		else
		{
			printf("ERROR: Recording file was not recorded in master/sub mode: %s\n", filename);
			result = K4A_RESULT_FAILED;
			break;
		}

		k4a_result_t calibrationResult = k4a_playback_get_calibration(files[i].handle, &calibrations[i]);
		if (calibrationResult != K4A_RESULT_SUCCEEDED) {
			printf("Error getting calibration for: %d\n", i);
			return false;
		}

		// Set up body tracking if body tracking file suffix is defined and the capture file names ends with that suffix
		if (btFileSuffix.compare("") != 0 && IOUtils::endsWith(filename, btFileSuffix)) {
			k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
			result = k4abt_tracker_create(&calibrations[i], tracker_config, &tracker);
			if (result != K4A_RESULT_SUCCEEDED)
			{
				std::cerr << "Body tracker initialization failed!" << std::endl;
				result = K4A_RESULT_FAILED;
				break;
			}
		}

		// Read the first capture of each recording into memory.
		k4a_stream_result_t stream_result = k4a_playback_get_next_capture(files[i].handle, &files[i].capture);
		if (stream_result == K4A_STREAM_RESULT_EOF)
		{
			printf("ERROR: Recording file is empty: %s\n", filename);
			result = K4A_RESULT_FAILED;
			break;
		}
		else if (stream_result == K4A_STREAM_RESULT_FAILED)
		{
			printf("ERROR: Failed to read first capture from file: %s\n", filename);
			result = K4A_RESULT_FAILED;
			break;
		}
	}

	if (tracker == NULL) {
		printf("Warning: Capture for body tracking was not found. Body tracking will not be processed.\n");
	}

	return result == K4A_RESULT_SUCCEEDED;
}

uint64_t KinectAzureProcessor::getMaxTimestampDiff(k4a_calibration_t** calibrations, int fileCount) {
	for (int i = 0; i < fileCount; i++) {
		// If any capture is running in max resolution, assume 15fps
		if (calibrations[0]->depth_camera_calibration.resolution_height == 1024) {
			return timestampDiff15;
		}
	}

	// Otherwise assume 30fps
	return timestampDiff30;
}

int KinectAzureProcessor::outputRecordingsToPlyFiles(std::unordered_map<std::string, Eigen::Matrix4Xd> transformations, std::string btFileSuffix, BodyTrackingUtils::BoundingBox captureSpaceBounds) {
	// Grab filenames to read in
	std::vector<std::string> mkvFiles = IOUtils::obtainMkvFilesFromDirectory(captureDirectory);

	int fileCount = mkvFiles.size();
	std::string masterFileSuffix = "Master.mkv";

	KinectAzureProcessor::recording_t* files = (KinectAzureProcessor::recording_t*)malloc(sizeof(KinectAzureProcessor::recording_t) * fileCount);
	k4a_calibration_t* calibrations = (k4a_calibration_t*)malloc(sizeof(k4a_calibration_t) * fileCount);
	// Allocate memory to store the state of N recordings.
	if (files == NULL)
	{
		printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(KinectAzureProcessor::recording_t) * fileCount);
		return 1;
	}
	memset(files, 0, sizeof(KinectAzureProcessor::recording_t) * fileCount);
	memset(calibrations, 0, sizeof(k4a_calibration_t) * fileCount);
	k4abt_tracker_t tracker = NULL;

	// Open files
	bool openRecordingSuccess = openFiles(&files, &calibrations, tracker, mkvFiles, btFileSuffix);
	if (!openRecordingSuccess) {
		return 1;
	}

	if (bodyTrackingOutputOnly && tracker == NULL) {
		cerr << "Error: Body tracking only argument was specified, but not body tracking file was specified in the transformation file. Ending processing." << endl;
		return 1;
	}

	k4a_result_t result = K4A_RESULT_SUCCEEDED;

	// Track frame indices 
	std::map<std::string, uint64_t> fileIndexCounter;
	for (int i = 0; i < fileCount; i++) {
		fileIndexCounter.insert({mkvFiles.at(i), 0});
	}

	// Calibration variables
	uint64_t previousTimestamp = (uint64_t)0;
	int discardFrameEnd = fileCount * 15;
	int orderingFrameEnd = fileCount * 15 * 2;
	std::vector<CalibrationInfo> calibrationInfo;

	// Processing variables
	int captureCount = 0;
	int groupCount = 0;
	int previousGroupCount = 0;
	uint64_t startGroupTimestamp = 0;
	std::string startFileName;
	std::string previousFileName;
	// Assume all files are run with the same frame rate
	uint64_t maxTimestampDiff = getMaxTimestampDiff(&calibrations, fileCount);
	std::vector<Ply> groupFrames;


	// Joint tracking variables
	bool trackerCaptureFound = (tracker != NULL);
	bool jointsObtained = false;
	std::vector<Eigen::RowVector3d> sub2Joints;
	ptree jointOutputJson, framesJson;
	std::vector<Eigen::RowVector3d> joints;
	double jointFrameModifier = 0.0;

	// Loop variables
	int endThreshold = 3; // Number of consecutive frames without a master capture before we decide to end the processing
	int missingFrameCount = 0; // Track the number of consecutive frames without all the captures
	uint64_t frame = 0;
	bool endProcessing = false;
	bool individualFrameProcessed = false;

	// Get frames in order
	while(true)
	{
		// First check to see if master frame has not been found for a consecutive number of frames
		// If individual frame is specified and has already been output, end processing
		if (endProcessing || missingFrameCount >= endThreshold) {
			// If so, stop processing and exit
			
			// Output joints json if tracker capture is in use
			if (trackerCaptureFound) {
				// Create an output stream
				std::ofstream outfile;

				// Remove Master.mkv
				// Add .json
				std::string outfileName = std::string(captureDirectory);
				outfileName = outfileName.append("\\joints.json");

				// Open the file to write to
				outfile.open(outfileName, ios::out | ios::trunc);

				jointOutputJson.add_child("frames", framesJson);
				BodyTrackingUtils::seedJointNames(&jointOutputJson);

				std::ostringstream oss;
				boost::property_tree::write_json(oss, jointOutputJson);
				std::regex reg("\\\"([0-9\-]+\\.{0,1}[0-9]*)\\\"");
				std::string result = std::regex_replace(oss.str(), reg, "$1");

				std::ofstream file;
				file.open(outfileName);
				file << result;
				file.close();
			}

			break;
		}
		else if (individualFrameProcessed) {
			// No need to write json when we are only processing a single frame
			// Just break out of loop
			break;
		}

		// Grab next frame
		KinectAzureProcessor::FrameInfo frameInfo = getNextFrame(fileCount, files);
		
		// Once the next frame is obtained, check if it is the end of the capture
		// The file will be null is the capture has ended
		if (frameInfo.file == NULL) {
			// If the file is null, end processing and continue to the next iteration, where the json joints will be output if applicable
			endProcessing = true;
			continue;
		}

		// Increment the frame's counter
		fileIndexCounter[frameInfo.file->filename] = fileIndexCounter[frameInfo.file->filename]++;

		// Get the timestamp from the frame
		uint64_t timestamp = (uint64_t)frameInfo.timestamp - previousTimestamp;

		if (frame >= discardFrameEnd && frame < orderingFrameEnd) {
			uint64_t timestamp = (uint64_t)frameInfo.timestamp - previousTimestamp;
			//std::cout << std::endl << "Frame" << frame << " Timestamp difference: " << timestamp << std::endl;
			previousTimestamp = frameInfo.timestamp;
			//print_capture_info(frameInfo.file);
			CalibrationInfo ci = { std::string(frameInfo.file->filename), timestamp };
			calibrationInfo.push_back(ci);
		}
		else if (frame == orderingFrameEnd) {
			// Perform all calibrations here
			startFileName = getStartRecording(files, calibrationInfo, fileCount);
			if (startFileName == "") {
				std::cerr << "Error evaluating start frame info." << std::endl;
				return 1;
			}
		}
		else if (frame > orderingFrameEnd) {
			// Group frames together
			uint64_t groupTimestampDiff = frameInfo.timestamp - startGroupTimestamp;
			if (startGroupTimestamp == 0) {
				// Start of processing
				// Continue until we get to the starting file
				if (std::string(frameInfo.file->filename).compare(startFileName) == 0) {
					startGroupTimestamp = frameInfo.timestamp;
				}
			}
			else if (startGroupTimestamp != 0 && groupCount == 0 && groupTimestampDiff <= maxTimestampDiff && std::string(frameInfo.file->filename).compare(previousFileName) == 0) {
				// Case where a capture appears twice in a row (can happen for Body tracking capture running at double fps of other captures)
				startGroupTimestamp = frameInfo.timestamp;
			}
			else if (groupTimestampDiff > maxTimestampDiff) {
				// Process previous group
				// Don't process first group
				if (groupCount != 0) {
					// Only process group if it contains all frames along with body tracking* (only if tracker capture exists)
					if (captureCount >= fileCount && (!trackerCaptureFound || jointsObtained)) {
						if (individualFrameIndex == -1 || groupCount >= individualFrameIndex) {
							missingFrameCount = 0;

							// Decrement frame count for the current frame, this will not be processed and outputted
							fileIndexCounter[frameInfo.file->filename] = fileIndexCounter[frameInfo.file->filename]--;

							// Output combined point cloud
							// If body tracking only is specified, skip this output and only focus on joint tracking
							if (!bodyTrackingOutputOnly) {
								outputPointCloudGroup(groupFrames, groupCount, transformations, joints, captureSpaceBounds, btFileSuffix);
							}

							// Re-increment frame count since it will be accurate for the next group processing
							fileIndexCounter[frameInfo.file->filename] = fileIndexCounter[frameInfo.file->filename]++;

							if (individualFrameIndex > 0) {
								individualFrameProcessed = true;
							}
						}
					}
					else if (captureCount < fileCount) {
						missingFrameCount++;
						std::cerr << "Frames missing for group " << groupCount << ". Skipping." << std::endl;
						if (groupCount == individualFrameIndex) {
							std::cerr << "Frames missing for specified output frame: " << groupCount << ". Next available set of frames will be output." << std::endl;
						}
					}
					else if (trackerCaptureFound && !jointsObtained) {
						std::cerr << "Body tracking missing for group " << groupCount << ". Skipping." << std::endl;
						if (groupCount == individualFrameIndex) {
							std::cerr << "Body tracking missing for specified output frame: " << groupCount << ". Next available set of frames will be output." << std::endl;
						}
					}
				}

				// Print frame info for next group
				groupCount++;
				if (groupCount != previousGroupCount && !individualFrameProcessed) {
					previousGroupCount = groupCount;
					std::cout << std::endl << std::endl << "Processing Group " << groupCount << std::endl;
					print_capture_info(frameInfo.file);
				}

				// Restart group
				jointsObtained = false;
				groupFrames.clear();
				groupFrames.shrink_to_fit();
				startGroupTimestamp = frameInfo.timestamp;
				joints.clear();
				jointFrameModifier = 0.0;
				captureCount = 0;

				// Add current frame to group
				captureCount++;
				if (!skipBtCloudProcessing || !IOUtils::endsWith(frameInfo.file->filename, btFileSuffix)) {
					groupFrames.push_back(generatePointCloud(frameInfo, calibrations));
				}

				// If capture is the desired body tracking file, process joint tracking data
				if (trackerCaptureFound && IOUtils::endsWith(frameInfo.file->filename, btFileSuffix) && !individualFrameProcessed) {
					jointsObtained = BodyTrackingUtils::predictJoints(&framesJson, groupCount + jointFrameModifier, tracker, frameInfo.file->capture, &joints);
					jointFrameModifier += 0.5;
				}
			}
			else {
				print_capture_info(frameInfo.file);
				captureCount++;

				// Add current frame to group
				if (!skipBtCloudProcessing || !IOUtils::endsWith(frameInfo.file->filename, btFileSuffix)) {
					groupFrames.push_back(generatePointCloud(frameInfo, calibrations));
				}

				// If capture is the desired body tracking file, process joint tracking data
				if (trackerCaptureFound && IOUtils::endsWith(frameInfo.file->filename, btFileSuffix) && groupCount != 0) {
					jointsObtained = BodyTrackingUtils::predictJoints(&framesJson, groupCount + jointFrameModifier, tracker, frameInfo.file->capture, &joints);
					jointFrameModifier += 0.5;
				}
			}
		}

		previousFileName = frameInfo.file->filename;
		k4a_capture_release(frameInfo.file->capture);
		frameInfo.file->capture = NULL;

		// Advance the recording with the lowest current timestamp forward.
		k4a_stream_result_t stream_result = k4a_playback_get_next_capture(frameInfo.file->handle, &frameInfo.file->capture);
		if (stream_result == K4A_STREAM_RESULT_FAILED)
		{
			printf("ERROR: Failed to read next capture from file: %s\n", frameInfo.file->filename.c_str());
			result = K4A_RESULT_FAILED;
			break;
		}

		frame++;
	}

	for (size_t i = 0; i < fileCount; i++)
	{
		if (files[i].handle != NULL)
		{
			k4a_playback_close(files[i].handle);
			files[i].handle = NULL;
		}
	}
	free(files);

	return result == K4A_RESULT_SUCCEEDED ? 0 : 1;
}

