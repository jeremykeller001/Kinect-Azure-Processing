//
// KinectAzureUtils.cpp
// Custom Utilities dealing with the Kinect Azure SDK
//
// Created by: Jeremy Keller
// 2/03/2020
//


#include "KinectAzureUtils.h"
#include "BodyTrackingUtils.h"

// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/fastpointcloud/main.cpp
void KinectAzureUtils::createXYTable(const k4a_calibration_t* calibration, k4a_image_t xy_table) {
	k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

	int width = calibration->depth_camera_calibration.resolution_width;
	int height = calibration->depth_camera_calibration.resolution_height;

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
				calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

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
}

Ply KinectAzureUtils::generatePly(KinectAzureUtils::FrameInfo frameInfo, const k4a_image_t xy_table)
{
	const k4a_image_t depthImage = k4a_capture_get_depth_image(frameInfo.file->capture);
	int width = k4a_image_get_width_pixels(depthImage);
	int height = k4a_image_get_height_pixels(depthImage);

	uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depthImage);
	k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

	Ply ply = Ply();
	ply.setFileName(frameInfo.file->filename);
	for (int i = 0; i < width * height; i++)
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

void KinectAzureUtils::print_capture_info(recording_t* file)
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

uint64_t KinectAzureUtils::first_capture_timestamp(k4a_capture_t capture)
{
	uint64_t timestamp = k4a_image_get_device_timestamp_usec(k4a_capture_get_depth_image(capture));
	k4a_image_release(k4a_capture_get_depth_image(capture));
	return timestamp;
}

std::string KinectAzureUtils::getStartRecording(recording_t* files, std::vector<CalibrationInfo> frameInfo, int fileCount) {
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

	CalibrationInfo startFrameInfo = { timestampDifferenceCounts.at(maxTimestampDiff).fileName, maxTimestampDiff };
	return timestampDifferenceCounts.at(maxTimestampDiff).fileName;
}

KinectAzureUtils::FrameInfo KinectAzureUtils::getNextFrame(int fileCount, recording_t* files) {
	uint64_t minTimestamp = (uint64_t)-1;
	KinectAzureUtils::recording_t* nextFile = NULL;

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

Ply KinectAzureUtils::generatePointCloud(KinectAzureUtils::FrameInfo frameInfo, k4a_calibration_t* calibrations) {
	// Generate xy table
	k4a_image_t xyTable = NULL;

	k4a_result_t result1 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		calibrations[frameInfo.index].depth_camera_calibration.resolution_width,
		calibrations[frameInfo.index].depth_camera_calibration.resolution_height,
		calibrations[frameInfo.index].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
		&xyTable);

	createXYTable(&calibrations[frameInfo.index], xyTable);

	return generatePly(frameInfo, xyTable);
}

void KinectAzureUtils::outputPointCloudGroup(std::vector<Ply> plys, uint64_t groupCount,
	std::unordered_map<std::string, Eigen::Matrix4Xd> transformations, std::string outputPath, std::vector<Eigen::RowVector3d> jointPositions) {

	// Apply transformations to move all point clouds into master coordinate system
	Ply combinedPly = MatrixUtils::applyTransforms(plys, transformations);
	
	// Filter points if joint positions are given
	if (jointPositions.size() > 0) {
		//
		// Filter out points not in bounding box;
		//

		// Grab transformation for Sub2 to apply to jointPositions
		Eigen::Matrix4Xd jointPositionTransformation;
		for (auto it = transformations.begin(); it != transformations.end(); it++) {
			// For each pc, loop through transform and find matches
			if (it->first.compare("Sub2.mkv") == 0) {
				jointPositionTransformation = it->second;
			}
		}

		// Apply transform to joint locations
		std::vector<Eigen::RowVector3d> jointPositionsTransformed = MatrixUtils::applyJointTrackingTransform(jointPositions, jointPositionTransformation);

		// Create bounding box
		BodyTrackingUtils::BoundingBox boundingBox = BodyTrackingUtils::createBoundingBox(jointPositionsTransformed);

		// Filter out points from the combined Ply if they do not fit within the bounding box
		Ply combinedFilteredPly;
		for (Eigen::RowVector3d point : combinedPly.getPoints()) {
			if (BodyTrackingUtils::withinBounds(point, boundingBox)) {
				combinedFilteredPly.addPoint(point);
			}
		}

		//
		// Output to file
		// Only if points exist in the cloud
		//
		if (combinedFilteredPly.getPointCount() > 0) {
			combinedFilteredPly.outputToFile(groupCount, outputPath);
		}
		else {
			std::cerr << "Warning: combined filtered point cloud in group " << groupCount << " had all of its points filtered out. This may be a potential bounding box error." << std::endl;
		}
	}
	else {
		// If there is no joint tracking, just output the raw point cloud data
		combinedPly.outputToFile(groupCount, outputPath);
	}

}

bool KinectAzureUtils::openFiles(KinectAzureUtils::recording_t** filess, k4a_calibration_t** calibrationss, k4abt_tracker_t &tracker, std::vector<std::string> mkvFiles) {
	if (mkvFiles.size() < 1) {
		printf("At least one mkv video must be found for processing");
		return false;
	}

	KinectAzureUtils::recording_t* files = *filess;
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

		// Set up body tracking if file is sub2
		if (IOUtils::endsWith(filename, "Sub2.mkv")) {
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
		printf("Warning: Sub2 capture for body tracking was not found. Body tracking will not be processed.\n");
	}

	return result == K4A_RESULT_SUCCEEDED;
}

void KinectAzureUtils::closeFiles(int fileCount, KinectAzureUtils::recording_t** files) {
	for (size_t i = 0; i < fileCount; i++)
	{
		if (files[i]->handle != NULL)
		{
			k4a_playback_close(files[i]->handle);
			files[i]->handle = NULL;
		}
	}
	free(files);
}

int KinectAzureUtils::outputRecordingsToPlyFiles(std::string dirPath, std::string transformFilePath) {
	// Grab filenames to read in
	std::vector<std::string> mkvFiles = IOUtils::obtainMkvFilesFromDirectory(dirPath);
	std::unordered_map<std::string, Eigen::Matrix4Xd> transformations = IOUtils::readTransformationFile(transformFilePath);
	int fileCount = mkvFiles.size();


	KinectAzureUtils::recording_t* files = (KinectAzureUtils::recording_t*)malloc(sizeof(KinectAzureUtils::recording_t) * fileCount);
	k4a_calibration_t* calibrations = (k4a_calibration_t*)malloc(sizeof(k4a_calibration_t) * fileCount);
	// Allocate memory to store the state of N recordings.
	if (files == NULL)
	{
		printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(KinectAzureUtils::recording_t) * fileCount);
		return 1;
	}
	memset(files, 0, sizeof(KinectAzureUtils::recording_t) * fileCount);
	memset(calibrations, 0, sizeof(k4a_calibration_t) * fileCount);
	k4abt_tracker_t tracker = NULL;

	// Open files
	bool openRecordingSuccess = openFiles(&files, &calibrations, tracker, mkvFiles);
	if (!openRecordingSuccess) {
		closeFiles(fileCount, &files);
		return 1;
	}

	k4a_result_t result = K4A_RESULT_SUCCEEDED;

	// Calibration variables
	uint64_t previousTimestamp = (uint64_t)0;
	int discardFrameEnd = fileCount * 15;
	int orderingFrameEnd = fileCount * 15 * 2;
	std::vector<CalibrationInfo> calibrationInfo;

	// Processing variables
	uint64_t groupCount = 0;
	uint64_t previousGroupCount = 0;
	uint64_t startGroupTimestamp = 0;
	std::string startFileName;
	// Assume all files are run with the same frame rate
	uint64_t maxTimestampDiff = calibrations[0].depth_camera_calibration.resolution_height == 1024 ? timestampDiff15 : timestampDiff30;
	std::vector<Ply> groupFrames;

	// Joint tracking variables
	bool trackerCaptureFound = (tracker != NULL);
	bool jointsObtained = false;
	std::vector<Eigen::RowVector3d> sub2Joints;

	// Loop variables
	int endThreshold = 5; // Number of consecutive frames without a master capture before we decide to end the processing
	int missingMasterCount = 0; // Track the number of consecutive frames without a master capture

	// Get frames in order
	for (int frame = 0; frame < 1000000; frame++)
	{
		if (missingMasterCount >= endThreshold) {
			frame = 1000000;
			break;
		}
		KinectAzureUtils::FrameInfo frameInfo = getNextFrame(fileCount, files);

		uint64_t timestamp = (uint64_t)frameInfo.timestamp - previousTimestamp;

		if (frame < discardFrameEnd) {
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
			// Print frame info
			if (groupCount != previousGroupCount) {
				previousGroupCount = groupCount;
				std::cout << "Processing Group " << groupCount << std::endl;
			}
			print_capture_info(frameInfo.file);

			// Group frames together
			uint64_t groupTimestampDiff = frameInfo.timestamp - startGroupTimestamp;
			if (startGroupTimestamp == 0) {
				// Start of processing
				// Continue until we get to the starting file
				if (std::string(frameInfo.file->filename).compare(startFileName) == 0) {
					startGroupTimestamp = frameInfo.timestamp;
					groupFrames.push_back(generatePointCloud(frameInfo, calibrations));
				}
			}
			else if (groupTimestampDiff > maxTimestampDiff) {
				// Process previous group
				// Don't process first group
				if (groupCount != 0) {
					// Check whether group contains master capture
					bool containsMaster = false;
					for (Ply& groupFrame : groupFrames) {
						if (IOUtils::endsWith(groupFrame.getFileName(), "Master.mkv")) {
							containsMaster = true;
							break;
						}
					}

					// Only process group if it contains master frame and body tracking (only if tracker capture exists)
					if (containsMaster && (!trackerCaptureFound || jointsObtained)) {
						missingMasterCount = 0;
						outputPointCloudGroup(groupFrames, groupCount, transformations, dirPath, sub2Joints);
					}
					else if (!containsMaster) {
						missingMasterCount++;
					}
					else if (trackerCaptureFound && !jointsObtained) {
						std::cerr << "Body tracking missing for group " << groupCount << ". Skipping." << std::endl;
					}
				}

				// Restart group
				std::cout << std::endl << std::endl;
				jointsObtained = false;
				groupFrames.clear();
				groupFrames.shrink_to_fit();
				groupCount++;
				startGroupTimestamp = frameInfo.timestamp;
				sub2Joints.clear();

				// Add current frame to group
				groupFrames.push_back(generatePointCloud(frameInfo, calibrations));

				// If capture is Sub2, process joint tracking data
				if (trackerCaptureFound && IOUtils::endsWith(frameInfo.file->filename, "Sub2.mkv")) {
					jointsObtained = BodyTrackingUtils::predictJoints(groupCount, tracker, frameInfo.file->capture, &sub2Joints);
				}
			}
			else {
				// Add current frame to group
				groupFrames.push_back(generatePointCloud(frameInfo, calibrations));

				// If capture is Sub2, process joint tracking data
				if (trackerCaptureFound && IOUtils::endsWith(frameInfo.file->filename, "Sub2.mkv")) {
					jointsObtained = BodyTrackingUtils::predictJoints(groupCount, tracker, frameInfo.file->capture, &sub2Joints);
				}
			}
		}

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
	}

	closeFiles(fileCount, &files);
	return result == K4A_RESULT_SUCCEEDED ? 0 : 1;
}

