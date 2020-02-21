//
// KinectAzureUtils.cpp
// Custom Utilities dealing with the Kinect Azure SDK
//
// Created by: Jeremy Keller
// 2/03/2020
//


#include "KinectAzureUtils.h"

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

void KinectAzureUtils::generate_point_cloud(const k4a_image_t depth_image,
	const k4a_image_t xy_table,
	k4a_image_t point_cloud,
	int* point_count)
{
	int width = k4a_image_get_width_pixels(depth_image);
	int height = k4a_image_get_height_pixels(depth_image);

	uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
	k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
	k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

	*point_count = 0;
	for (int i = 0; i < width * height; i++)
	{
		if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
		{
			point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
			point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
			point_cloud_data[i].xyz.z = (float)depth_data[i];
			(*point_count)++;
		}
		else
		{
			point_cloud_data[i].xyz.x = nanf("");
			point_cloud_data[i].xyz.y = nanf("");
			point_cloud_data[i].xyz.z = nanf("");
		}
	}
}

Ply KinectAzureUtils::generatePly(FrameInfo frameInfo, const k4a_image_t xy_table) 
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

void KinectAzureUtils::write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count)
{
	int width = k4a_image_get_width_pixels(point_cloud);
	int height = k4a_image_get_height_pixels(point_cloud);

	k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

	// save to the ply file
	std::ofstream ofs(file_name); // text mode first
	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "element vertex"
		<< " " << point_count << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "end_header" << std::endl;
	ofs.close();

	std::stringstream ss;
	for (int i = 0; i < width * height; i++)
	{
		if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
		{
			continue;
		}

		ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
			<< (float)point_cloud_data[i].xyz.z << std::endl;
	}

	std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
	ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

Ply KinectAzureUtils::generatePly(const char* file_name, const k4a_image_t point_cloud, int point_count) {
	int width = k4a_image_get_width_pixels(point_cloud);
	int height = k4a_image_get_height_pixels(point_cloud);

	k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

	Ply ply;
	ply.setFileName(std::string(file_name));

	for (int i = 0; i < width * height; i++)
	{
		if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
		{
			continue;
		}

		Eigen::RowVector3d point;
		point(0) = (double)point_cloud_data[i].xyz.x;
		point(1) = (double)point_cloud_data[i].xyz.y;
		point(2) = (double)point_cloud_data[i].xyz.z;
		ply.addPoint(point);
	}

	return ply;
}


void KinectAzureUtils::print_capture_info(recording_t* file)
{
	k4a_image_t image = k4a_capture_get_depth_image(file->capture);

	printf("%-32s", file->filename);

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

void KinectAzureUtils::orderAndIndexRecordings(recording_t* files, std::vector<CalibrationInfo> frameInfo, int fileCount) {
	// Assign the master file to be the first file, this is where we will start calibration
	std::string masterFile = std::string(files[0].filename);
	int startFrame = -1;
	for (int i = 0; i < frameInfo.size(); i++) {
		if (frameInfo.at(i).fileName.compare(masterFile) == 0) {
			startFrame = i;
			break;
		}
	}

	// If master frame is never found
	if (startFrame == -1) {
		// TODO: Throw exception
	}

	int endFrame = frameInfo.size() - 1;
	int currentIndex = 0;
	int previousStartFrameIndex = startFrame;


	// Initialize file index counters
	/*
	std::vector<FileIndexCounter> fileIndexCounters;
	for (int i = 1; i < fileCount; i++) {
		FileIndexCounter fileIndexCounter = { files[0].filename, fileCount };
		// Initialize all index counts to 0
		for (int j = 1; j < fileCount; j++) {
			fileIndexCounter.indexCounts[j] = 0;
		}
		fileIndexCounters.push_back(fileIndexCounter);
	}
	*/

	std::unordered_map<uint64_t, TimestampCounter> timestampDifferenceCounts;
	for (int i = startFrame + 1; i < endFrame; i++) {
		// Incremement timestampDifferenceCounts
		uint64_t timestampDiff = frameInfo.at(i).timestampDiff;
		std::string fileName = frameInfo.at(i).fileName;
		if (timestampDifferenceCounts.count(timestampDiff) > 0) {
			// Counter for frame timestamp diff is the same, increment
			TimestampCounter count = timestampDifferenceCounts.at(timestampDiff);
			count.timestampCount++;
			timestampDifferenceCounts.erase(timestampDiff);
			timestampDifferenceCounts.insert({timestampDiff, count});
		}
		else {
			// Counter for frame timestamp is new, insert with count of 1
			timestampDifferenceCounts.insert({ timestampDiff, {fileName, 1} });
		}

		/*
		// Reset each time master frame is found
		if(fileName.compare(masterFile) == 0) {
			previousStartFrameIndex = i;
		}
		else if (i - previousStartFrameIndex >= fileCount) {
			// If we do not hit the master file again after running the iteration for the file count, 
			// then this means the master file had a frame skip
			// Skip extra processing until the master file comes up again
			continue;
		}
		else {
			// Compare to file index counter info
			for (int j = 0; j < fileIndexCounters.size(); j++) {
				if (fileName.compare(fileIndexCounters.at(j).fileName) == 0) {
					// Compute diff between current index (i) count and previousStartFrameIndex
					int indexCountIndex = i - previousStartFrameIndex;
					fileIndexCounters.at(j).indexCounts[indexCountIndex]++;
				}
			}
		}
		*/
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

	// If no timestamp could be evaluated, throw exception
	if (maxTimestampDiff == 0) {
		// TODO: throw exception
	}

	CalibrationInfo startFrameInfo = { timestampDifferenceCounts.at(maxTimestampDiff).fileName, maxTimestampDiff };
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

	// If no timestamp could be evaluated, throw exception
	if (maxTimestampDiff == 0) {
		// TODO: throw exception
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

Ply KinectAzureUtils::generatePointCloud(FrameInfo frameInfo, k4a_calibration_t* calibrations) {
	// For debugging
	print_capture_info(frameInfo.file);

	// Generate xy table
	k4a_image_t xyTable = NULL;

	k4a_result_t result1 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		calibrations[0].depth_camera_calibration.resolution_width,
		calibrations[0].depth_camera_calibration.resolution_height,
		calibrations[0].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
		&xyTable);

	createXYTable(&calibrations[0], xyTable);

	return generatePly(frameInfo, xyTable);
}

Ply KinectAzureUtils::outputPointCloudGroup(std::vector<Ply> plys, uint64_t groupCount,
	std::unordered_map<std::string, Eigen::Matrix4Xd> transformations) {

	// TODO: figure out where body isolation fits in here

	// Match plys with transformations
	// apply transformation to each ply
	// Merge plys to master ply
	// Output merged ply to file with name: groupCount.ply
	return Ply();
}

int KinectAzureUtils::outputRecordingsToPlyFiles(std::string dirPath) {
	// Grab filenames to read in
	std::vector<std::string> mkvFiles = IOUtils::obtainMkvFilesFromDirectory(dirPath);
	std::unordered_map<std::string, Eigen::Matrix4Xd> transformations = IOUtils::readTransformationFile(dirPath);

	if (mkvFiles.size() < 1) {
		printf("At least one mkv video must be found for processing");
		return 1;
	}

	size_t file_count = (size_t)(mkvFiles.size());
	bool master_found = false;
	k4a_result_t result = K4A_RESULT_SUCCEEDED;

	// Allocate memory to store the state of N recordings.
	KinectAzureUtils::recording_t* files = (KinectAzureUtils::recording_t*)malloc(sizeof(KinectAzureUtils::recording_t) * file_count);
	k4a_calibration_t* calibrations = (k4a_calibration_t*)malloc(sizeof(k4a_calibration_t) * file_count);
	if (files == NULL)
	{
		printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(KinectAzureUtils::recording_t) * file_count);
		return 1;
	}
	memset(files, 0, sizeof(KinectAzureUtils::recording_t) * file_count);
	memset(calibrations, 0, sizeof(k4a_calibration_t) * file_count);

	// Open each recording file and validate they were recorded in master/subordinate mode.
	for (size_t i = 0; i < file_count; i++)
	{
		files[i].filename = const_cast<char*>(mkvFiles.at(i).c_str());
		result = k4a_playback_open(files[i].filename, &files[i].handle);

		if (result != K4A_RESULT_SUCCEEDED)
		{
			printf("Failed to open file: %s\n", files[i].filename);
			break;
		}

		result = k4a_playback_get_record_configuration(files[i].handle, &files[i].record_config);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			printf("Failed to get record configuration for file: %s\n", files[i].filename);
			break;
		}

		if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
		{
			printf("Opened master recording file: %s\n", files[i].filename);
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
			printf("Opened subordinate recording file: %s\n", files[i].filename);
		}
		else
		{
			printf("ERROR: Recording file was not recorded in master/sub mode: %s\n", files[i].filename);
			result = K4A_RESULT_FAILED;
			break;
		}

		// Start DU MOD

		k4a_result_t calibrationResult = k4a_playback_get_calibration(files[i].handle, &calibrations[i]);
		if (calibrationResult != K4A_RESULT_SUCCEEDED) {
			printf("Error getting calibration for: %d\n", i);
			return 1;
		}

		// End DU MOD


		// Read the first capture of each recording into memory.
		k4a_stream_result_t stream_result = k4a_playback_get_next_capture(files[i].handle, &files[i].capture);
		if (stream_result == K4A_STREAM_RESULT_EOF)
		{
			printf("ERROR: Recording file is empty: %s\n", files[i].filename);
			result = K4A_RESULT_FAILED;
			break;
		}
		else if (stream_result == K4A_STREAM_RESULT_FAILED)
		{
			printf("ERROR: Failed to read first capture from file: %s\n", files[i].filename);
			result = K4A_RESULT_FAILED;
			break;
		}
	}

	if (result == K4A_RESULT_SUCCEEDED)
	{
		// Calibration variables
		uint64_t previousTimestamp = (uint64_t)0;
		int discardFrameEnd = file_count * 15;
		int orderingFrameEnd = file_count * 15 * 2;
		std::vector<CalibrationInfo> calibrationInfo;

		// Processing variables
		uint64_t groupCount = 0;
		uint64_t startGroupTimestamp = 0;
		std::string startFileName;
		// Assume all files are run with the same calibration
		uint64_t maxTimestampDiff = calibrations[0].depth_camera_calibration.resolution_height == 1024 ? timestampDiff15 : timestampDiff30;
		std::vector<Ply> groupFrames;

		// Get frames in order
		for (int frame = 0; frame <= 1000000; frame++)
		{
			FrameInfo frameInfo = getNextFrame(file_count, files);

			uint64_t timestamp = (uint64_t)frameInfo.timestamp - previousTimestamp;
			std::cout << std::endl << "Frame" << frame << " Timestamp difference: " << timestamp << std::endl;

			if (frame > discardFrameEnd) {
				uint64_t timestamp = (uint64_t) frameInfo.timestamp - previousTimestamp;
				std::cout << std::endl << "Frame" << frame << " Timestamp difference: " << timestamp << std::endl;
				previousTimestamp = frameInfo.timestamp;
				print_capture_info(frameInfo.file);
				CalibrationInfo ci = { std::string(frameInfo.file->filename), timestamp };
				calibrationInfo.push_back(ci);
			}
			else if (frame == orderingFrameEnd) {
				// Perform all calibrations here
				startFileName = getStartRecording(files, calibrationInfo, file_count);
			}
			else if (frame > orderingFrameEnd) {
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


					// Restart group
					startGroupTimestamp = frameInfo.timestamp;
					
				}
				else {

				}
			}

			k4a_capture_release(frameInfo.file->capture);
			frameInfo.file->capture = NULL;

			// Advance the recording with the lowest current timestamp forward.
			k4a_stream_result_t stream_result = k4a_playback_get_next_capture(frameInfo.file->handle, &frameInfo.file->capture);
			if (stream_result == K4A_STREAM_RESULT_FAILED)
			{
				printf("ERROR: Failed to read next capture from file: %s\n", frameInfo.file->filename);
				result = K4A_RESULT_FAILED;
				break;
			}
		}
	}

	for (size_t i = 0; i < file_count; i++)
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

