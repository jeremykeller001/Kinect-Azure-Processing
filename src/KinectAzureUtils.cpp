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

uint64_t KinectAzureUtils::first_capture_timestamp(k4a_capture_t capture)
{
	// DU MOD
	uint64_t timestamp = k4a_image_get_device_timestamp_usec(k4a_capture_get_depth_image(capture));
	k4a_image_release(k4a_capture_get_depth_image(capture));
	return timestamp;
	// DU MOD
}

int KinectAzureUtils::outputRecordingsToPlyFiles(std::string dirPath) {
	// Grab filenames to read in
	std::vector<std::string> mkvFiles = IOUtils::obtainMkvFilesFromDirectory(dirPath);

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
		printf("%-32s  %12s  %12s  %12s\n", "Source file", "COLOR", "DEPTH", "IR");
		printf("==========================================================================\n");

		// Get frames in order
		// By default, set to be 1 second in + a random amount of time up to 3 seconds later
		int randMod = (file_count) * 15 * 3;
		srand(time(NULL));
		randMod = rand() % randMod;
		int startFrame = (file_count) * 15 + randMod;
		int endFrame = startFrame + (file_count);
		std::cout << "Start Frame: " << startFrame << std::endl;
		for (int frame = 0; frame <= endFrame; frame++)
		{
			uint64_t min_timestamp = (uint64_t)-1;
			KinectAzureUtils::recording_t* min_file = NULL;

			int index = 0;
			// Find the lowest timestamp out of each of the current captures.
			for (size_t i = 0; i < file_count; i++)
			{
				if (files[i].capture != NULL)
				{
					uint64_t timestamp = first_capture_timestamp(files[i].capture);
					if (timestamp < min_timestamp)
					{
						index = i;
						min_timestamp = timestamp;
						min_file = &files[i];
					}
				}
			}

			// DU MODIFICATION: Begin

			// Generate point cloud for this frame
			// Modify this value to cut off initial part of capture
			if (frame > startFrame) {
				k4a_image_t depthImage = k4a_capture_get_depth_image(min_file->capture);

				// Generate xy table
				// Generate point cloud
				auto t1 = std::chrono::steady_clock::now();
				k4a_image_t depth_image = NULL;
				k4a_image_t xy_table = NULL;
				k4a_image_t point_cloud = NULL;

				k4a_result_t result1 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
					calibrations[index].depth_camera_calibration.resolution_width,
					calibrations[index].depth_camera_calibration.resolution_height,
					calibrations[index].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
					&xy_table);

				createXYTable(&calibrations[index], xy_table);

				k4a_result_t result2 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
					calibrations[index].depth_camera_calibration.resolution_width,
					calibrations[index].depth_camera_calibration.resolution_height,
					calibrations[index].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
					&point_cloud);

				depth_image = k4a_capture_get_depth_image(min_file->capture);

				auto t2 = std::chrono::steady_clock::now();

				int point_count = 0;
				generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);

				auto t3 = std::chrono::steady_clock::now();

				std::ostringstream outputFileName;
				std::string fileName = min_file->filename;
				fileName = fileName.substr(0, fileName.find(".mkv"));
				outputFileName << fileName << "_" << frame << ".ply";

				std::cout << "output: " << outputFileName.str() << std::endl << std::endl;
				write_point_cloud(outputFileName.str().c_str(), point_cloud, point_count);

				auto t4 = std::chrono::steady_clock::now();
				// DU MODIFICATION: End
				// Uncomment this if you would like to pause in between frame outputs
			}

			k4a_capture_release(min_file->capture);
			min_file->capture = NULL;

			// Advance the recording with the lowest current timestamp forward.
			k4a_stream_result_t stream_result = k4a_playback_get_next_capture(min_file->handle, &min_file->capture);
			if (stream_result == K4A_STREAM_RESULT_FAILED)
			{
				printf("ERROR: Failed to read next capture from file: %s\n", min_file->filename);
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

