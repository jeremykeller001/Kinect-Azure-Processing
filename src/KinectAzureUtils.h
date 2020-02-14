//
// KinectAzureUtils.h
// Custom Utilities dealing with the Kinect Azure SDK
//
// Created by: Jeremy Keller
// 2/03/2020
//


#pragma once
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <malloc.h>
#include <vector>
#include "IOUtils.h"
#include "Ply.h"

#ifndef KINECT_AZURE_UTILS_H
#define KINECT_AZURE_UTILS_H

class KinectAzureUtils
{
private:
	Ply generatePly(const char* file_name, const k4a_image_t point_cloud, int point_count);

	typedef struct
	{
		std::string fileName;
		uint64_t timestampDiff;
	} CalibrationInfo;

	typedef struct
	{
		char* filename;
		k4a_playback_t handle;
		k4a_record_configuration_t record_config;
		k4a_capture_t capture;
		int index;
	} recording_t;

	typedef struct
	{
		std::string fileName;
		int fileCount;
		int* indexCounts = new int[fileCount - 1];
	} FileIndexCounter;
	 
public:


	static void orderAndIndexRecordings(recording_t* files,  std::vector<CalibrationInfo> frameInfo, int fileCount);

	static void createXYTable(const k4a_calibration_t* calibration, k4a_image_t xy_table);

	static void generate_point_cloud(const k4a_image_t depth_image,
		const k4a_image_t xy_table,
		k4a_image_t point_cloud,
		int* point_count);

	static void write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count);
	
	static void print_capture_info(recording_t* file);

	static uint64_t first_capture_timestamp(k4a_capture_t capture);

	static int outputRecordingsToPlyFiles(std::string dirPath);
};
#endif
